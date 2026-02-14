#!/usr/bin/env python3

import argparse
import csv
import os
import re
import tempfile
from datetime import datetime, timezone
from collections import defaultdict
from urllib.parse import urlparse

import numpy as np


TRANSITION_STATUSES = {"off_track", "pause", "prepare"}
ITERATION_FILENAME_PATTERN = re.compile(r"^(?P<iteration>\d+)-iteration\.csv$")
MS_PER_SECOND = 1000.0


def load_step_deltas(trace_file):
    per_episode_timestamps = defaultdict(list)

    with open(trace_file, newline="", encoding="utf-8") as file_obj:
        reader = csv.DictReader(file_obj)
        required_fields = {"episode", "tstamp", "episode_status"}
        missing = required_fields - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing required column(s): {', '.join(sorted(missing))}")

        for row in reader:
            status = (row.get("episode_status") or "").strip().lower()
            if status in TRANSITION_STATUSES:
                continue

            episode = int(float(row["episode"]))
            tstamp = float(row["tstamp"])
            per_episode_timestamps[episode].append(tstamp)

    per_episode_deltas = {}
    for episode, timestamps in per_episode_timestamps.items():
        if len(timestamps) < 2:
            continue
        deltas = np.diff(np.array(timestamps, dtype=np.float64))
        per_episode_deltas[episode] = deltas[deltas >= 0]

    return per_episode_deltas


def summarize(values):
    if values.size == 0:
        return None
    return {
        "avg": float(np.mean(values)),
        "std": float(np.std(values)),
        "max": float(np.max(values)),
        "p95": float(np.percentile(values, 95)),
        "count": int(values.size),
    }


def print_summary(title, stats):
    if not stats:
        print(f"{title}: no valid step deltas")
        return
    avg_ms = stats["avg"] * MS_PER_SECOND
    max_ms = stats["max"] * MS_PER_SECOND
    p95_ms = stats["p95"] * MS_PER_SECOND
    std_ms = stats["std"] * MS_PER_SECOND
    print(
        f"{title}: count={stats['count']}, avg_ms={avg_ms:.1f}, "
        f"max_ms={max_ms:.1f}, p95_ms={p95_ms:.1f}, std_ms={std_ms:.1f}"
    )


def parse_s3_uri(uri):
    parsed = urlparse(uri)
    if parsed.scheme != "s3" or not parsed.netloc:
        raise ValueError(f"Invalid S3 URI: {uri}")
    bucket = parsed.netloc
    prefix = parsed.path.lstrip("/")
    return bucket, prefix


def list_s3_keys(bucket, prefix):
    try:
        import boto3
    except ImportError as error:
        raise RuntimeError("boto3 is required for S3 input. Install with: pip install boto3") from error

    client = boto3.client("s3")
    paginator = client.get_paginator("list_objects_v2")
    keys = []
    for page in paginator.paginate(Bucket=bucket, Prefix=prefix):
        for entry in page.get("Contents", []):
            key = entry.get("Key", "")
            if key and not key.endswith("/"):
                keys.append(key)

    return sorted(keys)


def download_s3_prefix(bucket, prefix, download_root):
    try:
        import boto3
    except ImportError as error:
        raise RuntimeError("boto3 is required for S3 input. Install with: pip install boto3") from error

    keys = list_s3_keys(bucket, prefix)
    if not keys:
        return []

    print(f"Found {len(keys)} file(s) in s3://{bucket}/{prefix}")
    client = boto3.client("s3")

    local_paths = []
    for index, key in enumerate(keys, start=1):
        relative = key[len(prefix) :].lstrip("/") if prefix and key.startswith(prefix) else key
        if not relative:
            relative = os.path.basename(key)
        target_path = os.path.join(download_root, relative)
        os.makedirs(os.path.dirname(target_path), exist_ok=True)
        client.download_file(bucket, key, target_path)
        local_paths.append(target_path)
        print(f"Download [{index}/{len(keys)}]: s3://{bucket}/{key}")

    return local_paths


def flatten_deltas(per_episode_deltas):
    chunks = [values for values in per_episode_deltas.values() if values.size > 0]
    if not chunks:
        return np.array([], dtype=np.float64)
    return np.concatenate(chunks)


def extract_iteration(file_path):
    filename = os.path.basename(file_path)
    match = ITERATION_FILENAME_PATTERN.match(filename)
    if match:
        return int(match.group("iteration"))
    return None


def sort_key_for_file(file_path):
    iteration = extract_iteration(file_path)
    if iteration is None:
        return (1, file_path)
    return (0, iteration, file_path)


def list_csv_files_from_directory(source_dir):
    csv_files = []
    for root, _, files in os.walk(source_dir):
        for name in files:
            if name.lower().endswith(".csv"):
                csv_files.append(os.path.join(root, name))
    return sorted(csv_files, key=sort_key_for_file)


def print_iteration_table(iteration_rows):
    if not iteration_rows:
        print("Per-iteration summary: no valid iteration rows")
        return

    print("Per-iteration summary:")
    print(
        f"{'iteration':>9} {'steps':>10} {'avg_ms':>10} {'max_ms':>10} "
        f"{'p95_ms':>10} {'std_ms':>10} {'file':<s}"
    )
    print("-" * 110)
    for row in sorted(iteration_rows, key=lambda item: sort_key_for_file(item["file"])):
        iteration_label = str(row["iteration"]) if row["iteration"] is not None else "n/a"
        avg_ms = row["avg"] * MS_PER_SECOND
        max_ms = row["max"] * MS_PER_SECOND
        p95_ms = row["p95"] * MS_PER_SECOND
        std_ms = row["std"] * MS_PER_SECOND
        print(
            f"{iteration_label:>9} {row['count']:>10d} {avg_ms:>10.1f} "
            f"{max_ms:>10.1f} {p95_ms:>10.1f} {std_ms:>10.1f} {os.path.basename(row['file'])}"
        )


def process_files(file_paths):
    processed = 0
    all_deltas_chunks = []
    iteration_rows = []

    for index, file_path in enumerate(file_paths, start=1):
        try:
            per_episode_deltas = load_step_deltas(file_path)
        except Exception as error:
            print(f"Process [{index}/{len(file_paths)}]: skipped {file_path} ({error})")
            continue

        file_deltas = flatten_deltas(per_episode_deltas)
        if file_deltas.size == 0:
            print(f"Process [{index}/{len(file_paths)}]: skipped {file_path} (no valid deltas)")
            continue

        processed += 1
        all_deltas_chunks.append(file_deltas)

        file_stats = summarize(file_deltas)
        cumulative_stats = summarize(np.concatenate(all_deltas_chunks))
        iteration_rows.append(
            {
                "iteration": extract_iteration(file_path),
                "file": file_path,
                "count": file_stats["count"],
                "avg": file_stats["avg"],
                "max": file_stats["max"],
                "p95": file_stats["p95"],
                "std": file_stats["std"],
            }
        )
        file_avg_ms = file_stats["avg"] * MS_PER_SECOND
        file_max_ms = file_stats["max"] * MS_PER_SECOND
        file_p95_ms = file_stats["p95"] * MS_PER_SECOND
        file_std_ms = file_stats["std"] * MS_PER_SECOND
        cumulative_avg_ms = cumulative_stats["avg"] * MS_PER_SECOND
        cumulative_max_ms = cumulative_stats["max"] * MS_PER_SECOND
        cumulative_p95_ms = cumulative_stats["p95"] * MS_PER_SECOND
        cumulative_std_ms = cumulative_stats["std"] * MS_PER_SECOND
        print(
            f"Process [{index}/{len(file_paths)}]: {file_path} | "
            f"file(avg_ms={file_avg_ms:.1f}, max_ms={file_max_ms:.1f}, "
            f"p95_ms={file_p95_ms:.1f}, std_ms={file_std_ms:.1f}) | "
            f"cumulative(avg_ms={cumulative_avg_ms:.1f}, max_ms={cumulative_max_ms:.1f}, "
            f"p95_ms={cumulative_p95_ms:.1f}, std_ms={cumulative_std_ms:.1f})"
        )

    if processed == 0:
        return np.array([], dtype=np.float64), []
    return np.concatenate(all_deltas_chunks), iteration_rows


def default_download_dir(bucket, prefix):
    safe_prefix = prefix.strip("/").replace("/", "_") if prefix else "root"
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return os.path.abspath(
        os.path.join(os.getcwd(), f"simtrace_download_{bucket}_{safe_prefix}_{timestamp}")
    )


def print_evolution(per_episode_deltas):
    rows = []
    for episode in sorted(per_episode_deltas):
        deltas = per_episode_deltas[episode]
        if deltas.size == 0:
            continue
        rows.append(
            (
                episode,
                int(deltas.size),
                float(np.mean(deltas)),
                float(np.max(deltas)),
                float(np.percentile(deltas, 95)),
            )
        )

    if len(rows) < 2:
        print("Evolution: not enough episodes with valid deltas")
        return

    episodes = np.array([row[0] for row in rows], dtype=np.float64)
    avg_values = np.array([row[2] for row in rows], dtype=np.float64)
    max_values = np.array([row[3] for row in rows], dtype=np.float64)
    p95_values = np.array([row[4] for row in rows], dtype=np.float64)

    avg_slope = float(np.polyfit(episodes, avg_values, 1)[0])
    max_slope = float(np.polyfit(episodes, max_values, 1)[0])
    p95_slope = float(np.polyfit(episodes, p95_values, 1)[0])

    quartile_size = max(1, len(rows) // 4)
    first_quartile = rows[:quartile_size]
    last_quartile = rows[-quartile_size:]

    def mean_of(index, items):
        return float(np.mean([item[index] for item in items]))

    def pct_change(start, end):
        return ((end - start) / start * 100.0) if start != 0 else float("nan")

    first_avg, last_avg = mean_of(2, first_quartile), mean_of(2, last_quartile)
    first_max, last_max = mean_of(3, first_quartile), mean_of(3, last_quartile)
    first_p95, last_p95 = mean_of(4, first_quartile), mean_of(4, last_quartile)

    print(
        f"Evolution episodes: {len(rows)} (from {int(episodes.min())} to {int(episodes.max())})"
    )
    print("Linear trend slope per episode:")
    print(f"  avg slope_ms/ep={avg_slope * MS_PER_SECOND:.1f}")
    print(f"  max slope_ms/ep={max_slope * MS_PER_SECOND:.1f}")
    print(f"  p95 slope_ms/ep={p95_slope * MS_PER_SECOND:.1f}")
    print("First quartile vs last quartile mean:")
    print(
        f"  avg_ms: {first_avg * MS_PER_SECOND:.1f} -> {last_avg * MS_PER_SECOND:.1f} "
        f"({pct_change(first_avg, last_avg):+.2f}%)"
    )
    print(
        f"  max_ms: {first_max * MS_PER_SECOND:.1f} -> {last_max * MS_PER_SECOND:.1f} "
        f"({pct_change(first_max, last_max):+.2f}%)"
    )
    print(
        f"  p95_ms: {first_p95 * MS_PER_SECOND:.1f} -> {last_p95 * MS_PER_SECOND:.1f} "
        f"({pct_change(first_p95, last_p95):+.2f}%)"
    )

    window = 5
    if len(rows) >= window:
        print("Last 5 moving-average p95 points (ms):")
        moving_average = []
        for index in range(len(rows) - window + 1):
            end_episode = rows[index + window - 1][0]
            value = float(np.mean(p95_values[index : index + window]))
            moving_average.append((end_episode, value))
        for episode, value in moving_average[-5:]:
            print(f"  up_to_ep={episode}: p95_ma5_ms={value * MS_PER_SECOND:.1f}")

    ratios = []
    for row in rows:
        episode, count, avg_value, max_value, p95_value = row
        if p95_value > 0:
            ratios.append((max_value / p95_value, episode, count, avg_value, max_value, p95_value))
    ratios.sort(reverse=True)
    print("Top 5 spike ratios (max/p95):")
    for ratio, episode, count, avg_value, max_value, p95_value in ratios[:5]:
        print(
            f"  ep={episode}: ratio={ratio:.3f}, count={count}, avg_ms={avg_value * MS_PER_SECOND:.1f}, "
            f"max_ms={max_value * MS_PER_SECOND:.1f}, p95_ms={p95_value * MS_PER_SECOND:.1f}"
        )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze sim trace timing stability by computing tstamp step deltas per episode, "
            "excluding transition rows (off_track, pause, prepare)."
        )
    )
    parser.add_argument(
        "trace_source",
        help=(
            "Path to sim trace CSV file, local directory containing CSV files, "
            "or S3 URI such as s3://bucket/prefix"
        ),
    )
    parser.add_argument(
        "--overall-only",
        action="store_true",
        help="Only print aggregate stats across all episodes",
    )
    parser.add_argument(
        "--evolution",
        action="store_true",
        help="Print how avg/max/p95 evolve across episodes",
    )
    parser.add_argument(
        "--keep-downloaded",
        action="store_true",
        help="Keep downloaded S3 files in a local directory.",
    )
    parser.add_argument(
        "--download-dir",
        help=(
            "Directory to store S3 downloads when --keep-downloaded is set. "
            "If omitted, a timestamped directory is created in the current working directory."
        ),
    )
    args = parser.parse_args()

    if args.trace_source.startswith("s3://"):
        bucket, prefix = parse_s3_uri(args.trace_source)
        if args.keep_downloaded:
            download_dir = (
                os.path.abspath(args.download_dir)
                if args.download_dir
                else default_download_dir(bucket, prefix)
            )
            os.makedirs(download_dir, exist_ok=True)
            print(f"Keeping downloaded files in: {download_dir}")
            local_files = download_s3_prefix(bucket, prefix, download_dir)
            if not local_files:
                print("No files found at S3 prefix.")
                return
            local_files = sorted(local_files, key=sort_key_for_file)
            all_deltas, iteration_rows = process_files(local_files)
            if all_deltas.size == 0:
                print("No valid in-episode step deltas found after filtering transition rows.")
                return
            if not args.overall_only:
                print_iteration_table(iteration_rows)
            print_summary("Overall", summarize(all_deltas))
        else:
            with tempfile.TemporaryDirectory(prefix="simtrace_s3_") as temp_dir:
                local_files = download_s3_prefix(bucket, prefix, temp_dir)
                if not local_files:
                    print("No files found at S3 prefix.")
                    return
                local_files = sorted(local_files, key=sort_key_for_file)
                all_deltas, iteration_rows = process_files(local_files)
                if all_deltas.size == 0:
                    print("No valid in-episode step deltas found after filtering transition rows.")
                    return
                if not args.overall_only:
                    print_iteration_table(iteration_rows)
                print_summary("Overall", summarize(all_deltas))

        if args.download_dir and not args.keep_downloaded:
            print("Ignoring --download-dir because --keep-downloaded was not set.")
        return

    if os.path.isdir(args.trace_source):
        local_files = list_csv_files_from_directory(args.trace_source)
        if not local_files:
            print("No CSV files found in local directory.")
            return
        print(f"Found {len(local_files)} CSV file(s) in {args.trace_source}")
        all_deltas, iteration_rows = process_files(local_files)
        if all_deltas.size == 0:
            print("No valid in-episode step deltas found after filtering transition rows.")
            return
        if not args.overall_only:
            print_iteration_table(iteration_rows)
        print_summary("Overall", summarize(all_deltas))
        return

    per_episode_deltas = load_step_deltas(args.trace_source)
    if not per_episode_deltas:
        print("No valid in-episode step deltas found after filtering transition rows.")
        return

    if not args.overall_only:
        for episode in sorted(per_episode_deltas):
            print_summary(f"Episode {episode}", summarize(per_episode_deltas[episode]))

    all_deltas = flatten_deltas(per_episode_deltas)
    print_summary("Overall", summarize(all_deltas))

    if args.evolution:
        print_evolution(per_episode_deltas)


if __name__ == "__main__":
    main()