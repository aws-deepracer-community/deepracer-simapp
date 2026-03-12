#!/usr/bin/env python3
"""
Route (.npy) geometry checker for deepracer-simapp.

Checks each route for:
- Loadability and expected array shape (Nx6+)
- Non-finite values (NaN / +/-inf)
- Degenerate consecutive segments (zero length)
- Loop closure consistency across center/inner/outer lines
- Near-zero lane width points

Usage examples:
  python3 utils/route_check.py --track Mexico_track
  python3 utils/route_check.py --track Mexico_track --track Canada_Training
  python3 utils/route_check.py --all
"""

import argparse
import glob
import os
import sys
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
BUNDLE = os.path.join(ROOT, '..', 'bundle')
ROUTES_DIR = os.path.join(BUNDLE, 'routes')

RED = '\033[91m'
YELLOW = '\033[93m'
GREEN = '\033[92m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'


def get_route_names():
    return sorted(
        (os.path.splitext(os.path.basename(p))[0]
         for p in glob.glob(os.path.join(ROUTES_DIR, '*.npy'))),
        key=str.casefold,
    )


def check_route_npy(route_path, label):
    errors = []
    warnings = []

    try:
        waypoints = np.load(route_path, allow_pickle=False)
    except Exception as e:
        return [f'{label}: failed to load route npy: {e}'], warnings

    if waypoints.ndim != 2:
        return [f'{label}: route array must be 2D, got ndim={waypoints.ndim}'], warnings

    if waypoints.shape[0] < 2:
        return [f'{label}: route must have at least 2 rows, got {waypoints.shape[0]}'], warnings

    if waypoints.shape[1] < 6:
        return [
            f'{label}: route must have at least 6 columns (center/inner/outer XY), got {waypoints.shape[1]}'
        ], warnings

    core = waypoints[:, :6]
    if not np.isfinite(core).all():
        bad = np.argwhere(~np.isfinite(core))
        first = bad[0]
        errors.append(f'{label}: non-finite value at row={int(first[0])}, col={int(first[1])}')

    center = core[:, 0:2]
    inner = core[:, 2:4]
    outer = core[:, 4:6]

    def _check_polyline(name, pts):
        local_errors = []
        local_warnings = []

        unique_count = np.unique(pts, axis=0).shape[0]
        if unique_count < 2:
            local_errors.append(f'{label}: {name} has <2 unique points')
            return local_errors, local_warnings

        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        zero_seg_idx = np.where(seg <= 1e-9)[0]
        if len(zero_seg_idx) > 0:
            local_warnings.append(
                f'{label}: {name} has {len(zero_seg_idx)} zero-length consecutive segment(s)'
            )
            # Add precise locations for the first few problematic segments.
            cum = np.concatenate([[0.0], np.cumsum(seg)])
            total = cum[-1] if cum[-1] > 0 else 1.0
            show = min(3, len(zero_seg_idx))
            for i in zero_seg_idx[:show]:
                progress = (cum[int(i)] / total) * 100.0
                p0 = pts[int(i)]
                p1 = pts[int(i) + 1]
                local_warnings.append(
                    f'{label}: {name} zero-seg idx {int(i)}->{int(i)+1} '
                    f'at ~{progress:.2f}% lap, p0={p0.tolist()}, p1={p1.tolist()}'
                )
            if len(zero_seg_idx) > show:
                local_warnings.append(
                    f'{label}: {name} has {len(zero_seg_idx) - show} additional zero-length segment(s)'
                )

        return local_errors, local_warnings

    for line_name, line_pts in [('center', center), ('inner', inner), ('outer', outer)]:
        e, w = _check_polyline(line_name, line_pts)
        errors.extend(e)
        warnings.extend(w)

    center_loop = np.allclose(center[0], center[-1], atol=1e-9)
    inner_loop = np.allclose(inner[0], inner[-1], atol=1e-9)
    outer_loop = np.allclose(outer[0], outer[-1], atol=1e-9)
    if center_loop != inner_loop or center_loop != outer_loop:
        warnings.append(
            f'{label}: loop closure mismatch center={center_loop}, inner={inner_loop}, outer={outer_loop}'
        )

    inner_width = np.linalg.norm(center - inner, axis=1)
    outer_width = np.linalg.norm(center - outer, axis=1)
    bad_inner = int(np.count_nonzero(inner_width <= 1e-9))
    bad_outer = int(np.count_nonzero(outer_width <= 1e-9))
    if bad_inner > 0 or bad_outer > 0:
        warnings.append(
            f'{label}: near-zero lane width points inner={bad_inner}, outer={bad_outer}'
        )

    return errors, warnings


def main():
    parser = argparse.ArgumentParser(description='Route geometry checker for deepracer-simapp')
    parser.add_argument('--track', action='append', default=[],
                        help='Track/route name (without .npy). Can be repeated.')
    parser.add_argument('--all', action='store_true',
                        help='Check all route .npy files in bundle/routes')
    args = parser.parse_args()

    if args.track and args.all:
        print('Use either --track or --all, not both.')
        sys.exit(2)

    if args.track:
        route_names = sorted(set(args.track), key=str.casefold)
    else:
        route_names = get_route_names()

    total_errors = 0
    total_warnings = 0
    ok_count = 0
    issue_count = 0

    print(f'{BOLD}Route Geometry Check{RESET}')
    print(f'Checking {len(route_names)} route(s)...\n')

    for name in route_names:
        route_path = os.path.join(ROUTES_DIR, f'{name}.npy')
        errors = []
        warnings = []

        if not os.path.exists(route_path):
            errors.append(f'Route routes/{name}.npy does not exist')
        else:
            errors, warnings = check_route_npy(route_path, f'Route routes/{name}.npy')

        if errors or warnings:
            issue_count += 1
            print(f'{BOLD}{CYAN}{name}{RESET}')
            for e in errors:
                print(f'  {RED}ERROR{RESET}  {e}')
                total_errors += 1
            for w in warnings:
                print(f'  {YELLOW}WARN {RESET}  {w}')
                total_warnings += 1
            print()
        else:
            ok_count += 1

    print(f'{BOLD}Summary{RESET}')
    print(f'  Routes checked: {len(route_names)}')
    print(f'  {GREEN}OK: {ok_count}{RESET}')
    if issue_count:
        print(f'  {RED}Issues: {issue_count}{RESET}')
    print(f'  Total errors:   {total_errors}')
    print(f'  Total warnings: {total_warnings}')

    sys.exit(1 if total_errors > 0 else 0)


if __name__ == '__main__':
    main()
