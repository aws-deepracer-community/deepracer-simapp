#!/usr/bin/env python3
"""
Track consistency checker for deepracer-simapp.

For each world file, validates the full chain:
  World (.world) -> Model (model.config + .sdf) -> Meshes (.dae)
  Track iconography (.png)
  Route (.npy)

Also checks that tracks.txt, routes, iconography, worlds, models,
and meshes are all consistent with each other.
"""

import argparse
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

BUNDLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'bundle')
WORLDS_DIR = os.path.join(BUNDLE, 'worlds')
MODELS_DIR = os.path.join(BUNDLE, 'models')
MESHES_DIR = os.path.join(BUNDLE, 'meshes')
ROUTES_DIR = os.path.join(BUNDLE, 'routes')
ICONS_DIR = os.path.join(BUNDLE, 'track_iconography')
TRACKS_TXT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tracks.txt')

RED = '\033[91m'
YELLOW = '\033[93m'
GREEN = '\033[92m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'


def load_tracks_txt():
    if not os.path.exists(TRACKS_TXT):
        return None
    with open(TRACKS_TXT) as f:
        return [line.strip() for line in f if line.strip()]


def get_world_names():
    return sorted(
        (os.path.splitext(os.path.basename(w))[0]
         for w in glob.glob(os.path.join(WORLDS_DIR, '*.world'))),
        key=str.casefold
    )


def parse_world_includes(world_path):
    """Return list of model URIs referenced from <include> in the world file."""
    tree = ET.parse(world_path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        return [], None

    sdf_version = root.get('version')
    model_uris = []
    for inc in world.findall('include'):
        uri_el = inc.find('uri')
        if uri_el is not None and uri_el.text:
            text = uri_el.text.strip()
            # Skip sun includes
            if 'sun' in text.lower() and 'model' not in text.replace('model://', ''):
                continue
            model_uris.append(text)
    return model_uris, sdf_version


def resolve_model_uri(uri):
    """Convert 'model://models/foo' to the local models/foo path."""
    prefix = 'model://models/'
    if uri.startswith(prefix):
        return uri[len(prefix):]
    prefix2 = 'model://'
    if uri.startswith(prefix2):
        return uri[len(prefix2):]
    return uri


def get_model_sdf_files(model_dir):
    """Find the SDF file(s) in a model directory using model.config or fallback."""
    config_path = os.path.join(model_dir, 'model.config')
    sdf_files = []
    if os.path.exists(config_path):
        try:
            tree = ET.parse(config_path)
            for sdf_el in tree.getroot().findall('sdf'):
                if sdf_el.text:
                    sdf_files.append(sdf_el.text.strip())
        except ET.ParseError:
            pass

    if not sdf_files:
        # Fallback: any .sdf in the directory
        sdf_files = [
            os.path.basename(f) for f in glob.glob(os.path.join(model_dir, '*.sdf'))
        ]
    return sdf_files


def parse_mesh_uris(sdf_path):
    """Extract all mesh URIs from an SDF file."""
    uris = []
    try:
        tree = ET.parse(sdf_path)
        for uri_el in tree.iter('uri'):
            if uri_el.text and 'mesh' in uri_el.text.lower():
                uris.append(uri_el.text.strip())
    except ET.ParseError:
        pass
    return uris


def resolve_mesh_uri(uri):
    """Convert 'model://meshes/foo/bar.dae' to a path relative to bundle/."""
    prefix = 'model://'
    if uri.startswith(prefix):
        return uri[len(prefix):]
    return uri


def check_sdf_version(filepath, label):
    """Check SDF version of a file. Returns list of issues."""
    issues = []
    try:
        with open(filepath) as f:
            content = f.read(500)
        m = re.search(r'sdf version=["\']([^"\']+)["\']', content)
        if m:
            ver = m.group(1)
            if ver not in ('1.7', '1.8', '1.9'):
                issues.append(f'{label}: SDF version is {ver} (expected 1.7+)')
        else:
            issues.append(f'{label}: no SDF version found')
    except Exception as e:
        issues.append(f'{label}: error reading file: {e}')
    return issues


def check_world_plugins(world_path, label):
    """Check that required Gazebo Harmonic plugins are present."""
    issues = []
    required_plugins = [
        'gz::sim::systems::Physics',
        'gz::sim::systems::SceneBroadcaster',
        'gz::sim::systems::Sensors',
        'gz::sim::systems::UserCommands',
        'gz::sim::systems::DeepRacerGazeboSystemPlugin',
    ]
    try:
        tree = ET.parse(world_path)
        world = tree.getroot().find('world')
        if world is None:
            issues.append(f'{label}: no <world> element')
            return issues
        found_plugins = set()
        for plugin in world.findall('plugin'):
            name = plugin.get('name', '')
            found_plugins.add(name)
        for rp in required_plugins:
            if rp not in found_plugins:
                issues.append(f'{label}: missing plugin {rp}')
    except ET.ParseError as e:
        issues.append(f'{label}: XML parse error: {e}')
    return issues


def check_track(track_name):
    """Run all checks for a single track. Returns (errors, warnings)."""
    errors = []
    warnings = []

    # 1. World file
    world_path = os.path.join(WORLDS_DIR, f'{track_name}.world')
    if not os.path.exists(world_path):
        errors.append(f'Missing world file: {track_name}.world')
        return errors, warnings

    # World SDF version
    errors.extend(check_sdf_version(world_path, 'World'))

    # World plugins
    errors.extend(check_world_plugins(world_path, 'World'))

    # 2. Model references from world
    model_uris, _ = parse_world_includes(world_path)
    if not model_uris:
        errors.append('World has no model <include> URIs')
        return errors, warnings

    for uri in model_uris:
        model_name = resolve_model_uri(uri)
        model_dir = os.path.join(MODELS_DIR, model_name)

        if not os.path.isdir(model_dir):
            errors.append(f'Missing model directory: models/{model_name}/')
            continue

        # model.config
        config_path = os.path.join(model_dir, 'model.config')
        if not os.path.exists(config_path):
            warnings.append(f'Missing model.config in models/{model_name}/')

        # SDF files
        sdf_files = get_model_sdf_files(model_dir)
        if not sdf_files:
            errors.append(f'No SDF file found in models/{model_name}/')
            continue

        for sdf_file in sdf_files:
            sdf_path = os.path.join(model_dir, sdf_file)
            if not os.path.exists(sdf_path):
                errors.append(f'model.config references missing SDF: models/{model_name}/{sdf_file}')
                continue

            # Model SDF version
            errors.extend(check_sdf_version(sdf_path, f'Model {model_name}/{sdf_file}'))

            # 3. Mesh references from model SDF
            mesh_uris = parse_mesh_uris(sdf_path)
            if not mesh_uris:
                warnings.append(f'No mesh URIs found in models/{model_name}/{sdf_file}')

            for mesh_uri in mesh_uris:
                rel_path = resolve_mesh_uri(mesh_uri)
                abs_path = os.path.join(BUNDLE, rel_path)
                if not os.path.exists(abs_path):
                    errors.append(f'Missing mesh: {rel_path} (from models/{model_name}/{sdf_file})')

    # 4. Route file
    route_path = os.path.join(ROUTES_DIR, f'{track_name}.npy')
    if not os.path.exists(route_path):
        errors.append(f'Missing route: routes/{track_name}.npy')

    # 5. Track iconography
    icon_path = os.path.join(ICONS_DIR, f'{track_name}.png')
    if not os.path.exists(icon_path):
        warnings.append(f'Missing track icon: track_iconography/{track_name}.png')

    return errors, warnings


def find_orphans():
    """Find files/dirs that exist but aren't referenced by any world."""
    issues = []

    world_names = set(get_world_names())

    # Collect all model dirs referenced by worlds
    referenced_models = set()
    for wn in world_names:
        wp = os.path.join(WORLDS_DIR, f'{wn}.world')
        uris, _ = parse_world_includes(wp)
        for u in uris:
            referenced_models.add(resolve_model_uri(u))

    # Collect all mesh dirs referenced by those models
    referenced_mesh_dirs = set()
    for model_name in referenced_models:
        model_dir = os.path.join(MODELS_DIR, model_name)
        if not os.path.isdir(model_dir):
            continue
        for sdf_file in get_model_sdf_files(model_dir):
            sdf_path = os.path.join(model_dir, sdf_file)
            if not os.path.exists(sdf_path):
                continue
            for mesh_uri in parse_mesh_uris(sdf_path):
                rel = resolve_mesh_uri(mesh_uri)
                # Extract the directory part (e.g., meshes/foo from meshes/foo/bar.dae)
                parts = rel.split('/')
                if len(parts) >= 2:
                    referenced_mesh_dirs.add(parts[1] if parts[0] == 'meshes' else parts[0])

    # Non-track model dirs (cars, obstacles, cameras, etc.)
    NON_TRACK_MODELS = {'amazon_box_obstacle', 'bot_car', 'box_obstacle',
                        'camera', 'deepracer_box_obstacle', 'top_camera'}

    # Check for orphan model directories
    if os.path.isdir(MODELS_DIR):
        all_model_dirs = set(
            d for d in os.listdir(MODELS_DIR)
            if os.path.isdir(os.path.join(MODELS_DIR, d))
        )
        for d in sorted(all_model_dirs - referenced_models - NON_TRACK_MODELS, key=str.lower):
            issues.append(f'Orphan model directory: models/{d}/ (not referenced by any world)')

    # Mesh dirs used for non-track assets (cars, obstacles, effects)
    NON_TRACK_MESHES = {'amazon_box_obstacle', 'confetti', 'deepracer',
                        'deepracer_box_obstacle', 'f1', 'mit'}

    # Check for orphan mesh directories
    if os.path.isdir(MESHES_DIR):
        for d in sorted(os.listdir(MESHES_DIR)):
            if d in NON_TRACK_MESHES:
                continue
            if os.path.isdir(os.path.join(MESHES_DIR, d)) and d not in referenced_mesh_dirs:
                issues.append(f'Orphan mesh directory: meshes/{d}/ (not referenced by any model)')

    # Check for orphan route files
    for f in sorted(os.listdir(ROUTES_DIR)):
        if f.endswith('.npy'):
            name = f[:-4]
            if name not in world_names:
                issues.append(f'Orphan route: routes/{f} (no matching world)')

    return issues


def generate_tracks_txt():
    """Regenerate tracks.txt from world files that have complete assets."""
    world_names = get_world_names()
    valid_tracks = []

    for track_name in world_names:
        errors, _ = check_track(track_name)
        if not errors:
            valid_tracks.append(track_name)
        else:
            print(f'  {YELLOW}Excluding{RESET} {track_name}: {errors[0]}')

    with open(TRACKS_TXT, 'w') as f:
        for t in sorted(valid_tracks, key=str.casefold):
            f.write(t + '\n')

    print(f'\nWrote {len(valid_tracks)} tracks to {os.path.relpath(TRACKS_TXT)}')
    if len(world_names) - len(valid_tracks) > 0:
        print(f'Excluded {len(world_names) - len(valid_tracks)} tracks with errors')


def main():
    parser = argparse.ArgumentParser(description='Track consistency checker for deepracer-simapp')
    parser.add_argument('--generate-tracks-txt', action='store_true',
                        help='Regenerate tracks.txt from valid world files')
    args = parser.parse_args()

    if args.generate_tracks_txt:
        generate_tracks_txt()
        return

    # Determine track list
    tracks_txt = load_tracks_txt()
    world_names = get_world_names()

    if tracks_txt:
        track_list = sorted(set(tracks_txt), key=str.casefold)
    else:
        track_list = world_names

    total_errors = 0
    total_warnings = 0
    tracks_ok = 0
    tracks_with_issues = []

    print(f'{BOLD}Track Consistency Check{RESET}')
    print(f'Checking {len(track_list)} tracks...\n')

    # Check tracks.txt vs worlds
    if tracks_txt:
        txt_set = set(tracks_txt)
        world_set = set(world_names)
        for t in sorted(txt_set - world_set):
            print(f'  {RED}ERROR{RESET}  tracks.txt lists "{t}" but no world file exists')
            total_errors += 1
        for t in sorted(world_set - txt_set):
            print(f'  {YELLOW}WARN {RESET}  World "{t}.world" exists but not in tracks.txt')
            total_warnings += 1
        if txt_set - world_set or world_set - txt_set:
            print()

    for track_name in track_list:
        errors, warnings = check_track(track_name)

        if errors or warnings:
            tracks_with_issues.append(track_name)
            print(f'{BOLD}{CYAN}{track_name}{RESET}')
            for e in errors:
                print(f'  {RED}ERROR{RESET}  {e}')
                total_errors += 1
            for w in warnings:
                print(f'  {YELLOW}WARN {RESET}  {w}')
                total_warnings += 1
            print()
        else:
            tracks_ok += 1

    # Orphan check
    print(f'{BOLD}Orphan Check{RESET}')
    orphans = find_orphans()
    if orphans:
        for o in orphans:
            print(f'  {YELLOW}WARN {RESET}  {o}')
            total_warnings += 1
        print()
    else:
        print(f'  {GREEN}No orphans found{RESET}\n')

    # Summary
    print(f'{BOLD}Summary{RESET}')
    print(f'  Tracks checked: {len(track_list)}')
    print(f'  {GREEN}OK: {tracks_ok}{RESET}')
    if tracks_with_issues:
        print(f'  {RED}Issues: {len(tracks_with_issues)}{RESET}')
    print(f'  Total errors:   {total_errors}')
    print(f'  Total warnings: {total_warnings}')

    sys.exit(1 if total_errors > 0 else 0)


if __name__ == '__main__':
    main()
