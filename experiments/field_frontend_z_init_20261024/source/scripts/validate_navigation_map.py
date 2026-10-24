#!/usr/bin/env python3
"""Validate a 2D map's reference before the existing map-frame Nav2 launch."""
import argparse
import math
from pathlib import Path
import sys
import yaml


def validate(bundle: Path, navigation_yaml: Path) -> Path:
    metadata = yaml.safe_load((bundle / 'metadata.yaml').read_text())
    grid = yaml.safe_load(navigation_yaml.read_text())
    if not isinstance(metadata, dict) or metadata.get('map_frame') != 'map':
        raise ValueError('Map Bundle frame must be map')
    if not isinstance(grid, dict) or not isinstance(grid.get('image'), str) or not grid['image']:
        raise ValueError('map YAML has no image entry')
    if not math.isfinite(float(grid.get('resolution', 0))) or float(grid.get('resolution', 0)) <= 0:
        raise ValueError('map resolution must be finite and positive')
    origin = grid.get('origin')
    if not isinstance(origin, list) or len(origin) != 3 or not all(math.isfinite(float(x)) for x in origin):
        raise ValueError('map origin must contain three finite values')
    image = Path(grid['image'])
    if not image.is_absolute():
        image = navigation_yaml.parent / image
    if not image.is_file():
        raise ValueError('map image not found')
    conversion_path = navigation_yaml.parent / 'conversion.yaml'
    if conversion_path.is_file():
        conversion = yaml.safe_load(conversion_path.read_text())
        if not isinstance(conversion, dict) or conversion.get('schema') != 'a2w_nav_map_v1':
            raise ValueError('unrecognized conversion metadata')
        if conversion.get('map_frame') != 'map':
            raise ValueError('projection frame differs from map; resolve the recorded transform before Nav2')
        source_uuid = conversion.get('source_bundle_uuid')
        if source_uuid and source_uuid != metadata.get('bundle_uuid'):
            raise ValueError('2D map was generated from a different Map Bundle UUID')
    return image.resolve()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('map_yaml', type=Path)
    args = parser.parse_args()
    try:
        print(validate(args.bundle, args.map_yaml))
    except (OSError, ValueError, TypeError, yaml.YAMLError) as error:
        print(f'A2W Navigation: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
