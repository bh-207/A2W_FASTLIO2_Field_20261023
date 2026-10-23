"""Command-line entry point for Map Bundle conversion."""

import argparse
from dataclasses import fields
from pathlib import Path
import yaml

from .converter import ConversionConfig, ConversionError, convert_bundle, convert_pcd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--map-bundle", type=Path)
    source.add_argument("--pcd", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--source-frame", default="map")
    parser.add_argument("--overwrite", action="store_true")
    reference = parser.add_mutually_exclusive_group()
    reference.add_argument("--confirm-map-z", action="store_true", default=None,
                           help="confirm source +Z is the independently verified vertical")
    reference.add_argument("--up", type=float, nargs=3, dest="up_in_map",
                           help="independently verified up direction in the source frame")
    parser.add_argument("--method", choices=["height", "pmf"])
    integer = {"min_obstacle_points", "threads", "max_window_size", "mean_k"}
    for field in fields(ConversionConfig):
        if field.name not in {"map_z_confirmed", "up_in_map", "method"}:
            parser.add_argument("--" + field.name.replace("_", "-"),
                                type=int if field.name in integer else float)
    args = parser.parse_args()
    try:
        values = yaml.safe_load(args.config.read_text()) if args.config else {}
        if not isinstance(values, dict):
            raise ConversionError("configuration must be a mapping")
        for field in fields(ConversionConfig):
            value = args.confirm_map_z if field.name == "map_z_confirmed" else getattr(args, field.name, None)
            if value is not None:
                values[field.name] = value
        if args.confirm_map_z:
            values["up_in_map"] = None
        config = ConversionConfig(**values)
        if args.map_bundle:
            result = convert_bundle(args.map_bundle, args.output, config, overwrite=args.overwrite)
        else:
            result = convert_pcd(args.pcd, args.output, config, args.source_frame, overwrite=args.overwrite)
    except (ConversionError, OSError, TypeError, yaml.YAMLError) as error:
        parser.error(str(error))
    print(
        f"generated {args.output / 'nav_map.yaml'}: frame={result.frame} "
        f"size={result.width}x{result.height} occupied={result.occupied_cells} "
        f"free={result.free_cells}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

