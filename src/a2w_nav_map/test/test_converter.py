#!/usr/bin/env python3

import hashlib
import math
import struct
import tempfile
import unittest
from pathlib import Path

import yaml

from a2w_nav_map.converter import ConversionConfig, ConversionError, convert_bundle


def write_binary_pcd(path: Path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
        "TYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    payload = b"".join(struct.pack("<ffff", *point) for point in points)
    path.write_bytes(header + payload)


def make_bundle(root: Path, points, frame="map"):
    pcd = root / "global_map.pcd"
    write_binary_pcd(pcd, points)
    metadata = {
        "schema": "a2w_map_bundle_v1",
        "bundle_uuid": "synthetic-bundle",
        "map_frame": frame,
    }
    (root / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    entries = []
    for name in ("global_map.pcd", "metadata.yaml"):
        digest = hashlib.sha256((root / name).read_bytes()).hexdigest()
        entries.append(f"{digest}  {name}\n")
    (root / "manifest.sha256").write_text("".join(entries), encoding="utf-8")


class ConverterTests(unittest.TestCase):
    def test_preserves_map_coordinates_and_classifies_ground_and_obstacle(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / "bundle"
            output = Path(directory) / "navigation"
            bundle.mkdir()
            make_bundle(bundle, [
                (0.0, 0.0, -0.5, 1.0),
                (1.0, 0.0, -0.5, 1.0),
                (1.0, 1.0, 0.5, 1.0),
                (math.nan, 1.0, 0.5, 1.0),
            ])

            result = convert_bundle(
                bundle, output,
                ConversionConfig(resolution=0.5, padding=0.5, map_z_confirmed=True,
                                 ground_min_z=-0.8, ground_max_z=-0.3,
                                 obstacle_min_z=-0.2, obstacle_max_z=1.5),
            )

            map_yaml = yaml.safe_load((output / "nav_map.yaml").read_text())
            self.assertEqual(map_yaml["image"], "nav_map.pgm")
            self.assertEqual(map_yaml["resolution"], 0.5)
            self.assertEqual(map_yaml["origin"], [-0.5, -0.5, 0.0])
            self.assertEqual(result.frame, "map")
            self.assertEqual(result.finite_points, 3)
            self.assertGreater(result.free_cells, 0)
            self.assertGreater(result.occupied_cells, 0)
            metadata = yaml.safe_load((output / "conversion.yaml").read_text())
            self.assertEqual(metadata["source_bundle_uuid"], "synthetic-bundle")
            self.assertEqual(metadata["map_frame"], "map")

    def test_rejects_wrong_frame_and_corrupt_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "bundle"
            bundle.mkdir()
            make_bundle(bundle, [(0.0, 0.0, 0.5, 0.0)], frame="camera_init")
            with self.assertRaisesRegex(ConversionError, "map_frame"):
                convert_bundle(bundle, root / "out", ConversionConfig())

            make_bundle(bundle, [(0.0, 0.0, 0.5, 0.0)])
            (bundle / "global_map.pcd").write_bytes(b"corrupt")
            with self.assertRaisesRegex(ConversionError, "checksum"):
                convert_bundle(bundle, root / "out", ConversionConfig())

    def test_rejects_empty_or_unsupported_pcd(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "bundle"
            bundle.mkdir()
            make_bundle(bundle, [])
            with self.assertRaisesRegex(ConversionError, "no finite points"):
                convert_bundle(bundle, root / "out", ConversionConfig(map_z_confirmed=True))


if __name__ == "__main__":
    unittest.main()
