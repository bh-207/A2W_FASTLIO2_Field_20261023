"""Offline projection reference and one-command export regressions."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from test_converter import make_bundle, write_binary_pcd
from a2w_nav_map.converter import ConversionConfig, ConversionError, convert_pcd


class ProjectionCliTests(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, '-m', 'a2w_nav_map.cli', *map(str, args)],
                              text=True, capture_output=True, check=False)

    def test_bundle_requires_explicit_vertical_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            make_bundle(root, [(0, 0, -0.5, 1), (1, 0, 0.5, 1)])
            result = self.run_cli('--map-bundle', root, '--output', root / 'out')
            self.assertNotEqual(result.returncode, 0, 'unverified map Z must not be silently assumed vertical')
            self.assertIn('reference', result.stderr.lower())
            self.assertFalse((root / 'out').exists())

    def test_quoted_false_is_not_a_confirmed_vertical_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source.pcd'
            write_binary_pcd(source, [(0, 0, -.5, 1)])
            with self.assertRaisesRegex(ConversionError, 'boolean'):
                convert_pcd(source, root / 'out', ConversionConfig(map_z_confirmed='false'))
            self.assertFalse((root / 'out').exists())

    def test_unwritable_target_does_not_replace_existing_image(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source.pcd'
            write_binary_pcd(source, [(0, 0, -.5, 1)])
            out = root / 'out'
            out.mkdir()
            (out / 'nav_map.pgm').write_bytes(b'previous map')
            (out / 'nav_map.yaml').mkdir()
            with self.assertRaises((ConversionError, OSError)):
                convert_pcd(source, out, ConversionConfig(map_z_confirmed=True), overwrite=True)
            self.assertEqual((out / 'nav_map.pgm').read_bytes(), b'previous map')

    def test_input_cannot_be_one_of_the_output_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'nav_map.pgm'
            write_binary_pcd(source, [(0, 0, -.5, 1)])
            before = source.read_bytes()
            with self.assertRaisesRegex(ConversionError, 'source'):
                convert_pcd(source, root, ConversionConfig(map_z_confirmed=True), overwrite=True)
            self.assertEqual(source.read_bytes(), before)

    def test_late_publication_failure_restores_the_previous_complete_set(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source.pcd'
            write_binary_pcd(source, [(0, 0, -.5, 1)])
            out = root / 'out'
            out.mkdir()
            old = {name: ('old ' + name).encode() for name in ['nav_map.pgm', 'nav_map.yaml', 'conversion.yaml']}
            for name, data in old.items():
                (out / name).write_bytes(data)
            real_replace = os.replace
            def fail_second_target(src, dst):
                if Path(dst) == out / 'nav_map.yaml':
                    raise OSError('simulated target write failure')
                return real_replace(src, dst)
            with patch('a2w_nav_map.converter.os.replace', side_effect=fail_second_target):
                with self.assertRaises(OSError):
                    convert_pcd(source, out, ConversionConfig(map_z_confirmed=True), overwrite=True)
            self.assertEqual({name: (out / name).read_bytes() for name in old}, old)

    def test_known_tilt_preserves_projected_coordinates_and_records_transform(self):
        with tempfile.TemporaryDirectory(prefix='2d projection ') as directory:
            root = Path(directory)
            angle = np.deg2rad(20)
            rotation = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)],
                                 [0, np.sin(angle), np.cos(angle)]])
            points = np.array([[0, 0, -0.5], [1, 0, -0.5], [1, 1, 0.5]]) @ rotation.T
            source = root / 'source.pcd'
            write_binary_pcd(source, [(*point, 1) for point in points])
            before = hashlib.sha256(source.read_bytes()).hexdigest()
            result = self.run_cli('--pcd', source, '--output', root / 'out', '--up', *rotation[:, 2],
                                  '--resolution', '0.5', '--padding', '0.5')
            self.assertEqual(result.returncode, 0, result.stderr)
            metadata = yaml.safe_load((root / 'out' / 'conversion.yaml').read_text())
            self.assertEqual(metadata['map_frame'], 'map_projection')
            np.testing.assert_allclose(metadata['projection_T_source'][:3], np.c_[rotation.T, np.zeros(3)], atol=1e-7)
            self.assertGreater(metadata['result']['free_cells'], 0)
            self.assertGreater(metadata['result']['occupied_cells'], 0)
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), before)
            (root / 'metadata.yaml').write_text(yaml.safe_dump({'map_frame': 'map', 'bundle_uuid': 'test'}))
            validator = Path(__file__).resolve().parents[3] / 'scripts' / 'validate_navigation_map.py'
            checked = subprocess.run([sys.executable, str(validator), str(root), str(root / 'out' / 'nav_map.yaml')],
                                     text=True, capture_output=True)
            self.assertNotEqual(checked.returncode, 0)
            self.assertIn('frame', checked.stderr)

    def test_confirmed_map_z_output_uses_existing_navigation_names(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            make_bundle(root, [(0, 0, -0.5, 1), (1, 0, 0.5, 1)])
            result = self.run_cli('--map-bundle', root, '--output', root / 'out', '--confirm-map-z')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((root / 'out' / 'nav_map.pgm').is_file())
            metadata = yaml.safe_load((root / 'out' / 'conversion.yaml').read_text())
            self.assertEqual(metadata['map_frame'], 'map')
            validator = Path(__file__).resolve().parents[3] / 'scripts' / 'validate_navigation_map.py'
            checked = subprocess.run([sys.executable, str(validator), str(root), str(root / 'out' / 'nav_map.yaml')],
                                     text=True, capture_output=True)
            self.assertEqual(checked.returncode, 0, checked.stderr)
            second = self.run_cli('--map-bundle', root, '--output', root / 'out', '--confirm-map-z')
            self.assertNotEqual(second.returncode, 0, 'existing maps require explicit overwrite')

    @unittest.skipUnless(os.environ.get('A2W_PCD_FILTER_BIN'), 'PMF integration requires built helper')
    def test_recovered_pmf_cli_exports_ground_only_and_ground_with_box(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ground = [(x * .2, y * .2, 0., 1.) for x in range(-10, 11) for y in range(-10, 11)]
            for name, points, has_obstacle in [
                    ('ground', ground, False),
                    ('box', ground + [(x * .1, y * .1, .8, 1.) for x in range(5) for y in range(5)], True)]:
                source = root / (name + '.pcd')
                write_binary_pcd(source, points)
                result = self.run_cli('--pcd', source, '--output', root / name, '--confirm-map-z',
                                      '--method', 'pmf', '--voxel-leaf', '0', '--mean-k', '0')
                self.assertEqual(result.returncode, 0, result.stderr)
                metadata = yaml.safe_load((root / name / 'conversion.yaml').read_text())
                self.assertGreater(metadata['result']['free_cells'], 0)
                self.assertEqual(metadata['result']['occupied_cells'] > 0, has_obstacle)


if __name__ == '__main__':
    unittest.main()
