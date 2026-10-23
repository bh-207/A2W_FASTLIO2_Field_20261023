"""Geometry evaluator contracts; synthetic data do not certify factory geometry."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation


SPEC = importlib.util.spec_from_file_location(
    "loop_recall_geometry", Path(__file__).with_name("loop_recall_geometry.py"))
geometry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(geometry)


def poses(count=2):
    result = np.zeros((count, 7))
    result[:, 6] = 1
    return result


class GeometryEvaluatorTest(unittest.TestCase):
    def test_pose_csv_sorts_ids_and_accepts_named_aliases(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.csv"
            path.write_text("keyframe_id,x,y,z,qx,qy,qz,qw\n1,2,3,4,0,0,0,1\n0,0,0,0,0,0,0,1\n")
            actual = geometry.read_pose_csv(path, 2)
            np.testing.assert_allclose(actual[1, :3], [2, 3, 4])
            path.write_text("id,tx,ty,tz,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n0,0,0,0,0,0,0,1\n")
            with self.assertRaisesRegex(ValueError, "IDs"):
                geometry.read_pose_csv(path, 2)

    def test_pose_csv_rejects_missing_frames_and_nonfinite_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.csv"
            path.write_text("id,tx,ty,tz,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n")
            with self.assertRaises(ValueError):
                geometry.read_pose_csv(path, 2)
            path.write_text("id,tx,ty,tz,qx,qy,qz,qw\n0,nan,0,0,0,0,0,1\n")
            with self.assertRaises(ValueError):
                geometry.read_pose_csv(path, 1)

    def test_ground_height_uses_same_fixed_map_xy_in_both_variants(self):
        pair = dict(i=0, j=1, na=[0, 0, 1], nb=[0, 0, 1],
                    da=.6, db=.6, xy=[3, 4], partition="heldout",
                    scope="local", revisit=True)
        original = poses()
        moved = original.copy()
        moved[1, 2] = .5
        self.assertAlmostEqual(geometry.ground_rows(original, [pair])[0][2], 0)
        self.assertAlmostEqual(geometry.ground_rows(moved, [pair])[0][2], .5)
        self.assertAlmostEqual(geometry.ground_rows(moved, [pair])[0][3], 0)

    def test_odometry_residual_is_invariant_under_global_rigid_transform(self):
        raw = poses(3)
        raw[:, 0] = [0, 1, 2]
        raw[:, 3:] = Rotation.from_euler("z", [0, .2, .3]).as_quat()
        global_rotation = Rotation.from_euler("xyz", [.3, -.2, .7])
        transformed = raw.copy()
        transformed[:, :3] = global_rotation.apply(raw[:, :3]) + [8, -2, 4]
        transformed[:, 3:] = (global_rotation * Rotation.from_quat(raw[:, 3:])).as_quat()
        np.testing.assert_allclose(geometry.odometry_rows(raw, transformed)[:, 2:], 0, atol=1e-12)
        transformed[2, 2] += .4
        self.assertGreater(geometry.odometry_rows(raw, transformed)[1, 2], .39)

    def test_loop_measurement_direction_and_missing_measurement_are_explicit(self):
        trajectory = poses()
        trajectory[1, 0] = 2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "loops.csv"
            path.write_text("from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n1,0,-2,0,0,0,0,0,1\n")
            rows, _ = geometry.loop_rows(path, trajectory)
            np.testing.assert_allclose(rows[0][2:], 0, atol=1e-12)
            path.write_text("from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n1,0,2,0,0,0,0,0,1\n")
            rows, _ = geometry.loop_rows(path, trajectory)
            self.assertAlmostEqual(rows[0][2], 4)
        self.assertIn("unavailable", geometry.loop_rows(None, trajectory)[1])

    def test_fixed_matches_keep_far_points_in_denominator(self):
        a = np.array([[0., 0, 1], [1., 0, 1], [100., 0, 1]])
        fixed = [dict(i=0, j=1, a=a, b=a.copy(), ab=np.arange(3), ba=np.arange(3),
                      scope="local", region="A_reference")]
        original = poses()
        moved = original.copy()
        moved[1, 0] = 1000
        before = geometry.surface_rows(original, fixed)[0]
        after = geometry.surface_rows(moved, fixed)[0]
        self.assertEqual(before[2:4], after[2:4])
        self.assertEqual(after[3], 3)
        self.assertAlmostEqual(after[7], 1000)
        self.assertAlmostEqual(after[8], 1000)
        self.assertEqual(after[9], 0)

    def test_regression_warnings_catch_normal_region_worsening(self):
        baseline = {"A_reference": {"height_m": geometry.summary([.01, .02])}}
        enhanced = {"A_reference": {"height_m": geometry.summary([.1, .2])}}
        warnings = geometry.regression_warnings(baseline, enhanced)
        self.assertTrue(any("A_reference" in row["metric"] for row in warnings))

    def test_one_of_101_fixed_pairs_worsens_without_changing_aggregate_quantiles(self):
        heights = np.linspace(.005, .1, 101)
        baseline = [[i, i + 1, value] for i, value in enumerate(heights)]
        enhanced = [row.copy() for row in baseline]
        enhanced[0][2] = .02
        self.assertEqual(geometry.summary(heights), geometry.summary([row[2] for row in enhanced]))
        result = geometry.paired_regressions(
            baseline, enhanced, {"height_m": (2, False)}, {"A_reference": {0, 1}})
        self.assertEqual(result["compared_pairs"], 101)
        self.assertEqual(result["worsened_pairs"], 1)
        self.assertEqual(result["metrics"]["height_m"]["worst_keyframes"], [0, 1])
        self.assertAlmostEqual(result["metrics"]["height_m"]["worst_delta"], .015)
        self.assertEqual(result["regions"]["A_reference"]["worsened_pairs"], 1)

    def test_pairwise_warnings_cover_surface_coverage_and_each_odometry_factor(self):
        baseline = [[0, 1, .1, .9], [1, 2, .2, .8]]
        enhanced = [[1, 2, .3, .8], [0, 1, .1, .7]]  # Matching uses IDs, not row order.
        result = geometry.paired_regressions(
            baseline, enhanced, {"distance_m": (2, False), "coverage": (3, True)}, {})
        self.assertEqual(result["worsened_pairs"], 2)
        self.assertAlmostEqual(result["metrics"]["coverage"]["worst_delta"], .2)
        odometry = geometry.paired_regressions(
            baseline, enhanced, {"translation_log_m": (2, False)}, {"A_reference": {2}})
        self.assertEqual(odometry["warnings"][0]["keyframes"], [1, 2])
        self.assertEqual(odometry["regions"]["A_reference"]["worsened_pairs"], 1)
        with self.assertRaisesRegex(ValueError, "same fixed pair"):
            geometry.paired_regressions(baseline, enhanced[:1], {"distance_m": (2, False)}, {})

    def test_maximum_error_increase_warns_when_median_and_p95_do_not_change(self):
        before = {"A_reference": {"height_m": geometry.summary([0.] * 100 + [1.])}}
        after = {"A_reference": {"height_m": geometry.summary([0.] * 100 + [2.])}}
        warnings = geometry.regression_warnings(before, after)
        self.assertTrue(any(row["metric"] == "A_reference.height_m.max" for row in warnings))

    def test_output_must_not_overwrite_any_readonly_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "bundle"
            evidence = root / "evidence"
            bundle.mkdir()
            evidence.mkdir()
            for output in [bundle, bundle / "out", evidence, evidence / "out"]:
                with self.assertRaises(ValueError):
                    geometry.check_destination(output, bundle, evidence)
            geometry.check_destination(root / "new", bundle, evidence)


class OriginalEvidenceRegressionTest(unittest.TestCase):
    @unittest.skipUnless(Path("artifacts/map_repair_1016/comparison.json").exists(),
                         "original factory evidence is not distributed with the repository")
    def test_saved_and_unlooped_ground_metrics_reproduce_original_report(self):
        directory = Path("artifacts/map_repair_1016")
        pairs = json.loads((directory / "ground_pairs.json").read_text())
        previous = json.loads((directory / "comparison.json").read_text())
        for name in ("saved_poses", "no_loops"):
            trajectory = geometry.read_pose_csv(directory / (name + ".csv"), 1490)
            measured = geometry.ground_metrics(geometry.ground_rows(trajectory, pairs), pairs, {})
            for scope in ("heldout_local_revisit", "heldout_outside_revisit"):
                for metric in ("height_m", "normal_deg"):
                    self.assertAlmostEqual(measured[scope][metric]["p95"],
                                           previous[name][scope][metric]["p95"], places=10)


if __name__ == "__main__":
    unittest.main()
