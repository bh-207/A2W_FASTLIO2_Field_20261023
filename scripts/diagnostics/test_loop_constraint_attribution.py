import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

SPEC = importlib.util.spec_from_file_location(
    "loop_constraint_attribution", Path(__file__).with_name("loop_constraint_attribution.py"))
attribution = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(attribution)


class AttributionTest(unittest.TestCase):
    def test_effect_classification_does_not_sum_metres_and_degrees(self):
        self.assertEqual(attribution.classify_effect({"distance_m": 1}, {"distance_m": 2})[0], "beneficial")
        self.assertEqual(attribution.classify_effect({"distance_m": 2}, {"distance_m": 1})[0], "harmful")
        self.assertEqual(attribution.classify_effect(
            {"distance_m": 1, "normal_deg": 2}, {"distance_m": 2, "normal_deg": 1})[0], "uncertain")
        self.assertEqual(attribution.classify_effect({"coverage": .8}, {"coverage": .9})[0], "harmful")

    def test_invariant_floor_distance_survives_global_rigid_transform(self):
        poses = np.array([[0, 0, 0, 0, 0, 0, 1], [0, 0, .2, 0, 0, 0, 1]], float)
        pair = dict(i=0, j=1, na=[0, 0, 1], da=.6, nb=[0, 0, 1], db=.6, xy=[2, 3])
        anchors = attribution.make_ground_anchors(poses, [pair])
        expected = attribution.invariant_ground_rows(poses, [pair], anchors)
        moved = poses.copy()
        rotation = Rotation.from_euler("xyz", [.3, -.5, .8])
        moved[:, :3] = rotation.apply(poses[:, :3]) + [3, -7, 2]
        moved[:, 3:] = (rotation * Rotation.from_quat(poses[:, 3:])).as_quat()
        actual = attribution.invariant_ground_rows(moved, [pair], anchors)
        np.testing.assert_allclose(actual[:, 2:], expected[:, 2:], atol=1e-12)
        self.assertAlmostEqual(actual[0, 2], .2)

    def test_unavailable_future_focus_is_not_filled_with_zero(self):
        metric = attribution.focus_values(
            np.empty((0, 4)), [], np.empty((0, 4)), [], [], count=1200)
        self.assertFalse(metric["1170_1214"]["available"])
        self.assertEqual(metric["1170_1214"]["metrics"], {})

    def test_original_fixed_matches_identical_points_have_zero_distance(self):
        poses = np.zeros((2, 7));poses[:, -1] = 1
        points = np.array([[0, 0, 0], [1, 1, 1]], float)
        fixed = dict(i=0, j=1, a=points, b=points, ab=np.arange(2), ba=np.arange(2))
        rows = attribution.geometry.surface_rows(poses, [fixed])
        self.assertAlmostEqual(rows[0][7], 0)
        self.assertAlmostEqual(rows[0][8], 0)


class FixedGraphReplayTest(unittest.TestCase):
    binary = Path(__file__).resolve().parents[2] / "artifacts/loop_recall_v3_20261020/build/loop_constraint_ablation"

    @unittest.skipUnless(binary.exists(), "build the offline graph tool for its integration tests")
    def test_synthetic_graph_retains_odometry_and_injects_edge_only_at_source_time(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "bundle"
            (bundle / "keyframes").mkdir(parents=True)
            (bundle / "config").mkdir()
            (bundle / "keyframes/index.csv").write_text(
                "id,tx,ty,tz,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n1,1,0,0,0,0,0,1\n2,2,0,0,0,0,0,1\n")
            config = {"prior_rotation_sigma_rad": .001, "prior_translation_sigma_m": .001,
                      "odometry_rotation_sigma_rad": .05, "odometry_translation_sigma_m": .1,
                      "loop_rotation_sigma_rad": .05, "loop_translation_sigma_m": .1,
                      "robust_kernel_scale": 1, "relinearization_threshold": .1,
                      "relinearization_skip": 1, "robust_kernel": "huber"}
            (bundle / "config/mapping_effective.yaml").write_text(
                "\n".join(f"pose_graph.{key}: {value}" for key, value in config.items()) + "\n")
            loops = root / "loops.csv"
            loops.write_text("current_id,candidate_id,relative_tx,relative_ty,relative_tz,relative_qx,relative_qy,relative_qz,relative_qw\n2,0,-1.8,0,0,0,0,0,1\n")
            output = root / "output"
            completed = subprocess.run([str(self.binary), str(bundle), str(loops), str(output)], capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            no_loops = attribution.geometry.read_pose_csv(output / "odometry_only.csv", 3)
            np.testing.assert_allclose(no_loops[:, 0], [0, 1, 2], atol=1e-10)
            full = attribution.geometry.read_pose_csv(output / "all.csv", 3)
            self.assertLess(full[-1, 0], 2)
            control = attribution.geometry.read_pose_csv(output / "snapshots/without_0_at_2.csv", 3)
            np.testing.assert_allclose(control, no_loops, atol=1e-10)
            self.assertEqual(sorted(path.name for path in (output / "snapshots").glob("after_*.csv")), ["after_2.csv"])
            original = (output / "all.csv").read_bytes()
            repeated = subprocess.run([str(self.binary), str(bundle), str(loops), str(output)], capture_output=True)
            self.assertNotEqual(repeated.returncode, 0)
            self.assertEqual((output / "all.csv").read_bytes(), original)
            loops.write_text("current_id,candidate_id,relative_tx,relative_ty,relative_tz,relative_qx,relative_qy,relative_qz,relative_qw\n1,2,0,0,0,0,0,0,1\n")
            illegal_output = root / "future_factor"
            illegal = subprocess.run([str(self.binary), str(bundle), str(loops), str(illegal_output)], capture_output=True, text=True)
            self.assertNotEqual(illegal.returncode, 0)
            self.assertFalse(illegal_output.exists())


if __name__ == "__main__":
    unittest.main()
