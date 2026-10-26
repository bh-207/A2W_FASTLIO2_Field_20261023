"""Mechanism tests do not certify real gravity or factory loop correctness."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

PATH = Path(__file__).with_name("loop_roof_audit.py")
audit = None
if PATH.exists():
    spec = importlib.util.spec_from_file_location("loop_roof_audit", PATH)
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)


class RoofAuditTest(unittest.TestCase):
    def implementation(self):
        self.assertIsNotNone(audit, "roof audit implementation is absent")
        return audit

    def test_actual_sc_uses_max_positive_z_plus_two_and_clamped_bins(self):
        impl = self.implementation()
        cloud = np.array([[0, 0, -3], [1, 0, 1], [1, 0, 5], [81, 0, 100]], dtype=np.float32)
        desc = impl.describe(cloud)
        self.assertEqual(desc.shape, (20, 60))
        self.assertEqual(desc[0, 0], 7)
        self.assertEqual(np.count_nonzero(desc), 1)

    def test_few_high_points_can_dominate_all_sc_maximum_cells(self):
        impl = self.implementation()
        low = np.tile(np.array([[1., 0, 0], [0, 1, 0]], dtype=np.float32), (100, 1))
        cloud = np.vstack([low, [[1, 0, 5], [0, 1, 5]]]).astype(np.float32)
        stats = impl.height_stats(cloud, None)
        self.assertLess(stats["body_proxy_high_point_fraction"], .02)
        self.assertEqual(stats["body_proxy_high_max_cell_fraction"], 1)

    def test_descriptor_rejects_empty_or_nonfinite_cloud_like_production(self):
        impl = self.implementation()
        with self.assertRaises(ValueError):
            impl.describe(np.empty((0, 3), dtype=np.float32))
        with self.assertRaises(ValueError):
            impl.describe(np.array([[1, 0, np.nan]], dtype=np.float32))

    def test_local_ground_proxy_preserves_sloped_original_coordinates(self):
        impl = self.implementation()
        xx, yy = np.meshgrid(np.arange(-6., 7.), np.arange(-6., 7.))
        ground = np.column_stack([xx.ravel(), yy.ravel(), .1*xx.ravel() + .05*yy.ravel() - 1])
        cloud = np.vstack([ground, ground + [0, 0, 6]]).astype(np.float32)
        original = cloud.copy()
        plane = impl.fit_local_ground(cloud)
        self.assertIsNotNone(plane)
        np.testing.assert_allclose(plane[:3], [.1, .05, -1], atol=1e-5)
        np.testing.assert_allclose(impl.ground_heights(ground, plane), 0, atol=1e-5)
        np.testing.assert_array_equal(cloud, original)
        self.assertGreater(np.ptp(ground[:, 2]), 1)

    def test_missing_ground_reference_is_explicit_and_cropping_falls_back(self):
        impl = self.implementation()
        cloud = np.array([[1., 0, 1], [2, 0, 2]], dtype=np.float32)
        self.assertIsNone(impl.fit_local_ground(cloud))
        subsets = impl.retrieval_subsets(cloud, None)
        np.testing.assert_array_equal(subsets["ground_crop_proxy_v3"], cloud)
        self.assertIsNone(impl.height_stats(cloud, None)["local_ground_high_point_fraction"])

    def test_fft_matches_literal_production_formula_and_yaw_boundary(self):
        impl = self.implementation()
        rng = np.random.default_rng(17)
        query = rng.random((20, 60)).astype(np.float32)
        candidates = np.stack([query, np.roll(query, 30, axis=1), np.roll(query, 31, axis=1)])
        distance, yaw = impl.compare_all(query, candidates)
        for i, candidate in enumerate(candidates):
            exact = impl.direct_compare(query, candidate)
            self.assertAlmostEqual(distance[i], exact[0], places=13)
            self.assertAlmostEqual(yaw[i], exact[1], places=13)
        self.assertAlmostEqual(yaw[1], np.pi)
        self.assertAlmostEqual(yaw[2], -29*np.pi/30)


if __name__ == "__main__":
    unittest.main()
