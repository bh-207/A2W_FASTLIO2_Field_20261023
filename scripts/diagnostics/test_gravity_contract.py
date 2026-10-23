"""Known-coordinate tests, not a real-sensor/initialization acceptance test."""
import unittest
import numpy as np

try:
    import gravity_contract as g
except ImportError:
    g = None


class GravityContractTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(g, "gravity/frame conversion contract is not implemented")

    def ref(self, vector=(0, 0, -9.81), **kwargs):
        return dict(vector=list(vector), frame_id="body", stamp_ns=10,
                    quality_valid=True, source="synthetic_known_gravity", **kwargs)

    def test_no_gravity_is_not_replaced_by_map_z(self):
        with self.assertRaisesRegex(ValueError, "gravity"):
            g.alignment(None, "body", 10)

    def test_rejects_wrong_frame_and_future_measurement(self):
        with self.assertRaisesRegex(ValueError, "frame"):
            g.alignment(self.ref(), "lidar", 10)
        with self.assertRaisesRegex(ValueError, "future"):
            g.alignment(self.ref(), "body", 9)

    def test_epoch_must_be_exact_unless_explicit_age_budget_is_supplied(self):
        with self.assertRaisesRegex(ValueError, "age"):
            g.alignment(self.ref(), "body", 11)
        aligned = g.alignment(self.ref(), "body", 11, max_reference_age_ns=1)
        self.assertEqual(aligned["max_reference_age_ns"], 1)
        for stamp in [-1, float("nan"), 10.5, True]:
            with self.assertRaises(ValueError):
                g.alignment(self.ref(), "body", stamp)

    def test_rejects_low_quality_and_bad_norm(self):
        ref = self.ref()
        ref["quality_valid"] = False
        with self.assertRaisesRegex(ValueError, "quality"):
            g.alignment(ref, "body", 10)
        for value in [(0,0,0), (0,0,-1), (np.nan,0,-9.81)]:
            with self.assertRaises(ValueError):
                g.alignment(self.ref(value), "body", 10)

    def test_known_roll_maps_gravity_to_negative_z(self):
        a = g.alignment(self.ref((0, 9.81, 0)), "body", 10)
        np.testing.assert_allclose(a["matrix"][:3,:3] @ [0,9.81,0], [0,0,-9.81], atol=1e-10)
        self.assertEqual(a["gravity_source"], "synthetic_known_gravity")
        self.assertEqual(a["version"], "gravity-rp-v3-1")

    def test_nearly_opposite_gravity_is_still_a_valid_rigid_rotation(self):
        for x in [0.0, 1e-5, 1e-4]:
            gravity = np.array([x, 0., 9.81])
            a = g.alignment(self.ref(gravity), "body", 10)["matrix"]
            np.testing.assert_allclose(a[:3,:3] @ gravity,
                                       [0, 0, -np.linalg.norm(gravity)], atol=1e-12)

    def test_same_start_orientation_is_identity(self):
        np.testing.assert_allclose(g.alignment(self.ref(), "body", 10)["matrix"], np.eye(4), atol=1e-12)

    def test_repeated_normalization_rejected(self):
        with self.assertRaisesRegex(ValueError, "already"):
            g.alignment(self.ref(), "body", 10, input_version="gravity-rp-v3-1")

    def test_independent_source_target_transform_recovery(self):
        # A_s=R_x(+90deg), A_t=R_y(+90deg), T_n translates [1,2,3].
        a_s=np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]],float)
        a_t=np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]],float)
        t_n=np.eye(4); t_n[:3,3]=[1,2,3]
        expected=np.array([[0,-1,0,-3],[0,0,-1,2],[1,0,0,1],[0,0,0,1]],float)
        actual=g.restore_registration(t_n,a_s,a_t)
        np.testing.assert_allclose(actual,expected,atol=1e-12)
        np.testing.assert_allclose(g.transform_points([[2,3,4]],actual),[[-6,-2,3]],atol=1e-12)

    def test_transform_roundtrip_preserves_points(self):
        t=np.array([[0,-1,0,3],[1,0,0,-2],[0,0,1,7],[0,0,0,1]],float)
        points=np.array([[0,0,0],[2,3,4],[-1,8,-2]],float)
        np.testing.assert_allclose(g.transform_points(g.transform_points(points,t),np.linalg.inv(t)),points,atol=1e-12)

    def test_known_slope_preserved_after_global_rotation(self):
        # Two planes with normals [0,0,1] and [0,-.5,sqrt(3)/2] differ by 30deg.
        n1=np.array([0.,0.,1.]); n2=np.array([0.,-.5,np.sqrt(3)/2])
        a=g.alignment(self.ref((0,9.81,0)),"body",10)["matrix"]
        n1r=a[:3,:3]@n1; n2r=a[:3,:3]@n2
        self.assertAlmostEqual(float(np.degrees(np.arccos(n1r@n2r))),30.0,places=10)
        points=np.array([[0,0,0],[0,2,2/np.sqrt(3)]],float)
        moved=g.transform_points(points,a)
        self.assertAlmostEqual(float(np.linalg.norm(moved[1]-moved[0])),4/np.sqrt(3),places=10)

    def test_rejects_non_rigid_scale_and_reflection(self):
        for t in [np.diag([2.,1.,1.,1.]), np.diag([-1.,1.,1.,1.])]:
            with self.assertRaisesRegex(ValueError,"rigid"):
                g.restore_registration(t,np.eye(4),np.eye(4))

    def test_alignment_records_reference_not_a_physical_extrinsic(self):
        a=g.alignment(self.ref(),"body",10)
        self.assertEqual(a["source_frame"],"body")
        self.assertEqual(a["target_frame"],"body_gravity_rp")
        self.assertEqual(a["gravity_stamp_ns"],10)
        self.assertFalse(a["physical_extrinsic"])

    def test_map_pose_rotation_preserves_relative_pose_and_full_3d(self):
        a=np.array([[1,0,0,2],[0,0,-1,3],[0,1,0,4],[0,0,0,1]],float)
        p=np.eye(4); p[:3,3]=[1,2,3]
        q=np.array([[0,-1,0,4],[1,0,0,5],[0,0,1,6],[0,0,0,1]],float)
        np.testing.assert_allclose(np.linalg.inv(a@p)@(a@q),np.linalg.inv(p)@q,atol=1e-12)
        self.assertAlmostEqual(g.transform_points([[1,2,3]],a)[0,2],6.)


if __name__ == "__main__":
    unittest.main()
