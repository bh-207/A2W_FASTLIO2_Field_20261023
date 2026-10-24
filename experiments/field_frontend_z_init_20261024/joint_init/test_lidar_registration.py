import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from lidar_registration import register_point_to_plane,deskew_constant_velocity,window_rows


class LidarRegistrationTests(unittest.TestCase):
    def test_recovers_known_metric_transform_without_imu_or_fastlio(self):
        rng=np.random.default_rng(738)
        uv=rng.uniform(-2,2,(1200,2))
        target=np.vstack([np.c_[uv,0*uv[:,0]],np.c_[uv[:,0],0*uv[:,0],uv[:,1]],np.c_[0*uv[:,0],uv]])
        rotation=Rotation.from_rotvec([.015,-.02,.025]).as_matrix()
        translation=np.array([.07,-.05,.03])
        source=(target-translation)@rotation
        source+=rng.normal(0,.0005,source.shape)
        result=register_point_to_plane(source,target)
        self.assertTrue(result["accepted"],result)
        np.testing.assert_allclose(result["R"],rotation,atol=.001)
        np.testing.assert_allclose(result["t"],translation,atol=.001)
        self.assertGreater(np.linalg.eigvalsh(result["covariance"]).min(),0.)
        self.assertIn("objective_history",result)
        self.assertLessEqual(np.max(np.diff(result["objective_history"])),1e-12)

    def test_plane_cannot_constrain_six_degree_registration(self):
        rng=np.random.default_rng(2);uv=rng.uniform(-2,2,(2000,2));points=np.c_[uv,uv[:,0]*0]
        result=register_point_to_plane(points,points)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["reason"],"geometry_degenerate")

    def test_deskew_preserves_scan_end_and_applies_signed_motion(self):
        points=np.array([[1.,0,0],[1.,0,0]])
        result=deskew_constant_velocity(points,np.array([0.,.1]),np.array([1.,0,0]),np.zeros(3))
        np.testing.assert_allclose(result,[[.9,0,0],[1.,0,0]],atol=1e-12)

    def test_later_motion_window_preserves_absolute_ns_and_scan_identity(self):
        rows=[dict(stamp_ns=str(1792724411000000000+i*1000000000),scan_id=str(i)) for i in range(230)]
        selected=window_rows(rows,150.,65.,3)
        self.assertEqual(selected[0]["scan_id"],"150")
        self.assertEqual(selected[-1]["scan_id"],"213")
        self.assertEqual(len(selected),22)


if __name__=="__main__":unittest.main()
