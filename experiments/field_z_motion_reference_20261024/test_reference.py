import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import reference_sequence as rs
import motion_audit as ma


class ReferenceTests(unittest.TestCase):
    def test_submap_is_bounded_causal_and_in_requested_frame(self):
        self.assertTrue(hasattr(rs, 'build_submap'), 'bounded causal submap not implemented')
        history=[]
        for i in range(5):
            history.append(dict(scan_id=i, cloud=np.array([[0.,0,2],[1.,0,2],[0,1.,2]]),
                                R=np.eye(3), p=np.array([i*.1,0.,0.])))
        cloud,ids=rs.build_submap(history,np.eye(3),np.array([.4,0,0]),2)
        self.assertEqual(ids,[3,4]);self.assertEqual(len(cloud),6)
        np.testing.assert_allclose(cloud[:3],history[3]['cloud']+[-.1,0,0])

    def test_submap_rejects_empty_or_unbounded_window(self):
        self.assertTrue(hasattr(rs, 'build_submap'))
        with self.assertRaises(ValueError):rs.build_submap([],np.eye(3),np.zeros(3),0)

    def test_true_objective_polishing_is_opt_in(self):
        import inspect
        self.assertIn('polish_on_stall',inspect.signature(rs.register_point_to_plane).parameters)
        self.assertFalse(inspect.signature(rs.register_point_to_plane).parameters['polish_on_stall'].default)

    def test_gicp_reference_recovers_metric_transform_and_rejects_plane(self):
        self.assertTrue(hasattr(rs,'register_gicp'), 'independent GICP diagnostic not implemented')
        rng=np.random.default_rng(738);uv=rng.uniform(-2,2,(600,2))
        target=np.vstack([np.c_[uv,np.zeros(600)],np.c_[uv[:,0],np.zeros(600),uv[:,1]],np.c_[np.zeros(600),uv]])
        r=Rotation.from_rotvec([.015,-.02,.025]).as_matrix();t=np.array([.07,-.05,.03])
        a=rs.register_gicp((target-t)@r,target)
        self.assertTrue(a['accepted'],a)
        np.testing.assert_allclose(a['R'],r,atol=.001)
        np.testing.assert_allclose(a['t'],t,atol=.001)
        b=rs.register_gicp(np.c_[uv,np.zeros(600)],np.c_[uv,np.zeros(600)])
        self.assertFalse(b['accepted'])

    def test_trajectory_deskew_uses_only_available_scan_end_poses(self):
        self.assertTrue(hasattr(rs,'deskew_trajectory'))
        cloud=np.array([[4.,0,1],[3.9,0,1.]])
        ref=dict(stamp_ns=np.array([0,100000000,200000000]),p=np.array([[0.,0,0],[.1,0,0],[9.,0,0]]),R=np.tile(np.eye(3),(3,1,1)))
        corrected=rs.deskew_trajectory(cloud,np.array([0.,.1]),0,100000000,ref)
        np.testing.assert_allclose(corrected,[[3.9,0,1],[3.9,0,1]],atol=1e-12)

    def test_chord_shape_invariant_under_one_global_se3(self):
        self.assertTrue(hasattr(ma, 'chord_metrics'), 'XYZ chord shape diagnostic not implemented')
        p=np.array([[0.,0,0],[2,1.,3],[4,0.,0]])
        a=ma.chord_metrics(p);r=Rotation.from_rotvec([.2,-.4,.1]).as_matrix()
        b=ma.chord_metrics(p@r.T+[5.,2.,-1.])
        np.testing.assert_allclose(a['chord_distance_m'],[0, np.sqrt(10),0],atol=1e-12)
        np.testing.assert_allclose(a['chord_distance_m'],b['chord_distance_m'],atol=1e-12)
        np.testing.assert_allclose(a['turn_angle_deg'],b['turn_angle_deg'],atol=1e-12)

    def test_displacement_residual_recovers_known_bias_without_differentiating(self):
        self.assertTrue(hasattr(ma, 'window_residual'))
        t=np.arange(0,2.01,.01);f=np.tile([.2,-.4,0.],(len(t),1))
        out=ma.window_residual(t,f,np.zeros(3),np.zeros(3),np.zeros(3))
        np.testing.assert_allclose(out['position_residual_m'],[.4,-.8,0],atol=1e-10)
        np.testing.assert_allclose(out['velocity_residual_m_s'],[.4,-.8,0],atol=1e-10)

if __name__=='__main__':unittest.main()
