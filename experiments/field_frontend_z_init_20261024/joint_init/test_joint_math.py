"""Behavior tests for independent LiDAR / raw-SI IMU joint initialization.

Wrong gravity/bias signs, treating yaw-only motion as observable, dropping pose
uncertainty, or accepting a LiDAR/IMU timestamp gap must break these tests.
"""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from joint_math import JointInput, solve_joint


def fixture(motion="rich", pose_sigma=.001):
    ti=np.linspace(0.,10.,2001)
    tl=np.linspace(0.,10.,51)
    def rotations(t):
        if motion=="static": angles=np.zeros((len(t),3))
        elif motion=="yaw": angles=np.c_[0*t,0*t,.3*t]
        elif motion=="roll": angles=np.c_[.3*t,0*t,0*t]
        else: angles=np.c_[.5*np.sin(.8*t),.4*np.sin(.6*t),.3*t]
        return Rotation.from_euler("xyz",angles).as_matrix()
    g=np.array([.18,-.27,-9.803626474932222])
    ba=np.array([.12,-.08,.045]); bg=np.array([.012,-.009,.006])
    r=rotations(ti)
    # Analytic world translation and its exact second derivative.
    p=np.c_[.2*tl**2,.1*np.sin(tl),.06*np.cos(.7*tl)]
    aw=np.c_[0*ti+.4,-.1*np.sin(ti),-.0294*np.cos(.7*ti)]
    if motion=="static":p[:]=0.;aw[:]=0.
    a=np.einsum("nji,nj->ni",r,aw-g)+ba
    # Midpoint body angular rate from exact adjacent SO(3) samples; endpoints
    # average the two neighboring intervals independently of solver helpers.
    w_interval=Rotation.from_matrix(np.einsum("nji,njk->nik",r[:-1],r[1:])).as_rotvec()/(ti[1:]-ti[:-1])[:,None]
    w=np.vstack([w_interval[0],.5*(w_interval[:-1]+w_interval[1:]),w_interval[-1]])+bg
    cov=np.tile(np.diag([pose_sigma**2]*3+[.0005**2]*3),(len(tl),1,1))
    data=JointInput(tl,p,rotations(tl),ti,a,w,cov,
                    provenance="raw_scan_registration",accel_sigma=.025,gyro_sigma=.001,
                    t_il=np.zeros(3))
    return data,g,ba,bg


class JointMathTests(unittest.TestCase):
    def test_recovers_nonzero_gravity_and_both_biases_in_si(self):
        data,g,ba,bg=fixture()
        result=solve_joint(data)
        self.assertTrue(result["accepted"],result)
        np.testing.assert_allclose(result["g_W"],g,atol=.018)
        np.testing.assert_allclose(result["ba_I"],ba,atol=.018)
        np.testing.assert_allclose(result["bg_I"],bg,atol=.0005)
        cov=np.array(result["covariance_g_ba_bg"])
        self.assertEqual(cov.shape,(9,9))
        eigenvalues=np.linalg.eigvalsh(cov)
        self.assertGreaterEqual(eigenvalues.min(),-1e-12)
        self.assertGreater(eigenvalues[1],0.)  # S2 removes only radial g DOF.

    def test_static_data_reports_three_unidentifiable_g_ba_directions(self):
        result=solve_joint(fixture("static")[0])
        self.assertFalse(result["accepted"])
        self.assertEqual(result["g_ba_rank"],3)
        self.assertEqual(np.array(result["g_ba_nullspace"]).shape,(3,6))
        self.assertIn("s2_g_ba_rank",result)
        self.assertEqual(result["s2_g_ba_rank"],3)
        self.assertEqual(np.array(result["s2_g_ba_nullspace"]).shape,(2,5))

    def test_yaw_with_known_gravity_length_identifies_direction_and_bias(self):
        data,g,ba,bg=fixture("yaw")
        result=solve_joint(data)
        self.assertTrue(result["accepted"],result)
        self.assertEqual(result["g_ba_rank"],5)
        null=np.array(result["g_ba_nullspace"])[0]
        self.assertGreater(abs(null[2]),.65)
        self.assertGreater(abs(null[5]),.65)
        self.assertEqual(result["s2_g_ba_rank"],5)
        np.testing.assert_allclose(result["g_W"],g,atol=.018)
        np.testing.assert_allclose(result["ba_I"],ba,atol=.018)
        np.testing.assert_allclose(result["bg_I"],bg,atol=.0005)

    def test_axis_perpendicular_to_gravity_retains_direction_ambiguity(self):
        data,*_=fixture("roll")
        # Exact gravity perpendicular to the only angular axis. A length
        # constraint cannot eliminate its tangent g_x / ba_x ambiguity.
        correction=np.array([-.18,.27,9.803626474932222-9.809])
        r=Rotation.from_euler("x",.3*data.imu_t).as_matrix()
        data.acc-=np.einsum("nji,j->ni",r,correction)
        result=solve_joint(data)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["g_ba_rank"],5)
        self.assertIn("s2_g_ba_rank",result)
        self.assertEqual(result["s2_g_ba_rank"],4)

    def test_two_physical_s2_branches_within_bias_bound_are_rejected(self):
        data,g0,ba0,_=fixture("roll")
        g1=np.array([.8,0.,-np.sqrt(9.809**2-.8**2)])
        ba1=np.array([.8,0.,0.])
        r=Rotation.from_euler("x",.3*data.imu_t).as_matrix()
        data.acc+=np.einsum("nji,j->ni",r,g0-g1)+(ba1-ba0)
        # Reflecting only g_x and ba_x leaves the actual force samples equal.
        g2=g1.copy();g2[0]=-.8;ba2=ba1.copy();ba2[0]=-.8
        np.testing.assert_allclose(np.einsum("nji,j->ni",r,g1-g2)-(ba1-ba2),0.,atol=1e-12)
        result=solve_joint(data)
        self.assertFalse(result["accepted"],result)
        self.assertEqual(result["reason"],"ambiguous_s2_branch")
        self.assertGreaterEqual(len(result["s2_branch_candidates"]),2)

    def test_pose_uncertainty_increases_output_covariance(self):
        low=solve_joint(fixture(pose_sigma=.0005)[0])
        high=solve_joint(fixture(pose_sigma=.02)[0])
        self.assertTrue(low["accepted"]); self.assertTrue(high["accepted"])
        self.assertGreater(np.trace(high["covariance_g_ba_bg"]),np.trace(low["covariance_g_ba_bg"])*2.)

    def test_rejects_fastlio_pose_provenance(self):
        data,*_=fixture();data.provenance="FAST_LIO_body_cloud"
        with self.assertRaisesRegex(ValueError,"independent"):
            solve_joint(data)

    def test_rejects_raw_imu_gap(self):
        data,*_=fixture(); keep=(data.imu_t<4.)|(data.imu_t>4.15)
        data.imu_t=data.imu_t[keep];data.acc=data.acc[keep];data.gyro=data.gyro[keep]
        with self.assertRaisesRegex(ValueError,"gap"):
            solve_joint(data)

    def test_s2_constraint_retains_direction_and_zero_radial_uncertainty(self):
        result=solve_joint(fixture()[0])
        self.assertAlmostEqual(np.linalg.norm(result["g_W"]),9.809,places=8)
        g=np.array(result["g_W"])
        self.assertGreater(abs(g[0]),.1)
        self.assertLess(abs(g@np.array(result["covariance_g_ba_bg"])[:3,:3]@g),1e-10)

    def test_explicit_legacy_scale_and_lever_arm_use_internal_bias_domain(self):
        data,g,ba,bg=fixture()
        data.acc_scale=1.0135127790522112;data.acc=data.acc/data.acc_scale
        data.t_il=np.array([-.00862,.00914,.03916])
        data.lidar_p+=np.einsum("nij,j->ni",data.lidar_R,data.t_il)
        result=solve_joint(data)
        self.assertTrue(result["accepted"],result)
        np.testing.assert_allclose(result["ba_I"],ba,atol=.018)
        np.testing.assert_allclose(result["ba_raw_equivalent_report_only"],ba/data.acc_scale,atol=.018)
        self.assertEqual(np.array(result["full_covariance"]).shape,(18,18))


if __name__=="__main__": unittest.main()
