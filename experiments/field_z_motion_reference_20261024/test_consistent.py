import unittest
import numpy as np
from pathlib import Path
import tempfile,csv
import consistent_init as ci


class ConsistentIntegrationTests(unittest.TestCase):
    def test_static_nonobservable_fit_returns_rejected_candidate_not_exception(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);times=np.arange(17)*.25;origin=1792724411000000000
            np.savez(root/'poses.npz',stamp_ns=origin+np.rint(times*1e9).astype(np.int64),p=np.zeros((17,3)),R=np.tile(np.eye(3),(17,1,1)),pose_cov=np.eye(102)*1e-4,scan_ids=np.arange(17),provenance='raw_scan_registration')
            with (root/'imu.csv').open('w') as f:
                w=csv.writer(f);w.writerow(['header_ns','ax','ay','az','gx','gy','gz'])
                for t in np.arange(0,4.005,.005):w.writerow([origin+round(t*1e9),0,0,9.809,0,0,0])
            result=ci.run(root/'poses.npz',root/'imu.csv',root/'result.json',alpha=1.)
            self.assertFalse(result['accepted'])
            self.assertEqual(result['reason'],'insufficient_conditioned_g_ba_excitation')

    def test_reference_contract_rejects_wrong_provenance_rotation_and_covariance(self):
        self.assertTrue(hasattr(ci,'validate_poses'))
        q=dict(stamp_ns=np.arange(6,dtype=np.int64)*100000000,p=np.zeros((6,3)),R=np.tile(np.eye(3),(6,1,1)),pose_cov=np.eye(36),provenance='raw_scan_registration',scan_ids=np.arange(6))
        ci.validate_poses(q)
        for field,value in [('provenance','FAST_LIO'),('R',np.zeros((6,3,3))),('pose_cov',np.eye(36)*-1)]:
            bad=dict(q);bad[field]=value
            with self.assertRaises(ValueError):ci.validate_poses(bad)

    def test_common_prefix_gauge_uncertainty_cancels_in_relative_pose_prior(self):
        self.assertTrue(hasattr(ci,'relative_imu_pose_covariance'))
        from scipy.spatial.transform import Rotation
        from joint_init.lidar_registration import skew
        r=Rotation.from_rotvec([[.1,-.2,.4],[.2,-.1,.6]]).as_matrix();p=np.array([[1.,2.,.3],[2.,3.,.2]])
        j=np.zeros((12,6))
        for i in range(2):
            j[6*i:6*i+3,:3]=np.eye(3);j[6*i:6*i+3,3:]=-skew(p[i]);j[6*i+3:6*i+6,3:]=r[i].T
        cov,T=ci.relative_imu_pose_covariance(p,r,j@j.T,np.array([-.00862,.00914,.03916]))
        np.testing.assert_allclose(cov,0,atol=1e-12)
        np.testing.assert_allclose(T@j,0,atol=1e-12)

    def test_mean_only_does_not_allocate_a_quadratic_pose_covariance(self):
        it=np.arange(0,1.005,.005);a=np.tile([0,0,9.809],(len(it),1))
        s,C,_,_=ci._integrate_all(it,a,np.zeros_like(a),it,np.r_[0,0,-9.809,np.zeros(9)],1.,np.zeros(3),.035,.002,False)
        self.assertLessEqual(C.size,1)

    def test_state_and_measurement_share_the_same_gyro_propagated_orientation(self):
        self.assertTrue(hasattr(ci,'integrate'), 'consistent propagation not implemented')
        t=np.arange(0,2.005,.005);times=np.array([0.,1.,2.])
        a=np.tile([.2,0,9.809],(len(t),1));w=np.zeros_like(a)
        s,c,x=ci.integrate(t,a,w,times,np.array([0,0,-9.809,0,0,0,0,0,0,0,0,0]),1.,np.zeros(3),.035,.002,True)
        np.testing.assert_allclose(s[-1,:3],[.4,0,0],atol=1e-10)
        np.testing.assert_allclose(s[-1,7:],[.4,0,0],atol=1e-10)
        np.testing.assert_allclose(s[-1,3:7],[0,0,0,1],atol=1e-12)
        self.assertEqual(c.shape,(18,18));self.assertEqual(x.shape,(9,18))
        self.assertGreaterEqual(np.linalg.eigvalsh(c).min(),-1e-12)

    def test_unbracketed_input_is_rejected(self):
        self.assertTrue(hasattr(ci,'integrate'))
        t=np.arange(0,1,.005);a=np.tile([0,0,9.809],(len(t),1))
        with self.assertRaises(ValueError):ci.integrate(t,a,np.zeros_like(a),[0,2],np.zeros(12),1.,np.zeros(3),.035,.002,False)

    def test_shared_raw_sample_covariance_matches_direct_finite_differences(self):
        from scipy.spatial.transform import Rotation
        it=np.arange(0,.201,.01);times=np.array([.005,.055,.15,.195]);n=len(times)
        acc=np.tile([.2,.1,9.809],(len(it),1));gyro=np.tile([.01,.02,.03],(len(it),1))
        par=np.r_[0,0,-9.809,.1,.05,.01,.001,.002,.003,.2,0,.1]
        s,C,X,P=ci._integrate_all(it,acc,gyro,times,par,1.013,np.zeros(3),.035,.002,True)
        baseR=Rotation.from_quat(s[:,3:7]);jp=[];js=[]
        for sample in range(len(it)):
            for kind in range(6):
                a=acc.copy();w=gyro.copy();eps=1e-5
                if kind<3:a[sample,kind]+=eps
                else:w[sample,kind-3]+=eps
                q=ci.integrate(it,a,w,times,par,1.013,np.zeros(3),.035,.002,False)[0]
                dtheta=(baseR.inv()*Rotation.from_quat(q[:,3:7])).as_rotvec()
                scale=.035 if kind<3 else .002
                jp.append(np.c_[q[:,:3]-s[:,:3],dtheta].ravel()/eps*scale)
                js.append(np.r_[q[-1,:3]-s[-1,:3],dtheta[-1],q[-1,7:]-s[-1,7:]]/eps*scale)
        jp=np.array(jp).T;js=np.array(js).T
        np.testing.assert_allclose(C,jp@jp.T,rtol=.01,atol=1e-9)
        np.testing.assert_allclose(X,js@jp.T,rtol=.01,atol=1e-9)
        np.testing.assert_allclose(P,js@js.T,rtol=.01,atol=1e-9)

if __name__=='__main__':unittest.main()
