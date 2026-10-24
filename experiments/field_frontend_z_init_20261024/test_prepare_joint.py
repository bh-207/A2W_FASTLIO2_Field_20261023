import csv,hashlib,json,tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_joint_candidate import prepare
class GaugeTest(unittest.TestCase):
 def test_state_and_cross_covariance_use_one_SE3(self):
  with tempfile.TemporaryDirectory() as d:
   d=Path(d);poses=d/'poses.npz';np.savez(poses,scan_ids=np.array([13,14]));R=Rotation.from_rotvec([.1,-.2,.3]).as_matrix();q=Rotation.from_matrix(R).as_quat();idx=d/'index.csv'
   idx.write_text('source_scan_id,tx,ty,tz,qx,qy,qz,qw\n13,1,2,3,'+','.join(map(str,q))+'\n');tim=d/'timing.csv';tim.write_text('raw_scan_id,header_ns\n14,123\n')
   rng=np.random.default_rng(7);B=rng.normal(size=(18,18));C=B@B.T*.00001;X=Rotation.from_rotvec([.2,.3,-.2]).as_matrix();p=np.array([.3,1.,2.]);n=dict(accepted=True,source_poses_sha256=hashlib.sha256(poses.read_bytes()).hexdigest(),source_imu_sha256='test',fixed_extrinsic_T=[-.00862,.00914,.03916],fixed_extrinsic_R=np.eye(3).tolist(),commit_scan_id=14,R_WI_commit=X.tolist(),p_WI_commit=p.tolist(),v_WI_commit=[1,2,3],g_W=[0,0,-9.809],ba_I=[.01,.02,.03],bg_I=[.001,.002,.003],full_covariance=C.tolist());src=d/'candidate.json';src.write_text(json.dumps(n));z=prepare(src,poses,idx,tim,d/'mapped.json');t=np.array([1,2,3])+R@np.array(n['fixed_extrinsic_T']);point=np.array([3.,1.,-2.])
   np.testing.assert_allclose(np.array(z['R_WI_commit'])@point+z['p_WI_commit'],R@(X@point+p)+t,atol=1e-12)
   J=np.eye(18)
   for k in [0,6,9]:J[k:k+3,k:k+3]=R
   np.testing.assert_allclose(z['full_covariance'],J@C@J.T,atol=1e-15);self.assertEqual(z['ba_I'],n['ba_I']);self.assertEqual(z['bg_I'],n['bg_I']);self.assertTrue(z['frontend_world_alignment']['not_gravity_or_motion_truth'])
if __name__=='__main__':unittest.main()
