import tempfile,json,hashlib,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_candidate import prepare


class PreparationTests(unittest.TestCase):
    def test_rejected_candidate_never_creates_a_fallback_output(self):
        with tempfile.TemporaryDirectory() as f:
            p=Path(f);(p/'c.json').write_text('{"accepted":false}')
            with self.assertRaisesRegex(ValueError,'no fallback'):prepare(p/'c.json',p/'none',p/'none',p/'out.json')
            self.assertFalse((p/'out.json').exists())

    def test_fixed_extrinsic_contract_is_not_silently_changed(self):
        with tempfile.TemporaryDirectory() as f:
            p=Path(f);c=dict(accepted=True,world_gauge='training_first_LiDAR',fixed_extrinsic_R=np.eye(3).tolist(),fixed_extrinsic_T=[0,0,0],extrinsic_est_en=False,time_offset=0)
            for key in ['source_poses','source_imu']:
                x=p/key;x.write_text('test-only');c[key]=str(x);c[key+'_sha256']=hashlib.sha256(x.read_bytes()).hexdigest()
            c.update(g_ba_constrained_rank=5,heldout_validation=dict(passed=True,used_in_fit=False))
            (p/'c.json').write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError,'fixed'):prepare(p/'c.json',p/'missing',p/'missing',p/'out.json')

    def test_explicit_training_gauge_is_placed_once_with_body_rotation_tangent(self):
        with tempfile.TemporaryDirectory() as f:
            p=Path(f);lever=np.array([-.00862,.00914,.03916]);rot=Rotation.from_euler('z',90,degrees=True);quat=rot.as_quat()
            (p/'b.csv').write_text('source_scan_id,tx,ty,tz,qx,qy,qz,qw\n17,1,2,3,'+','.join(map(str,quat))+'\n')
            (p/'t.csv').write_text('raw_scan_id,header_ns\n99,2000000000\n')
            cov=np.eye(18);cov[0,0]=1;cov[1,1]=2
            c=dict(accepted=True,world_gauge='training_first_LiDAR',world_gauge_scan_id=17,fixed_extrinsic_R=np.eye(3).tolist(),fixed_extrinsic_T=lever.tolist(),extrinsic_est_en=False,time_offset=0,
                   g_ba_constrained_rank=5,heldout_validation=dict(passed=True,used_in_fit=False),commit_scan_id=99,
                   p_WI_commit=(-lever).tolist(),R_WI_commit=np.eye(3).tolist(),v_WI_commit=[1,0,0],g_W=[0,0,-9.809],full_covariance=cov.tolist())
            for key in ['source_poses','source_imu']:
                x=p/key;x.write_text('synthetic test only');c[key]=str(x);c[key+'_sha256']=hashlib.sha256(x.read_bytes()).hexdigest()
            (p/'c.json').write_text(json.dumps(c));a=prepare(p/'c.json',p/'b.csv',p/'t.csv',p/'o.json')
            np.testing.assert_allclose(a['p_WI_commit'],[1,2,3]);np.testing.assert_allclose(a['R_WI_commit'],rot.as_matrix(),atol=1e-12)
            np.testing.assert_allclose(a['v_WI_commit'],[0,1,0],atol=1e-12)
            self.assertAlmostEqual(a['full_covariance'][0][0],2);self.assertAlmostEqual(a['full_covariance'][3][3],1)
            self.assertEqual(a['frontend_world_alignment']['anchor_scan_id'],17)

if __name__=='__main__':unittest.main()
