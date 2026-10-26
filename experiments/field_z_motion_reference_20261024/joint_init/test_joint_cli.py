import unittest
import csv
import tempfile
from pathlib import Path
import numpy as np
from joint_cli import propagate_state,run
from test_joint_math import fixture


class PropagationTests(unittest.TestCase):
    def test_raw_si_static_samples_preserve_tilted_gravity_and_bias(self):
        state=dict(t=0.,p=np.array([1.,2.,3.]),R=np.eye(3),v=np.array([.1,-.2,.3]),
                   g=np.array([.2,-.3,-9.8]),ba=np.array([.1,-.04,.07]),bg=np.array([.01,-.02,.03]),cov=np.eye(18)*.001)
        times=np.linspace(0,1.,201);alpha=1.013
        accel=np.tile((-state["g"]+state["ba"])/alpha,(len(times),1))
        gyro=np.tile(state["bg"],(len(times),1))
        result=propagate_state(state,times,accel,gyro,np.array([.5,1.]),alpha,.03,.001)
        self.assertEqual(len(result),2)
        np.testing.assert_allclose(result[-1]["p"],[1.1,1.8,3.3],atol=1e-10)
        np.testing.assert_allclose(result[-1]["R"],np.eye(3),atol=1e-12)
        self.assertGreater(np.trace(result[-1]["cov"]),np.trace(state["cov"]))

    def write_inputs(self,directory,holdout_shift=0.):
        data,*_=fixture();origin=1792724411000000000
        p=data.lidar_p+np.einsum("nij,j->ni",data.lidar_R,[-.00862,.00914,.03916])
        p[data.lidar_t>7.]+=holdout_shift
        poses=directory/"poses.npz";imu=directory/"imu.csv"
        np.savez(poses,stamp_ns=origin+np.rint(data.lidar_t*1e9).astype(np.int64),p=p,R=data.lidar_R,pose_cov=data.pose_cov,
                 scan_ids=np.arange(len(p)),provenance="raw_scan_registration")
        with imu.open("w") as stream:
            writer=csv.writer(stream);writer.writerow(["header_ns","ax","ay","az","gx","gy","gz"])
            for t,a,g in zip(data.imu_t,data.acc,data.gyro):writer.writerow([origin+round(t*1e9),*a,*g])
        return poses,imu

    def test_run_rejects_empty_or_too_small_holdout(self):
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp);poses,imu=self.write_inputs(directory)
            for fraction in (1.,1.1,0.,.99):
                with self.subTest(fraction=fraction):
                    with self.assertRaisesRegex(ValueError,"holdout|train_fraction"):
                        run(poses,imu,directory/"out.json",1.,train_fraction=fraction)

    def test_real_run_fit_excludes_holdout_and_commits_only_after_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp);poses,imu=self.write_inputs(directory)
            good=run(poses,imu,directory/"good.json",1.,train_fraction=.7)
            self.assertTrue(good["accepted"],good)
            self.assertFalse(set(good["training_scan_ids"])&set(good["heldout_scan_ids"]))
            self.assertFalse(good["candidate_holdout"]["used_in_fit"])
            self.assertEqual(good["training_last_stamp_ns"],1792724418000000000)
            self.assertEqual(good["commit_stamp_ns"],1792724421000000000)
            self.assertEqual(good["commit_scan_id"],50)
            self.assertEqual(good["residual_degrees_of_freedom"],good["residual_dimension"]-8)
            poses,imu=self.write_inputs(directory,holdout_shift=100.)
            bad=run(poses,imu,directory/"bad.json",1.,train_fraction=.7)
            self.assertFalse(bad["accepted"])
            self.assertEqual(bad["reason"],"independent_lidar_holdout_prediction_failed")
            for field in ("g_W","ba_I","bg_I"):
                np.testing.assert_allclose(good[field],bad[field],atol=1e-12)


if __name__=="__main__":unittest.main()
