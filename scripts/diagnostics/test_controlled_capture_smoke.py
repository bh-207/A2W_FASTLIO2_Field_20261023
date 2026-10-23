"""Offline smoke-runner contracts. Does not initialize ROS or launch processes."""
import importlib.util
import json
from pathlib import Path
import struct
import unittest

SCRIPT=Path(__file__).with_name('controlled_capture_smoke.py')
if SCRIPT.exists():
    spec=importlib.util.spec_from_file_location('controlled_smoke',SCRIPT)
    tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
else:tool=None

class ControlledCaptureSmokeTests(unittest.TestCase):
    def runner(self):
        self.assertIsNotNone(tool,'CONTROLLED CAPTURE RUNNER MISSING: full real-node chain has no owned orchestration')
        return tool
    def test_default_plan_never_starts_ros_and_uses_synthetic_profile(self):
        plan=self.runner().make_plan('/repo','/output','/build','/fixture',218)
        self.assertFalse(plan['starts_ros']);self.assertEqual(plan['source_provenance'],'synthetic')
        self.assertIn('--synthetic-test',plan['init_command']);self.assertIn('--base-config',plan['init_command'])
        self.assertEqual(plan['raw_record_cache_mib'],1)
    def test_loopback_explicit_nonzero_domain_and_no_external_dds(self):
        runner=self.runner();env={'ROS_LOCALHOST_ONLY':'1','ROS_DOMAIN_ID':'218'}
        runner.validate_environment(env,218,True)
        for bad,domain,authorized in [({},218,True),(env,0,True),(env,218,False),
            ({**env,'ROS_DISCOVERY_SERVER':'remote:11811'},218,True),({**env,'CYCLONEDDS_URI':'file:remote.xml'},218,True)]:
            with self.assertRaises(ValueError):runner.validate_environment(bad,domain,authorized)
    def test_one_delta_shifts_imu_header_and_every_point_without_mutating_fixture(self):
        runner=self.runner();packer=struct.Struct('<ffffH6xd')
        original=packer.pack(1,2,3,40,0,1000.)+packer.pack(4,5,6,50,1,1000.05)
        delta=1_000_000_007
        begin,end,data=runner.shift_scan((1_000_000_000_000,1_000_050_000_000,original),delta)
        self.assertEqual(begin,1_001_000_000_007);self.assertEqual(end,1_001_050_000_007)
        self.assertAlmostEqual(struct.unpack_from('<d',data,24)[0],1001.000000007,places=10)
        self.assertAlmostEqual(struct.unpack_from('<d',data,56)[0],1001.050000007,places=10)
        self.assertEqual(data[:24],original[:24]);self.assertEqual(data[32:56],original[32:56])
        self.assertEqual(struct.unpack_from('<d',original,24)[0],1000.)
        shifted=runner.shift_imu([1000.,1,2,3,.01,.02,.03],delta)
        self.assertEqual(shifted['stamp_ns'],begin);self.assertEqual(shifted['accel'],[1,2,3])
    def test_frozen_anchor_after_ready_has_grid_continuity(self):
        anchor=self.runner().choose_frozen_anchor(1_000_000_000,1_234_000_000)
        self.assertEqual(anchor,1_600_000_000);self.assertGreaterEqual(anchor,1_534_000_000)
        self.assertEqual((anchor-1_000_000_000)%100_000_000,0)
        self.assertEqual((anchor-1_000_000_000)%5_000_000,0)
    def test_real_cli_json_not_last_nested_object(self):
        data=self.runner().parse_cli_json('ROS diagnostic\n'+json.dumps({'ready':True,'nested':{'value':2}},indent=2)+'\n')
        self.assertEqual(data,{'ready':True,'nested':{'value':2}})
    def test_fixture_rotation_is_synthetic_not_guessed_or_world_alignment(self):
        q=self.runner().matrix_quaternion([0.,-1.,0.,1.,0.,0.,0.,0.,1.])
        self.assertAlmostEqual(q[0],0.);self.assertAlmostEqual(q[1],0.)
        self.assertAlmostEqual(q[2],.7071067811865476);self.assertAlmostEqual(q[3],.7071067811865476)
    def test_recorded_static_tf_accepts_actual_last_transforms_schema(self):
        runner=self.runner()
        self.assertTrue(callable(getattr(runner,'recorded_synthetic_tf',None)),
            'smoke must consume actual verify headers.last_transforms, not missing transforms')
        # Consumer boundary from full_capture_smoke_v4 verification.json. No ROS or fake TF publication.
        for key in ['last_transforms','transforms']:
            with self.subTest(schema=key):
                verified={'headers':{'/tf_static':{key:[{'parent':'body','child':'synthetic_sensor'}]}}}
                before=json.dumps(verified,sort_keys=True)
                self.assertTrue(runner.recorded_synthetic_tf(verified))
                self.assertEqual(before,json.dumps(verified,sort_keys=True))
    def test_recorded_static_tf_rejects_missing_wrong_frame_and_dynamic_only(self):
        runner=self.runner()
        self.assertTrue(callable(getattr(runner,'recorded_synthetic_tf',None)),
            'smoke must inspect recorded static TF without fabricating absent evidence')
        for verified in [{},{'headers':{'/tf_static':{'last_transforms':[]}}},
            {'headers':{'/tf_static':{'last_transforms':[{'parent':'other','child':'synthetic_sensor'}]}}},
            {'headers':{'/tf':{'last_transforms':[{'parent':'body','child':'synthetic_sensor'}]}}},
            {'headers':{'/tf_static':{'last_transforms':[],
                'transforms':[{'parent':'body','child':'synthetic_sensor'}]}}}]:
            with self.subTest(verified=verified):self.assertFalse(runner.recorded_synthetic_tf(verified))

if __name__=='__main__':unittest.main()
