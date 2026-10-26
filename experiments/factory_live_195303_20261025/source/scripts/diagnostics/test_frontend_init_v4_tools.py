"""Integration packaging contracts; source submodule remains read-only."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
SCRIPT=Path(__file__).with_name('frontend_init_v4_tools.py')
impl=None
if SCRIPT.exists():
    spec=importlib.util.spec_from_file_location('frontend_init_v4_tools',SCRIPT)
    impl=importlib.util.module_from_spec(spec);spec.loader.exec_module(impl)


class FrontendInitPackagingTest(unittest.TestCase):
    def test_disabled_v4_does_not_validate_or_round_legacy_offset(self):
        script=Path(__file__).with_name('frontend_init_v4_patchgen.py')
        spec=importlib.util.spec_from_file_location('patchgen',script);p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
        config=p.NODE_CONFIG
        self.assertLess(config.index('if(frontend_input_config_v4.enabled)'),config.index('if(!std::isfinite(time_diff_lidar_to_imu)'))

    def test_effective_config_sidecar_is_written(self):
        script=Path(__file__).with_name('frontend_init_v4_patchgen.py')
        spec=importlib.util.spec_from_file_location('patchgen',script);p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
        self.assertIn('effective_config_v4',p.IMU_METHODS)
        self.assertIn('gyro_evidence',p.IMU_METHODS)
        self.assertIn('maximum_mean_gyro_norm_rad_s',p.IMU_METHODS)

    def test_strict_positive_callback_loopback_notifies_gate(self):
        script=Path(__file__).with_name('frontend_init_v4_patchgen.py')
        spec=importlib.util.spec_from_file_location('patchgen',script);p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
        modified=p.modified_sources(ROOT/'src/FAST_LIO_Hesai')['src/laserMapping.cpp'][1]
        start=modified.index('    if (timestamp < last_timestamp_imu)')
        end=modified.index('    last_timestamp_imu = timestamp;',start)
        block=modified[start:end]
        self.assertIn('if(frontend_input_config_v4.enabled)p_imu->notify_frontend_input_rejection_v4();',block)

    def api(self):
        self.assertIsNotNone(impl,'frontend_init_v4 integration packaging absent')
        return impl

    def test_applies_to_exact_clean_base_without_touching_original(self):
        a=self.api();source=ROOT/'src/FAST_LIO_Hesai'
        before=subprocess.check_output(['git','-C',str(source),'status','--porcelain'])
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)/'isolated'
            manifest=a.prepare(source,output)
            self.assertEqual(manifest['base_commit'],'16e97cdcc4260b9ba6241518a19ad8384224ba48')
            self.assertTrue((output/'source/include/frontend_init_v4.hpp').is_file())
            self.assertTrue((output/'source/src/IMU_Processing.hpp').is_file())
            self.assertTrue(manifest['patch_sha256'])
            self.assertFalse(manifest['ros_started'])
            with self.assertRaises(ValueError):a.prepare(source,output)
        self.assertEqual(before,subprocess.check_output(['git','-C',str(source),'status','--porcelain']))

    def test_source_or_parent_output_rejected(self):
        a=self.api();source=ROOT/'src/FAST_LIO_Hesai'
        for output in [source,source/'inside',source.parent]:
            with self.assertRaises(ValueError):a.prepare(source,output)

    def test_unverified_external_result_cannot_export_config(self):
        a=self.api()
        data=dict(source_provenance='synthetic',R_I_L=[[1,0,0],[0,1,0],[0,0,1]],t_I_L=[.1,.2,.3],
            offset_s=.02,offset_definition='t_imu_raw_minus_t_lidar_same_event',verified=False,
            imu_gyro_unit='rad/s',imu_accel_unit='m/s^2',unit_evidence='fixture',frame_evidence='known',
            temporal_stability_evidence='known',excitation_evidence='known')
        with self.assertRaises(ValueError):a.review_external(data)
        data['verified']=True
        result=a.review_external(data)
        self.assertFalse(result['production_accepted'])
        self.assertEqual(result['time_offset_lidar_to_imu'],.02)
        self.assertEqual(result['physical_transform'],'T_I_L')

    def test_external_wrong_units_time_sign_frame_and_scale_rejected(self):
        a=self.api()
        base=dict(source_provenance='synthetic',R_I_L=[[1,0,0],[0,1,0],[0,0,1]],t_I_L=[0,0,0],
            offset_s=.02,offset_definition='t_imu_raw_minus_t_lidar_same_event',verified=True,
            imu_gyro_unit='rad/s',imu_accel_unit='m/s^2',unit_evidence='fixture',frame_evidence='known',
            temporal_stability_evidence='known',excitation_evidence='known')
        for change in [dict(imu_gyro_unit='unknown'),dict(offset_definition='unknown'),
                       dict(R_I_L=[[2,0,0],[0,1,0],[0,0,1]]),dict(frame_evidence=''),
                       dict(temporal_stability_evidence=''),dict(source_provenance='guess')]:
            with self.subTest(change=change),self.assertRaises(ValueError):a.review_external({**base,**change})


if __name__=='__main__':unittest.main()
