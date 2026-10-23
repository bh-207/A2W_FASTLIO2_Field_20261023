"""Compile real standard-C++ core; no ROS or production mutation."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
CORE=ROOT/'patches/fast_lio_v4/frontend_init_v4.hpp'


class FrontendInitCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.binary=None
        if CORE.exists():
            cls.binary=Path(cls.temp.name)/'core_test'
            subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-pedantic',
                '-I'+str(CORE.parent),str(CORE.parent/'test_frontend_init_v4_core.cpp'),'-o',str(cls.binary)],check=True)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def run_case(self,name):
        self.assertIsNotNone(self.binary,'frontend_init_v4 core implementation absent')
        result=subprocess.run([str(self.binary),name],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_static_inclined_passes_without_world_Z_assumption(self):self.run_case('inclined')
    def test_count_AND_continuous_duration(self):self.run_case('count_duration')
    def test_motion_discards_old_window(self):self.run_case('motion_reset')
    def test_late_bad_sample_cancels_midgroup_readiness(self):self.run_case('late_bad_group')
    def test_duplicate_backward_gap_reset(self):self.run_case('timing_reset')
    def test_acc_gyro_variance_and_direction_instability_reset(self):self.run_case('variance_direction')
    def test_nonfinite_unknown_units_and_bad_config_rejected(self):self.run_case('invalid')
    def test_corrected_SI_not_scaled_twice(self):self.run_case('units_once')
    def test_offset_imu_minus_lidar_subtracted_exactly_once(self):self.run_case('offset_sign')
    def test_missing_evidence_frame_or_unit_mismatch_rejected(self):self.run_case('unknown_mismatch')
    def test_production_default_off(self):self.run_case('off_default')


if __name__=='__main__':unittest.main()
