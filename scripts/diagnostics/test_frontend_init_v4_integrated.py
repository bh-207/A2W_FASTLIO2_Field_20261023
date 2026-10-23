"""Actual patched ImuProcess / ESKF, no ROS node or publisher."""
import json
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
BINARY=Path(os.environ.get('FRONTEND_INIT_V4_ISOLATION',str(ROOT/'artifacts/loop_v4_20261021/frontend/isolation_v9')))/'build/frontend_init_v4_harness'

class ActualProcessTest(unittest.TestCase):
    def test_positive_backtrack_legacy_drop_control_reproduces_unsafe_initialization(self):
        self.assertTrue(BINARY.is_file(),'actual Process harness not built')
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'RED'
            result=subprocess.run([str(BINARY),'positive_backtrack_legacy_drop',str(output)],capture_output=True,text=True)
            self.assertEqual(result.returncode,2,result.stdout+result.stderr)
            with (output/'init_quality.csv').open() as stream:rows=list(csv.DictReader(stream))
            initialized=[r for r in rows if r['stage']=='initialized']
            self.assertEqual(len(initialized),1)
            self.assertEqual(int(initialized[0]['accepted_samples']),219)

    def test_actual_process_scenarios(self):
        self.assertTrue(BINARY.is_file(),'actual Process harness not built')
        for case in ['tilted_static','moving_then_static','moving_never_static','late_bad_group','late_bad_input_nan','late_bad_input_stamp','positive_backtrack_buffered','off']:
            with self.subTest(case=case),tempfile.TemporaryDirectory() as directory:
                output=Path(directory)/case
                result=subprocess.run([str(BINARY),case,str(output)],capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                report=json.loads((output/'result.json').read_text())
                self.assertTrue(report['mechanism_checks_passed'])
                self.assertFalse(report['ros_started'])
                self.assertFalse(report['ros_published'])
                self.assertEqual(report['source_provenance'],'synthetic')
                self.assertEqual(report['quality_gate_enabled'],case!='off')
                if case!='off':
                    metadata=json.loads((output/'init_quality.csv.metadata.json').read_text())
                    self.assertEqual(metadata['source_provenance'],'synthetic')
                    self.assertEqual(metadata['gate']['minimum_samples'],200)
                    self.assertEqual(metadata['input']['gyro_unit'],'rad/s')
                    self.assertEqual(metadata['input']['imu_minus_lidar_offset_ns'],0)
                again=subprocess.run([str(BINARY),case,str(output)],capture_output=True,text=True)
                self.assertNotEqual(again.returncode,0,'refuse output overwrite')

if __name__=='__main__':unittest.main()
