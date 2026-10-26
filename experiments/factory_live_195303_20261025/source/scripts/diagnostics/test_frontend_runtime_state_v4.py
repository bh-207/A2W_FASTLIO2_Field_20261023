"""Run real writer lifecycle, immutable snapshots, overload and disk failure."""
import csv
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]

class RuntimeStateWriterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.binary=Path(cls.temp.name)/'writer'
        subprocess.run(['g++','-std=c++17','-O2','-pthread','-I',str(ROOT/'patches/fast_lio_v4'),
            str(ROOT/'patches/fast_lio_v4/test_frontend_runtime_state_v4.cpp'),'-o',str(cls.binary)],check=True)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def execute(self,case):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        path=Path(directory.name)/'states.csv'
        result=subprocess.run([str(self.binary),case,str(path)],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        return path,result
    def test_default_off_has_no_files_or_side_effects(self):
        path,_=self.execute('disabled');self.assertEqual(list(path.parent.iterdir()),[])
    def test_snapshot_source_time_frames_and_init_angle(self):
        path,_=self.execute('golden')
        with path.open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),2);self.assertEqual(rows[0]['stage'],'POST_LIDAR_UPDATE')
        self.assertEqual(rows[0]['state_stamp_ns'],'1099999999');self.assertEqual(rows[0]['pos_O_I_x'],'1')
        self.assertEqual(rows[1]['stage'],'INIT_COMMIT');self.assertEqual(rows[1]['pos_O_I_x'],'999')
        self.assertEqual(rows[1]['state_stamp_ns'],'1095000000')
        # Hand-checked atan(4.9/8.5), not a value derived by writer helper.
        self.assertAlmostEqual(float(rows[1]['anti_g_O_to_plus_Z_O_rad']),.5229387635711719,places=10)
        meta=json.loads(Path(str(path)+'.metadata.json').read_text())
        self.assertFalse(meta['gravity_alignment']);self.assertEqual(meta['frames'],{'O':'camera_init','I':'body','L':'LiDAR'})
        status=json.loads(Path(str(path)+'.status.json').read_text())
        self.assertEqual(status['written'],2);self.assertTrue(status['closed']);self.assertTrue(status['flush_ok'])
    def test_overflow_is_counted_and_alarmed_without_unbounded_queue(self):
        path,result=self.execute('overflow');status=json.loads(Path(str(path)+'.status.json').read_text())
        self.assertGreater(status['dropped'],0);self.assertEqual(status['capacity'],1)
        self.assertLessEqual(status['maximum_pending'],1);self.assertIn('QUEUE_DROP',result.stderr)
        self.assertEqual(status['attempted'],status['enqueued']+status['dropped'])
    def test_real_disk_write_failure_is_not_a_flush_pass(self):
        path,result=self.execute('write_error');status=json.loads(Path(str(path)+'.status.json').read_text())
        self.assertGreater(status['write_errors'],0);self.assertFalse(status['flush_ok'])
        self.assertIn('WRITE_ERROR',result.stderr)
    def test_existing_result_refuses_overwrite(self):
        path,_=self.execute('golden');before=path.read_bytes()
        result=subprocess.run([str(self.binary),'golden',str(path)],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0);self.assertEqual(path.read_bytes(),before)
    def test_init_commit_flushes_before_shutdown_for_readiness(self):
        path,_=self.execute('live_init')
        with path.open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),1);self.assertEqual(rows[0]['stage'],'INIT_COMMIT')

if __name__=='__main__':unittest.main()
