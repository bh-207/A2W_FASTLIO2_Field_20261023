import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT=Path(__file__).with_name('frontend_init_v4_fixture.py')
class FrozenFixtureTest(unittest.TestCase):
    def test_raw_frozen_export_and_no_overwrite(self):
        self.assertTrue(SCRIPT.is_file(),'raw frozen fixture extractor absent')
        spec=importlib.util.spec_from_file_location('fixture',SCRIPT);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        with tempfile.TemporaryDirectory() as temporary:
            output=Path(temporary)/'fixture';report=m.export(m.DEFAULT_SOURCE,output,2)
            self.assertEqual(report['source_provenance'],'synthetic')
            self.assertFalse(report['truth_arrays_loaded'])
            self.assertEqual(report['units']['accel'],'m/s^2')
            self.assertEqual(len(report['source_sha256']),3)
            self.assertTrue((output/'scan_0001.csv').is_file())
            with self.assertRaises(ValueError):m.export(m.DEFAULT_SOURCE,output,2)

if __name__=='__main__':unittest.main()
