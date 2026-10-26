import importlib.util
from pathlib import Path
import unittest

class RunContractTest(unittest.TestCase):
    def test_readonly_run_contract(self):
        script=Path(__file__).with_name('frontend_init_v4_run.py')
        self.assertTrue(script.is_file(),'mechanism artifact runner absent')
        spec=importlib.util.spec_from_file_location('runner',script);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
        self.assertEqual(len(m.CASES),8)
        with self.assertRaises(ValueError):m.run(Path('/not/an/isolation'),Path('/not/a/fixture'),Path('/tmp'))

if __name__=='__main__':unittest.main()
