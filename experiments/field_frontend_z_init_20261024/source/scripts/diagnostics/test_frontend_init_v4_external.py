import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT=Path(__file__).with_name('frontend_init_v4_tools.py')
spec=importlib.util.spec_from_file_location('tools',SCRIPT);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
TEXT='''Initialization result:
Time Lag IMU to LiDAR (second) = 0.01
Homogeneous Transformation Matrix from LiDAR to IMU:
1 0 0 0.1
0 1 0 0.2
0 0 1 0.3
0 0 0 1
Refinement result:
Time Lag IMU to LiDAR (second) = 0.02
Homogeneous Transformation Matrix from LiDAR to IMU:
1 0 0 0.4
0 1 0 0.5
0 0 1 0.6
0 0 0 1
'''
class ExternalReviewTest(unittest.TestCase):
    def test_official_text_last_refinement_not_initial(self):
        self.assertTrue(hasattr(m,'parse_official_text'),'official result parser absent')
        result=m.parse_official_text(TEXT)
        self.assertEqual(result['offset_s'],.02)
        self.assertEqual(result['t_I_L'],[.4,.5,.6])
        self.assertFalse(result['verified'])
        self.assertEqual(result['source_provenance'],'unknown')
        with self.assertRaises(ValueError):m.review_external(result)

    def test_initial_only_and_ambiguous_matrix_fail_closed(self):
        self.assertTrue(hasattr(m,'parse_official_text'))
        for text in [TEXT.split('Refinement result:')[0],TEXT.replace('0 0 0 1\n','0 0 0 2\n')]:
            with self.assertRaises(ValueError):m.parse_official_text(text)

    def test_offline_external_plan_executes_nothing(self):
        self.assertTrue(hasattr(m,'external_plan'),'independent upstream entry absent')
        with tempfile.TemporaryDirectory() as tmp:
            result=m.external_plan(Path(tmp)/'plan')
            self.assertEqual(result['upstream_run_status'],'NOT_RUN')
            self.assertFalse(result['ros_published'])
            self.assertFalse(result['production_accepted'])
            self.assertTrue((Path(tmp)/'plan/entry_plan.json').is_file())

if __name__=='__main__':unittest.main()
