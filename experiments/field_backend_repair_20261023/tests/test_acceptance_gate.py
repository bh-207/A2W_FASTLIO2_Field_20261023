import importlib.util
from pathlib import Path
import unittest
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from accept_run import judge

class GateTest(unittest.TestCase):
    def test_helper_success_does_not_override_actual_missing_imu(self):
        result=dict(valid=True,rate=1.,requested_duration_s=20.,exit_codes={r:0 for r in ['frontend','ingress','backend','recorder','field_observer']},
            ordered_stop=dict(success=True),fixed_state_closed=dict(valid=True),runtime_shutdown=dict(valid=True),handoff=dict(valid=True),full_output_handoff=dict(valid=True),
            recorder_log=dict(checked=True,loss_reported=False,errors=[]))
        accounting={k:dict(complete=True) for k in ['published_vs_callback_full','callback_vs_recorded_full','published_vs_recorded_full']}
        self.assertEqual(judge(result,accounting,dict(complete=True))['status'],'PASS_SCOPED_OFFLINE')
        accounting['published_vs_callback_full']=dict(complete=False,a_only_count=1,b_only_count=0)
        failure=judge(result,accounting,dict(complete=True))
        self.assertEqual(failure['status'],'FAIL')
        self.assertIn('published_to_imu_callback_exact',failure['failed_checks'])
        accounting['published_vs_callback_full']=dict(complete=True)
        result['rate']=.25
        self.assertIn('original_rate',judge(result,accounting,dict(complete=True))['failed_checks'])
    def test_missing_accounting_is_failure(self):
        self.assertEqual(judge({}, {}, {})['status'],'FAIL')

if __name__=='__main__':unittest.main()
