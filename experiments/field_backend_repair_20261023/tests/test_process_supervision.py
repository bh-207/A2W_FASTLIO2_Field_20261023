import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1] / 'source'
sys.path.insert(0, str(ROOT / 'scripts'))
import mapping_capture as capture


class ProcessSupervisionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.session = Path(self.tmp.name)
        (self.session / 'logs').mkdir()
        (self.session / 'processes.json').write_text('{}')

    def tearDown(self):
        self.tmp.cleanup()

    def test_normal_exit_is_recorded_by_waiter(self):
        child, identity = capture.spawn_owned(self.session, 'test',
            [sys.executable, '-c', 'import sys,time; time.sleep(.15)', str(self.session)])
        self.assertEqual(child.wait(timeout=5), 0)
        result = capture.owned_exit_report(self.session, 'test', identity)
        self.assertTrue(result['valid'], result)
        self.assertEqual(result['exit_code'], 0)

    def test_nonzero_exit_cannot_be_normal(self):
        child, identity = capture.spawn_owned(self.session, 'test',
            [sys.executable, '-c', 'import sys,time; time.sleep(.15); sys.exit(7)', str(self.session)])
        self.assertEqual(child.wait(timeout=5), 7)
        result = capture.owned_exit_report(self.session, 'test', identity)
        self.assertFalse(result['valid'])
        self.assertEqual(result['exit_code'], 7)

    def test_pid_disappearance_without_status_is_not_success(self):
        result = capture.owned_exit_report(self.session, 'test',
            {'pid': 99999999, 'start_ticks': 1, 'owner_session': str(self.session), 'role': 'test'})
        self.assertFalse(result['valid'])
        self.assertIsNone(result['exit_code'])

    def test_signal_exit_remains_distinguishable(self):
        child, identity = capture.spawn_owned(self.session, 'test',
            [sys.executable, '-c', 'import time; time.sleep(30)', str(self.session)])
        capture.signal_owned(identity, signal.SIGTERM)
        child.wait(timeout=5)
        result = capture.owned_exit_report(self.session, 'test', identity)
        self.assertFalse(result['valid'])
        self.assertEqual(result['exit_code'], -signal.SIGTERM)


if __name__ == '__main__':
    unittest.main()
