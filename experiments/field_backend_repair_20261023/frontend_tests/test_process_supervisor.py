"""Exercise actual waiters and conservative stop verdicts without ROS or DDS."""
from pathlib import Path
import copy
import importlib.util
import json
import signal
import subprocess
import sys
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "source"
SCRIPTS = SOURCE / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("repair_mapping_capture", SCRIPTS / "mapping_capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class ProcessExitEvidence(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="capture-exit-contract-")
        self.addCleanup(self.temporary.cleanup)
        self.session = Path(self.temporary.name)
        (self.session / "logs").mkdir()
        (self.session / "process_exits").mkdir()
        self.identity = dict(pid=99999999, start_ticks=1234, cmdline=["/nonexistent/owned/frontend"],
            boot_id="fixture-boot", owner_session=str(self.session), role="frontend",
            supervisor_pid=99999998, exit_status=str(self.session / "process_exits/frontend.json"))

    def persist(self, **overrides):
        status = dict(schema="owned_process_exit_v1", identity=copy.deepcopy(self.identity),
            exit_code=0, exited_unix_ns=100, log_flushed=True, log_closed=True)
        status.update(overrides)
        (self.session / "process_exits/frontend.json").write_text(json.dumps(status))

    def report(self):
        return capture.owned_exit_report(self.session, "frontend", self.identity)

    def test_missing_waitpid_status_cannot_pass_for_absent_pid(self):
        result = self.report()
        self.assertFalse(result["valid"])
        self.assertIsNone(result["exit_code"])

    def test_actual_nonzero_exit_cannot_pass(self):
        self.persist(exit_code=7)
        result = self.report()
        self.assertFalse(result["valid"])
        self.assertEqual(result["exit_code"], 7)

    def test_actual_signal_exit_cannot_pass(self):
        self.persist(exit_code=-signal.SIGTERM)
        self.assertFalse(self.report()["valid"])

    def test_open_log_cannot_pass(self):
        self.persist(log_closed=False)
        self.assertFalse(self.report()["valid"])

    def test_unflushed_log_cannot_pass(self):
        self.persist(log_flushed=False)
        self.assertFalse(self.report()["valid"])

    def test_stale_pid_generation_cannot_substitute_for_owned_exit(self):
        stale = copy.deepcopy(self.identity)
        stale["start_ticks"] -= 1
        self.persist(identity=stale)
        self.assertFalse(self.report()["valid"])

    def test_complete_owned_waitpid_status_passes(self):
        self.persist()
        self.assertTrue(self.report()["valid"])

    def test_stop_reports_failure_when_pid_is_absent_and_exit_status_missing(self):
        (self.session / "session.json").write_text(json.dumps(dict(
            schema="mapping_capture_session_v1", session=str(self.session))))
        (self.session / "processes.json").write_text(json.dumps(dict(frontend=self.identity)))
        result = capture.stop_session(self.session, discard_unsaved=True, timeout=0)
        self.assertFalse(result["success"])
        self.assertFalse(result["processes"]["frontend"]["valid"])
        self.assertFalse(result["closures"]["frontend"]["valid"])
        self.assertTrue((self.session / "stop.json").is_file())


class ActualSupervisor(unittest.TestCase):
    def execute(self, code, inherited_sigchld_ignored=False):
        directory = tempfile.TemporaryDirectory(prefix="capture-real-waiter-")
        self.addCleanup(directory.cleanup)
        session = Path(directory.name)
        (session / "logs").mkdir()
        child = [sys.executable, "-c", f"import time,sys; print('real child output',flush=True); time.sleep(.12); sys.exit({code})"]
        command = [sys.executable, str(SCRIPTS / "capture_process_supervisor.py"), str(session), "probe", *child]
        if inherited_sigchld_ignored:
            wrapper = "import os,signal,sys; signal.signal(signal.SIGCHLD,signal.SIG_IGN); os.execv(sys.executable,[sys.executable,*sys.argv[1:]])"
            command = [sys.executable, "-c", wrapper, *command[1:]]
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        status_path = session / "process_exits/probe.json"
        self.assertTrue(status_path.is_file(), result.stderr)
        return session, result, json.loads(status_path.read_text())

    def test_actual_zero_child_has_closed_output_and_zero_status(self):
        session, result, status = self.execute(0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(status["exit_code"], 0)
        self.assertTrue(status["log_flushed"] and status["log_closed"])
        self.assertIn("real child output", (session / "logs/probe.log").read_text())

    def test_actual_nonzero_child_retains_its_exit_code(self):
        _, result, status = self.execute(7)
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertEqual(status["exit_code"], 7)

    def test_inherited_sigchld_ignore_cannot_fabricate_zero_exit(self):
        _, result, status = self.execute(7, inherited_sigchld_ignored=True)
        self.assertEqual(status["exit_code"], 7, "SIGCHLD auto-reap must not fabricate a normal exit")
        self.assertEqual(result.returncode, 7, result.stderr)


if __name__ == "__main__":
    unittest.main()
