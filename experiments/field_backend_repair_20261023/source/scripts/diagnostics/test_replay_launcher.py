"""Check the isolated replay launcher without ROS, sensor data, or registration."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("run_loop_replay_experiment.sh")


class ReplayLauncherTest(unittest.TestCase):
    def test_cached_mode_cannot_start_without_matching_isolated_libraries(self):
        with tempfile.TemporaryDirectory(prefix="a2w_replay_launcher_") as directory:
            root = Path(directory)
            bundle = root / "bundle"
            bundle.mkdir()
            (bundle / "manifest.sha256").write_text("synthetic fixture only\n")
            output = root / "new"
            result = subprocess.run(
                ["bash", str(SCRIPT), "/bin/true", str(bundle),
                 str(output), "scheduled_cached", "0"],
                capture_output=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(output.exists())

    def test_failure_exit_status_and_existing_output_are_preserved(self):
        with tempfile.TemporaryDirectory(prefix="a2w_replay_launcher_") as directory:
            root = Path(directory)
            bundle = root / "bundle"
            bundle.mkdir()
            (bundle / "manifest.sha256").write_text("synthetic fixture only\n")
            output = root / "new"
            result = subprocess.run(["bash", str(SCRIPT), "/bin/false", str(bundle),
                                     str(output), "legacy", "0"], capture_output=True, timeout=10)
            self.assertEqual(result.returncode, 1)
            evidence = root / "new.execution"
            self.assertEqual((evidence / "exit_code.txt").read_text(), "1\n")
            identity = (evidence / "command_identity.txt").read_bytes()
            again = subprocess.run(["bash", str(SCRIPT), "/bin/true", str(bundle),
                                    str(output), "legacy", "0"], capture_output=True, timeout=10)
            self.assertNotEqual(again.returncode, 0)
            self.assertEqual((evidence / "command_identity.txt").read_bytes(), identity)

    def test_input_bundle_cannot_be_an_output_parent(self):
        with tempfile.TemporaryDirectory(prefix="a2w_replay_launcher_") as directory:
            bundle = Path(directory)
            result = subprocess.run(["bash", str(SCRIPT), "/bin/true", str(bundle),
                                     str(bundle / "bad_output"), "scheduled", "1"],
                                    capture_output=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(list(bundle.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
