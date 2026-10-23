from pathlib import Path
import json
import subprocess
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
FRONTEND = HERE.parent / "source/frontend"

class Helpers(unittest.TestCase):
    def test_trace_real_disk_failure(self):
        with tempfile.TemporaryDirectory(prefix="frontend-disk-error-") as directory:
            executable = str(Path(directory) / "disk_error")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-misleading-indentation", "-pthread",
                f"-I{FRONTEND / 'include'}", f"-I{FRONTEND / 'src'}", str(HERE / "test_trace_write_failure.cpp"), "-o", executable], check=True)
            subprocess.run([executable, directory], check=True)
            status = json.loads((Path(directory) / "failed_handoff_trace_status.json").read_text())
            self.assertTrue(status["closed"])
            self.assertFalse(status["flush_ok"])
            self.assertGreater(status["errors"], 0)

    def test_required_endpoint_subset(self):
        with tempfile.TemporaryDirectory(prefix="frontend-endpoints-") as directory:
            executable = str(Path(directory) / "endpoints")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-misleading-indentation", "-pthread",
                f"-I{FRONTEND / 'include'}", f"-I{FRONTEND / 'src'}", str(HERE / "test_required_endpoints.cpp"), "-o", executable], check=True)
            subprocess.run([executable], check=True)

    def test_real_helpers(self):
        for header in ("frontend_lifecycle.hpp", "pose_covariance.hpp"):
            self.assertTrue((FRONTEND / "include" / header).is_file(), f"missing production helper: {header}")
        with tempfile.TemporaryDirectory(prefix="frontend-contracts-") as directory:
            executable = str(Path(directory) / "contracts")
            subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-misleading-indentation", "-pthread",
                "-I/usr/include/eigen3", f"-I{FRONTEND / 'include'}", f"-I{FRONTEND / 'src'}",
                str(HERE / "test_frontend_contracts.cpp"), "-o", executable], check=True)
            subprocess.run([executable, directory], check=True)
            captured = Path(directory) / "capture/run"
            trace_status = json.loads((captured / "frontend_handoff_trace_status.json").read_text())
            self.assertEqual(trace_status["written"], 500)
            self.assertEqual(trace_status["attempted"], 500)
            self.assertEqual(trace_status["errors"], 0)
            self.assertTrue(trace_status["closed"] and trace_status["flush_ok"])
            self.assertEqual(len((captured / "frontend_handoff.csv").read_text().splitlines()), 501)
            exit_status = json.loads((captured / "frontend_exit_status.json").read_text())
            self.assertEqual(exit_status["exit_code"], 2)
            self.assertEqual(exit_status["signal"], 2)
            self.assertEqual(exit_status["error"], "a real callback error")
            self.assertTrue(exit_status["handoff_trace_closed"] and exit_status["handoff_trace_flush_ok"])

if __name__ == "__main__":
    unittest.main()
