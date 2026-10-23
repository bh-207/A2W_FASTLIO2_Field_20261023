"""CLI output-preservation check. Requires the isolated C++ probe binary."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
BINARY = Path(os.environ.get("A2W_REGISTRATION_PROBE_BINARY", str(
    ROOT / "artifacts/loop_recall_v3_20261020/build/registration_frame_probe")))


@unittest.skipUnless(BINARY.is_file(), "isolated registration_frame_probe has not been built")
class ProbeOutputTest(unittest.TestCase):
    def test_nonempty_output_is_rejected_without_overwriting_any_file(self):
        with tempfile.TemporaryDirectory(prefix="a2w_probe_output_") as directory:
            output = Path(directory)
            sentinel = output / "runs.csv"
            sentinel.write_text("prior evidence must survive\n")
            result = subprocess.run([str(BINARY), str(ROOT / "maps/factory_map_1016"),
                                     str(output), "1"], capture_output=True, text=True, timeout=60)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(sentinel.read_text(), "prior evidence must survive\n")
            self.assertEqual([p.name for p in output.iterdir()], ["runs.csv"])


if __name__ == "__main__":
    unittest.main()
