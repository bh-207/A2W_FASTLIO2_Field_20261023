"""Read-only parser contracts; synthetic fixtures do not certify real gravity."""
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).with_name("frontend_state_audit.py")
impl = None
if SCRIPT.exists():
    spec = importlib.util.spec_from_file_location("frontend_state_audit", SCRIPT)
    impl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(impl)

TRACE = ("1.2 predicted 1 2 3 0 0 0 1 .1 .2 .3 0 0 0 1 0 0 -9.809 .01 .02 .03 .1 .2 .3\n"
         "1.2 updated 1 2 4 0 0 0 1 .1 .2 .4 0 0 0 1 0 0 -9.809 .01 .02 .03 .1 .2 .3\n")
SENSOR = ("imu 1 1 2 3 0 0 .1\nimu 1.1 3 4 5 0 0 .3\n"
          "cloud 1 20 123456\nsync 1 1.2 2 1 1.1\n")


class FrontendStateAuditTest(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(impl, "frontend_state_audit implementation absent")
        return impl

    def fixture(self, folder):
        trace, sensor = folder / "dbg.txt", folder / "sensor_audit.txt"
        trace.write_text(TRACE)
        sensor.write_text(SENSOR)
        return trace, sensor

    def run_cli(self, folder, *extra):
        trace, sensor = self.fixture(folder)
        return subprocess.run([sys.executable, str(SCRIPT), "--trace", str(trace),
            "--sensor-audit", str(sensor), "--output", str(folder / "out"),
            "--provenance", "synthetic", "--provenance-note", "known fixture",
            "--gyro-unit", "rad/s", *extra], capture_output=True, text=True)

    def test_parses_real_v4_column_order_and_body_gravity(self):
        a = self.api()
        rows = a.parse_trace(TRACE.splitlines())
        self.assertEqual(rows[0]["ext_tz"], .3)
        self.assertEqual(rows[1]["tz"], 4)
        self.assertEqual(rows[0]["g_I_z"], -9.809)
        self.assertAlmostEqual(rows[0]["bg_norm_rad_s"], .03741657386773942)

    def test_proxy_means_variances_do_not_claim_internal_window(self):
        a = self.api()
        sensors = a.parse_sensor(SENSOR.splitlines())
        result = a.startup_statistics(sensors, 1.2, "rad/s", "unknown", .5)
        self.assertEqual(result["sample_count"], 2)
        self.assertAlmostEqual(result["span_s"], .1)
        self.assertEqual(result["acc_mean_raw"], [2., 3., 4.])
        self.assertEqual(result["acc_variance_population_raw"], [1., 1., 1.])
        self.assertAlmostEqual(result["gyro_mean_norm_rad_s"], .2)
        self.assertEqual(result["internal_initialization_window"]["status"], "missing")
        self.assertIsNone(result["internal_initialization_window"]["internal_N"])

    def test_bad_width_nonfinite_and_bad_quaternion_rejected(self):
        a = self.api()
        for text in ["1 predicted 0", TRACE.replace("-9.809", "nan"),
                     TRACE.replace("0 0 0 1 .1", "0 0 0 0 .1", 1)]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                a.parse_trace(text.splitlines())
        with self.assertRaises(ValueError):
            a.parse_sensor(["sync 1 2 2 1"])

    def test_negative_order_and_gyro_quality_are_preserved(self):
        a = self.api()
        sensors = a.parse_sensor(["imu 1.1 1 2 3 0 0 1", "imu 1 1 2 3 0 0 1", "imu 1 1 2 3 0 0 1"])
        result = a.startup_statistics(sensors, 1.2, "rad/s", "unknown", .5)
        self.assertEqual(result["sample_count"], 3)
        self.assertEqual(result["duplicate_timestamp_count"], 1)
        self.assertEqual(result["backward_timestamp_count"], 1)
        self.assertFalse(result["gyro_mean_norm_check_pass"])

    def test_unknown_gyro_unit_does_not_forge_si_check(self):
        a = self.api()
        result = a.startup_statistics(a.parse_sensor(SENSOR.splitlines()), 1.2, "unknown", "unknown", .5)
        self.assertIsNone(result["gyro_mean_norm_rad_s"])
        self.assertIsNone(result["gyro_mean_norm_check_pass"])
        self.assertIsNone(a.startup_statistics(None, 1.2, "unknown", "unknown", .5)["sample_count"])

    def test_cli_negative_sensor_quality_is_visible_in_warning_file(self):
        self.api()
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            trace, sensor = self.fixture(folder)
            sensor.write_text("imu 1.1 0 0 9.81 0 0 1\nimu 1 0 0 9.81 0 0 1\nimu 1 0 0 9.81 0 0 1\n")
            result = subprocess.run([sys.executable, str(SCRIPT), "--trace", str(trace),
                "--sensor-audit", str(sensor), "--output", str(folder / "out"),
                "--provenance", "synthetic", "--provenance-note", "negative fixture", "--gyro-unit", "rad/s"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            warnings = (folder / "out/WARNINGS.txt").read_text()
            self.assertIn("gyro norm exceeds", warnings)
            self.assertIn("backward or duplicate IMU", warnings)

    def test_native_parser_labels_relative_begin_and_57_3_scaling(self):
        a = self.api()
        row = a.parse_native([" ".join(map(str, [0, 57.3, 0, 0, 1, 2, 3, 0, 0, 0,
            .1, .2, .3, 0, 0, 0, .01, .02, .03, .1, .2, .3, 0, 0, -9.809, 123]))])[0]
        self.assertEqual(row["time_basis"], "relative_scan_begin_label_state_at_scan_end")
        self.assertAlmostEqual(row["roll_rad"], 1)
        self.assertEqual(row["point_count"], 123)

    def test_cli_outputs_hashes_curves_svg_and_missing_qualification(self):
        self.api()
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            result = self.run_cli(folder)
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((folder / "out/summary.json").read_text())
            self.assertEqual(summary["provenance"]["kind"], "synthetic")
            self.assertEqual(summary["frames"], {"W": "camera_init", "I": "body", "L": "LiDAR"})
            self.assertEqual(summary["trace"]["updated_count"], 1)
            self.assertTrue(summary["inputs"][0]["sha256"])
            with (folder / "out/states.csv").open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 2)
            for name in ["gravity.svg", "bias.svg", "extrinsic.svg"]:
                self.assertEqual(ET.parse(folder / "out" / name).getroot().tag, "{http://www.w3.org/2000/svg}svg")
            self.assertIn("unknown", " ".join(summary["warnings"]))

    def test_cli_requires_provenance_and_refuses_existing_output(self):
        self.api()
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.assertEqual(self.run_cli(folder).returncode, 0)
            before = (folder / "out/summary.json").read_bytes()
            self.assertNotEqual(self.run_cli(folder).returncode, 0)
            self.assertEqual(before, (folder / "out/summary.json").read_bytes())
            result = subprocess.run([sys.executable, str(SCRIPT), "--trace", str(folder / "dbg.txt"),
                "--output", str(folder / "another")], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((folder / "another").exists())

    def test_missing_updated_and_sensor_are_explicit(self):
        self.api()
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            trace = folder / "dbg.txt"
            trace.write_text(TRACE.splitlines()[0] + "\n")
            result = subprocess.run([sys.executable, str(SCRIPT), "--trace", str(trace),
                "--output", str(folder / "out"), "--provenance", "real", "--provenance-note", "unverified real capture"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((folder / "out/summary.json").read_text())
            self.assertEqual(summary["trace"]["updated_count"], 0)
            self.assertIsNone(summary["startup_imu_proxy"]["sample_count"])
            self.assertIn("missing updated", " ".join(summary["warnings"]))


if __name__ == "__main__":
    unittest.main()
