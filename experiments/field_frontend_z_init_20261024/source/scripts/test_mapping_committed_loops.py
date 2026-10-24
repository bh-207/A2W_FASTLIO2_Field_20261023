"""Pure rosout-filter tests: no rclpy initialization or ROS transport."""
import csv
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).with_name("mapping_committed_loops.py")
VALID = ("[LoopCommitted] audit_v=1 source_id=42 target_id=7 sensor_stamp_ns=123456789000 "
         "commit_wall_ns=987654321000 process_to_commit_ms=12.5 process_clock_status=ok "
         "sensor_wall_latency_ms=unknown clock_relation=unverified fitness=0.125 overlap=0.75 "
         "correspondences=123 factors=44 optimized_translation_m=0.25 optimized_rotation_rad=0 gtsam_ms=2.5")


class CommittedLoopFilterTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCRIPT.is_file(), "committed-loop monitor is not implemented")
        spec = importlib.util.spec_from_file_location("mapping_committed_loops", SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_only_exact_node_and_committed_prefix_are_accepted(self):
        parse = self.module.parse_committed_loop
        for node in ("other_node", "/mapping_backend_node", "mapping_backend_node_extra"):
            self.assertIsNone(parse(node, VALID))
        for message in ("[回环成功] 当前关键帧=42；约束待图优化提交。",
                        "[图优化失败] gtsam_update_failed", "x" + VALID,
                        "[LoopProfile] accepted=true", "[LoopCommittedCandidate] " + VALID):
            self.assertIsNone(parse("mapping_backend_node", message))
        row = parse("mapping_backend_node", VALID)
        self.assertIsNotNone(row)
        self.assertEqual(row["source_id"], 42)
        self.assertEqual(row["target_id"], 7)
        self.assertEqual(row["sensor_stamp_ns"], 123456789000)
        self.assertEqual(row["commit_wall_ns"], 987654321000)
        self.assertEqual(row["process_to_commit_ms"], 12.5)
        self.assertIsNone(row["sensor_wall_latency_ms"])

    def test_custom_logger_is_explicit_not_suffix_matching(self):
        self.assertIsNotNone(self.module.parse_committed_loop("capture.backend", VALID, "capture.backend"))
        self.assertIsNone(self.module.parse_committed_loop("other.capture.backend", VALID, "capture.backend"))

    def test_missing_duplicate_malformed_and_false_clock_fields_fail_closed(self):
        for message in (VALID.replace("source_id=42 ", ""), VALID+" source_id=43",
                        VALID.replace("source_id=42", "source_id=-1"),
                        VALID.replace("commit_wall_ns=987654321000", "commit_wall_ns=nan"),
                        VALID.replace("process_to_commit_ms=12.5", "process_to_commit_ms=-1"),
                        VALID.replace("fitness=0.125", "fitness=nan"),
                        VALID.replace("clock_relation=unverified", "clock_relation=same_clock"),
                        VALID.replace("sensor_wall_latency_ms=unknown", "sensor_wall_latency_ms=-12")):
            self.assertIsNone(self.module.parse_committed_loop("mapping_backend_node", message), message)

    def test_steady_anomaly_is_reported_unknown_not_negative_delay(self):
        message = VALID.replace("process_to_commit_ms=12.5 process_clock_status=ok",
                                "process_to_commit_ms=unknown process_clock_status=anomaly")
        row = self.module.parse_committed_loop("mapping_backend_node", message)
        self.assertIsNotNone(row)
        self.assertIsNone(row["process_to_commit_ms"])
        text = self.module.format_chinese(row)
        self.assertIn("已提交", text)
        self.assertIn("unknown", text)
        self.assertNotIn("保存成功", text)

    def test_csv_contains_exact_int64_and_explicit_unknown_clock(self):
        message = VALID.replace("sensor_stamp_ns=123456789000", "sensor_stamp_ns=1792153757699892044")
        row = self.module.parse_committed_loop("mapping_backend_node", message)
        self.assertIsNotNone(row)
        stream = io.StringIO()
        writer = self.module.csv_writer(stream)
        writer.writerow(row)
        record = list(csv.DictReader(io.StringIO(stream.getvalue())))[0]
        self.assertEqual(record["sensor_stamp_ns"], "1792153757699892044")
        self.assertEqual(record["clock_relation"], "unverified")
        self.assertEqual(record["sensor_wall_latency_ms"], "")
        self.assertEqual(record["source_id"], "42")

    def test_humble_rosout_qos_is_constructible_without_starting_ros(self):
        self.assertTrue(callable(getattr(self.module, "rosout_qos", None)),
                        "monitor must expose its actual subscription QoS without initializing ROS")
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        import rclpy
        self.assertFalse(rclpy.ok())
        qos = self.module.rosout_qos()
        self.assertEqual(qos.depth, 1000)
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        self.assertFalse(rclpy.ok())

    def test_existing_csv_refuses_overwrite_before_starting_ros(self):
        import rclpy
        with tempfile.TemporaryDirectory(prefix="mapping-commit-filter-") as directory:
            path = Path(directory)/"committed.csv"
            path.write_text("preserved evidence\n")
            with self.assertRaises(FileExistsError):
                self.module.main(["--csv", str(path)])
            self.assertEqual(path.read_text(), "preserved evidence\n")
        self.assertFalse(rclpy.ok())


if __name__ == "__main__":
    unittest.main()
