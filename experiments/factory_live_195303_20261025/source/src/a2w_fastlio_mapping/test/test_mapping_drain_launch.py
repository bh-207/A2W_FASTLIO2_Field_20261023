"""Exercise real backend backlog, frozen saves and signal shutdown without hardware."""
import csv
import importlib.util
import os
import signal
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from a2w_fastlio_msgs.msg import RegistrationStatus
from a2w_fastlio_msgs.srv import SaveMapBundle
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import PointCloud2

_helpers_spec = importlib.util.spec_from_file_location(
    "mapping_test_helpers", Path(__file__).with_name("test_mapping_outputs_launch.py"))
_helpers = importlib.util.module_from_spec(_helpers_spec)
_helpers_spec.loader.exec_module(_helpers)
make_cloud, make_odom = _helpers.make_cloud, _helpers.make_odom


BUNDLE_ROOT = tempfile.mkdtemp(prefix="a2w-mapping-drain-")


def backend(prefix, period_ms):
    return launch_ros.actions.Node(
        package="a2w_fastlio_mapping", executable="mapping_backend_node",
        name=prefix, output="screen", parameters=[{
            "keyframe_odom_topic": f"/{prefix}/odom",
            "keyframe_cloud_topic": f"/{prefix}/cloud",
            "registration_status_topic": f"/{prefix}/status",
            "global_tf_owner_topic": f"/{prefix}/owner",
            "optimized_map_preview_topic": f"/{prefix}/preview",
            "save_map_bundle_service": f"/{prefix}/save",
            "bundle_root": BUNDLE_ROOT,
            "worker_period_ms": period_ms,
            "status_period_ms": 100,
            "preview_period_ms": 5000,
            "scan_context.exclude_recent": 1000,
            # A deliberately tiny legacy queue must not drop mapping data.
            "loop_pipeline.queue_capacity": 2,
            "full_map_voxel_leaf_m": 0.0,
        }])


def generate_test_description():
    drain = backend("drain", 30)
    stop_int = backend("stop_int", 200)
    stop_term = backend("stop_term", 200)
    bad_input = backend("bad_input", 30)
    return launch.LaunchDescription([
        drain, stop_int, stop_term, bad_input, launch_testing.actions.ReadyToTest(),
    ]), {"drain": drain, "stop_int": stop_int, "stop_term": stop_term, "bad_input": bad_input}


class TestMappingDrain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("mapping_drain_driver")
        self.stats = {}
        self.previews = []

    def tearDown(self):
        self.node.destroy_node()

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if predicate():
                return True
        return False

    def inputs(self, prefix):
        def status(message):
            if message.stage == "backend":
                self.stats[prefix] = dict(
                    token.split("=", 1) for token in message.reason.split() if "=" in token)
        self.node.create_subscription(RegistrationStatus, f"/{prefix}/status", status, 100)
        self.node.create_subscription(PointCloud2, f"/{prefix}/preview", self.previews.append, 10)
        pubs = (
            self.node.create_publisher(Odometry, f"/{prefix}/odom", 100),
            self.node.create_publisher(PointCloud2, f"/{prefix}/cloud", 100))
        self.assertTrue(self.wait(lambda: all(p.get_subscription_count() for p in pubs)))
        return pubs

    def send(self, pubs, count, start=0):
        for i in range(start, start + count):
            stamp = (i + 1) * 100_000_000
            pubs[0].publish(make_odom(stamp, i * 0.1))
            pubs[1].publish(make_cloud(stamp))
            # Keep DDS delivery below its queue depth, while deliberately outrunning processing.
            until = time.monotonic() + 0.004
            while time.monotonic() < until:
                rclpy.spin_once(self.node, timeout_sec=0.001)

    def test_391_frames_drain_before_save_and_full_map_built_only_once(self, drain, proc_output):
        pubs = self.inputs("drain")
        self.send(pubs, 391)
        self.assertTrue(self.wait(
            lambda: self.stats.get("drain", {}).get("received") == "391"),
            "Backend must report all 391 received frames, without queue-capacity drops")
        self.assertGreater(int(self.stats["drain"]["pending"]), 0)
        self.assertEqual(self.stats["drain"]["full_map_builds"], "0")
        self.assertLessEqual(len(self.previews), 1, "Preview must not be rebuilt for each keyframe")
        client = self.node.create_client(SaveMapBundle, "/drain/save")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        future = client.call_async(SaveMapBundle.Request(output_path="all_391"))
        self.assertTrue(self.wait(lambda: self.stats["drain"].get("state") == "draining"))
        self.assertFalse(future.done(), "Save cannot succeed while frames are pending")
        early_output = b"".join(event.text for event in proc_output[drain]).decode(errors="replace")
        self.assertNotIn("[地图就绪]", early_output)
        self.assertNotIn("[保存成功]", early_output)
        # Frames arriving after the save boundary must not alter the frozen snapshot.
        self.send(pubs, 5, start=391)
        concurrent = client.call_async(SaveMapBundle.Request(output_path="concurrent"))
        self.assertTrue(self.wait(lambda: concurrent.done()))
        self.assertFalse(concurrent.result().success)
        self.assertEqual(concurrent.result().message, "save_busy")
        self.assertTrue(self.wait(lambda: future.done(), timeout=25))
        response = future.result()
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.keyframe_count, 391)
        self.assertTrue(self.wait(lambda: self.stats["drain"].get("state") == "saved"))
        stats = self.stats["drain"]
        self.assertEqual(stats["received"], stats["processed"])
        self.assertEqual(stats["pending"], "0")
        self.assertEqual(stats["active"], "none")
        self.assertEqual(stats["full_map_builds"], "1")
        self.assertGreater(int(stats["last_completed_stamp_ns"]), 0)
        self.assertLessEqual(int(stats["preview_map_builds"]), 4)
        bundle = Path(response.resolved_path)
        with (bundle / "keyframes/index.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 391)
        self.assertEqual(rows[-1]["stamp_ns"], "39100000000")
        with (bundle / "global_map.pcd").open("rb") as stream:
            header = stream.read(250).split(b"DATA binary")[0]
        self.assertIn(b"POINTS 1173\n", header)  # 391 frames * 3 original points
        for message in (
                "[结束采集]", "[队列排空] received=391 processed=391 pending=0 active=none",
                "[最终优化]", "[地图同步]", "[地图就绪]", "[保存成功]",
                "尚无已提交的回环"):
            proc_output.assertWaitFor(message, process=drain, timeout=2)
        output = b"".join(event.text for event in proc_output[drain]).decode(errors="replace")
        phases = [output.index(tag) for tag in (
            "[结束采集]", "[队列排空]", "[最终优化]", "[地图同步]", "[地图就绪]", "[保存成功]")]
        self.assertEqual(phases, sorted(phases))

    def test_invalid_input_cannot_be_saved_as_a_complete_map(self, bad_input, proc_output):
        pubs = self.inputs("bad_input")
        self.send(pubs, 1)
        self.assertTrue(self.wait(lambda: self.stats.get("bad_input", {}).get("processed") == "1"))
        cloud = make_cloud(200_000_000)
        cloud.header.frame_id = "incorrect_frame"
        pubs[0].publish(make_odom(200_000_000, 1.0))
        pubs[1].publish(cloud)
        self.assertTrue(self.wait(lambda: self.stats["bad_input"].get("state") == "fault"))
        client = self.node.create_client(SaveMapBundle, "/bad_input/save")
        self.assertTrue(client.wait_for_service(timeout_sec=5))
        future = client.call_async(SaveMapBundle.Request(output_path="must_not_exist"))
        self.assertTrue(self.wait(lambda: future.done()))
        self.assertFalse(future.result().success)
        self.assertIn("mapping_processing_failed", future.result().message)
        self.assertFalse((Path(BUNDLE_ROOT) / "must_not_exist").exists())
        proc_output.assertWaitFor("[保存失败]", process=bad_input, timeout=2)
        output = b"".join(event.text for event in proc_output[bad_input]).decode(errors="replace")
        self.assertNotIn("[地图就绪]", output)
        self.assertNotIn("[保存成功]", output)

    def test_signals_drain_and_preserve_an_in_progress_save(self, stop_int, stop_term, proc_info, proc_output):
        for prefix, process, sig in (
                ("stop_int", stop_int, signal.SIGINT),
                ("stop_term", stop_term, signal.SIGTERM)):
            pubs = self.inputs(prefix)
            self.send(pubs, 20)
            self.assertTrue(self.wait(lambda: self.stats.get(prefix, {}).get("received") == "20"))
            self.assertGreater(int(self.stats[prefix]["pending"]), 0)
            future = None
            if sig == signal.SIGTERM:
                client = self.node.create_client(SaveMapBundle, f"/{prefix}/save")
                self.assertTrue(client.wait_for_service(timeout_sec=5))
                future = client.call_async(SaveMapBundle.Request(output_path="save_during_shutdown"))
                self.assertTrue(self.wait(lambda: self.stats[prefix].get("state") == "draining"))
            os.kill(process.process_details["pid"], sig)
            if future is not None:
                self.assertTrue(self.wait(lambda: future.done()))
                self.assertTrue(future.result().success, future.result().message)
                self.assertEqual(future.result().keyframe_count, 20)
            proc_info.assertWaitForShutdown(process=process, timeout=12)
            self.assertEqual(proc_info[process].returncode, 0)
            proc_output.assertWaitFor("received=20 processed=20 pending=0", process=process)
            proc_output.assertWaitFor(
                f"full_map_builds={1 if future else 0}", process=process)
            proc_output.assertWaitFor("shutdown_drained", process=process)


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
