import hashlib
import os
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from ament_index_python.packages import get_package_prefix
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from a2w_fastlio_msgs.msg import LocalizationStatus, RegistrationStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


BUNDLE_PARENT = tempfile.mkdtemp(prefix="a2w-stage9-")
BUNDLE_PATH = os.path.join(BUNDLE_PARENT, "bundle")
GENERATOR = os.path.join(
    get_package_prefix("a2w_fastlio_localization"),
    "lib", "a2w_fastlio_localization", "create_test_bundle")
subprocess.run([GENERATOR, BUNDLE_PATH], check=True)


def bundle_digest():
    digest = hashlib.sha256()
    for path in sorted(Path(BUNDLE_PATH).rglob("*")):
        if path.is_file():
            digest.update(str(path.relative_to(BUNDLE_PATH)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


BEFORE_DIGEST = bundle_digest()


def generate_test_description():
    node = launch_ros.actions.Node(
        package="a2w_fastlio_localization",
        executable="localization_node",
        output="screen",
        parameters=[{
            "map_bundle_path": BUNDLE_PATH,
            "odom_topic": "/stage9/odom",
            "body_cloud_topic": "/stage9/cloud",
            "pose_topic": "/stage9/pose",
            "localized_odom_topic": "/stage9/localized_odom",
            "path_topic": "/stage9/path",
            "registration_status_topic": "/stage9/registration_status",
            "localization_status_topic": "/stage9/localization_status",
            "global_tf_owner_topic": "/stage9/owner",
            "owner_conflict_window_ms": 100,
            "owner_heartbeat_ms": 20,
            "match_interval_ms": 50,
            "monitor.initialization_successes_required": 1,
            "monitor.degraded_failures_required": 1,
            "monitor.lost_failures_required": 2,
            # Exercise match-failure transitions independently of the watchdog.
            "monitor.correction_stale_after_ms": 60000,
            "input_timeout_ms": 5000,
            "monitor.strong_minimum_correspondences": 1000,
            "global_relocalization.enabled": True,
            "diagnostics.handoff_ab": True,
            "global_relocalization.top_k": 5,
            "global_relocalization.neighbor_keyframes_before": 0,
            "global_relocalization.neighbor_keyframes_after": 0,
            "global_relocalization.confirmation_count": 2,
            "global_relocalization.strong_minimum_correspondences": 1000,
            "global_relocalization.worker_queue_capacity": 1,
            # A completed valid result must survive a performance-budget warning.
            "global_relocalization.evaluation_time_budget_ms": 1,
            "initialization.mode": "fixed_region",
            "initialization.center_x": 5.0,
            "initialization.center_y": 0.0,
            "initialization.size_x": 3.0,
            "initialization.size_y": 3.0,
            "initialization.anchor_keyframe_id": 0,
            "initialization.keyframe_window": 30,
            "local_map.selection_radius_m": 50.0,
            "local_map.minimum_neighbors": 1,
            "local_map.voxel_leaf_m": 0.0,
            "quatro.minimum_points": 20,
            "nano_gicp.minimum_points": 20,
            "validation.minimum_correspondences": 20,
            "validation.minimum_overlap": 0.2,
            "validation.maximum_fitness": 0.5,
        }],
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {"localization": node}


def set_stamp(header, nanoseconds):
    header.stamp.sec = nanoseconds // 1_000_000_000
    header.stamp.nanosec = nanoseconds % 1_000_000_000


def make_odom(stamp, x):
    message = Odometry()
    set_stamp(message.header, stamp)
    message.header.frame_id = "camera_init"
    message.child_frame_id = "body"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def make_cloud(stamp):
    message = PointCloud2()
    set_stamp(message.header, stamp)
    message.header.frame_id = "body"
    points = []
    for x in range(5):
        for y in range(5):
            for z in range(4):
                points.append((
                    0.4 * x + 0.03 * y * z,
                    0.35 * y + 0.02 * x * z,
                    0.3 * z + 0.01 * x * y,
                    float(x * 20 + y * 4 + z),
                ))
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 16
    message.row_step = 16 * len(points)
    message.data = list(b"".join(struct.pack("<ffff", *point) for point in points))
    message.is_dense = True
    return message


class TestGlobalRelocalizationLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("stage9_global_relocalization_test")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.node.create_publisher(Odometry, "/stage9/odom", qos)
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/stage9/cloud", qos)
        self.states, self.audits, self.transforms = [], [], []
        self.node.create_subscription(
            LocalizationStatus, "/stage9/localization_status", self.states.append, qos)
        self.node.create_subscription(
            RegistrationStatus, "/stage9/registration_status", self.audits.append, qos)
        self.node.create_subscription(TFMessage, "/tf", self.transforms.append, 100)
        self.spin_until(lambda: (
            self.odom_pub.get_subscription_count() > 0 and
            self.cloud_pub.get_subscription_count() > 0))

    def tearDown(self):
        self.node.destroy_node()

    def publish_pair(self, stamp, odom_x):
        self.odom_pub.publish(make_odom(stamp, odom_x))
        self.cloud_pub.publish(make_cloud(stamp))

    def spin_until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate())

    def map_odom_transforms(self):
        return [
            transform for batch in self.transforms for transform in batch.transforms
            if transform.header.frame_id == "map" and
            transform.child_frame_id == "camera_init"
        ]

    def test_lost_rejection_then_consistent_global_recovery(self, proc_output, localization):
        stamp = 10_000_000_000
        for _ in range(5):
            self.publish_pair(stamp, 0.0)
            stamp += 1_000_000
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(state.state == LocalizationStatus.LOCALIZED for state in self.states):
                break
        self.spin_until(lambda: any(
            state.state == LocalizationStatus.LOCALIZED for state in self.states))
        initial_localized = next(
            state for state in self.states
            if state.state == LocalizationStatus.LOCALIZED)
        self.assertEqual(
            initial_localized.reason, "initial_global_localization_confirmed")
        self.assertTrue(any(
            audit.stage == "global_relocalization" and audit.accepted
            for audit in self.audits))
        self.assertFalse(any(
            audit.stage == "localization" and audit.accepted
            for audit in self.audits))
        self.spin_until(lambda: any(
            abs(transform.transform.translation.x - 5.0) < 0.3
            for transform in self.map_odom_transforms()))

        # Retain one real accepted fine-only context before provoking failure.
        stamp += 1_000_000
        self.publish_pair(stamp, 0.0)
        self.spin_until(lambda: any(
            audit.stage == "localization" and audit.accepted for audit in self.audits))

        stamp += 30_000_000
        self.publish_pair(stamp, 20.0)
        self.spin_until(lambda: any(
            state.state == LocalizationStatus.LOCALIZED and
            state.reason == "full_recovery_pending" for state in self.states))
        self.assertFalse(any(
            state.state in (LocalizationStatus.DEGRADED, LocalizationStatus.LOST)
            for state in self.states))

        # The next fresh frame performs the requested full recovery. Only its
        # failure is allowed to move LOCALIZED -> DEGRADED.
        stamp += 1_000_000
        self.publish_pair(stamp, 20.0)
        self.spin_until(lambda: any(
            state.state == LocalizationStatus.DEGRADED for state in self.states))
        self.assertFalse(any(
            state.state == LocalizationStatus.LOST for state in self.states))
        # A/B is armed only after initial global acceptance and records both
        # modes for the same new synchronized frame. Diagnostic B must not
        # promote the state; the separately scheduled full recovery owns the
        # transition to DEGRADED.
        proc_output.assertWaitFor("[handoff_ab]", process=localization, timeout=5)
        proc_output.assertWaitFor("A_fine_only", process=localization, timeout=5)
        proc_output.assertWaitFor("B_full", process=localization, timeout=5)
        proc_output.assertWaitFor("[fine_only_failure]", process=localization, timeout=5)
        proc_output.assertWaitFor("上一成功={", process=localization, timeout=5)

        stamp += 1_000_000
        self.publish_pair(stamp, 21.0)
        self.spin_until(lambda: any(
            state.state == LocalizationStatus.RELOCALIZING for state in self.states))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        frozen_count = len(self.map_odom_transforms())
        deadline = time.monotonic() + 0.12
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(len(self.map_odom_transforms()), frozen_count)

        stamp += 1_000_000
        recovery_start = len(self.states)
        audit_start = len(self.audits)
        self.publish_pair(stamp, 20.0)
        self.spin_until(lambda: any(
            audit.stage == "global_relocalization" and audit.accepted
            for audit in self.audits[audit_start:]))
        self.assertFalse(any(
            state.state == LocalizationStatus.LOCALIZED
            for state in self.states[recovery_start:]))

        stamp += 1_000_000
        self.publish_pair(stamp, 20.0)
        self.spin_until(lambda: any(
            state.state == LocalizationStatus.LOCALIZED and
            state.reason == "relocalization_accepted" for state in self.states))
        self.spin_until(lambda: any(
            abs(transform.transform.translation.x + 15.0) < 0.3
            for transform in self.map_odom_transforms()))
        self.assertTrue(all(
            state.hardware_validation_pending for state in self.states))
        self.assertEqual(bundle_digest(), BEFORE_DIGEST)


@launch_testing.post_shutdown_test()
class TestGlobalRelocalizationExit(unittest.TestCase):
    def test_clean_shutdown(self, proc_info, localization):
        launch_testing.asserts.assertExitCodes(proc_info, process=localization)
