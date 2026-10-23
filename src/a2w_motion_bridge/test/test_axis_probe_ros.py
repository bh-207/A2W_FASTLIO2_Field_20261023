"""Run only on a deliberately isolated ROS domain; never send hardware requests."""
import json
import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("A2W_ISOLATED_ROS_TEST") != "1",
    reason="requires explicit isolated ROS domain")


@pytest.mark.parametrize("limit,command_values,expected", [
    (.15, (.149, -.003, -.014), {"x": .149, "y": 0., "z": 0.}),
    (.30, (0., .30, 0.), {"x": 0., "y": .30, "z": 0.}),
    (.15, (0., .30, 0.), {"x": 0., "y": .15, "z": 0.}),
])
def test_bridge_audits_exact_post_deadband_sport_request(limit, command_values, expected):
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from unitree_api.msg import Request
    from a2w_motion_bridge.node import A2WMotionBridge
    from rclpy.executors import SingleThreadedExecutor
    assert int(os.environ.get("ROS_DOMAIN_ID", "0")) == 87
    rclpy.init(args=["--ros-args", "-p", "start_enabled:=true",
                    "-p", f"max_lateral:={limit}",
                    "-p", "configure_motion_profile:=false",
                    "-p", "input_topic:=/test/axis_input",
                    "-p", "output_topic:=/test/sport_request"])
    bridge = A2WMotionBridge()
    probe = rclpy.create_node("bridge_audit_test")
    commands, audits = [], []
    subs = [probe.create_subscription(Request, "/test/sport_request", commands.append, 10),
            probe.create_subscription(String, "/a2w_motion_bridge/request_audit",
                                      lambda m: audits.append(json.loads(m.data)), 10)]
    publisher = probe.create_publisher(Twist, "/test/axis_input", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and (not commands or not audits):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = command_values
            publisher.publish(msg)
            executor.spin_once(timeout_sec=.02)
        assert commands and audits
        command = next(m for m in commands if m.header.identity.api_id == 1008)
        audit = next(a for a in audits if a["request_id"] == command.header.identity.id)
        assert audit["parameter"] == json.loads(command.parameter) == expected
        assert audit["api_id"] == 1008
        assert audit["steady_ns"] > 0
    finally:
        executor.shutdown()
        bridge.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


def test_passive_probe_creates_no_command_publisher_and_records_source():
    import io
    import rclpy
    from nav_msgs.msg import Odometry
    from a2w_motion_bridge.axis_probe import AxisProbe
    rclpy.init()
    stream = io.StringIO()
    node = AxisProbe(stream)
    try:
        assert node.publisher is None
        assert not node.healthy()
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = "camera_init", "body"
        msg.pose.pose.orientation.w = 1.0
        node.on_odom(msg)
        events = [json.loads(line) for line in stream.getvalue().splitlines()]
        assert [event["kind"] for event in events] == ["odom", "synchronized_pose"]
        assert events[-1]["data"]["base"] is None  # No invented TF.
        node.stop()  # Passive stop cannot call enable/disable or emit velocity.
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_probe_receives_sport_request_through_humble_executor():
    """Catch the unsupported two-argument callback using actual DDS delivery."""
    import io
    import rclpy
    from unitree_api.msg import Request
    from a2w_motion_bridge.axis_probe import AxisProbe
    assert int(os.environ.get("ROS_DOMAIN_ID", "0")) == 87
    rclpy.init(args=["--ros-args", "-p", "sport_topic:=/test/probe_sport"])
    stream = io.StringIO()
    node = AxisProbe(stream)
    sender = rclpy.create_node("probe_request_sender")
    publisher = sender.create_publisher(Request, "/test/probe_sport", 10)
    try:
        msg = Request()
        msg.header.identity.id = 123
        msg.header.identity.api_id = 1008
        msg.parameter = '{"x":0.1,"y":0.0,"z":0.0}'
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and '"sport_request"' not in stream.getvalue():
            publisher.publish(msg)
            node.spin_for(.05)
        events = [json.loads(line) for line in stream.getvalue().splitlines()]
        request = next(e["data"] for e in events if e["kind"] == "sport_request")
        assert request["message"]["header"]["identity"]["id"] == 123
        assert request["publisher_gid"] is None  # Humble provides no per-message GID.
        assert node.publisher is None
    finally:
        node.destroy_node()
        sender.destroy_node()
        rclpy.shutdown()
