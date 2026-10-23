"""Isolated ROS check of terminal vy compensation; never use robot DDS domain."""
import json
import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("A2W_ISOLATED_ROS_TEST") != "1",
    reason="requires isolated ROS test domain")


def test_bridge_boosts_only_fresh_terminal_error():
    assert os.environ.get("ROS_DOMAIN_ID") in ("87", "88")
    import rclpy
    from geometry_msgs.msg import Twist, Vector3Stamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from unitree_api.msg import Request
    from std_msgs.msg import String
    from a2w_motion_bridge.node import A2WMotionBridge

    rclpy.init(args=["--ros-args", "-p", "output_topic:=/test/terminal_sport_request",
                     "-p", "max_lateral:=0.30", "-p", "min_effective_lateral:=0.30",
                     "-p", "start_enabled:=true", "-p", "configure_motion_profile:=false"])
    bridge = A2WMotionBridge()
    peer = Node("terminal_bridge_test_peer")
    received = []
    audits = []
    audit_sub = peer.create_subscription(
        String, '/a2w_motion_bridge/request_audit',
        lambda m: audits.append(json.loads(m.data)), 50)
    sub = peer.create_subscription(
        Request, "/test/terminal_sport_request", received.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(peer)
    try:
        deadline = time.monotonic()+2.
        while peer.count_publishers("/test/terminal_sport_request") == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert peer.count_publishers("/test/terminal_sport_request") > 0

        command = Twist()
        command.linear.y = -0.04
        error = Vector3Stamped()
        error.header.frame_id = "base_link"
        error.header.stamp = bridge.get_clock().now().to_msg()
        error.vector.y, error.vector.z = -0.35, 0.4
        bridge._on_terminal_error(error)
        bridge._on_command(command)
        deadline = time.monotonic()+1.
        while (not received or not audits) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert received
        assert json.loads(received[-1].parameter)["y"] == pytest.approx(-0.30)
        assert audits[-1]['terminal_lateral']['input_vy'] == pytest.approx(-0.04)
        assert audits[-1]['terminal_lateral']['output_vy'] == pytest.approx(-0.30)
        assert audits[-1]['terminal_lateral']['reason'] == 'boosted'

        received.clear()
        audits.clear()
        error.header.stamp.sec -= 1
        bridge._on_terminal_error(error)
        bridge._on_command(command)
        deadline = time.monotonic()+1.
        while (not received or not audits) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert received
        assert json.loads(received[-1].parameter)["y"] == pytest.approx(-0.04)
        assert audits[-1]['terminal_lateral']['reason'] == 'error_stale'

        # Literal cases verify explanations against the actual published Sport
        # request, including gates which deliberately must NOT boost motion.
        cases = [
            (-.04, None, .4, 0, -.04, 'error_missing'),
            (-.04, -.35, .4, 1, -.04, 'error_future'),
            (-.04, -.35, 1., 0, -.04, 'outside_terminal'),
            (-.04, -.03, .4, 0, 0., 'lateral_deadzone'),
            (-.04, -.07, .4, 0, -.04, 'below_boost_error'),
            (-.005, -.35, .4, 0, 0., 'command_deadband'),
            (-.04, .35, .4, 0, -.04, 'direction_mismatch'),
            (.32, .35, .4, 0, .30, 'already_effective'),
            (.04, .35, .4, 0, .30, 'boosted'),
        ]
        for vy, lateral, distance, future_s, output, reason in cases:
            received.clear()
            audits.clear()
            command.linear.y = vy
            if lateral is None:
                bridge._terminal_error = None
            else:
                error.header.stamp = bridge.get_clock().now().to_msg()
                error.header.stamp.sec += future_s
                error.vector.y, error.vector.z = lateral, distance
                bridge._on_terminal_error(error)
            bridge._on_command(command)
            deadline = time.monotonic()+1.
            while (not received or not audits) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            request = received[-1]
            actual_y = json.loads(request.parameter)['y'] if request.parameter else 0.
            assert actual_y == pytest.approx(output), reason
            diagnostic = audits[-1]['terminal_lateral']
            assert diagnostic['output_vy'] == pytest.approx(actual_y)
            assert diagnostic['reason'] == reason
    finally:
        executor.shutdown()
        peer.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
