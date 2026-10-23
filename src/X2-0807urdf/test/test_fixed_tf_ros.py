"""Verify the installed ROS publisher, isolated from the robot DDS domain."""
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import yaml


@pytest.mark.skipif(os.environ.get("A2W_ISOLATED_ROS_TEST") != "1",
                    reason="requires explicit isolated ROS domain")
def test_robot_state_publisher_exposes_body_to_robot_base_transform(tmp_path):
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from ament_index_python.packages import get_package_prefix

    assert os.environ.get("ROS_DOMAIN_ID") == "87"
    model = Path(__file__).resolve().parents[1] / "urdf" / "a2w_navigation.urdf"
    executable = Path(get_package_prefix("robot_state_publisher")) / "lib/robot_state_publisher/robot_state_publisher"
    rclpy.init()
    node = rclpy.create_node("fixed_tf_contract_test")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    params = tmp_path / "robot_description.yaml"
    params.write_text(yaml.safe_dump({"robot_state_publisher": {
        "ros__parameters": {"robot_description": model.read_text()}}}))
    log_path = tmp_path / "publisher.log"
    log_stream = log_path.open("w")
    process = subprocess.Popen([str(executable), "--ros-args", "--params-file", str(params)],
                               stdout=log_stream, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline and not buffer.can_transform("body", "base_link", Time()):
            assert process.poll() is None, log_path.read_text()[:3000]
            rclpy.spin_once(node, timeout_sec=.05)
        assert buffer.can_transform("body", "base_link", Time()), (
            node.get_node_names(), log_path.read_text()[:4000])
        transform = buffer.lookup_transform("body", "base_link", Time()).transform
        p, q = transform.translation, transform.rotation
        assert (p.x, p.y, p.z) == pytest.approx((.33767, 0., -.08134))
        assert (q.x, q.y, abs(q.z), abs(q.w)) == pytest.approx((0., 0., 1., 0.), abs=1e-6)
        # Body-frame recorded forward/left displacement expressed in base.
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        for body_xy, expected_base_xy in [((-.12825, 0.), (.12825, 0.)),
                                          ((0., -.09326), (0., .09326))]:
            x, y = body_xy
            assert (math.cos(yaw)*x + math.sin(yaw)*y,
                    -math.sin(yaw)*x + math.cos(yaw)*y) == pytest.approx(expected_base_xy, abs=1e-6)
        assert not buffer.can_transform("map", "body", Time())  # No fake upstream TF.
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        log_stream.close()
        node.destroy_node()
        rclpy.shutdown()
