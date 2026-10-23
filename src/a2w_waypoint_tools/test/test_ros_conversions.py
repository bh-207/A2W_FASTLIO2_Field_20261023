import math

from geometry_msgs.msg import TransformStamped

from a2w_waypoint_tools.ros_conversions import route_to_path, waypoint_from_transform


def test_transform_capture_and_path_keep_the_same_map_pose():
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.header.stamp.sec = 77
    transform.header.stamp.nanosec = 88
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = 1.5
    transform.transform.translation.y = -0.75
    transform.transform.translation.z = 0.2
    transform.transform.rotation.z = math.sin(math.pi / 8.0)
    transform.transform.rotation.w = math.cos(math.pi / 8.0)

    waypoint = waypoint_from_transform(transform, index=3)
    path = route_to_path("map", [waypoint], transform.header.stamp)

    assert waypoint.index == 3
    assert math.isclose(waypoint.yaw, math.pi / 4.0, abs_tol=1.0e-12)
    assert path.header.frame_id == "map"
    assert len(path.poses) == 1
    assert path.poses[0].header.frame_id == "map"
    assert path.poses[0].pose.position.x == 1.5
    assert path.poses[0].pose.position.y == -0.75
    assert path.poses[0].pose.orientation.z == transform.transform.rotation.z
    assert path.poses[0].pose.orientation.w == transform.transform.rotation.w
