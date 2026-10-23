"""Conversions between TF/ROS messages and the persisted route model."""

from typing import Iterable

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from .route_storage import Waypoint, quaternion_yaw


def waypoint_from_transform(transform: TransformStamped, index: int) -> Waypoint:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return Waypoint(
        index=index,
        stamp_sec=transform.header.stamp.sec,
        stamp_nanosec=transform.header.stamp.nanosec,
        x=translation.x,
        y=translation.y,
        z=translation.z,
        qx=rotation.x,
        qy=rotation.y,
        qz=rotation.z,
        qw=rotation.w,
        yaw=quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def route_to_path(frame_id: str, waypoints: Iterable[Waypoint], stamp: Time) -> Path:
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for waypoint in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = waypoint.z
        pose.pose.orientation.x = waypoint.qx
        pose.pose.orientation.y = waypoint.qy
        pose.pose.orientation.z = waypoint.qz
        pose.pose.orientation.w = waypoint.qw
        path.poses.append(pose)
    return path
