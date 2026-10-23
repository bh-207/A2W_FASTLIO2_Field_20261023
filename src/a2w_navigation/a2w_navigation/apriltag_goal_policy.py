"""ROS-independent conversion from the validated AprilTag Teach pose to a Nav2 goal."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class NavGoalDecision:
    accepted: bool
    reason: str
    x: float
    y: float
    yaw: float
    relative_x: float
    relative_y: float
    relative_yaw: float
    distance: float


class StableDetectionGate:
    """Require consecutive accepted IPPE poses before a goal may be requested."""

    def __init__(self, *, required_frames, timeout_s):
        if required_frames <= 0 or not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("invalid stable detection gate configuration")
        self.required_frames = int(required_frames)
        self.timeout_s = float(timeout_s)
        self._streak = 0
        self._pose = None
        self._stamp_s = None

    def accept(self, pose, *, stamp_s):
        value = _validate_transform("tag_body_current", pose)
        if not math.isfinite(stamp_s):
            raise ValueError("detection stamp must be finite")
        self._pose = value.copy()
        self._stamp_s = float(stamp_s)
        self._streak += 1

    def reject(self):
        self._streak = 0
        self._pose = None
        self._stamp_s = None

    def current(self, *, now_s):
        if self._streak < self.required_frames or self._pose is None:
            return None, "pose_not_stable"
        if not math.isfinite(now_s) or now_s - self._stamp_s > self.timeout_s:
            return None, "pose_stale"
        return self._pose.copy(), "accepted"


def bgr8_from_buffer(data, *, height, width, step, encoding):
    """Convert sensor_msgs/Image storage without depending on cv_bridge."""
    if encoding != "bgr8":
        raise ValueError(f"expected calibrated bgr8 image, got {encoding!r}")
    if min(height, width, step) <= 0 or step < width * 3:
        raise ValueError("invalid bgr8 image dimensions")
    raw = np.frombuffer(data, dtype=np.uint8)
    expected_size = height * step
    if raw.size != expected_size:
        raise ValueError(
            f"invalid bgr8 data size: expected {expected_size}, got {raw.size}")
    rows = raw.reshape(height, step)
    return rows[:, :width * 3].reshape(height, width, 3).copy()


def _validate_transform(name, transform):
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 transform")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain only finite values")
    if not np.allclose(value[3, :], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError(f"{name} has an invalid homogeneous row")
    return value


def _yaw(rotation):
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_nav_goal(
        map_body_current,
        tag_body_current,
        tag_body_target,
        *,
        min_distance_m,
        max_distance_m,
        min_yaw_rad,
        max_yaw_rad):
    """Compose map->body(current) with the script's current->Teach transform."""
    if (min(min_distance_m, max_distance_m, min_yaw_rad, max_yaw_rad) < 0.0 or
            min_distance_m > max_distance_m or min_yaw_rad > max_yaw_rad):
        raise ValueError("invalid AprilTag goal limits")

    map_body = _validate_transform("map_body_current", map_body_current)
    tag_body = _validate_transform("tag_body_current", tag_body_current)
    tag_target = _validate_transform("tag_body_target", tag_body_target)

    body_target = np.linalg.inv(tag_body) @ tag_target
    relative_x = float(body_target[0, 3])
    relative_y = float(body_target[1, 3])
    relative_yaw = _wrap_pi(_yaw(body_target[:3, :3]))
    distance = math.hypot(relative_x, relative_y)

    current_yaw = _yaw(map_body[:3, :3])
    cosine = math.cos(current_yaw)
    sine = math.sin(current_yaw)
    goal_x = float(map_body[0, 3]) + cosine * relative_x - sine * relative_y
    goal_y = float(map_body[1, 3]) + sine * relative_x + cosine * relative_y
    goal_yaw = _wrap_pi(current_yaw + relative_yaw)

    if distance < min_distance_m and abs(relative_yaw) < min_yaw_rad:
        accepted = False
        reason = "already_at_target"
    elif distance > max_distance_m:
        accepted = False
        reason = "goal_too_far"
    elif abs(relative_yaw) > max_yaw_rad:
        accepted = False
        reason = "goal_yaw_too_large"
    else:
        accepted = True
        reason = "accepted"

    return NavGoalDecision(
        accepted=accepted,
        reason=reason,
        x=goal_x,
        y=goal_y,
        yaw=goal_yaw,
        relative_x=relative_x,
        relative_y=relative_y,
        relative_yaw=relative_yaw,
        distance=distance,
    )
