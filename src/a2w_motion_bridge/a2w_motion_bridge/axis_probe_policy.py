"""Bounded, opt-in pulses and frame math; no ROS or robot side effects."""

import math
import json


class RequestAuditTracker:
    """Correlate bridge audits, not DDS identity, on Humble's message-only API.

    Cross-topic delivery may be reordered. Unmatched Move/StopMove gets a
    bounded 150ms grace period, then permanently fails this diagnostic run.
    This is an accidental competing-controller check, not authentication.
    """

    def __init__(self):
        self.audits = {}
        self.pending = {}
        self.failed = False

    @staticmethod
    def _key(request_id, api_id, parameters):
        return request_id, api_id, json.dumps(parameters, sort_keys=True)

    def request(self, request_id, api_id, parameters, *, now):
        key = self._key(request_id, api_id, parameters)
        if now - self.audits.get(key, -math.inf) > 2.:
            self.pending.setdefault(key, now)

    def audit(self, request_id, api_id, parameters, *, now):
        self.expired(now=now)  # A late audit must not undo an expired deadline.
        key = self._key(request_id, api_id, parameters)
        self.audits = {k: t for k, t in self.audits.items() if now - t <= 2.}
        self.audits[key] = now
        self.pending.pop(key, None)

    def expired(self, *, now):
        self.failed |= any(now - t > .15 for t in self.pending.values())
        return self.failed


class AxisPulse:
    def __init__(self, axis, duration_s=1.0, enabled=False, *, lateral_speed=.1):
        if not math.isfinite(lateral_speed) or not 0 < lateral_speed <= .3:
            raise ValueError("vy speed must be in (0, 0.30] m/s")
        self.commands = {"vx": (0.1, 0.0, 0.0), "vy": (0.0, lateral_speed, 0.0),
                         "wz": (0.0, 0.0, 0.2)}
        if axis not in self.commands or not math.isfinite(duration_s) or not 0 < duration_s <= 2:
            raise ValueError("axis must be vx/vy/wz; duration must be in (0, 2] seconds")
        self.axis, self.duration_s, self.enabled = axis, duration_s, enabled
        self.stopped = False

    def check_lateral_limit(self, limit):
        if self.axis == "vy" and (not math.isfinite(limit) or limit < self.commands["vy"][1]):
            raise ValueError("运行中的横移限幅不足；请重启更新后的诊断模式，禁止把被限幅的测试当成指定速度")

    def command(self, elapsed_s, *, healthy):
        if not healthy or not math.isfinite(elapsed_s) or elapsed_s < 0 or elapsed_s >= self.duration_s:
            self.stopped = True
        if not self.enabled or self.stopped:
            return (0.0, 0.0, 0.0)
        return self.commands[self.axis]


def _quaternion(q):
    if len(q) != 4 or not all(math.isfinite(x) for x in q):
        raise ValueError("invalid quaternion")
    norm = math.sqrt(sum(x * x for x in q))
    if norm < 1e-9:
        raise ValueError("zero quaternion")
    return tuple(x / norm for x in q)


def _multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X + x*W + y*Z - z*Y, w*Y - x*Z + y*W + z*X,
            w*Z + x*Y - y*X + z*W, w*W - x*X - y*Y - z*Z)


def _rotate(q, p):
    if len(p) != 3 or not all(math.isfinite(x) for x in p):
        raise ValueError("invalid translation")
    return _multiply(_multiply(q, (*p, 0.0)), (-q[0], -q[1], -q[2], q[3]))[:3]


def compose_pose(p_a, q_a, p_b, q_b):
    if len(p_a) != 3 or not all(math.isfinite(x) for x in p_a):
        raise ValueError("invalid translation")
    q_a, q_b = _quaternion(q_a), _quaternion(q_b)
    offset = _rotate(q_a, p_b)
    return tuple(a + b for a, b in zip(p_a, offset)), _multiply(q_a, q_b)


def relative_motion(p_start, q_start, p_end, q_end):
    q_start, q_end = _quaternion(q_start), _quaternion(q_end)
    inverse = (-q_start[0], -q_start[1], -q_start[2], q_start[3])
    delta = _rotate(inverse, tuple(b-a for a, b in zip(p_start, p_end)))
    x, y, z, w = _multiply(inverse, q_end)
    return {"translation": delta, "quaternion_xyzw": (x, y, z, w),
            "yaw_rad": math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))}
