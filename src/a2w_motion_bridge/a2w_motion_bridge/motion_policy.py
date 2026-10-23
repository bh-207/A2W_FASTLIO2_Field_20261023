"""ROS-independent A2-W command validation and watchdog policy."""

from dataclasses import dataclass
import json
import math
from typing import Optional


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003
SWITCH_GAIT_API_ID = 1011
SPEED_LEVEL_API_ID = 1015


@dataclass(frozen=True)
class MotionPolicyConfig:
    max_forward: float = 0.15
    max_reverse: float = 0.05
    max_lateral: float = 0.15
    max_yaw: float = 0.50
    min_effective_forward: float = 0.0
    min_effective_lateral: float = 0.0
    min_effective_yaw: float = 0.0
    deadband_forward: float = 0.01
    deadband_lateral: float = 0.01
    deadband_yaw: float = 0.02
    terminal_lateral_stop_m: float = 0.06
    terminal_lateral_boost_m: float = 0.08
    terminal_activation_radius_m: float = 0.70
    command_timeout_s: float = 0.30
    start_enabled: bool = False


@dataclass(frozen=True)
class RequestSpec:
    api_id: int
    parameter: str = ""


def motion_profile_requests(gait_type: int, speed_level: int):
    """Build the two A2 sport requests that match the proven hand-control profile."""
    if (isinstance(gait_type, bool) or isinstance(speed_level, bool) or
            not isinstance(gait_type, int) or not isinstance(speed_level, int) or
            gait_type < 0 or speed_level < 0):
        raise ValueError("gait_type and speed_level must be non-negative integers")
    return (
        RequestSpec(
            SWITCH_GAIT_API_ID,
            json.dumps({"data": gait_type}, separators=(",", ":")),
        ),
        RequestSpec(
            SPEED_LEVEL_API_ID,
            json.dumps({"data": speed_level}, separators=(",", ":")),
        ),
    )


class MotionPolicy:
    """Fail-closed state machine for the final robot command boundary."""

    def __init__(self, config: MotionPolicyConfig):
        if min(config.max_forward, config.max_reverse, config.max_lateral,
               config.max_yaw, config.min_effective_forward,
               config.min_effective_lateral, config.min_effective_yaw,
               config.deadband_forward, config.deadband_lateral, config.deadband_yaw,
               config.terminal_lateral_stop_m, config.terminal_lateral_boost_m,
               config.terminal_activation_radius_m, config.command_timeout_s) < 0.0:
            raise ValueError("motion limits and timeout must be non-negative")
        if not all(math.isfinite(value) for value in config.__dict__.values()):
            raise ValueError("motion policy values must be finite")
        if (config.min_effective_forward > min(config.max_forward, config.max_reverse) or
                config.min_effective_lateral > config.max_lateral or
                config.min_effective_yaw > config.max_yaw):
            raise ValueError("effective minimum must not exceed its motion limit")
        if config.terminal_lateral_stop_m >= config.terminal_lateral_boost_m:
            raise ValueError("terminal lateral stop must be below boost threshold")
        if config.terminal_activation_radius_m <= 0.0:
            raise ValueError("terminal activation radius must be positive")
        if (config.deadband_forward > min(config.max_forward, config.max_reverse) or
                config.deadband_lateral > config.max_lateral or
                config.deadband_yaw > config.max_yaw):
            raise ValueError("deadband must not exceed its motion limit")
        self.config = config
        self.enabled = config.start_enabled
        self.reason = "startup_enabled" if self.enabled else "startup_disabled"
        self._last_command_s: Optional[float] = None
        self._timeout_reported = False

    @staticmethod
    def stop() -> RequestSpec:
        return RequestSpec(STOP_MOVE_API_ID)

    def set_enabled(self, enabled: bool) -> Optional[RequestSpec]:
        self.enabled = bool(enabled)
        self._last_command_s = None
        self._timeout_reported = False
        self.reason = "enabled" if enabled else "operator_disabled"
        return None if enabled else self.stop()

    def command(self, vx: float, vy: float, wz: float, now_s: float,
                terminal_lateral_error_m: Optional[float] = None,
                terminal_distance_m: Optional[float] = None) -> RequestSpec:
        if not all(math.isfinite(value) for value in (vx, vy, wz, now_s)):
            self.enabled = False
            self.reason = "non_finite_command"
            return self.stop()
        if not self.enabled:
            self.reason = "hardware_output_disabled"
            return self.stop()
        x = max(-self.config.max_reverse, min(self.config.max_forward, vx))
        y = max(-self.config.max_lateral, min(self.config.max_lateral, vy))
        z = max(-self.config.max_yaw, min(self.config.max_yaw, wz))
        x = 0.0 if abs(x) < self.config.deadband_forward else x
        y = 0.0 if abs(y) < self.config.deadband_lateral else y
        z = 0.0 if abs(z) < self.config.deadband_yaw else z
        if (self.config.min_effective_lateral > 0.0 and
                terminal_lateral_error_m is not None and terminal_distance_m is not None and
                math.isfinite(terminal_lateral_error_m) and math.isfinite(terminal_distance_m) and
                0.0 <= terminal_distance_m <= self.config.terminal_activation_radius_m):
            error = terminal_lateral_error_m
            if abs(error) <= self.config.terminal_lateral_stop_m:
                y = 0.0
            elif (abs(error) >= self.config.terminal_lateral_boost_m and
                  y * error > 0.0 and abs(y) < self.config.min_effective_lateral):
                y = math.copysign(self.config.min_effective_lateral, y)
        if abs(x) < 1.0e-6 and abs(y) < 1.0e-6 and abs(z) < 1.0e-6:
            self._last_command_s = None
            self._timeout_reported = False
            self.reason = "zero_command"
            return self.stop()
        # Lateral minimum is opt-in and requires fresh, same-direction terminal error.
        self._last_command_s = now_s
        self._timeout_reported = False
        self.reason = "move"
        return RequestSpec(
            MOVE_API_ID,
            json.dumps({"x": x, "y": y, "z": z}, separators=(",", ":")),
        )

    def watchdog(self, now_s: float) -> Optional[RequestSpec]:
        if not self.enabled or self._last_command_s is None or self._timeout_reported:
            return None
        if now_s - self._last_command_s <= self.config.command_timeout_s:
            return None
        self.enabled = False
        self._timeout_reported = True
        self.reason = "command_timeout"
        return self.stop()

    def shutdown(self) -> RequestSpec:
        self.enabled = False
        self.reason = "shutdown"
        return self.stop()
