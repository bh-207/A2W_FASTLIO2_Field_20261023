"""ROS-independent localization admission and velocity gate."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class GuardPolicyConfig:
    status_timeout_s: float = 1.0
    max_forward: float = 0.15
    max_reverse: float = 0.05
    max_lateral: float = 0.15
    max_yaw: float = 0.50
    auto_arm_on_first_localized: bool = False


@dataclass(frozen=True)
class GuardEvent:
    publish_zero: bool = False
    cancel_goal: bool = False


class NavigationReadinessLatch:
    """Report only transitions into and out of the operator-ready state."""

    def __init__(self):
        self.ready = False

    def update(self, *, gate_armed: bool, pose_available: bool,
               hardware_output_enabled: bool) -> Optional[bool]:
        ready = bool(gate_armed and pose_available and hardware_output_enabled)
        if ready == self.ready:
            return None
        self.ready = ready
        return ready


class GuardPolicy:
    INITIALIZING = 0
    LOCALIZED = 1
    DEGRADED = 2
    LOST = 3
    RELOCALIZING = 4

    def __init__(self, config: GuardPolicyConfig):
        if min(config.status_timeout_s, config.max_forward, config.max_reverse,
               config.max_lateral, config.max_yaw) < 0.0:
            raise ValueError("guard limits and timeout must be non-negative")
        self.config = config
        self.armed = False
        self.motion_authorized = False
        self.reason = "localization_status_missing"
        self._state: Optional[int] = None
        self._last_liveness_s: Optional[float] = None
        self._stale_cancel_reported = False
        self._auto_arm_pending = config.auto_arm_on_first_localized
        self._navigation_ready = False
        self._pause_ack_required = False

    def _fresh_localized(self, now_s: float) -> bool:
        return self._navigation_ready and self._fresh_localization(now_s)

    def _fresh_localization(self, now_s: float) -> bool:
        return (
            self._state == self.LOCALIZED
            and self._last_liveness_s is not None
            and math.isfinite(now_s)
            and 0.0 <= now_s - self._last_liveness_s <= self.config.status_timeout_s
        )

    def update_status(self, state: int, now_s: float) -> GuardEvent:
        if not math.isfinite(now_s):
            self.armed = False
            self.motion_authorized = False
            self.reason = "invalid_status_time"
            return GuardEvent(True, True)
        previous_state = self._state
        self._state = state
        self._last_liveness_s = now_s
        self._stale_cancel_reported = False
        if state == self.LOCALIZED:
            if self._auto_arm_pending:
                self.reason = (
                    "localized_waiting_for_pose" if self._navigation_ready
                    else "localized_waiting_for_nav2")
            else:
                if not self.motion_authorized:
                    self.reason = "localized_waiting_for_arm"
                elif not self._navigation_ready:
                    self.reason = "localized_waiting_for_nav2"
                else:
                    self.reason = (
                        "localized_waiting_for_arm" if not self.armed else "motion_enabled")
            return GuardEvent()
        was_armed = self.armed
        if previous_state == self.LOCALIZED and self._navigation_ready:
            self._pause_ack_required = True
        self.armed = False
        self.motion_authorized = False
        labels = {
            self.INITIALIZING: "localization_initializing",
            self.DEGRADED: "localization_degraded",
            self.LOST: "localization_lost",
            self.RELOCALIZING: "localization_relocalizing",
        }
        self.reason = labels.get(state, "localization_state_invalid")
        return GuardEvent(
            publish_zero=True,
            cancel_goal=was_armed or previous_state == self.LOCALIZED,
        )

    def update_liveness(self, now_s: float) -> GuardEvent:
        if not math.isfinite(now_s):
            self.armed = False
            self.motion_authorized = False
            self.reason = "invalid_liveness_time"
            return GuardEvent(True, True)
        self._last_liveness_s = now_s
        self._stale_cancel_reported = False
        if self._state == self.LOCALIZED and self._auto_arm_pending:
            # Authorize lifecycle startup before arming the velocity output.
            self.motion_authorized = True
            self._auto_arm_pending = False
        if self._state == self.LOCALIZED and self.motion_authorized:
            if not self._navigation_ready:
                self.reason = "localized_waiting_for_nav2"
                return GuardEvent(publish_zero=True)
            self.armed = True
            self.reason = "motion_enabled"
            return GuardEvent()
        if self._state == self.LOCALIZED and not self.armed:
            self.reason = "localized_waiting_for_arm"
        return GuardEvent()

    def update_navigation_ready(self, ready: bool, now_s: float) -> GuardEvent:
        if not math.isfinite(now_s):
            self._navigation_ready = False
            self.armed = False
            self.motion_authorized = False
            self.reason = "invalid_nav2_ready_time"
            return GuardEvent(True, True)
        self._navigation_ready = bool(ready)
        if not self._navigation_ready:
            self._pause_ack_required = False
            was_armed = self.armed
            self.armed = False
            self.reason = "nav2_not_ready"
            return GuardEvent(publish_zero=True, cancel_goal=was_armed)
        if self.motion_authorized and self._fresh_localized(now_s):
            self.armed = True
            self._auto_arm_pending = False
            self.reason = "motion_enabled"
            return GuardEvent()
        if self._state == self.LOCALIZED and not self.armed:
            self.reason = "localized_waiting_for_arm"
        return GuardEvent()

    def arm(self, now_s: float) -> bool:
        if self._pause_ack_required:
            self.reason = "waiting_for_nav2_pause"
            return False
        if not self._fresh_localization(now_s):
            self.armed = False
            self.motion_authorized = False
            self.reason = "localization_not_ready"
            return False
        self.motion_authorized = True
        self.armed = self._navigation_ready
        self._auto_arm_pending = False
        self.reason = "motion_enabled" if self.armed else "localized_waiting_for_nav2"
        return True

    def disarm(self) -> GuardEvent:
        self._pause_ack_required = self._navigation_ready
        self.armed = False
        self.motion_authorized = False
        self._auto_arm_pending = False
        self.reason = "operator_disabled"
        return GuardEvent(publish_zero=True, cancel_goal=True)

    def tick(self, now_s: float) -> GuardEvent:
        if self._last_liveness_s is None:
            self.armed = False
            self.motion_authorized = False
            self.reason = "localization_status_missing"
            return GuardEvent(publish_zero=True)
        if (not math.isfinite(now_s) or
                not 0.0 <= now_s - self._last_liveness_s <= self.config.status_timeout_s):
            was_armed = self.armed
            if was_armed:
                self._pause_ack_required = self._navigation_ready
            self.armed = False
            self.motion_authorized = False
            self.reason = "localization_liveness_stale"
            cancel = was_armed and not self._stale_cancel_reported
            self._stale_cancel_reported = self._stale_cancel_reported or cancel
            return GuardEvent(publish_zero=True, cancel_goal=cancel)
        if not self.armed:
            return GuardEvent(publish_zero=True)
        return GuardEvent()

    def filter_command(self, vx: float, vy: float, wz: float,
                       now_s: float) -> Tuple[float, float, float]:
        if self.tick(now_s).publish_zero or not self.armed:
            return (0.0, 0.0, 0.0)
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            self.armed = False
            self.motion_authorized = False
            self.reason = "non_finite_command"
            return (0.0, 0.0, 0.0)
        return (
            max(-self.config.max_reverse, min(self.config.max_forward, vx)),
            max(-self.config.max_lateral, min(self.config.max_lateral, vy)),
            max(-self.config.max_yaw, min(self.config.max_yaw, wz)),
        )
