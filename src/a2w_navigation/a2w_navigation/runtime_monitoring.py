"""ROS-independent state used by Navigation startup and runtime diagnostics."""

from typing import Iterable, NamedTuple, Optional


INITIALIZING_STATE = 0
LOCALIZED_STATE = 1
DEGRADED_STATE = 2
LOST_STATE = 3
RELOCALIZING_STATE = 4

STARTUP_REQUEST = "startup"
PAUSE_REQUEST = "pause"
RESUME_REQUEST = "resume"


class CostmapCounts(NamedTuple):
    free: int
    inflated: int
    lethal: int
    unknown: int


def classify_costmap(cells: Iterable[int]) -> CostmapCounts:
    """Count the standard OccupancyGrid representation of Nav2 costs."""
    free = inflated = lethal = unknown = 0
    for value in cells:
        if value < 0:
            unknown += 1
        elif value == 0:
            free += 1
        elif value >= 99:
            lethal += 1
        else:
            inflated += 1
    return CostmapCounts(free, inflated, lethal, unknown)


def should_publish_zero_keepalive(
        *, armed: bool, last_command_s: Optional[float],
        now_s: float, interval_s: float) -> bool:
    """Keep the bridge watchdog alive with zero only while its gate is armed."""
    return bool(
        armed and
        (last_command_s is None or now_s - last_command_s >= interval_s))


class Nav2ActivationPolicy:
    """Serialize Nav2 startup, pause and resume around localization health."""

    def __init__(self, *, enabled: bool):
        self.enabled = bool(enabled)
        self.localized = False
        self.pose_available = False
        self.in_flight = False
        self.in_flight_request: Optional[str] = None
        self.active = False
        self.paused = False
        self._pause_required = False
        self.ready = False
        self.motion_authorized = False

    def observe_motion_authorization(self, authorized: bool) -> None:
        self.motion_authorized = bool(authorized)
        if not self.motion_authorized:
            if self.active or self.in_flight_request in (STARTUP_REQUEST, RESUME_REQUEST):
                self._pause_required = True
            self.ready = False

    def observe_localization(self, state: int) -> None:
        self.localized = state == LOCALIZED_STATE
        if not self.localized:
            # A pose seen before a degraded/lost transition is not sufficient
            # evidence for a later startup or resume attempt.
            self.pose_available = False
            if self.active or self.in_flight_request in (
                    STARTUP_REQUEST, RESUME_REQUEST):
                self._pause_required = True
            self.ready = False

    def observe_pose(self) -> None:
        if self.localized:
            self.pose_available = True

    def take_lifecycle_request(self) -> Optional[str]:
        if not self.enabled or self.in_flight:
            return None

        request = None
        if self._pause_required and self.active:
            request = PAUSE_REQUEST
        elif not self.localized or not self.pose_available or not self.motion_authorized:
            return None
        elif self.paused:
            request = RESUME_REQUEST
        elif not self.active:
            request = STARTUP_REQUEST
        else:
            return None

        self.in_flight = True
        self.in_flight_request = request
        return request

    def complete_lifecycle_request(self, success: bool) -> Optional[str]:
        request = self.in_flight_request
        self.in_flight = False
        self.in_flight_request = None

        if success and request in (STARTUP_REQUEST, RESUME_REQUEST):
            self.active = True
            self.paused = False
        elif success and request == PAUSE_REQUEST:
            self.active = False
            self.paused = True
            self._pause_required = False
        elif not success and request in (STARTUP_REQUEST, RESUME_REQUEST):
            # A failed activation cannot have left an active goal server, so a
            # pause requested while that activation was pending is unnecessary.
            self._pause_required = False

        self.ready = bool(
            self.active and self.localized and self.pose_available and
            self.motion_authorized and not self._pause_required)
        return request
