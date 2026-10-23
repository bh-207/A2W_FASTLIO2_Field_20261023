"""Localization readiness policy for waypoint capture."""


class CaptureReadiness:
    LOCALIZED = 1

    def __init__(self, status_timeout_s: float):
        if status_timeout_s <= 0.0:
            raise ValueError("status_timeout_s must be positive")
        self._status_timeout_s = status_timeout_s
        self._state = None
        self._last_status_s = None

    def update(self, state: int, now_s: float) -> None:
        self._state = int(state)
        self._last_status_s = float(now_s)

    def reason(self, now_s: float) -> str:
        if self._last_status_s is None:
            return "尚未收到定位状态"
        if float(now_s) - self._last_status_s > self._status_timeout_s:
            return "定位状态已超时"
        if self._state != self.LOCALIZED:
            return "定位状态不是 LOCALIZED"
        return ""
