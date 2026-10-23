from a2w_waypoint_tools.capture_readiness import CaptureReadiness


def test_capture_requires_a_fresh_localized_status():
    readiness = CaptureReadiness(status_timeout_s=2.0)

    assert readiness.reason(now_s=10.0) == "尚未收到定位状态"

    readiness.update(state=1, now_s=10.0)
    assert readiness.reason(now_s=11.9) == ""
    assert readiness.reason(now_s=12.1) == "定位状态已超时"


def test_degraded_localization_blocks_capture():
    readiness = CaptureReadiness(status_timeout_s=2.0)

    readiness.update(state=2, now_s=10.0)

    assert readiness.reason(now_s=10.1) == "定位状态不是 LOCALIZED"
