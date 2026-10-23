import math

import pytest

from a2w_motion_bridge import axis_probe_policy as policy


def test_pulse_is_disabled_by_default():
    pulse = policy.AxisPulse("vx", duration_s=1.0)
    assert pulse.command(0.2, healthy=True) == (0.0, 0.0, 0.0)


def test_explicit_lateral_speed_does_not_change_other_axes_or_default_opt_in():
    pulse = policy.AxisPulse("vy", enabled=True, lateral_speed=.3)
    assert pulse.command(.5, healthy=True) == (0., .3, 0.)
    assert pulse.command(1., healthy=True) == (0., 0., 0.)
    assert policy.AxisPulse("vy", lateral_speed=.3).command(.5, healthy=True) == (0., 0., 0.)
    assert policy.AxisPulse("vx", enabled=True, lateral_speed=.3).command(.5, healthy=True) == (.1, 0., 0.)


@pytest.mark.parametrize("speed", [0, -.1, .301, math.nan, math.inf])
def test_lateral_diagnostic_speed_cannot_exceed_approved_bound(speed):
    with pytest.raises(ValueError):
        policy.AxisPulse("vy", lateral_speed=speed)


def test_probe_rejects_requested_speed_above_running_gate_limits():
    pulse = policy.AxisPulse("vy", enabled=True, lateral_speed=.3)
    with pytest.raises(ValueError):
        pulse.check_lateral_limit(.15)
    pulse.check_lateral_limit(.3)
    with pytest.raises(ValueError):
        pulse.check_lateral_limit(math.nan)


@pytest.mark.parametrize("axis,want", [
    ("vx", (0.10, 0.0, 0.0)),
    ("vy", (0.0, 0.10, 0.0)),
    ("wz", (0.0, 0.0, 0.20)),
])
def test_pulse_commands_exactly_one_axis_then_stops(axis, want):
    pulse = policy.AxisPulse(axis, enabled=True)
    assert pulse.command(0.0, healthy=True) == want
    assert pulse.command(0.99, healthy=True) == want
    assert pulse.command(1.0, healthy=True) == (0.0, 0.0, 0.0)


def test_health_loss_latches_stop_and_never_resumes_old_pulse():
    pulse = policy.AxisPulse("vy", enabled=True)
    assert pulse.command(0.1, healthy=True)[1] == 0.1
    assert pulse.command(0.2, healthy=False) == (0.0, 0.0, 0.0)
    assert pulse.command(0.3, healthy=True) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("duration", [-1, 0, 2.01, math.nan, math.inf])
def test_rejects_unbounded_or_invalid_motion_duration(duration):
    with pytest.raises(ValueError):
        policy.AxisPulse("vx", duration_s=duration, enabled=True)


def test_relative_motion_uses_start_axes_not_map_axes():
    # Start +90 degrees: world +Y is start-body +X.
    q = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
    result = policy.relative_motion((1, 2, 0), q, (1, 2.1, 0), q)
    assert result["translation"] == pytest.approx([0.1, 0, 0])
    assert result["yaw_rad"] == pytest.approx(0)


def test_relative_yaw_wraps_at_pi():
    def q(yaw):
        return (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    result = policy.relative_motion((0, 0, 0), q(math.radians(179)),
                                    (0, 0, 0), q(math.radians(-179)))
    assert result["yaw_rad"] == pytest.approx(math.radians(2))


def test_relative_transform_composes_sensor_lever_arm():
    # A sensor .3m ahead of base rotates around a stationary base.
    p, q = policy.compose_pose((0, .3, 0), (0, 0, math.sqrt(.5), math.sqrt(.5)),
                               (-.3, 0, 0), (0, 0, 0, 1))
    assert p == pytest.approx((0, 0, 0))


def test_invalid_quaternion_is_not_silently_identity():
    with pytest.raises(ValueError):
        policy.relative_motion((0, 0, 0), (0, 0, 0, 0),
                               (0, 0, 0), (0, 0, 0, 1))


@pytest.mark.parametrize("audit_first", [False, True])
def test_request_audit_correlation_handles_cross_topic_delivery_order(audit_first):
    tracker = policy.RequestAuditTracker()
    key = (123, 1008, {"x": .1, "y": 0., "z": 0.})
    if audit_first:
        tracker.audit(*key, now=1.)
        tracker.request(*key, now=1.02)
    else:
        tracker.request(*key, now=1.)
        assert not tracker.expired(now=1.1)
        tracker.audit(*key, now=1.1)
    assert not tracker.expired(now=1.3)


@pytest.mark.parametrize("wrong_key", [
    (124, 1008, {"x": .1}), (123, 1003, {"x": .1}), (123, 1008, {"x": -.1})])
def test_unmatched_request_stops_even_if_only_one_field_differs(wrong_key):
    tracker = policy.RequestAuditTracker()
    tracker.audit(123, 1008, {"x": .1}, now=1.)
    tracker.request(*wrong_key, now=1.01)
    assert tracker.expired(now=1.2)


def test_old_audit_cannot_authorize_a_new_request_with_reused_id():
    tracker = policy.RequestAuditTracker()
    tracker.audit(123, 1008, {}, now=1.)
    tracker.request(123, 1008, {}, now=5.)
    assert tracker.expired(now=5.2)
