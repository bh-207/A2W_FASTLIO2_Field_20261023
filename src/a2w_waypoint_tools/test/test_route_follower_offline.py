"""Behavior tests on real follower methods without ROS initialization."""
import copy
import math
from types import SimpleNamespace as NS

import pytest

from a2w_waypoint_tools import route_path
from route_follower_offline_rig import Rig


def test_fresh_healthy_pose_jump_cancels_revokes_and_clears_progress():
    rig = Rig()
    rig.tick(1., (1., 0., 0.))
    assert rig.node.progress_s == pytest.approx(1.)

    rig.tick(1.1, (4., 2., .8))

    assert rig.node.phase == 'CANCELLING'
    assert rig.cancels and rig.disables == [False]
    assert rig.node.progress_s == 0.
    assert rig.node.remaining_m is None
    rig.tick(1.2, (.5, 0., 0.))
    assert rig.node.phase == 'CANCELLING'


def test_normal_correction_plus_physical_motion_and_mount_offset_remains_running():
    rig = Rig()
    rig.tick(1., (0., 0., math.pi-.01))
    # Body correction: 0.5m/10deg; static body/base offset adds <=0.0589m.
    # Normal physical translation/yaw over 100ms is included independently.
    lever_arm = 2.*.33767*math.sin(math.radians(10.)/2.)
    rig.tick(1.1, (.5+lever_arm+.06, 0., math.pi-.01+math.radians(10.)+.05))
    assert rig.node.phase == 'RUNNING'
    assert not rig.cancels and not rig.disables


def test_gradual_lateral_detour_is_allowed_without_changing_reference():
    rig = Rig()
    reference = copy.deepcopy(rig.node.path)
    for i in range(30):
        rig.tick(1.+.1*i, (.04*i, .8*math.sin(math.pi*i/29.), .2*math.sin(math.pi*i/29.)))
    assert rig.node.phase == 'RUNNING' and not rig.cancels
    assert rig.node.path.poses[-1].pose.position.x == reference.poses[-1].pose.position.x
    assert rig.node.raw_points[-1] == (5.4, 0., 0.)


def test_tf_source_time_rollback_stops_a_running_route():
    rig = Rig()
    rig.tick(1., (0., 0., 0.))
    rig.tf_age = .2
    rig.tick(1.1, (.01, 0., 0.))  # latest lookup is fresh but source went 1.0->0.9
    assert rig.node.phase == 'CANCELLING'
    assert rig.cancels and rig.disables == [False]


def test_explicit_rejoin_uses_selected_segment_and_checks_path_before_send():
    rig = Rig()
    rig.node.raw_points = [(0., 0., 0.), (2., 0., 0.), (2., 2., 1.)]
    rig.node.points = route_path.build_path(rig.node.raw_points)
    rig.node.path = rig.node._path(rig.node.points)
    original = copy.deepcopy(rig.node.raw_points)
    rig.node.phase, rig.node.started, rig.node.send_future = 'WAITING', -2., None
    rig.node.busy['follow'] = False
    rig.node.rejoin_segment_index = 1

    rig.tick(1., (2.2, .5, 0.))

    assert rig.node.phase == 'CHECK_MAP'
    assert len(rig.grid_requests) == 1
    assert rig.node.points[0][:2] == (2.2, .5)
    assert (2., .5) in [p[:2] for p in rig.node.points]
    assert rig.node.points[-1] == (2., 2., 1.)
    assert rig.node.raw_points == original
    assert rig.node.path.header.frame_id == 'map'
    assert rig.node.last_nav_pose is not None


def test_rejoin_does_not_bypass_closed_authorization():
    rig = Rig()
    rig.node.phase, rig.node.started, rig.node.send_future = 'WAITING', -2., None
    rig.node.rejoin_segment_index = 0
    rig.node.localized = False
    rig.tick(1., (1., .1, 0.))
    assert rig.node.phase == 'WAITING'
    assert not rig.grid_requests


def test_selected_rejoin_segment_never_guesses_a_distant_or_other_branch():
    points = [(0., 0., 0.), (2., 0., 0.), (2., 2., 1.)]
    with pytest.raises(ValueError):
        route_path.rejoin_route(points, (2.1, 1., 0.), 0, .5)
    with pytest.raises(ValueError):
        route_path.rejoin_route(points, (2., 1., 0.), 2, .5)


def test_jump_during_footprint_validation_never_sends_goal():
    rig = Rig()
    rig.node.phase, rig.node.send_future = 'CHECK_PATH', None
    rig.node.last_nav_pose = ((0., 0., 0.), 1.)
    rig.node.busy['follow'] = False
    rig.now, rig.pose = 1.1, (3., 2., .8)
    rig.node.safety = rig.node.bridge = (True, rig.now)
    rig.node.controller, rig.node.goal_checker = 'FollowPath', 'goal_checker'
    rig.node._path_checked.__globals__['FollowPath'] = NS(Goal=lambda **kw: NS(**kw))
    goals = []
    rig.node.action = NS(send_goal_async=lambda goal, **kw:
                        (goals.append(goal) or NS(add_done_callback=lambda cb: None)))

    rig.node._path_checked(NS(result=lambda: NS(is_valid=True)))

    assert not goals
    assert rig.node.phase == 'CANCELLING'


def test_async_validation_keeps_sampling_fresh_tf_until_goal_submission():
    rig = Rig()
    rig.node.phase, rig.node.send_future = 'CHECK_PATH', None
    rig.node.last_nav_pose = ((0., 0., 0.), 1.)
    rig.node.busy['follow'] = False
    rig.node.controller, rig.node.goal_checker = 'FollowPath', 'goal_checker'
    rig.node._path_checked.__globals__['FollowPath'] = NS(Goal=lambda **kw: NS(**kw))
    goals = []
    rig.node.action = NS(send_goal_async=lambda goal, **kw:
                        (goals.append(goal) or NS(add_done_callback=lambda cb: None)))
    for i in range(1, 11):
        rig.tick(1.+i*.2, (0., 0., 0.))
    rig.node._path_checked(NS(result=lambda: NS(is_valid=True)))
    assert len(goals) == 1
    assert rig.node.phase == 'SENDING'


@pytest.mark.parametrize('pose,index,expected_xy', [
    ((1., 0., 0.), 0, [(1., 0.), (2., 0.), (2., 2.)]),
    ((2., 0., 0.), 0, [(2., 0.), (2., 2.)]),
])
def test_rejoin_on_reference_does_not_add_zero_length_connector(pose, index, expected_xy):
    points = [(0., 0., 0.), (2., 0., 0.), (2., 2., 1.)]
    result = route_path.rejoin_route(points, pose, index, .5)
    assert [p[:2] for p in result] == expected_xy
    assert result[-1] == points[-1]
