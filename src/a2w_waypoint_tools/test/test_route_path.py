def test_map_frame_remaining_distance_decreases_despite_wrong_nav_feedback():
    route = module()
    path = [(100., 0., 0.), (110., 0., 0.)]
    assert route.remaining_path_distance((102., 0., 0.), path)[0] == pytest.approx(8.)
    assert route.remaining_path_distance((105., 0., 0.), path)[0] == pytest.approx(5.)


def test_terminal_goal_error_is_expressed_in_robot_axes():
    route = module()
    x, y, distance = route.terminal_error_body((0., 0., math.pi/2), (0., 1., 0.))
    assert (x, y, distance) == pytest.approx((1., 0., 1.))
    x, y, distance = route.terminal_error_body((0., .35, 0.), (1., 0., 0.))
    assert (x, y, distance) == pytest.approx((1., -.35, math.hypot(1., .35)))


import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml


def module():
    path = Path(__file__).parents[1] / 'a2w_waypoint_tools/route_path.py'
    assert path.exists(), 'route path implementation missing'
    spec = importlib.util.spec_from_file_location('route_path', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_resample_preserves_every_original_waypoint_and_final_yaw():
    result = module().build_path([(0., 0., 0.), (1., 0., 0.), (1., 1., 1.2)], .2, 5, 2)
    assert result[0][:2] == (0., 0.)
    assert result[-1] == (1., 1., 1.2)
    assert len(result) == 11
    assert all(math.isfinite(v) for point in result for v in point)
    assert result[5][:2] == (1., 0.)  # Original corner is an immutable anchor.


def test_straight_route_has_tangent_yaw():
    result = module().build_path([(0., 0., 2.), (1., 0., 1.)], .25, 1, 0)
    assert result == [(0., 0., 0.), (.25, 0., 0.), (.5, 0., 0.), (.75, 0., 0.), (1., 0., 1.)]


def test_default_densification_does_not_cut_or_skip_original_corners():
    result = module().build_path([(0., 0., 0.), (1., 0., 0.), (1., 1., 1.2)])
    assert len(result) == 11
    assert result[0][:2] == (0., 0.) and result[-1] == (1., 1., 1.2)
    assert result[5][:2] == (1., 0.)


def test_non_grid_waypoint_remains_an_anchor_and_optional_smoothing_is_bounded():
    original = [(0., 0., 0.), (1.03, 0., 0.), (1.03, 1.11, 1.2)]
    baseline = module().build_path(original, .2, 5, 0)
    softened = module().build_path(original, .2, 5, 2)
    assert len(baseline) == len(softened) == 13
    for waypoint in original:
        assert waypoint[:2] in [p[:2] for p in baseline]
        assert waypoint[:2] in [p[:2] for p in softened]
    assert max(math.hypot(a[0]-b[0], a[1]-b[1]) for a, b in zip(baseline, softened)) <= .030001


def test_route_metrics_use_segments_and_wrap_goal_yaw():
    route = module()
    assert route.distance_to_polyline((.5, .2), [(0., 0., 0.), (1., 0., 0.)]) == pytest.approx(.2)
    assert route.distance_to_polyline((1.2, .0), [(0., 0., 0.), (1., 0., 0.)]) == pytest.approx(.2)
    dx, dy, distance, yaw = route.goal_error((.9, .1, math.pi-.1),
                                              (1., 0., -math.pi+.1))
    assert (dx, dy, distance, yaw) == pytest.approx((-.1, .1, math.sqrt(.02), -.2))


@pytest.mark.parametrize('points,spacing,window,passes', [
    ([(0, 0, 0)], .2, 5, 2), ([(0, 0, 0), (0, 0, 1)], .2, 5, 2),
    ([(0, 0, 0), (float('nan'), 0, 0)], .2, 5, 2),
    ([(0, 0, 0), (1, 0, 0)], 0, 5, 2),
    ([(0, 0, 0), (1, 0, 0)], .2, 0, 2),
])
def test_invalid_route_never_creates_fallback_goal(points, spacing, window, passes):
    with pytest.raises(ValueError):
        module().build_path(points, spacing, window, passes)


def test_recorder_yaml_load_and_frame_check(tmp_path):
    p = tmp_path / 'route.yaml'
    document = {'schema': 'a2w_waypoint_route_v1', 'frame_id': 'map', 'robot_frame': 'base_link',
                'waypoints': [{'position': {'x': 1., 'y': 2.}, 'yaw': .3},
                              {'position': {'x': 2., 'y': 2.}, 'yaw': .4}]}
    p.write_text(yaml.safe_dump(document))
    assert module().load_route(p, 'map', 'base_link') == [(1., 2., .3), (2., 2., .4)]
    with pytest.raises(ValueError):
        module().load_route(p, 'camera_init', 'base_link')
    with pytest.raises(FileNotFoundError):
        module().load_route(tmp_path / 'absent.yaml', 'map', 'base_link')


def test_old_three_column_file_is_read_but_not_go_back_missions(tmp_path):
    p = tmp_path / 'route.txt'
    p.write_text('0, 0, 0\n1, 0, 1.2\n')
    assert module().load_route(p, 'map', 'base_link')[-1] == (1., 0., 1.2)
    p.write_text('GO\n0,0,0\nBACK\n1,0,0\n')
    with pytest.raises(ValueError):
        module().load_route(p, 'map', 'base_link')


def test_centerline_unknown_outside_and_obstacle_are_rejected():
    origin = NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    grid = NS(header=NS(frame_id='map'), metadata=NS(size_x=4, size_y=4, resolution=.5, origin=origin),
              data=[0]*16)
    points = [(.25, .25, 0.), (1.25, .25, 0.)]
    check = module().check_centerline
    check(points, grid, 'map')
    for cost in (253, 254, 255):
        grid.data[1] = cost
        with pytest.raises(ValueError):
            check(points, grid, 'map')  # obstacle BETWEEN the input points
    grid.data[1] = 0
    with pytest.raises(ValueError):
        check([(-.01, .25, 0.), (.25, .25, 0.)], grid, 'map')


def test_safety_freshness_requires_all_live_gates():
    ready = module().ready_to_follow
    assert ready(localized=True, nav_ready=True, safety=(True, 10.), bridge=(True, 10.), now=10.1)
    assert not ready(localized=True, nav_ready=True, safety=(True, 9.), bridge=(True, 10.), now=10.1)
    assert not ready(localized=False, nav_ready=True, safety=(True, 10.), bridge=(True, 10.), now=10.1)
    assert not ready(localized=True, nav_ready=True, safety=(True, 10.), bridge=(False, 10.), now=10.1)
