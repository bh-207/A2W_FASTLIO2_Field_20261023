def test_running_route_publishes_robot_frame_terminal_error_and_live_remaining(rig):
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    assert rig.pump(1., lambda: rig.node.remaining_m is not None)
    initial_remaining = rig.node.remaining_m
    rig.x_position, rig.lateral_offset = .5, .15
    assert rig.pump(1., lambda: rig.node.remaining_m is not None and
                    rig.node.remaining_m < initial_remaining-.2 and
                    bool(rig.terminal_errors) and
                    abs(rig.terminal_errors[-1].vector.x-.5) < .03)
    error = rig.terminal_errors[-1]
    assert error.header.frame_id == 'base_link'
    assert error.vector.x == pytest.approx(.5, abs=.03)
    assert error.vector.y == pytest.approx(-.15, abs=.03)
    assert error.vector.z == pytest.approx(math.hypot(.5, .15), abs=.03)


"""Real ROS client against bounded test servers; NEVER uses the robot domain."""
import json
import math
import os
from pathlib import Path
import sys
import time

import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get('A2W_ISOLATED_ROS_TEST') != '1',
                              reason='requires isolated ROS test environment')
sys.path.insert(0, str(Path(__file__).parents[1]))


@pytest.fixture
def rig(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') in ('87', '88'), 'not an isolated test domain'
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.task import Future
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped, Vector3Stamped
    from a2w_fastlio_msgs.msg import LocalizationStatus
    from nav2_msgs.action import FollowPath
    from nav2_msgs.srv import GetCostmap, IsPathValid
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool
    from a2w_waypoint_tools.route_follower import RouteFollower

    filename = tmp_path / 'route.yaml'
    filename.write_text(yaml.safe_dump({
        'schema': 'a2w_waypoint_route_v1', 'frame_id': 'map', 'robot_frame': 'base_link',
        'waypoints': [{'position': {'x': 0., 'y': 0.}, 'yaw': 0.},
                      {'position': {'x': .5, 'y': .1}, 'yaw': 0.},
                      {'position': {'x': 1., 'y': 0.}, 'yaw': .8}]}))
    rclpy.init()
    peer = Node('route_test_peer')
    ex = SingleThreadedExecutor()
    ex.add_node(peer)
    class Rig:
        route_file = filename
        node = None
        state = LocalizationStatus.LOCALIZED
        goals = []
        cancels = []
        disables = []
        finish = None
        valid = True
        blocked = False
        delay_accept = None
        accepting = False
        lateral_offset = 0.0
        terminal_errors = []
        x_position = 0.0
        yaw_rad = 0.0
        abort = False

        def start(self, execute):
            self.node = RouteFollower(parameter_overrides=[
                Parameter('route_file', value=str(filename)), Parameter('execute', value=execute),
                Parameter('wait_timeout_s', value=8.)])
            ex.add_node(self.node)

        def pump(self, seconds=1., until=lambda: False):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                ex.spin_once(timeout_sec=.02)
                if until():
                    return True
            return until()
    rig = Rig()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_ready = peer.create_publisher(Bool, '/navigation/nav2_ready', qos)
    pub_loc = peer.create_publisher(LocalizationStatus, '/localization/status', 10)
    pub_guard = peer.create_publisher(String, '/navigation/safety_status', 10)
    pub_bridge = peer.create_publisher(String, '/a2w_motion_bridge/status', 10)
    terminal_sub = peer.create_subscription(
        Vector3Stamped, '/route_follower/terminal_error_body',
        rig.terminal_errors.append, 10)
    tf = TransformBroadcaster(peer)

    def publish():
        pub_loc.publish(LocalizationStatus(state=rig.state))
        pub_ready.publish(Bool(data=True))
        pub_guard.publish(String(data=json.dumps({'motion_enabled': True})))
        pub_bridge.publish(String(data=json.dumps({'hardware_output_enabled': True})))
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.header.stamp = peer.get_clock().now().to_msg()
        t.transform.translation.x = rig.x_position
        t.transform.translation.y = rig.lateral_offset
        t.transform.rotation.z = math.sin(rig.yaw_rad / 2.)
        t.transform.rotation.w = math.cos(rig.yaw_rad / 2.)
        tf.sendTransform(t)

    timer = peer.create_timer(.05, publish)
    def grid(req, res):
        res.map.header.frame_id = 'map'
        res.map.header.stamp = peer.get_clock().now().to_msg()
        m = res.map.metadata
        m.resolution, m.size_x, m.size_y = .1, 100, 100
        m.origin.position.x = m.origin.position.y = -5.
        m.origin.orientation.w = 1.
        res.map.data = [254 if rig.blocked else 0]*10000
        return res

    def valid(req, res):
        assert len(req.path.poses) > 10  # footprint check also samples between route points
        res.is_valid = rig.valid
        return res

    def disable(req, res):
        rig.disables.append(req.data)
        res.success = True
        return res

    async def accept(goal):
        rig.accepting = True
        if rig.delay_accept is not None:
            await rig.delay_accept
        return GoalResponse.ACCEPT

    async def execute(handle):
        rig.goals.append(handle.request)
        rig.finish = Future()
        await rig.finish
        if handle.is_cancel_requested:
            handle.canceled()
        elif rig.abort:
            handle.abort()
        else:
            handle.succeed()
        return FollowPath.Result()

    def cancel(handle):
        rig.cancels.append(handle)
        # Wake execute after cancellation state is installed by the action server.
        return CancelResponse.ACCEPT

    def complete_cancel():
        if rig.finish and not rig.finish.done() and any(h.is_cancel_requested for h in rig.cancels):
            rig.finish.set_result(None)

    timer_cancel = peer.create_timer(.02, complete_cancel)
    services = [peer.create_service(GetCostmap, '/global_costmap/get_costmap', grid),
                peer.create_service(IsPathValid, '/is_path_valid', valid),
                peer.create_service(SetBool, '/navigation/enable_motion', disable)]
    action = ActionServer(peer, FollowPath, '/follow_path', execute_callback=execute,
                          goal_callback=accept, cancel_callback=cancel,
                          callback_group=ReentrantCallbackGroup())
    try:
        yield rig
    finally:
        if rig.node:
            rig.node.stop('test teardown')
            rig.pump(.3)
            ex.remove_node(rig.node)
            rig.node.destroy_node()
        action.destroy()
        ex.shutdown()
        peer.destroy_node()
        rclpy.shutdown()


def test_preview_never_sends_motion_goal(rig):
    rig.start(False)
    rig.pump(1.5)
    assert not rig.goals
    assert not rig.accepting


def test_feedback_distinguishes_arc_remaining_from_terminal_error(rig, capfd):
    from nav2_msgs.action import FollowPath
    from types import SimpleNamespace
    document = yaml.safe_load(rig.route_file.read_text())
    for point in document['waypoints']:
        point['position']['y'] = 0.
    rig.route_file.write_text(yaml.safe_dump(document))
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    for step in range(1, 11):
        rig.x_position, rig.lateral_offset = step*.1, step*.045
        rig.pump(.3)
    assert rig.pump(1., lambda: rig.node.remaining_m == 0.)
    rig.node.last_feedback = 0.
    rig.node._feedback(SimpleNamespace(feedback=FollowPath.Feedback(speed=.06)))
    output = capfd.readouterr()
    text = output.out + output.err
    assert '沿线剩余 0.00m' in text
    assert '终点XY=0.450m' in text
    assert '横向误差=-0.450m' in text


def test_single_continuous_path_finishes_at_b(rig):
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    goal = rig.goals[0]
    assert goal.controller_id == 'FollowPath' and goal.goal_checker_id == 'goal_checker'
    assert len(goal.path.poses) > 3
    assert goal.path.header.frame_id == 'map'
    assert goal.path.poses[-1].pose.position.x == 1.
    assert goal.path.poses[-1].pose.position.y == 0.
    rig.finish.set_result(None)
    assert rig.pump(3., lambda: rig.node.done)
    assert rig.node.exit_code == 0
    rig.pump(.3)
    assert len(rig.goals) == 1


def test_default_node_keeps_the_original_corner_in_follow_path(rig):
    document = yaml.safe_load(rig.route_file.read_text())
    document['waypoints'] = [
        {'position': {'x': x, 'y': y}, 'yaw': yaw}
        for x, y, yaw in [(0., 0., 0.), (1., 0., 0.), (1., 1., 1.2)]]
    rig.route_file.write_text(yaml.safe_dump(document))
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    path = rig.goals[0].path
    assert len(path.poses) == 11
    corner = path.poses[5].pose.position
    assert (corner.x, corner.y) == pytest.approx((1., 0.))


def test_route_result_reports_estimated_endpoint_error_and_cross_track(rig):
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    for fraction in (.2, .4, .6, .8, 1.):
        rig.x_position, rig.lateral_offset, rig.yaw_rad = fraction, .12*fraction, .5*fraction
        rig.pump(.4)
    rig.pump(.2)
    assert rig.node.deviation_samples > 0
    assert rig.node.max_raw_deviation_m >= .1
    rig.finish.set_result(None)
    assert rig.pump(3., lambda: rig.node.done)
    assert rig.node.final_goal_error is not None
    dx, dy, xy, yaw = rig.node.final_goal_error
    assert (dx, dy, xy, yaw) == pytest.approx((0., .12, .12, -.3), abs=.03)


def test_lateral_detour_does_not_cancel_or_replace_the_reference_path(rig):
    # Client-contract test, NOT a simulation of MPPI obstacle avoidance.
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    # A physical detour evolves over time; an instantaneous map TF offset is a
    # localization jump and must now cancel. Peak remains above A tolerance.
    for offset in (.2, .4, .6, .8):
        rig.lateral_offset = offset
        rig.pump(.4)
    assert rig.node.phase == 'RUNNING'
    assert not rig.cancels and not rig.disables
    for offset in (.6, .4, .2, 0.):
        rig.lateral_offset = offset
        rig.pump(.4)
    assert rig.node.phase == 'RUNNING' and len(rig.goals) == 1
    rig.finish.set_result(None)
    assert rig.pump(3., lambda: rig.node.done)
    assert rig.node.exit_code == 0


def test_controller_abort_does_not_automatically_restart_route(rig):
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    rig.abort = True
    rig.finish.set_result(None)
    assert rig.pump(3., lambda: rig.node.done)
    assert rig.node.exit_code != 0
    rig.pump(.3)
    assert len(rig.goals) == 1


@pytest.mark.parametrize('reject', ['center', 'footprint'])
def test_invalid_route_never_sends_goal(rig, reject):
    rig.blocked, rig.valid = (reject == 'center'), (reject != 'footprint')
    rig.start(True)
    assert rig.pump(5., lambda: rig.node.done)
    assert rig.node.exit_code != 0
    assert not rig.accepting


def test_degraded_cancels_and_localized_never_resumes_old_route(rig):
    rig.start(True)
    assert rig.pump(5., lambda: bool(rig.goals))
    rig.state = 2
    assert rig.pump(3., lambda: bool(rig.cancels))
    assert rig.pump(3., lambda: rig.node.done)
    rig.state = 1
    rig.pump(.4)
    assert len(rig.goals) == 1
    assert False in rig.disables


def test_stop_during_goal_acceptance_cancels_late_accepted_goal(rig):
    from rclpy.task import Future
    rig.delay_accept = Future()
    rig.start(True)
    assert rig.pump(5., lambda: rig.accepting)
    rig.node.stop('Ctrl+C')
    rig.delay_accept.set_result(None)
    assert rig.pump(3., lambda: bool(rig.cancels))
    assert rig.pump(3., lambda: rig.node.done)
    assert rig.node.exit_code != 0
