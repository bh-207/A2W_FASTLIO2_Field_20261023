"""Preview a sampled route, or explicitly execute ONE Nav2 FollowPath goal.

No cmd_vel publisher, algorithm retuning, waypoint dwell, or automatic restart.
"""
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from a2w_fastlio_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import GetCostmap, IsPathValid
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .route_path import (build_path, check_centerline, distance_to_polyline,
                         goal_error, load_route, ready_to_follow,
                         remaining_path_distance, terminal_error_body,
                         pose_discontinuity, rejoin_route)
from .route_storage import quaternion_yaw


class RouteFollower(Node):
    def __init__(self, **kwargs):
        super().__init__('route_follower', **kwargs)
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.frame = param('global_frame', 'map')
        self.robot_frame = param('robot_frame', 'base_link')
        self.execute = param('execute', False)
        self.wait_timeout = float(param('wait_timeout_s', 120.))
        self.request_timeout = float(param('request_timeout_s', 5.))
        self.tf_max_age = float(param('tf_max_age_s', 1.))
        self.start_tolerance = float(param('start_tolerance_m', .5))
        self.pose_jump_translation_m = float(param('pose_jump_translation_m', .6))
        self.pose_jump_yaw_rad = float(param('pose_jump_yaw_rad', .2))
        self.pose_jump_max_speed_mps = float(param('pose_jump_max_speed_mps', .8))
        self.pose_jump_max_yaw_rate_rps = float(param('pose_jump_max_yaw_rate_rps', .75))
        self.last_nav_pose = None
        self.controller = param('controller_id', 'FollowPath')
        self.goal_checker = param('goal_checker_id', 'goal_checker')
        if any(not math.isfinite(v) or v <= 0 for v in (
                self.wait_timeout, self.request_timeout, self.tf_max_age, self.start_tolerance,
                self.pose_jump_translation_m, self.pose_jump_yaw_rad,
                self.pose_jump_max_speed_mps, self.pose_jump_max_yaw_rate_rps)):
            raise ValueError('超时/起点容差必须是有限正数')
        raw = load_route(param('route_file', ''), self.frame, self.robot_frame)
        self.raw_points = raw
        self.rejoin_segment_index = param('rejoin_segment_index', -1)
        if (isinstance(self.rejoin_segment_index, bool) or
                not isinstance(self.rejoin_segment_index, int) or
                not -1 <= self.rejoin_segment_index < len(raw)-1):
            raise ValueError('rejoin_segment_index 必须为 -1 或原始路线有效段索引')
        self.spacing_m = float(param('spacing_m', .2))
        self.smooth_window, self.smooth_passes = param('smooth_window', 5), param('smooth_passes', 0)
        self.points = build_path(raw, self.spacing_m, self.smooth_window, self.smooth_passes)
        self.path = self._path(self.points)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.preview = self.create_publisher(Path, '/route_follower/path', qos)
        self.raw_pub = self.create_publisher(Path, '/route_follower/raw_path', qos)
        self.trace_pub = self.create_publisher(Path, '/route_follower/executed_path', qos)
        self.terminal_error_pub = self.create_publisher(
            Vector3Stamped, '/route_follower/terminal_error_body', 10)
        self.remaining_m = None
        self.progress_s = 0.
        self.trace = Path()
        self.trace.header.frame_id = self.frame
        self.deviation_samples = 0
        self.max_raw_deviation_m = self.raw_deviation_sum_m = 0.
        self.max_path_deviation_m = self.path_deviation_sum_m = 0.
        self.final_goal_error = None
        self.preview.publish(self.path)
        self.raw_pub.publish(self._path(raw))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.localized = False
        self.nav_ready = False
        self.safety = self.bridge = (False, -math.inf)
        self.busy = {'nav': False, 'follow': False}
        self.subscriptions_ = [
            self.create_subscription(LocalizationStatus, param(
                'localization_status_topic', '/localization/status'), self._localization, 10),
            self.create_subscription(Bool, '/navigation/nav2_ready', self._ready, qos),
            self.create_subscription(String, '/navigation/safety_status',
                                     lambda m: self._gate(m, 'safety', 'motion_enabled'), 10),
            self.create_subscription(String, '/a2w_motion_bridge/status',
                                     lambda m: self._gate(m, 'bridge', 'hardware_output_enabled'), 10),
        ]
        action_name = param('action_name', '/follow_path')
        for name, topic in [('nav', '/navigate_to_pose/_action/status'),
                            ('follow', action_name+'/_action/status')]:
            self.subscriptions_.append(self.create_subscription(
                GoalStatusArray, topic, lambda m, key=name: self._busy(m, key), qos))
        self.action = ActionClient(self, FollowPath, action_name)
        self.grid_client = self.create_client(GetCostmap, param(
            'costmap_service', '/global_costmap/get_costmap'))
        self.valid_client = self.create_client(IsPathValid, param(
            'path_valid_service', '/is_path_valid'))
        self.disable_client = self.create_client(SetBool, '/navigation/enable_motion')
        self.phase = 'WAITING' if self.execute else 'PREVIEW'
        self.phase_since = self.started = time.monotonic()
        self.goal_handle = None
        self.send_future = None
        self.done = False
        self.exit_code = 0
        self.cancel_requested = False
        self.last_feedback = self.last_trace = 0.
        self.timer = self.create_timer(.1, self._tick)
        self.get_logger().info(
            f'采样 {len(raw)} 点 → 连续路径 {len(self.points)} 点；'
            + ('等待 LOCALIZED、Nav2 READY 和两道运动门；验证后只发送一次 FollowPath。'
               if self.execute else '仅预览，不发送运动目标。确认路线后用 execute:=true 单独启动。'))

    def _path(self, points):
        result = Path()
        result.header.frame_id = self.frame
        result.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = result.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            result.poses.append(pose)
        return result

    def _localization(self, message):
        self.localized = message.state == LocalizationStatus.LOCALIZED

    def _ready(self, message):
        self.nav_ready = message.data

    def _gate(self, message, attribute, field):
        try:
            flag = json.loads(message.data).get(field) is True
        except (ValueError, AttributeError):
            flag = False
        setattr(self, attribute, (flag, time.monotonic()))

    def _busy(self, message, key):
        self.busy[key] = any(s.status in (GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING) for s in message.status_list)

    def _transform(self):
        transform = self.buffer.lookup_transform(self.frame, self.robot_frame, Time())
        age = (self.get_clock().now().nanoseconds-Time.from_msg(transform.header.stamp).nanoseconds)*1e-9
        if not -.1 <= age <= self.tf_max_age:
            raise ValueError(f'TF 时间异常：age={age:.2f}s')
        return transform

    def _estimated_pose(self, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (translation.x, translation.y,
                quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w))

    def _record_deviation(self, pose):
        raw = distance_to_polyline(pose, self.raw_points)
        path = distance_to_polyline(pose, self.points)
        self.deviation_samples += 1
        self.raw_deviation_sum_m += raw
        self.path_deviation_sum_m += path
        self.max_raw_deviation_m = max(self.max_raw_deviation_m, raw)
        self.max_path_deviation_m = max(self.max_path_deviation_m, path)

    def _set_phase(self, phase):
        self.phase, self.phase_since = phase, time.monotonic()

    def _tick(self):
        if self.done or self.phase == 'PREVIEW':
            return
        now = time.monotonic()
        if self.phase == 'CANCELLING':
            if now-self.phase_since > self.request_timeout:
                self.get_logger().error('取消确认超时，已请求关闭安全门；请执行 scripts/a2w_stop_now.sh 并确认机器人停止。')
                self.done = True
            return
        ready = ready_to_follow(localized=self.localized, nav_ready=self.nav_ready,
                                safety=self.safety, bridge=self.bridge, now=now)
        try:
            transform = self._transform()
        except (TransformException, ValueError) as error:
            transform = None
            if self.phase != 'WAITING':
                self.stop(f'TF 不可用：{error}')
                return
        if self.phase == 'WAITING':
            if now-self.started > self.wait_timeout:
                self.stop('等待定位/Nav2/运动门超时；未发送目标')
                return
            if not ready or transform is None or now-self.started < 1.:
                return
            if any(self.busy.values()):
                self.stop('已有导航任务，请先取消 RViz/其他客户端目标')
                return
            if self.rejoin_segment_index < 0:
                p = transform.transform.translation
                distance = math.hypot(p.x-self.points[0][0], p.y-self.points[0][1])
                if distance > self.start_tolerance:
                    self.stop(f'距 A 点 {distance:.2f}m，超出 {self.start_tolerance:.2f}m；请先手控或单点导航回 A')
                    return
            if not (self.action.server_is_ready() and self.grid_client.service_is_ready()
                    and self.valid_client.service_is_ready()):
                return
            if self.rejoin_segment_index >= 0:
                try:
                    remaining = rejoin_route(self.raw_points, self._estimated_pose(transform),
                                             self.rejoin_segment_index, self.start_tolerance)
                    self.points = build_path(remaining, self.spacing_m,
                                             self.smooth_window, self.smooth_passes)
                    self.path = self._path(self.points)
                    self.preview.publish(self.path)
                except ValueError as error:
                    self.stop(f'未发送显式重接路径：{error}')
                    return
            self.last_nav_pose = (self._estimated_pose(transform),
                                  Time.from_msg(transform.header.stamp).nanoseconds*1.e-9)
            self._set_phase('CHECK_MAP')
            self.grid_client.call_async(GetCostmap.Request()).add_done_callback(self._grid_checked)
            return
        if not ready or self.busy['nav']:
            self.stop('定位/安全门/Nav2 就绪状态失效，或存在其他导航任务；不自动续跑')
            return
        if self.phase != 'RUNNING' and now-self.phase_since > self.request_timeout:
            self.stop('验证/目标响应超时')
            return
        current = self._estimated_pose(transform)
        stamp_s = Time.from_msg(transform.header.stamp).nanoseconds*1.e-9
        if (not all(math.isfinite(v) for v in current) or
                (self.last_nav_pose is not None and pose_discontinuity(
                    self.last_nav_pose[0], current, stamp_s-self.last_nav_pose[1],
                    translation_slack_m=self.pose_jump_translation_m,
                    yaw_slack_rad=self.pose_jump_yaw_rad,
                    max_speed_mps=self.pose_jump_max_speed_mps,
                    max_yaw_rate_rps=self.pose_jump_max_yaw_rate_rps,
                    max_interval_s=self.tf_max_age))):
            self.stop('map→base_link 位姿/时标跳变：取消旧路线并关闭安全门；确认定位后重新授权，显式重接新任务')
            return
        self.last_nav_pose = (current, stamp_s)
        if self.phase == 'RUNNING':
            self.remaining_m, self.progress_s = remaining_path_distance(
                current, self.points, self.progress_s)
            tf_age = (self.get_clock().now().nanoseconds -
                      Time.from_msg(transform.header.stamp).nanoseconds)*1e-9
            if 0. <= tf_age <= .2:
                forward, lateral, distance = terminal_error_body(current, self.points[-1])
                error = Vector3Stamped()
                error.header.stamp = self.get_clock().now().to_msg()
                error.header.frame_id = self.robot_frame
                error.vector.x, error.vector.y, error.vector.z = forward, lateral, distance
                self.terminal_error_pub.publish(error)
        if self.phase == 'RUNNING' and now-self.last_trace >= .5:
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            self._record_deviation(self._estimated_pose(transform))
            self.trace.header.stamp = pose.header.stamp
            self.trace.poses.append(pose)
            self.trace.poses = self.trace.poses[-5000:]
            self.trace_pub.publish(self.trace)
            self.last_trace = now

    def _grid_checked(self, future):
        if self.phase != 'CHECK_MAP':
            return
        try:
            grid = future.result().map
            check_centerline(self.points, grid, self.frame)
            # Densely sample footprint poses, preserving every route vertex and
            # interpolating yaw along the shortest angular arc.
            dense = []
            for a, b in zip(self.points, self.points[1:]):
                count = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/(grid.metadata.resolution*.5)))
                dyaw = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
                dense.extend((a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count,
                              a[2]+dyaw*i/count) for i in range(count))
            dense.append(self.points[-1])
            self._set_phase('CHECK_PATH')
            self.valid_client.call_async(IsPathValid.Request(
                path=self._path(dense))).add_done_callback(self._path_checked)
        except Exception as error:
            self.stop(f'路径地图检查失败：{error}')

    def _path_checked(self, future):
        if self.phase != 'CHECK_PATH':
            return
        try:
            if not future.result().is_valid:
                raise ValueError('Nav2 footprint 路径碰撞检查未通过；请调整采样路线或减小平滑幅度')
            transform = self._transform()
            current = self._estimated_pose(transform)
            stamp_s = Time.from_msg(transform.header.stamp).nanoseconds*1.e-9
            if self.last_nav_pose is not None and pose_discontinuity(
                    self.last_nav_pose[0], current, stamp_s-self.last_nav_pose[1],
                    translation_slack_m=self.pose_jump_translation_m,
                    yaw_slack_rad=self.pose_jump_yaw_rad,
                    max_speed_mps=self.pose_jump_max_speed_mps,
                    max_yaw_rate_rps=self.pose_jump_max_yaw_rate_rps,
                    max_interval_s=self.tf_max_age):
                raise ValueError('路径验证期间 map→base_link 位姿/时标跳变，请重新确认定位')
            self.last_nav_pose = (current, stamp_s)
            if not ready_to_follow(localized=self.localized, nav_ready=self.nav_ready,
                    safety=self.safety, bridge=self.bridge, now=time.monotonic()) or any(self.busy.values()):
                raise ValueError('发送前安全/任务独占检查未通过')
            self.path = self._path(self.points)
            goal = FollowPath.Goal(path=self.path, controller_id=self.controller,
                                   goal_checker_id=self.goal_checker)
            self._set_phase('SENDING')
            self.send_future = self.action.send_goal_async(goal, feedback_callback=self._feedback)
            self.send_future.add_done_callback(self._accepted)
        except Exception as error:
            self.stop(f'未发送路径：{error}')

    def _accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.stop('FollowPath 被拒绝')
                self.done = True
                return
            self.goal_handle.get_result_async().add_done_callback(self._result)
            if self.phase == 'CANCELLING' or self.done:
                self._cancel()
            else:
                self._set_phase('RUNNING')
                self.get_logger().info('开始连续跟踪 A→B；中间点不停留，Ctrl+C 取消本任务。')
        except Exception as error:
            self.stop(f'目标响应异常：{error}')

    def _feedback(self, message):
        if time.monotonic()-self.last_feedback >= 2.:
            f = message.feedback
            remaining = '等待 map TF' if self.remaining_m is None else f'{self.remaining_m:.2f}m'
            try:
                transform = self._transform()
                pose = self._estimated_pose(transform)
                forward, lateral, xy = terminal_error_body(pose, self.points[-1])
                yaw = goal_error(pose, self.points[-1])[3]
                tf_age = (self.get_clock().now().nanoseconds -
                          Time.from_msg(transform.header.stamp).nanoseconds) * 1e-9
                endpoint = (f'终点XY={xy:.3f}m，body纵向误差={forward:+.3f}m，'
                            f'横向误差={lateral:+.3f}m，yaw误差={yaw:+.3f}rad，'
                            f'TF年龄={tf_age:.3f}s，'
                            f'终点误差可发布={0. <= tf_age <= .2}')
            except (TransformException, ValueError) as error:
                endpoint = f'终点误差不可用：{error}'
            self.get_logger().info(
                f'沿线剩余 {remaining}（不代表到达）；{endpoint}；'
                f'控制反馈速度 {f.speed:.2f}m/s')
            self.last_feedback = time.monotonic()

    def _log_result_metrics(self):
        try:
            pose = self._estimated_pose(self._transform())
            self.final_goal_error = goal_error(pose, self.raw_points[-1])
            self._record_deviation(pose)
            dx, dy, xy, yaw = self.final_goal_error
            self.get_logger().info(
                f'终点 map→base_link 估计误差：dx={dx:+.3f}m dy={dy:+.3f}m '
                f'XY={xy:.3f}m yaw={yaw:+.3f}rad')
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'终点误差不可用：{error}')
        if self.deviation_samples:
            self.get_logger().info(
                f'轨迹最近路径距离（每 0.5s 一次，含绕障）：'
                f'原始采样线最大/平均={self.max_raw_deviation_m:.3f}/'
                f'{self.raw_deviation_sum_m/self.deviation_samples:.3f}m，'
                f'发送路径最大/平均={self.max_path_deviation_m:.3f}/'
                f'{self.path_deviation_sum_m/self.deviation_samples:.3f}m，'
                f'样本={self.deviation_samples}')

    def _result(self, future):
        try:
            status = future.result().status
            self._log_result_metrics()
            if status == GoalStatus.STATUS_SUCCEEDED and self.phase != 'CANCELLING':
                self.get_logger().info('A→B 跟踪完成，Nav2 已报告到达 B。')
            else:
                self.exit_code = 1
                self.get_logger().warning(f'任务结束 status={status}；不会自动重发或恢复。')
        except Exception as error:
            self.exit_code = 1
            self.get_logger().error(f'读取结果失败：{error}')
        self.done = True

    def _cancel(self):
        if self.goal_handle and self.goal_handle.accepted and not self.cancel_requested:
            self.cancel_requested = True
            self.goal_handle.cancel_goal_async()

    def stop(self, reason):
        if self.done or self.phase in ('CANCELLING', 'PREVIEW'):
            return
        self.exit_code = 1
        self.progress_s, self.remaining_m, self.last_nav_pose = 0., None, None
        self.get_logger().warning(reason)
        pending = self.send_future is not None
        self._set_phase('CANCELLING')
        # Closing the existing guard is a second, independent stop path when an
        # action response is late. Never publish raw Twist or Sport requests here.
        if pending and self.disable_client.service_is_ready():
            self.disable_client.call_async(SetBool.Request(data=False))
        self._cancel()
        if not pending:
            self.done = True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node, code = None, 1
    try:
        node = RouteFollower()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
        code = node.exit_code
    except KeyboardInterrupt:
        if node:
            node.stop('用户 Ctrl+C：取消路线并关闭导航安全门')
            deadline = time.monotonic()+node.request_timeout
            try:
                while not node.done and node.phase != 'PREVIEW' and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
            except KeyboardInterrupt:
                pass
            code = 0 if node.phase == 'PREVIEW' else 1
            if node.phase != 'PREVIEW' and not node.done:
                node.get_logger().error('未确认停车，请执行 scripts/a2w_stop_now.sh 或遥控急停。')
    except Exception as error:
        print(f'连续路径任务异常：{error}', file=sys.stderr)
        if node:
            node.stop(str(error))
            deadline = time.monotonic()+node.request_timeout
            while not node.done and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
