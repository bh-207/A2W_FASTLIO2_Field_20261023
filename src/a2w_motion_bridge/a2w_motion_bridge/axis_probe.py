"""Passive recorder by default; explicit, bounded pulses through the existing guard.

No direct Sport publisher. Requires axis_diagnostic_mode in the navigation launch.
Each axis is a separate operator-confirmed run, never an automatic three-axis tour.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.convert import message_to_ordereddict
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from a2w_fastlio_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from unitree_api.msg import Request, Response

from .axis_probe_policy import AxisPulse, RequestAuditTracker, compose_pose, relative_motion


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


class AxisProbe(Node):
    def __init__(self, output, enabled=False):
        super().__init__("a2w_axis_probe")
        self.output = output
        self.enabled = enabled
        self.latest = {}
        self.samples = []
        self.external_motion = False
        self.request_audits = RequestAuditTracker()
        self.active_goal = False
        self.bridge_gids = set()
        self.active = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.body_frame = self.declare_parameter("body_frame", "body").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.odom_frame = self.declare_parameter("odom_frame", "camera_init").value
        self.command_topic = self.declare_parameter("command_topic", "/a2w/axis_test/cmd_vel").value
        self.sport_topic = self.declare_parameter("sport_topic", "/api/sport/request").value
        self.guard_node = self.declare_parameter("guard_node", "/localization_guard").value
        self.bridge_node = self.declare_parameter("bridge_node", "/a2w_motion_bridge").value
        self.publisher = self.create_publisher(Twist, self.command_topic, 10) if enabled else None
        self.subscriptions_ = []
        self.subscribe(Odometry, "odom_topic", "/Odometry", self.on_odom)
        self.subscribe(PoseStamped, "pose_topic", "/localization/pose",
                       lambda m: self.remember("pose", m))
        self.subscribe(LocalizationStatus, "localization_status_topic", "/localization/status",
                       lambda m: self.remember("localization", m))
        self.subscribe(String, "guard_status_topic", "/navigation/safety_status",
                       lambda m: self.remember("guard", m))
        self.subscribe(String, "bridge_status_topic", "/a2w_motion_bridge/status",
                       lambda m: self.remember("bridge", m))
        self.subscribe(String, "audit_topic", "/a2w_motion_bridge/request_audit",
                       self.on_audit)
        self.subscribe(GoalStatusArray, "goal_status_topic", "/navigate_to_pose/_action/status",
                       self.on_goals)
        self.subscribe(Response, "response_topic", "/api/sport/response",
                       lambda m: self.record("sport_response", message_to_ordereddict(m)))
        self.subscriptions_.append(self.create_subscription(
            Request, self.sport_topic, self.on_request, qos_profile_sensor_data))
        self.guard_enable = self.create_client(SetBool, self.declare_parameter(
            "guard_enable_service", "/navigation/enable_motion").value)
        self.bridge_enable = self.create_client(SetBool, self.declare_parameter(
            "bridge_enable_service", "/a2w_motion_bridge/enable").value)
        self.cancel = self.create_client(CancelGoal, self.declare_parameter(
            "cancel_service", "/navigate_to_pose/_action/cancel_goal").value)

    def subscribe(self, cls, param, default, callback):
        topic = self.declare_parameter(param, default).value
        self.subscriptions_.append(self.create_subscription(cls, topic, callback, qos_profile_sensor_data))

    def record(self, kind, data):
        self.output.write(json.dumps({"kind": kind, "steady_ns": time.monotonic_ns(),
            "ros_ns": self.get_clock().now().nanoseconds, "data": data}, ensure_ascii=False) + "\n")
        self.output.flush()

    def remember(self, name, message):
        self.latest[name] = (time.monotonic(), message)
        self.record(name, message_to_ordereddict(message))

    def on_odom(self, msg):
        self.remember("odom", msg)
        if msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.body_frame:
            return
        p, q = pose_values(msg.pose.pose)
        sample = {"stamp_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                  "body": (p, q), "base": None}
        try:
            # Only the fixed body->base edge is needed, at the odometry stamp.
            t = self.buffer.lookup_transform(self.body_frame, self.base_frame,
                                             Time.from_msg(msg.header.stamp)).transform
            offset = (t.translation.x, t.translation.y, t.translation.z)
            rotation = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
            sample["base"] = compose_pose(p, q, offset, rotation)
        except (TransformException, ValueError):
            pass
        self.samples.append(sample)
        self.record("synchronized_pose", sample)

    def on_goals(self, msg):
        self.active_goal = any(s.status in (1, 2, 3) for s in msg.status_list)
        self.record("goal_status", message_to_ordereddict(msg))

    def on_audit(self, msg):
        self.remember("audit", msg)
        try:
            audit = json.loads(msg.data)
            self.request_audits.audit(audit["request_id"], audit["api_id"],
                                      audit["parameter"], now=time.monotonic())
        except (ValueError, KeyError, TypeError):
            if self.active:
                self.external_motion = True

    def on_request(self, msg):
        # Humble's executor passes only msg; per-message publisher GID is not
        # exposed. Never substitute a graph endpoint GID for the actual sender.
        self.record("sport_request", {"publisher_gid": None,
                                     "origin_check": "bridge_request_audit",
                                     "message": message_to_ordereddict(msg)})
        if self.active and msg.header.identity.api_id in (1003, 1008):
            try:
                parameters = json.loads(msg.parameter or "{}")
            except ValueError:
                self.external_motion = True
                return
            self.request_audits.request(msg.header.identity.id, msg.header.identity.api_id,
                                        parameters, now=time.monotonic())

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError("所需服务不可用")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if not future.done() or future.result() is None:
            raise RuntimeError("服务响应超时")
        return future.result()

    def check_route(self, pulse=None):
        for remote, wanted in ((self.guard_node, self.command_topic),
                               (self.bridge_node, "/cmd_vel_guarded")):
            client = self.create_client(GetParameters, remote + "/get_parameters")
            try:
                result = self.call(client, GetParameters.Request(names=["input_topic", "max_lateral"]))
            finally:
                self.destroy_client(client)
            if len(result.values) != 2 or result.values[0].string_value != wanted:
                raise RuntimeError("拒绝运动：没有通过 axis_diagnostic_mode 隔离 Nav2 速度")
            if pulse is not None:
                pulse.check_lateral_limit(result.values[1].double_value)
        if self.count_publishers(self.command_topic) != 1:
            raise RuntimeError("诊断输入存在其他发布者，拒绝运动")
        self.bridge_gids = {bytes(p.endpoint_gid).hex() for p in
            self.get_publishers_info_by_topic(self.sport_topic)
            if p.node_name == self.bridge_node.rsplit("/", 1)[-1]}
        if len(self.bridge_gids) != 1:
            raise RuntimeError("无法唯一识别 MotionBridge 的 Sport 发布源")

    def healthy(self):
        now = time.monotonic()
        if self.request_audits.expired(now=now):
            if not self.external_motion:
                self.record("unconfirmed_sport_request", "Sport 请求未在150ms内匹配 MotionBridge 审计")
            self.external_motion = True
        for name in ("odom", "pose", "guard", "bridge"):
            if name not in self.latest or now - self.latest[name][0] > .35:
                return False
        if "localization" not in self.latest or self.latest["localization"][1].state != 1:
            return False
        odom = self.latest["odom"][1]
        age = (self.get_clock().now().nanoseconds -
               (odom.header.stamp.sec * 10**9 + odom.header.stamp.nanosec)) / 1e9
        if not -.05 <= age <= .35 or odom.header.frame_id != self.odom_frame or odom.child_frame_id != self.body_frame:
            return False
        try:
            guard = json.loads(self.latest["guard"][1].data)["motion_enabled"]
            bridge = json.loads(self.latest["bridge"][1].data)["hardware_output_enabled"]
        except (ValueError, KeyError):
            return False
        return (guard and bridge and bool(self.samples) and self.samples[-1]["base"] is not None
                and not self.active_goal and not self.external_motion)

    def publish(self, command):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command
        self.publisher.publish(msg)
        self.record("probe_command", command)

    def stop(self):
        self.active = False
        if self.publisher is None:
            return True
        self.publish((0., 0., 0.))
        # Disable final hardware first; failure is surfaced, never called a successful stop.
        errors = []
        for client in (self.bridge_enable, self.guard_enable):
            try:
                if not self.call(client, SetBool.Request(data=False)).success:
                    errors.append("disable rejected")
            except Exception as error:
                errors.append(str(error))
        if errors:
            print("软件停止未确认，请立即使用遥控急停：" + "; ".join(errors), file=sys.stderr)
        return not errors


def main(args=None):
    argv = sys.argv if args is None else ["axis_probe", *args]
    parser = argparse.ArgumentParser(description="单轴方向诊断；默认只记录，不运动")
    parser.add_argument("--axis", choices=("vx", "vy", "wz"), default="vx")
    parser.add_argument("--duration", type=float, default=1.0, help="运动持续时间，最大2秒")
    parser.add_argument("--vy-speed", type=float, default=.1, help="横移诊断速度，0 < vy <= 0.30 m/s")
    parser.add_argument("--record-seconds", type=float, default=10.)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--output-root", default="log/axis_diagnostics")
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    pulse = AxisPulse(options.axis, options.duration, options.enable_motion,
                      lateral_speed=options.vy_speed)
    if not 0 < options.record_seconds <= 300:
        parser.error("record-seconds must be in (0,300]")
    folder = Path(options.output_root).resolve() / f"{options.axis}_{time.time_ns()}"
    folder.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=argv[1:], signal_handler_options=SignalHandlerOptions.NO)
    print(f"诊断记录：{folder}")
    with (folder / "events.jsonl").open("x", encoding="utf-8") as stream:
        node = AxisProbe(stream, options.enable_motion)
        attempted = False
        try:
            node.record("options", vars(options))
            if not options.enable_motion:
                node.spin_for(options.record_seconds)
                print("只读采集完成，未发送运动命令。")
                return
            node.spin_for(2.)
            node.check_route(pulse)
            print("空旷场地、遥控急停在手。诊断入口不经过 Collision Monitor；单轴最长2秒。")
            print(f"本次 vx/vy/wz={pulse.commands[options.axis]}，持续 {options.duration:.2f} 秒。")
            if input(f"确认执行 +{options.axis}，请输入 {options.axis}：").strip() != options.axis:
                print("已取消，未启用运动。")
                return
            attempted = True
            response = node.call(node.cancel, CancelGoal.Request())
            if response.return_code != 0:
                raise RuntimeError("取消 Nav2 Goal 失败")
            node.spin_for(.5)
            for client in (node.bridge_enable, node.guard_enable):
                result = node.call(client, SetBool.Request(data=True))
                if not result.success:
                    raise RuntimeError(result.message)
            node.spin_for(.2)
            node.check_route(pulse)
            if not node.healthy():
                raise RuntimeError("定位/TF/安全门未就绪或 Goal 未结束，拒绝运动")
            before = node.samples[-1]
            node.active = True
            started = time.monotonic()
            while time.monotonic() - started < options.duration:
                healthy = node.healthy() and node.count_publishers(node.command_topic) == 1
                node.publish(pulse.command(time.monotonic() - started, healthy=healthy))
                if pulse.stopped:
                    raise RuntimeError("数据超时、外部运动请求或安全门变化：诊断已中止")
                node.spin_for(.05)
            if not node.stop():
                raise RuntimeError("未确认停车，本次测试不能判为完成")
            attempted = False
            node.spin_for(.5)
            after = node.samples[-1]
            if after["stamp_ns"] <= before["stamp_ns"] or not after["base"]:
                raise RuntimeError("停止后没有新鲜配对位姿，不能计算结果")
            result = {"axis": options.axis, "before": before, "after": after,
                      "body": relative_motion(*before["body"], *after["body"]),
                      "base_link": relative_motion(*before["base"], *after["base"]),
                      "hardware_direction_validated": False}
            node.record("result", result)
            (folder / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
            print(json.dumps(result, ensure_ascii=False, indent=2))
            print("请记录肉眼/AprilTag真实方向；单位外参下 body/base_link 相同不构成独立验证。")
        except KeyboardInterrupt:
            print("已中断，保留已采集记录。")
        except Exception as error:
            node.record("aborted", str(error))
            raise SystemExit(str(error))
        finally:
            if attempted:
                node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
