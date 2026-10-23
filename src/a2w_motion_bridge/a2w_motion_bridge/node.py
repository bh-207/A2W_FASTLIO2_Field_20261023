"""ROS 2 node implementing the guarded Unitree A2-W motion boundary."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool
from unitree_api.msg import Request

from .motion_policy import (
    MotionPolicy,
    MotionPolicyConfig,
    RequestSpec,
    motion_profile_requests,
)


class A2WMotionBridge(Node):
    def __init__(self):
        super().__init__("a2w_motion_bridge")
        input_topic = self.declare_parameter("input_topic", "/cmd_vel_guarded").value
        output_topic = self.declare_parameter("output_topic", "/api/sport/request").value
        config = MotionPolicyConfig(
            max_forward=float(self.declare_parameter("max_forward", 0.15).value),
            max_reverse=float(self.declare_parameter("max_reverse", 0.05).value),
            max_lateral=float(self.declare_parameter("max_lateral", 0.15).value),
            max_yaw=float(self.declare_parameter("max_yaw", 0.50).value),
            min_effective_forward=float(self.declare_parameter(
                "min_effective_forward", 0.0).value),
            min_effective_lateral=float(self.declare_parameter(
                "min_effective_lateral", 0.0).value),
            min_effective_yaw=float(self.declare_parameter(
                "min_effective_yaw", 0.0).value),
            deadband_forward=float(self.declare_parameter("deadband_forward", 0.01).value),
            deadband_lateral=float(self.declare_parameter("deadband_lateral", 0.01).value),
            deadband_yaw=float(self.declare_parameter("deadband_yaw", 0.02).value),
            terminal_lateral_stop_m=float(self.declare_parameter(
                "terminal_lateral_stop_m", 0.06).value),
            terminal_lateral_boost_m=float(self.declare_parameter(
                "terminal_lateral_boost_m", 0.08).value),
            terminal_activation_radius_m=float(self.declare_parameter(
                "terminal_activation_radius_m", 0.70).value),
            command_timeout_s=float(self.declare_parameter("command_timeout_s", 0.30).value),
            start_enabled=bool(self.declare_parameter("start_enabled", False).value),
        )
        self._configure_motion_profile = bool(self.declare_parameter(
            "configure_motion_profile", True).value)
        self._gait_type = int(self.declare_parameter("gait_type", 0).value)
        self._speed_level = int(self.declare_parameter("speed_level", 0).value)
        self._policy = MotionPolicy(config)
        if config.min_effective_forward or config.min_effective_yaw:
            self.get_logger().warn("vx/wz 的 min_effective 参数仍未启用")
        if config.min_effective_lateral:
            self.get_logger().warn(
                f"终点横向补偿待命：vy_min_effective={config.min_effective_lateral:.2f}m/s；"
                "只接受新鲜且方向一致的路线终点误差")
        self._request_id = 0
        self._publisher = self.create_publisher(Request, output_topic, 10)
        self._audit = self.create_publisher(String, self.declare_parameter(
            "request_audit_topic", "/a2w_motion_bridge/request_audit").value, 50)
        self._status = self.create_publisher(String, "/a2w_motion_bridge/status", 10)
        self._subscription = self.create_subscription(Twist, input_topic, self._on_command, 10)
        self._terminal_error = None
        self._last_terminal_log_s = -math.inf
        self._terminal_error_timeout_s = float(self.declare_parameter(
            "terminal_error_timeout_s", 0.20).value)
        if not 0.0 < self._terminal_error_timeout_s <= 1.0:
            raise ValueError("terminal_error_timeout_s must be in (0,1]")
        self._terminal_sub = self.create_subscription(
            Vector3Stamped, self.declare_parameter(
                "terminal_error_topic", "/route_follower/terminal_error_body").value,
            self._on_terminal_error, 10)
        self._enable_service = self.create_service(
            SetBool, "/a2w_motion_bridge/enable", self._on_enable)
        self._watchdog_timer = self.create_timer(0.05, self._on_watchdog)
        self._profile_timer = None
        if self._policy.enabled and self._configure_motion_profile:
            # Localization initialization leaves ample time for DDS discovery;
            # publish once after startup instead of racing the sport service.
            self._profile_timer = self.create_timer(0.5, self._publish_profile_once)
        self._publish_status()
        if self._policy.enabled:
            self.get_logger().warn(
                "A2-W hardware output prepared by launch; LocalizationGuard still blocks motion")
        else:
            self.get_logger().warn(
                "A2-W hardware output is DISABLED; call /a2w_motion_bridge/enable only after checks")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _publish_request(self, spec: RequestSpec, terminal_lateral=None) -> None:
        request = Request()
        self._request_id += 1
        request.header.identity.id = self._request_id
        request.header.identity.api_id = spec.api_id
        request.parameter = spec.parameter
        self._publisher.publish(request)
        audit = String()
        audit_data = {
            "request_id": self._request_id, "api_id": spec.api_id,
            "parameter": json.loads(spec.parameter) if spec.parameter else {},
            "stamp_ns": self.get_clock().now().nanoseconds,
            "steady_ns": time.monotonic_ns(),
            "reason": self._policy.reason,
        }
        if terminal_lateral is not None:
            audit_data['terminal_lateral'] = terminal_lateral
        audit.data = json.dumps(audit_data, separators=(",", ":"))
        self._audit.publish(audit)
        self._publish_status()

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps({
            "hardware_output_enabled": self._policy.enabled,
            "reason": self._policy.reason,
            "gait_type": self._gait_type,
            "speed_level": self._speed_level,
        }, separators=(",", ":"))
        self._status.publish(message)

    def _publish_motion_profile(self) -> None:
        if not self._configure_motion_profile:
            return
        for request in motion_profile_requests(self._gait_type, self._speed_level):
            self._publish_request(request)
        self.get_logger().info(
            "A2-W motion profile requested: "
            f"gait_type={self._gait_type} speed_level={self._speed_level}")

    def _publish_profile_once(self) -> None:
        self._publish_motion_profile()
        if self._profile_timer is not None:
            self._profile_timer.cancel()

    def _on_terminal_error(self, message: Vector3Stamped) -> None:
        if message.header.frame_id != "base_link":
            return
        self._terminal_error = (
            message.vector.y, message.vector.z,
            rclpy.time.Time.from_msg(message.header.stamp).nanoseconds)

    def _on_command(self, message: Twist) -> None:
        now_ns = self.get_clock().now().nanoseconds
        error = distance = None
        if self._terminal_error is not None:
            lateral, xy_distance, stamp_ns = self._terminal_error
            age_ns = now_ns - stamp_ns
            if 0 <= age_ns <= self._terminal_error_timeout_s * 1e9:
                error, distance = lateral, xy_distance
        spec = self._policy.command(
            message.linear.x, message.linear.y, message.angular.z, now_ns * 1e-9,
            terminal_lateral_error_m=error, terminal_distance_m=distance)
        diagnostic = self._terminal_diagnostic(message.linear.y, spec, now_ns)
        self._publish_request(spec, terminal_lateral=diagnostic)
        # Observe the actual published request; never change the command here.
        if (self._policy.config.min_effective_lateral > 0.0 and
                message.linear.y != 0.0 and
                time.monotonic() - self._last_terminal_log_s >= 2.0):
            self._last_terminal_log_s = time.monotonic()
            self.get_logger().info(
                '[终点横移] guarded vy=%+.3f → Sport请求 vy=%+.3f m/s；'
                '横向误差=%s m 终点XY=%s m 误差消息年龄=%s ms；%s [%s]' % (
                    diagnostic['input_vy'], diagnostic['output_vy'],
                    diagnostic['lateral_error_m'], diagnostic['distance_m'],
                    diagnostic['error_age_ms'], diagnostic['reason_zh'], diagnostic['reason']))

    def _terminal_diagnostic(self, vy, spec, now_ns):
        """Read-only explanation of the existing terminal compensation gates."""
        config = self._policy.config
        lateral = distance = age_ms = None
        if self._terminal_error is not None:
            lateral, distance, stamp_ns = self._terminal_error
            age_ms = (now_ns - stamp_ns) * 1e-6
        bounded = max(-config.max_lateral, min(config.max_lateral, vy))
        output = json.loads(spec.parameter).get('y', 0.0) if spec.parameter else 0.0
        if not self._policy.enabled:
            reason, description = 'hardware_disabled', '运动输出已关闭'
        elif config.min_effective_lateral <= 0.0:
            reason, description = 'disabled', '未启用横向补偿'
        elif lateral is None:
            reason, description = 'error_missing', '未收到有效base_link终点误差'
        elif age_ms < 0.0:
            reason, description = 'error_future', '误差时间戳超前'
        elif age_ms > self._terminal_error_timeout_s * 1000.0:
            reason, description = 'error_stale', '终点误差消息已过期'
        elif not all(math.isfinite(v) for v in (lateral, distance)):
            reason, description = 'error_invalid', '终点误差非有限数'
        elif not 0.0 <= distance <= config.terminal_activation_radius_m:
            reason, description = 'outside_terminal', '未进入终点补偿范围'
        elif abs(lateral) <= config.terminal_lateral_stop_m:
            reason, description = 'lateral_deadzone', '横向误差已进入停止死区'
        elif abs(lateral) < config.terminal_lateral_boost_m:
            reason, description = 'below_boost_error', '横向误差未达到补偿门限'
        elif bounded == 0.0 or abs(bounded) < config.deadband_lateral:
            reason, description = 'command_deadband', '输入vy为零或处于指令死区'
        elif bounded * lateral <= 0.0:
            reason, description = 'direction_mismatch', 'vy与终点横向误差方向不一致'
        elif abs(bounded) >= config.min_effective_lateral:
            reason, description = 'already_effective', '输入vy已达到有效速度'
        else:
            reason, description = 'boosted', '横向最小有效速度补偿已触发'
        return dict(input_vy=vy, output_vy=output, lateral_error_m=lateral,
                    distance_m=distance, error_age_ms=age_ms,
                    reason=reason, reason_zh=description)

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        stop = self._policy.set_enabled(request.data)
        if stop is not None:
            self._publish_request(stop)
        else:
            self._publish_motion_profile()
            self._publish_status()
        response.success = True
        response.message = (
            "A2-W hardware output enabled; watchdog active"
            if request.data else "A2-W StopMove sent and hardware output disabled")
        return response

    def _on_watchdog(self) -> None:
        stop = self._policy.watchdog(self._now_s())
        if stop is not None:
            self.get_logger().error("A2-W command timeout: StopMove sent; re-enable required")
            self._publish_request(stop)

    def stop_now(self) -> None:
        self._publish_request(self._policy.shutdown())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = A2WMotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_now()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
