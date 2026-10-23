"""Localization-status gate between Nav2 and the A2-W motion bridge."""

import json

import rclpy
from rclpy.signals import SignalHandlerOptions
from action_msgs.srv import CancelGoal
from a2w_fastlio_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .guard_policy import (
    GuardEvent, GuardPolicy, GuardPolicyConfig, NavigationReadinessLatch)
from .runtime_monitoring import should_publish_zero_keepalive


class LocalizationGuardNode(Node):
    def __init__(self):
        super().__init__("localization_guard")
        config = GuardPolicyConfig(
            status_timeout_s=float(self.declare_parameter("status_timeout_s", 1.0).value),
            max_forward=float(self.declare_parameter("max_forward", 0.15).value),
            max_reverse=float(self.declare_parameter("max_reverse", 0.05).value),
            max_lateral=float(self.declare_parameter("max_lateral", 0.15).value),
            max_yaw=float(self.declare_parameter("max_yaw", 0.50).value),
            auto_arm_on_first_localized=bool(self.declare_parameter(
                "auto_arm_on_first_localized", False).value),
        )
        self._policy = GuardPolicy(config)
        self._readiness = NavigationReadinessLatch()
        self._pose_available = False
        self._hardware_output_enabled = False
        status_topic = self.declare_parameter(
            "localization_status_topic", "/localization/status").value
        pose_topic = self.declare_parameter(
            "localization_pose_topic", "/localization/pose").value
        nav2_ready_topic = self.declare_parameter(
            "nav2_ready_topic", "/navigation/nav2_ready").value
        self._command_keepalive_s = float(self.declare_parameter(
            "command_keepalive_s", 0.1).value)
        if self._command_keepalive_s <= 0.0:
            raise ValueError("command_keepalive_s must be positive")
        input_topic = self.declare_parameter("input_topic", "/cmd_vel_safe").value
        output_topic = self.declare_parameter("output_topic", "/cmd_vel_guarded").value
        cancel_service = self.declare_parameter(
            "navigate_to_pose_cancel_service", "/navigate_to_pose/_action/cancel_goal").value
        self._command_publisher = self.create_publisher(Twist, output_topic, 10)
        self._status_publisher = self.create_publisher(String, "/navigation/safety_status", 10)
        self._status_subscription = self.create_subscription(
            LocalizationStatus, status_topic, self._on_status, 20)
        self._pose_subscription = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, 20)
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._nav2_ready_subscription = self.create_subscription(
            Bool, nav2_ready_topic, self._on_nav2_ready, ready_qos)
        self._bridge_status_subscription = self.create_subscription(
            String, "/a2w_motion_bridge/status", self._on_bridge_status, 10)
        self._command_subscription = self.create_subscription(
            Twist, input_topic, self._on_command, 10)
        self._enable_service = self.create_service(
            SetBool, "/navigation/enable_motion", self._on_enable)
        self._cancel_client = self.create_client(CancelGoal, cancel_service)
        self._cancel_pending = False
        self._last_command_s = None
        self._timer = self.create_timer(0.1, self._on_timer)
        self._publish_zero()
        self._publish_status()
        self.get_logger().warn("navigation blocked: localization not ready")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _publish_zero(self) -> None:
        self._command_publisher.publish(Twist())

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps({
            "motion_enabled": self._policy.armed,
            "motion_authorized": self._policy.motion_authorized,
            "reason": self._policy.reason,
        }, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _apply_event(self, event: GuardEvent) -> None:
        if event.publish_zero:
            self._publish_zero()
        if event.cancel_goal:
            self._cancel_pending = True
        self._try_cancel_goal()
        self._publish_status()
        self._update_readiness()

    def _update_readiness(self) -> None:
        transition = self._readiness.update(
            gate_armed=self._policy.armed,
            pose_available=self._pose_available,
            hardware_output_enabled=self._hardware_output_enabled,
        )
        if transition is True:
            self.get_logger().info(
                "\n========================================\n"
                "定位与运动安全门已就绪\n"
                "请等待 Nav2 已激活提示后再设置 Nav2 Goal\n"
                "========================================")
        elif transition is False:
            self.get_logger().error(
                "导航已暂停：定位或运动安全条件不再满足，已输出零速度")

    def _try_cancel_goal(self) -> None:
        if not self._cancel_pending or not self._cancel_client.service_is_ready():
            return
        self._cancel_client.call_async(CancelGoal.Request())
        self._cancel_pending = False
        self.get_logger().error("NavigateToPose cancel requested by LocalizationGuard")

    def _on_status(self, message: LocalizationStatus) -> None:
        self._apply_event(self._policy.update_status(message.state, self._now_s()))

    def _on_pose(self, _message: PoseStamped) -> None:
        self._pose_available = True
        event = self._policy.update_liveness(self._now_s())
        self._apply_event(event)

    def _on_bridge_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            self._hardware_output_enabled = bool(
                status.get("hardware_output_enabled", False))
        except (json.JSONDecodeError, AttributeError):
            self._hardware_output_enabled = False
            self.get_logger().error("invalid A2-W motion bridge status")
        self._update_readiness()

    def _on_nav2_ready(self, message: Bool) -> None:
        self._apply_event(
            self._policy.update_navigation_ready(message.data, self._now_s()))

    def _on_command(self, message: Twist) -> None:
        now_s = self._now_s()
        self._last_command_s = now_s
        vx, vy, wz = self._policy.filter_command(
            message.linear.x, message.linear.y, message.angular.z, now_s)
        output = Twist()
        output.linear.x = vx
        output.linear.y = vy
        output.angular.z = wz
        self._command_publisher.publish(output)
        self._publish_status()

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            response.success = self._policy.arm(self._now_s())
            response.message = (
                ("navigation motion enabled; send a NEW goal" if self._policy.armed
                 else "motion authorized; waiting for Nav2 resume, then send a NEW goal")
                if response.success
                else "navigation blocked: " + self._policy.reason)
        else:
            self._apply_event(self._policy.disarm())
            response.success = True
            response.message = "navigation motion disabled; zero command published"
        self._publish_status()
        self._update_readiness()
        return response

    def _on_timer(self) -> None:
        now_s = self._now_s()
        self._apply_event(self._policy.tick(now_s))
        if should_publish_zero_keepalive(
                armed=self._policy.armed,
                last_command_s=self._last_command_s,
                now_s=now_s,
                interval_s=self._command_keepalive_s):
            self._publish_zero()


def main(args=None):
    # Keep the ROS context valid while Python handles SIGINT so the final
    # zero command can be published before shutdown invalidates publishers.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LocalizationGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
