"""Start the Nav2 lifecycle only after Localization has a usable map pose."""

import json
import math
import time

import rclpy
from a2w_fastlio_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from .runtime_monitoring import (
    PAUSE_REQUEST, RESUME_REQUEST, STARTUP_REQUEST, Nav2ActivationPolicy)


class Nav2ActivationGate(Node):
    def __init__(self):
        super().__init__("nav2_activation_gate")
        enabled = bool(self.declare_parameter("enabled", True).value)
        status_topic = self.declare_parameter(
            "localization_status_topic", "/localization/status").value
        pose_topic = self.declare_parameter(
            "localization_pose_topic", "/localization/pose").value
        manager_service = self.declare_parameter(
            "lifecycle_manager_service",
            "/lifecycle_manager_navigation/manage_nodes").value
        retry_period_s = float(self.declare_parameter("retry_period_s", 0.5).value)
        if retry_period_s <= 0.0:
            raise ValueError("retry_period_s must be positive")

        self._policy = Nav2ActivationPolicy(enabled=enabled)
        self._authorization_timeout_s = float(self.declare_parameter(
            "authorization_timeout_s", 1.0).value)
        if not math.isfinite(self._authorization_timeout_s) or self._authorization_timeout_s <= 0:
            raise ValueError("authorization_timeout_s must be positive and finite")
        self._last_authorization_s = None
        safety_topic = self.declare_parameter(
            "safety_status_topic", "/navigation/safety_status").value
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_publisher = self.create_publisher(
            Bool, "/navigation/nav2_ready", ready_qos)
        self._last_published_ready = None
        self._status_subscription = self.create_subscription(
            LocalizationStatus, status_topic, self._on_status, 20)
        self._pose_subscription = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, 20)
        self._safety_subscription = self.create_subscription(
            String, safety_topic, self._on_safety, 10)
        self._manager_client = self.create_client(
            ManageLifecycleNodes, manager_service)
        self._timer = self.create_timer(retry_period_s, self._tick)
        self._publish_ready(False)
        if enabled:
            self.get_logger().info(
                "Nav2 activation deferred: waiting for LOCALIZED state and localization pose")
        else:
            self.get_logger().info("automatic Nav2 activation disabled")

    def _publish_ready(self, ready: bool) -> None:
        # Latched state, not a heartbeat: Guard publishes safety_status in
        # response, so echoing unchanged readiness creates a feedback loop.
        if ready == self._last_published_ready:
            return
        message = Bool()
        message.data = ready
        self._ready_publisher.publish(message)
        self._last_published_ready = ready

    def _on_status(self, message: LocalizationStatus) -> None:
        self._policy.observe_localization(message.state)
        self._publish_ready(self._policy.ready)
        self._request_lifecycle_transition()

    def _on_pose(self, _message: PoseStamped) -> None:
        self._policy.observe_pose()
        self._request_lifecycle_transition()

    def _tick(self) -> None:
        if (self._last_authorization_s is None or
                time.monotonic() - self._last_authorization_s > self._authorization_timeout_s):
            self._policy.observe_motion_authorization(False)
            self._publish_ready(False)
        self._request_lifecycle_transition()

    def _on_safety(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            authorized = status.get("motion_authorized") is True
        except (ValueError, TypeError, AttributeError):
            authorized = False
        self._last_authorization_s = time.monotonic()
        self._policy.observe_motion_authorization(authorized)
        self._publish_ready(self._policy.ready)
        self._request_lifecycle_transition()

    def _request_lifecycle_transition(self) -> None:
        if not self._manager_client.service_is_ready():
            return
        request_kind = self._policy.take_lifecycle_request()
        if request_kind is None:
            return
        commands = {
            STARTUP_REQUEST: ManageLifecycleNodes.Request.STARTUP,
            PAUSE_REQUEST: ManageLifecycleNodes.Request.PAUSE,
            RESUME_REQUEST: ManageLifecycleNodes.Request.RESUME,
        }
        request = ManageLifecycleNodes.Request()
        request.command = commands[request_kind]
        future = self._manager_client.call_async(request)
        future.add_done_callback(self._on_lifecycle_complete)
        self.get_logger().info(
            f"requesting Nav2 lifecycle transition: {request_kind}")

    def _on_lifecycle_complete(self, future) -> None:
        try:
            response = future.result()
            success = bool(response.success)
        except Exception as error:  # rclpy futures surface transport errors here.
            success = False
            self.get_logger().error(f"Nav2 lifecycle transition failed: {error}")
        request_kind = self._policy.complete_lifecycle_request(success)
        self._publish_ready(self._policy.ready)
        if success and request_kind in (STARTUP_REQUEST, RESUME_REQUEST):
            self.get_logger().info(
                "\n========================================\n"
                "Nav2 已激活：Planner / Controller / Costmap 已就绪\n"
                "========================================")
        elif success and request_kind == PAUSE_REQUEST:
            self.get_logger().error(
                "Nav2 已暂停：定位或运动授权未就绪；新 Goal 将被拒绝。"
                "定位恢复后请重新授权 /navigation/enable_motion，再发送新 Goal。")
        else:
            self.get_logger().warn(
                f"Nav2 {request_kind} failed; lifecycle transition will retry")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Nav2ActivationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
