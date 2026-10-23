"""Low-rate terminal summary for Localization, costmap, and velocity stages."""

import json
import time
from typing import Dict, Optional, Tuple

import rclpy
from a2w_fastlio_msgs.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from .runtime_monitoring import CostmapCounts, classify_costmap


Velocity = Tuple[float, float, float]


class NavigationRuntimeMonitor(Node):
    def __init__(self):
        super().__init__("navigation_runtime_monitor")
        report_interval_s = float(self.declare_parameter(
            "report_interval_s", 2.0).value)
        if report_interval_s < 1.0:
            raise ValueError("report_interval_s must be at least 1.0 second")
        self._use_collision_monitor = bool(self.declare_parameter(
            "use_collision_monitor", True).value)

        self._localization_label = "WAITING"
        self._last_reported_localization: Optional[Tuple[int, str]] = None
        self._nav2_ready = False
        self._motion_enabled = False
        self._safety_reason = "waiting_for_safety_status"
        self._safety_time = 0.0
        self._local_costmap: Optional[CostmapCounts] = None
        self._velocities: Dict[str, Optional[Velocity]] = {
            "raw": None, "safe": None, "guarded": None,
        }

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_subscription = self.create_subscription(
            LocalizationStatus, "/localization/status", self._on_localization, 20)
        self._ready_subscription = self.create_subscription(
            Bool, "/navigation/nav2_ready", self._on_nav2_ready, latched_qos)
        self._safety_subscription = self.create_subscription(
            String, "/navigation/safety_status", self._on_safety, 10)
        self._costmap_subscription = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_local_costmap, latched_qos)
        self._raw_subscription = self.create_subscription(
            Twist, "/cmd_vel", lambda message: self._on_velocity("raw", message), 10)
        self._safe_subscription = None
        if self._use_collision_monitor:
            self._safe_subscription = self.create_subscription(
                Twist, "/cmd_vel_safe", lambda message: self._on_velocity("safe", message), 10)
        self._guarded_subscription = self.create_subscription(
            Twist, "/cmd_vel_guarded",
            lambda message: self._on_velocity("guarded", message), 10)
        self._timer = self.create_timer(report_interval_s, self._report)
        self.get_logger().info(
            f"navigation runtime summary enabled ({report_interval_s:.1f} s interval)")

    def _on_localization(self, message: LocalizationStatus) -> None:
        self._localization_label = message.state_label
        signature = (int(message.state), message.reason)
        if signature == self._last_reported_localization:
            return
        self._last_reported_localization = signature
        self.get_logger().info(
            "定位状态=%s reason=%s fitness=%.4f overlap=%.3f correspondences=%d" % (
                message.state_label, message.reason, message.fitness,
                message.overlap, message.correspondence_count))

    def _on_nav2_ready(self, message: Bool) -> None:
        if message.data == self._nav2_ready:
            return
        self._nav2_ready = message.data
        self.get_logger().info("Nav2状态=%s" % ("READY" if message.data else "WAITING"))

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        self._local_costmap = classify_costmap(message.data)

    def _on_safety(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            self._motion_enabled = status.get("motion_enabled") is True
            self._safety_reason = str(status.get("reason", "unknown"))
        except (ValueError, TypeError, AttributeError):
            self._motion_enabled = False
            self._safety_reason = "invalid_safety_status"
        self._safety_time = time.monotonic()

    def _on_velocity(self, stage: str, message: Twist) -> None:
        self._velocities[stage] = (
            message.linear.x, message.linear.y, message.angular.z)

    @staticmethod
    def _format_velocity(value: Optional[Velocity]) -> str:
        if value is None:
            return "waiting"
        return "(%.3f,%.3f,%.3f)" % value

    def _format_costmap(self) -> str:
        if self._local_costmap is None:
            return "waiting"
        counts = self._local_costmap
        return "obstacle=%d inflation=%d free=%d unknown=%d" % (
            counts.lethal, counts.inflated, counts.free, counts.unknown)

    def _report(self) -> None:
        if time.monotonic() - self._safety_time > 1.0:
            self._motion_enabled = False
            self._safety_reason = "safety_status_stale"
        self.get_logger().info(
            "导航摘要 localization=%s nav2=%s motion=%s reason=%s local_costmap[%s] "
            "cmd_vel raw=%s -> safe=%s -> guarded=%s" % (
                self._localization_label,
                "READY" if self._nav2_ready else "WAITING",
                "ENABLED" if self._motion_enabled else "DISABLED",
                self._safety_reason,
                self._format_costmap(),
                self._format_velocity(self._velocities["raw"]),
                self._format_velocity(self._velocities["safe"])
                if self._use_collision_monitor else "BYPASSED(已旁路)",
                self._format_velocity(self._velocities["guarded"])))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationRuntimeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
