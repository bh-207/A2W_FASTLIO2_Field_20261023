"""Interactive map-frame waypoint recorder."""

from datetime import datetime, timezone
from pathlib import Path
import sys

import rclpy
from a2w_fastlio_msgs.msg import LocalizationStatus
from nav_msgs.msg import Path as PathMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .capture_readiness import CaptureReadiness
from .ros_conversions import route_to_path, waypoint_from_transform
from .route_storage import is_capture_key, save_route_files
from .terminal_input import TerminalKeyReader


class WaypointRecorderNode(Node):
    def __init__(self, key_reader):
        super().__init__("waypoint_recorder")
        self._key_reader = key_reader
        self._global_frame = str(self.declare_parameter("global_frame", "map").value)
        self._robot_frame = str(self.declare_parameter("robot_frame", "base_link").value)
        self._output_prefix = Path(str(
            self.declare_parameter("output_prefix", "route").value)).expanduser()
        self._tf_timeout_s = float(self.declare_parameter("tf_timeout_s", 0.5).value)
        status_timeout_s = float(
            self.declare_parameter("localization_status_timeout_s", 2.0).value)
        status_topic = str(self.declare_parameter(
            "localization_status_topic", "/localization/status").value)
        path_topic = str(self.declare_parameter(
            "path_topic", "/waypoint_recorder/path").value)
        if not self._global_frame or not self._robot_frame:
            raise ValueError("global_frame and robot_frame must be non-empty")
        if self._tf_timeout_s <= 0.0:
            raise ValueError("tf_timeout_s must be positive")

        self._readiness = CaptureReadiness(status_timeout_s)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_publisher = self.create_publisher(PathMessage, path_topic, path_qos)
        self._status_subscription = self.create_subscription(
            LocalizationStatus, status_topic, self._on_localization_status, 20)
        self._waypoints = []
        self._saved = False
        self._keyboard_timer = self.create_timer(0.05, self._poll_keyboard)

        self.get_logger().info(
            "路径点采集器已启动：按 S/s 或空格记录，Ctrl+C 保存并退出")
        self.get_logger().info(
            f"TF={self._global_frame} -> {self._robot_frame} "
            f"输出前缀={self._output_prefix}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_localization_status(self, message: LocalizationStatus) -> None:
        self._readiness.update(message.state, self._now_s())

    def _poll_keyboard(self) -> None:
        key = self._key_reader.read_available()
        if key is not None and is_capture_key(key):
            self._capture_waypoint()

    def _capture_waypoint(self) -> None:
        blocked_reason = self._readiness.reason(self._now_s())
        if blocked_reason:
            self.get_logger().warning(f"本次路径点未记录：{blocked_reason}")
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        except TransformException as error:
            self.get_logger().warning(f"本次路径点未记录：TF 不可用：{error}")
            return

        waypoint = waypoint_from_transform(transform, len(self._waypoints))
        self._waypoints.append(waypoint)
        path = route_to_path(
            self._global_frame, self._waypoints, self.get_clock().now().to_msg())
        self._path_publisher.publish(path)
        self.get_logger().info(
            f"已记录路径点 {waypoint.index}: "
            f"x={waypoint.x:.4f}, y={waypoint.y:.4f}, "
            f"z={waypoint.z:.4f}, yaw={waypoint.yaw:.4f} rad")

    def save(self):
        if self._saved:
            return None
        saved_at = datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
            "+00:00", "Z")
        paths = save_route_files(
            self._output_prefix,
            frame_id=self._global_frame,
            robot_frame=self._robot_frame,
            waypoints=self._waypoints,
            saved_at_utc=saved_at,
        )
        self._saved = True
        self.get_logger().info(
            f"路径采集完成：{len(self._waypoints)} 个点；"
            f"YAML={paths[0]}；CSV={paths[1]}")
        return paths


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0
    try:
        with TerminalKeyReader(sys.stdin) as key_reader:
            node = WaypointRecorderNode(key_reader)
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # Keep terminal restoration and report a concise failure.
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"路径点采集器异常：{error}")
        else:
            print(f"路径点采集器异常：{error}", file=sys.stderr)
    finally:
        if node is not None:
            try:
                node.save()
            except Exception as error:
                exit_code = 1
                node.get_logger().error(f"路径保存失败：{error}")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
