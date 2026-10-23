"""Adapt the existing A2W IPPE Teach pose to one short-range Nav2 goal."""

import importlib.util
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from a2w_fastlio_msgs.msg import LocalizationStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .apriltag_goal_policy import (
    StableDetectionGate,
    bgr8_from_buffer,
    compute_nav_goal,
)


def _load_pose_module(path):
    spec = importlib.util.spec_from_file_location("a2w_apriltag_ippe_source", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载 AprilTag IPPE 源文件：{path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    required = ("AprilTagIppeEstimator", "K_RAW", "DIST")
    missing = [name for name in required if not hasattr(module, name)]
    if missing:
        raise RuntimeError(f"AprilTag IPPE 源文件缺少：{', '.join(missing)}")
    return module


def _transform_matrix(transform):
    translation = transform.translation
    quaternion = transform.rotation
    q = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("TF quaternion is invalid")
    x, y, z, w = q / norm
    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = [translation.x, translation.y, translation.z]
    return result


class AprilTagNavGoalNode(Node):
    def __init__(self):
        super().__init__("apriltag_nav_goal")
        pose_module_value = str(self.declare_parameter("pose_module_path", "").value)
        target_value = str(self.declare_parameter("target_path", "").value)
        if not target_value:
            raise RuntimeError("请通过 target_path 指定本场地的 AprilTag Teach 文件")
        pose_module_path = Path(pose_module_value).expanduser() if pose_module_value else Path(__file__).with_name("apriltag_pose.py")
        target_path = Path(target_value).expanduser()
        image_topic = str(self.declare_parameter(
            "image_topic", "/front_camera/image_raw").value)
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._body_frame = str(self.declare_parameter("body_frame", "body").value)
        self._min_distance_m = float(self.declare_parameter(
            "min_goal_distance_m", 0.05).value)
        self._max_distance_m = float(self.declare_parameter(
            "max_goal_distance_m", 1.0).value)
        self._min_yaw_rad = math.radians(float(self.declare_parameter(
            "min_goal_yaw_deg", 2.0).value))
        self._max_yaw_rad = math.radians(float(self.declare_parameter(
            "max_goal_yaw_deg", 90.0).value))
        required_frames = int(self.declare_parameter("min_good_frames", 5).value)
        # The validated 1920x1080 IPPE pipeline can take longer than 0.5 s on
        # this PC. This is adapter freshness only; IPPE continuity thresholds
        # and camera calibration remain owned by the existing source module.
        pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 2.0).value)

        if not pose_module_path.is_file():
            raise RuntimeError(f"AprilTag IPPE 源文件不存在：{pose_module_path}")
        if not target_path.is_file():
            raise RuntimeError(f"AprilTag Teach 文件不存在：{target_path}")
        pose_module = _load_pose_module(pose_module_path)
        target_data = np.load(target_path, allow_pickle=True)
        required_target_keys = {
            "T_tag_base_target", "T_base_camera_optical", "K_RAW", "DIST",
            "tag_id", "tag_size_m", "pose_solver",
        }
        missing = required_target_keys.difference(target_data.files)
        if missing:
            raise RuntimeError(f"Teach 文件缺少字段：{', '.join(sorted(missing))}")
        pose_solver = str(np.asarray(target_data["pose_solver"]).item())
        if pose_solver != "SOLVEPNP_IPPE_SQUARE":
            raise RuntimeError(f"Teach pose solver 不匹配：{pose_solver}")
        if (not np.allclose(target_data["K_RAW"], pose_module.K_RAW) or
                not np.allclose(target_data["DIST"], pose_module.DIST)):
            raise RuntimeError("Teach 文件与当前 AprilTag 相机标定不一致")

        self._target = np.asarray(
            target_data["T_tag_base_target"], dtype=np.float64).copy()
        self._estimator = pose_module.AprilTagIppeEstimator(
            tag_id=int(np.asarray(target_data["tag_id"]).item()),
            tag_size_m=float(np.asarray(target_data["tag_size_m"]).item()),
            T_base_camera_optical=np.asarray(
                target_data["T_base_camera_optical"], dtype=np.float64),
        )
        self._detection_gate = StableDetectionGate(
            required_frames=required_frames,
            timeout_s=pose_timeout_s,
        )
        self._localized = False
        self._goal_pending = False
        self._last_detection_reason = "waiting_for_image"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._status_publisher = self.create_publisher(
            String, "/apriltag_dock/status", 10)
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/apriltag_dock/goal_preview", 10)
        self._image_subscription = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)
        self._localization_subscription = self.create_subscription(
            LocalizationStatus,
            "/localization/status",
            self._on_localization,
            20,
        )
        self._trigger_service = self.create_service(
            Trigger, "/apriltag_dock/send_goal", self._on_trigger)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "AprilTag→Nav2 已就绪："
            f"image={image_topic} target={target_path} "
            f"range=[{self._min_distance_m:.2f}, {self._max_distance_m:.2f}]m")

    def _on_localization(self, message):
        self._localized = message.state == LocalizationStatus.LOCALIZED

    def _on_image(self, message):
        now_s = time.monotonic()
        try:
            frame = bgr8_from_buffer(
                message.data,
                height=message.height,
                width=message.width,
                step=message.step,
                encoding=message.encoding,
            )
            tag_body, info = self._estimator.detect(frame, now=now_s)
        except Exception as error:  # Camera/pose failure is fail-closed.
            self._detection_gate.reject()
            self._last_detection_reason = f"pose_error:{error}"
            return
        if tag_body is None:
            self._detection_gate.reject()
            self._last_detection_reason = str(info.get("reason", "pose_rejected"))
            return
        self._detection_gate.accept(tag_body, stamp_s=now_s)
        self._last_detection_reason = "accepted"

    def _publish_status(self):
        pose, readiness = self._detection_gate.current(now_s=time.monotonic())
        message = String()
        message.data = json.dumps({
            "localized": self._localized,
            "pose_ready": pose is not None,
            "pose_reason": readiness,
            "last_detection_reason": self._last_detection_reason,
            "goal_pending": self._goal_pending,
        }, separators=(",", ":"), ensure_ascii=False)
        self._status_publisher.publish(message)

    def _on_trigger(self, _request, response):
        if not self._localized:
            response.success = False
            response.message = "定位不是 LOCALIZED，拒绝发送 AprilTag Goal"
            return response
        if self._goal_pending:
            response.success = False
            response.message = "已有 AprilTag Goal 正在执行"
            return response
        tag_body, reason = self._detection_gate.current(now_s=time.monotonic())
        if tag_body is None:
            response.success = False
            response.message = f"AprilTag pose 未就绪：{reason}"
            return response
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._body_frame, Time())
            map_body = _transform_matrix(transform.transform)
            decision = compute_nav_goal(
                map_body,
                tag_body,
                self._target,
                min_distance_m=self._min_distance_m,
                max_distance_m=self._max_distance_m,
                min_yaw_rad=self._min_yaw_rad,
                max_yaw_rad=self._max_yaw_rad,
            )
        except (TransformException, ValueError) as error:
            response.success = False
            response.message = f"Goal 变换失败：{error}"
            return response
        if not decision.accepted:
            response.success = False
            response.message = f"Goal 被范围门拒绝：{decision.reason}"
            return response
        if not self._action_client.wait_for_server(timeout_sec=0.5):
            response.success = False
            response.message = "NavigateToPose action 尚未就绪"
            return response

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = decision.x
        goal_pose.pose.position.y = decision.y
        goal_pose.pose.orientation.z = math.sin(0.5 * decision.yaw)
        goal_pose.pose.orientation.w = math.cos(0.5 * decision.yaw)
        self._goal_publisher.publish(goal_pose)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self._goal_pending = True
        response.success = True
        response.message = (
            f"AprilTag Goal 已提交：distance={decision.distance:.3f}m "
            f"relative=({decision.relative_x:.3f},{decision.relative_y:.3f},"
            f"{math.degrees(decision.relative_yaw):.1f}deg)")
        return response

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._goal_pending = False
            self.get_logger().error(f"AprilTag Goal 提交失败：{error}")
            return
        if not goal_handle.accepted:
            self._goal_pending = False
            self.get_logger().error("AprilTag Goal 被 Nav2 拒绝")
            return
        self.get_logger().info("AprilTag Goal 已由 Nav2 接受")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_pending = False
        try:
            status = future.result().status
            self.get_logger().info(f"AprilTag Goal 结束：status={status}")
        except Exception as error:
            self.get_logger().error(f"AprilTag Goal 结果异常：{error}")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AprilTagNavGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
