"""Run explicitly in an isolated ROS domain; never launches a hardware bridge."""
import os
import time
import unittest


@unittest.skipUnless(os.environ.get("A2W_ISOLATED_ROS_TEST") == "1",
                     "requires explicitly isolated ROS environment")
class MotionAuthorizationRosTest(unittest.TestCase):
    def test_start_pause_reauthorize_and_guard_disconnect(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from a2w_fastlio_msgs.msg import LocalizationStatus
        from geometry_msgs.msg import PoseStamped, Twist
        from nav2_msgs.srv import ManageLifecycleNodes
        from std_msgs.msg import Bool
        from std_srvs.srv import SetBool
        from a2w_navigation.localization_guard import LocalizationGuardNode
        from a2w_navigation.nav2_activation_gate import Nav2ActivationGate

        rclpy.init(args=["--ros-args", "-p", "auto_arm_on_first_localized:=true"])
        guard = LocalizationGuardNode()
        gate = Nav2ActivationGate()
        peer = Node("authorization_test_peer")
        executor = SingleThreadedExecutor()
        for node in (guard, gate, peer):
            executor.add_node(node)
        requests, outputs, readiness = [], [], []

        def lifecycle(request, response):
            requests.append(request.command)
            response.success = True
            return response

        server = peer.create_service(ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes", lifecycle)
        status_pub = peer.create_publisher(LocalizationStatus, "/localization/status", 20)
        pose_pub = peer.create_publisher(PoseStamped, "/localization/pose", 20)
        cmd_pub = peer.create_publisher(Twist, "/cmd_vel_safe", 10)
        subscription = peer.create_subscription(Twist, "/cmd_vel_guarded", outputs.append, 20)
        ready_subscription = peer.create_subscription(
            Bool, "/navigation/nav2_ready", readiness.append, 20)
        client = peer.create_client(SetBool, "/navigation/enable_motion")
        state = LocalizationStatus.LOCALIZED
        last_publish = 0.

        def pump(seconds=.1, condition=None):
            nonlocal last_publish
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now-last_publish > .1:
                    status = LocalizationStatus()
                    status.state = state
                    status_pub.publish(status)
                    pose = PoseStamped()
                    pose.header.stamp = peer.get_clock().now().to_msg()
                    pose.header.frame_id = "map"
                    pose.pose.orientation.w = 1.
                    pose_pub.publish(pose)
                    last_publish = now
                executor.spin_once(timeout_sec=.02)
                if condition and condition():
                    return True
            return condition() if condition else True

        def enable(value):
            future = client.call_async(SetBool.Request(data=value))
            self.assertTrue(pump(3., future.done))
            self.assertTrue(future.result().success, future.result().message)

        try:
            self.assertTrue(pump(5., lambda: gate._policy.ready and guard._policy.armed))
            self.assertEqual(requests, [ManageLifecycleNodes.Request.STARTUP])
            pump(.2)
            readiness.clear()
            pump(.3)
            self.assertLessEqual(len(readiness), 1,
                "safety_status and nav2_ready must not form a publish feedback loop")
            enable(False)
            self.assertTrue(pump(3., lambda: gate._policy.paused))
            self.assertFalse(guard._policy.armed)
            outputs.clear()
            command = Twist()
            command.linear.x = .1
            cmd_pub.publish(command)
            pump(.4)
            self.assertTrue(outputs)
            self.assertTrue(all(m.linear.x == 0. for m in outputs))
            enable(True)
            self.assertTrue(pump(3., lambda: gate._policy.ready and guard._policy.armed))
            self.assertEqual(requests[-1], ManageLifecycleNodes.Request.RESUME)
            state = LocalizationStatus.DEGRADED
            self.assertTrue(pump(3., lambda: gate._policy.paused))
            self.assertFalse(guard._policy.motion_authorized)
            count = len(requests)
            state = LocalizationStatus.LOCALIZED
            pump(.7)
            self.assertEqual(len(requests), count, "old authorization must not resume Nav2")
            self.assertFalse(gate._policy.ready)
            enable(True)
            self.assertTrue(pump(3., lambda: gate._policy.ready and guard._policy.armed))
            executor.remove_node(guard)
            guard.destroy_node()
            guard = None
            self.assertTrue(pump(3., lambda: gate._policy.paused),
                            "missing authorization heartbeat must pause Nav2")
        finally:
            executor.shutdown()
            if guard is not None:
                guard.destroy_node()
            gate.destroy_node()
            peer.destroy_node()
            rclpy.shutdown()
