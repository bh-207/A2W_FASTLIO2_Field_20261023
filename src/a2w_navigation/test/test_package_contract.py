#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]


class PackageContractTests(unittest.TestCase):
    def test_manifest_declares_only_the_approved_internal_dependency(self):
        root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        self.assertEqual(root.findtext("name"), "a2w_navigation")
        dependencies = {
            element.text
            for tag in ("depend", "build_depend", "exec_depend")
            for element in root.findall(tag)
            if element.text
        }
        self.assertIn("a2w_fastlio_msgs", dependencies)
        self.assertTrue({
            "ament_index_python", "launch", "launch_ros", "nav2_bringup",
            "nav2_bt_navigator", "nav2_common", "nav2_controller",
            "nav2_costmap_2d", "nav2_map_server", "nav2_mppi_controller",
            "nav2_planner", "nav2_smac_planner", "nav2_smoother",
            "nav2_behaviors", "nav2_lifecycle_manager",
            "nav2_velocity_smoother", "nav2_waypoint_follower",
            "nav2_collision_monitor", "rviz2",
            "nav2_msgs", "nav_msgs", "sensor_msgs", "tf2_ros", "python3-numpy",
        } <= dependencies)
        self.assertFalse({
            "a2w_fastlio_mapping", "a2w_fastlio_localization", "a2w_fastlio_map",
            "fast_lio", "go2_driver", "go2_description", "livox_ros_driver2",
            "nav2_amcl",
        } & dependencies)

    def test_cmake_registers_tests_without_building_runtime_code(self):
        source = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("find_package(ament_cmake REQUIRED)", source)
        self.assertIn("ament_add_pytest_test(test_package_contract", source)
        self.assertNotIn("add_executable(", source)
        self.assertNotIn("add_library(", source)
        self.assertIn("scripts/nav2_activation_gate", source)
        self.assertIn("scripts/navigation_runtime_monitor", source)
        self.assertIn("scripts/apriltag_nav_goal", source)
        self.assertIn("test_runtime_monitoring", source)
        self.assertIn("test_apriltag_goal_policy", source)

    def test_guard_uses_localized_pose_as_high_rate_liveness(self):
        source = (PACKAGE_ROOT / "a2w_navigation" / "localization_guard.py").read_text(
            encoding="utf-8")
        self.assertIn('"localization_pose_topic", "/localization/pose"', source)
        self.assertIn('"nav2_ready_topic", "/navigation/nav2_ready"', source)
        self.assertIn('"command_keepalive_s", 0.1', source)
        self.assertIn("self._policy.update_liveness", source)
        self.assertIn("self._policy.update_navigation_ready", source)
        self.assertIn("定位与运动安全门已就绪", source)
        self.assertNotIn("定位完成，导航系统已就绪", source)

    def test_runtime_nodes_own_signal_shutdown_without_double_shutdown_tracebacks(self):
        module_root = PACKAGE_ROOT / "a2w_navigation"
        for filename in ("nav2_activation_gate.py", "navigation_runtime_monitor.py"):
            with self.subTest(filename=filename):
                source = (module_root / filename).read_text(encoding="utf-8")
                self.assertIn("SignalHandlerOptions.NO", source)

    def test_documentation_describes_integrated_runtime_and_validation_limits(self):
        repository_root = PACKAGE_ROOT.parents[1]
        guide = (repository_root / "docs" / "NAVIGATION_DEVELOPMENT.md").read_text(
            encoding="utf-8")
        self.assertIn("map→camera_init→body→base_link", guide)
        self.assertIn("scripts/run_a2w_navigation.sh", guide)
        self.assertIn("LocalizationGuard → MotionBridge", guide)
        self.assertIn("默认旁路 Collision Monitor", guide)
        self.assertIn("不包含完整 NavigateToPose BT recovery", guide)
        self.assertIn("新部署设备与所有改动组合仍需", guide)
        self.assertIn(
            "Offline implementation and verification complete; "
            "JT128 hardware validation pending.", guide)


if __name__ == "__main__":
    unittest.main()
