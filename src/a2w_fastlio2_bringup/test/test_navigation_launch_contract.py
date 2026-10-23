#!/usr/bin/env python3

import importlib.util
import os
import pathlib
import re
import tempfile
import unittest
from unittest import mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters


BRINGUP_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BRINGUP_ROOT.parents[1]
LAUNCH_PATH = BRINGUP_ROOT / "launch" / "navigation.launch.py"
PACKAGE_PATH = BRINGUP_ROOT / "package.xml"
SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "run_a2w_navigation.sh"
STOP_SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "a2w_stop_now.sh"


def launch_context(description, overrides=None):
    """Resolve declared defaults without executing nodes or included launches."""
    context = LaunchContext()
    context.launch_configurations.update({
        "map_bundle_path": "/tmp/a2w_test_map_bundle",
        "navigation_map_yaml": "/tmp/a2w_test_map.yaml",
    })
    context.launch_configurations.update(overrides or {})
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


class NavigationLaunchContractTests(unittest.TestCase):
    def test_manual_capture_can_omit_sport_bridge_without_changing_navigation_default(self):
        spec = importlib.util.spec_from_file_location("navigation_capture_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        bridge = next(entity for entity in description.entities
                      if getattr(entity, "_Node__node_name", None) == "a2w_motion_bridge")
        self.assertIsNotNone(bridge.condition)
        for value, expected in (("false", False), ("true", True)):
            context = LaunchContext()
            context.launch_configurations["start_motion_bridge"] = value
            self.assertEqual(bridge.condition.evaluate(context), expected)

    @classmethod
    def setUpClass(cls):
        cls.launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
        cls.script_source = SCRIPT_PATH.read_text(encoding="utf-8")
        compile(cls.launch_source, str(LAUNCH_PATH), "exec")

    def test_declares_required_arguments_with_fail_closed_defaults(self):
        for name in (
                "map_bundle_path", "navigation_map_yaml", "nav2_params_file",
                "rviz", "autostart", "use_sim_time", "auto_enable_motion",
                "motion_vx_min_effective", "motion_vy_min_effective",
                "motion_wz_min_effective", "motion_gait_type",
                "motion_speed_level"):
            self.assertRegex(
                self.launch_source,
                rf'DeclareLaunchArgument\(\s*"{re.escape(name)}"',
            )
        self.assertIn(
            'DeclareLaunchArgument("autostart", default_value="false")',
            self.launch_source)
        self.assertIn(
            'DeclareLaunchArgument("rviz", default_value="false")',
            self.launch_source)
        self.assertIn(
            'DeclareLaunchArgument("auto_enable_motion", default_value="false")',
            self.launch_source)

    def test_composes_localization_nav2_guard_and_disabled_bridge(self):
        self.assertIn('package="robot_state_publisher"', self.launch_source)
        self.assertIn('executable="robot_state_publisher"', self.launch_source)
        self.assertIn('get_package_share_directory("a2w_description")', self.launch_source)
        self.assertIn('"robot_description"', self.launch_source)
        self.assertIn('localization.launch.py', self.launch_source)
        self.assertIn('nav2_core.launch.py', self.launch_source)
        self.assertIn('package="a2w_navigation"', self.launch_source)
        self.assertIn('executable="localization_guard"', self.launch_source)
        self.assertIn('package="a2w_motion_bridge"', self.launch_source)
        self.assertIn('executable="motion_bridge"', self.launch_source)
        self.assertIn('"output_topic": "/cmd_vel_guarded"', self.launch_source)

    def test_collision_bypass_keeps_localization_guard_in_velocity_chain(self):
        spec = importlib.util.spec_from_file_location("navigation_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for enabled, topic in (("true", "/cmd_vel_safe"), ("false", "/cmd_vel")):
            # ParameterValue caches its evaluated value within one launch.
            description = module.generate_launch_description()
            guard = next(entity for entity in description.entities
                         if getattr(entity, "_Node__node_name", None) == "localization_guard")
            context = launch_context(description, {
                "use_collision_monitor": enabled,
                "use_sim_time": "false", "auto_enable_motion": "false",
            })
            params = evaluate_parameters(context, guard._Node__parameters)[0]
            self.assertEqual(params["input_topic"], topic)
            self.assertEqual(params["output_topic"], "/cmd_vel_guarded")
            self.assertEqual(params["max_forward"], 0.60)
            self.assertFalse(params["auto_arm_on_first_localized"])

    def test_projects_jt128_cloud_through_finite_scan_for_costmap_clearing(self):
        self.assertIn('package="pointcloud_to_laserscan"', self.launch_source)
        self.assertIn('executable="pointcloud_to_laserscan_node"', self.launch_source)
        self.assertIn('executable="laserscan_to_pointcloud_node"', self.launch_source)
        self.assertIn('("cloud_in", "/cloud_registered_body")', self.launch_source)
        self.assertIn('("scan", "/a2w/scan")', self.launch_source)
        self.assertIn('("scan_in", "/a2w/scan")', self.launch_source)
        self.assertIn('("cloud", "/a2w/scan_cloud")', self.launch_source)
        self.assertIn('"target_frame": "body"', self.launch_source)
        self.assertIn('"use_inf": False', self.launch_source)
        self.assertIn('"inf_epsilon": -0.001', self.launch_source)
        self.assertIn(
            "<exec_depend>pointcloud_to_laserscan</exec_depend>",
            PACKAGE_PATH.read_text(encoding="utf-8"),
        )

    def test_higher_lateral_limit_is_only_for_isolated_diagnostic_mode(self):
        spec = importlib.util.spec_from_file_location("navigation_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for diagnostic, expected in (("false", .15), ("true", .30)):
            description = module.generate_launch_description()
            context = launch_context(description, {
                "axis_diagnostic_mode": diagnostic, "use_collision_monitor": "false",
                "use_sim_time": "false", "auto_enable_motion": "false",
                "motion_vx_min_effective": "0.0", "motion_vy_min_effective": "0.0",
                "motion_wz_min_effective": "0.0", "motion_gait_type": "0",
                "motion_speed_level": "0",
            })
            for name in ("localization_guard", "a2w_motion_bridge"):
                node = next(entity for entity in description.entities
                            if getattr(entity, "_Node__node_name", None) == name)
                params = evaluate_parameters(context, node._Node__parameters)[0]
                self.assertEqual(params["max_lateral"], expected)
                self.assertEqual(params["max_forward"], .60)
                self.assertEqual(params["max_yaw"], .5)
                if name == "localization_guard" and diagnostic == "true":
                    self.assertEqual(params["input_topic"], "/a2w/axis_test/cmd_vel")

    def test_never_starts_mapping_amcl_or_fake_tf(self):
        for forbidden in (
                "mapping.launch.py", "mapping_backend_node", "mapping_ingress_node",
                "nav2_amcl", "amcl", "static_transform_publisher", "go2", "livox"):
            self.assertNotIn(forbidden, self.launch_source.lower())

    def test_configured_speed_chain_passes_point_six_but_still_stops_when_degraded(self):
        """Catch a stale downstream cap without sending any ROS/Sport request."""
        import json
        import yaml
        from a2w_navigation.guard_policy import GuardPolicy, GuardPolicyConfig
        from a2w_motion_bridge.motion_policy import MotionPolicy, MotionPolicyConfig

        spec = importlib.util.spec_from_file_location("navigation_speed_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = launch_context(description, {
            "axis_diagnostic_mode": "false", "use_collision_monitor": "false",
            "use_sim_time": "false", "auto_enable_motion": "false",
            "motion_vx_min_effective": "0.0", "motion_vy_min_effective": "0.0",
            "motion_wz_min_effective": "0.0", "motion_gait_type": "0",
            "motion_speed_level": "0",
        })
        def config(name, config_type):
            node = next(entity for entity in description.entities
                        if getattr(entity, "_Node__node_name", None) == name)
            values = evaluate_parameters(context, node._Node__parameters)[0]
            return config_type(**{k: v for k, v in values.items()
                                  if k in config_type.__dataclass_fields__})

        guard = GuardPolicy(config("localization_guard", GuardPolicyConfig))
        bridge = MotionPolicy(config("a2w_motion_bridge", MotionPolicyConfig))
        guard.update_navigation_ready(True, .5)
        guard.update_status(guard.LOCALIZED, 1.)
        self.assertTrue(guard.arm(1.1))
        bridge.set_enabled(True)
        with (REPOSITORY_ROOT / 'src/a2w_navigation/config/nav2_params.yaml').open() as f:
            document = yaml.safe_load(f)
        follow = document['controller_server']['ros__parameters']['FollowPath']
        smoother = document['velocity_smoother']['ros__parameters']
        # The upstream configured saturation must not clip the requested 0.60.
        forward = min(.60, follow['vx_max'], smoother['max_velocity'][0])
        filtered = guard.filter_command(forward, .30, .80, 1.2)
        request = bridge.command(*filtered, now_s=1.2)
        self.assertEqual(request.api_id, 1008)
        self.assertEqual(json.loads(request.parameter), {'x': .60, 'y': .15, 'z': .50})
        request = bridge.command(*guard.filter_command(-.6, -.3, -.8, 1.2), now_s=1.2)
        self.assertEqual(json.loads(request.parameter), {'x': -.05, 'y': -.15, 'z': -.50})
        event = guard.update_status(guard.DEGRADED, 1.3)
        self.assertTrue(event.cancel_goal)
        request = bridge.command(*guard.filter_command(.60, 0., 0., 1.4), now_s=1.4)
        self.assertEqual(request.api_id, 1003)

    def test_robot_description_does_not_claim_fastlio_tf_ownership(self):
        self.assertNotIn('camera_init", "body', self.launch_source)
        self.assertNotIn('map", "camera_init', self.launch_source)

    def test_both_motion_gates_share_the_half_radian_yaw_limit(self):
        self.assertEqual(self.launch_source.count('"max_yaw": 0.50'), 2)

    def test_auto_enable_argument_controls_both_motion_gates(self):
        self.assertIn('"auto_arm_on_first_localized": ParameterValue(', self.launch_source)
        self.assertIn('"start_enabled": ParameterValue(', self.launch_source)

    def test_motion_bridge_uses_hand_control_profile_and_effective_minimums(self):
        spec = importlib.util.spec_from_file_location("navigation_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        bridge = next(entity for entity in description.entities
                      if getattr(entity, "_Node__node_name", None) == "a2w_motion_bridge")
        context = launch_context(description, {
            "use_sim_time": "false",
            "auto_enable_motion": "false",
            "motion_vx_min_effective": "0.04",
            "motion_vy_min_effective": "0.05",
            "motion_wz_min_effective": "0.08",
            "motion_gait_type": "0",
            "motion_speed_level": "0",
        })

        params = evaluate_parameters(context, bridge._Node__parameters)[0]

        self.assertEqual(params["min_effective_forward"], 0.04)
        self.assertEqual(params["min_effective_lateral"], 0.05)
        self.assertEqual(params["min_effective_yaw"], 0.08)
        self.assertEqual(params["gait_type"], 0)
        self.assertEqual(params["speed_level"], 0)

    def test_nav2_activation_waits_for_localization_and_runtime_summary_is_low_rate(self):
        self.assertIn('"navigation_autostart": "false"', self.launch_source)
        self.assertIn('executable="nav2_activation_gate"', self.launch_source)
        self.assertIn('executable="navigation_runtime_monitor"', self.launch_source)
        self.assertIn('"report_interval_s": 2.0', self.launch_source)

    def test_axis_diagnostic_mode_isolates_nav2_commands_but_keeps_guard(self):
        spec = importlib.util.spec_from_file_location("navigation_axis_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for diagnostic, collision, expected in (
                ("false", "false", "/cmd_vel"),
                ("false", "true", "/cmd_vel_safe"),
                ("true", "false", "/a2w/axis_test/cmd_vel"),
                ("true", "true", "/a2w/axis_test/cmd_vel")):
            description = module.generate_launch_description()
            guard = next(entity for entity in description.entities
                         if getattr(entity, "_Node__node_name", None) == "localization_guard")
            context = launch_context(description, {"use_sim_time": "false",
                "auto_enable_motion": "false", "axis_diagnostic_mode": diagnostic,
                "use_collision_monitor": collision})
            parameters = evaluate_parameters(context, guard._Node__parameters)[0]
            self.assertEqual(parameters["input_topic"], expected)
            self.assertEqual(parameters["output_topic"], "/cmd_vel_guarded")
            self.assertFalse(parameters["auto_arm_on_first_localized"])
        self.assertIn('"enabled": ParameterValue(autostart, value_type=bool)',
                      self.launch_source)

    def test_runner_defaults_to_ready_gated_live_navigation(self):
        self.assertIn('rviz:=true', self.script_source)
        self.assertIn('autostart:=true', self.script_source)
        self.assertIn('auto_enable_motion:=true', self.script_source)

    def test_localization_include_has_an_isolated_launch_configuration_scope(self):
        """Localization's rviz=false must not overwrite Nav2's rviz argument."""
        from launch.actions import GroupAction, IncludeLaunchDescription

        with tempfile.TemporaryDirectory() as log_dir, mock.patch.dict(
                os.environ, {"ROS_LOG_DIR": log_dir}):
            spec = importlib.util.spec_from_file_location(
                "a2w_navigation_composition_launch", LAUNCH_PATH)
            launch_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(launch_module)
            launch_description = launch_module.generate_launch_description()

        scoped_localization_includes = []
        outer_nav2_includes = []
        for entity in launch_description.entities:
            if isinstance(entity, GroupAction):
                for child in entity.get_sub_entities():
                    if isinstance(child, IncludeLaunchDescription):
                        argument_names = {name for name, _ in child.launch_arguments}
                        if "map_bundle_path" in argument_names:
                            scoped_localization_includes.append(child)
            elif isinstance(entity, IncludeLaunchDescription):
                argument_names = {name for name, _ in entity.launch_arguments}
                if "map" in argument_names and "params_file" in argument_names:
                    outer_nav2_includes.append(entity)

        self.assertEqual(len(scoped_localization_includes), 1)
        self.assertEqual(len(outer_nav2_includes), 1)

    def test_runner_resolves_and_validates_runtime_artifacts(self):
        self.assertIn('realpath -e -- "$1"', self.script_source)
        self.assertIn('manifest.sha256', self.script_source)
        self.assertIn('metadata.yaml', self.script_source)
        self.assertIn('navigation_map_yaml', self.script_source)
        self.assertIn('A2W_BRINGUP_LAUNCH=navigation.launch.py', self.script_source)
        self.assertNotIn('/home/' 'dndx', self.script_source)

    def test_runner_allows_optional_unitree_message_underlay(self):
        self.assertIn('A2W_UNITREE_ROS_SETUP', self.script_source)
        self.assertIn('unitree_ros_setup=${A2W_UNITREE_ROS_SETUP:-}', self.script_source)
        self.assertIn('if [[ -n $unitree_ros_setup ]]; then', self.script_source)
        self.assertNotIn('unitree_ros2/cyclonedds_ws/install/setup.bash', self.script_source)
        self.assertIn('source "$unitree_ros_setup"', self.script_source)

    def test_emergency_stop_script_disables_both_gates_and_cancels_goals(self):
        self.assertTrue(STOP_SCRIPT_PATH.is_file())
        stop_source = STOP_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn('/a2w_motion_bridge/enable', stop_source)
        self.assertIn('/navigation/enable_motion', stop_source)
        self.assertIn('/navigate_to_pose/_action/cancel_goal', stop_source)
        self.assertIn('std_srvs/srv/SetBool', stop_source)
        self.assertIn('action_msgs/srv/CancelGoal', stop_source)


if __name__ == "__main__":
    unittest.main()
