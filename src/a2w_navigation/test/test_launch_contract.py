#!/usr/bin/env python3

import importlib.util
import pathlib
import re
import unittest
import xml.etree.ElementTree as ET

import yaml

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "nav2_core.launch.py"


class LaunchContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = LAUNCH_PATH.read_text(encoding="utf-8")
        compile(cls.source, str(LAUNCH_PATH), "exec")

    def test_declares_required_arguments_and_safe_defaults(self):
        for name in (
                "map", "params_file", "bt_xml", "rviz", "rviz_config",
                "autostart", "navigation_autostart", "use_sim_time"):
            self.assertRegex(
                self.source,
                rf'DeclareLaunchArgument\(\s*"{re.escape(name)}"',
            )
        self.assertIn('DeclareLaunchArgument("autostart", default_value="false")',
                      self.source)
        self.assertIn('DeclareLaunchArgument("rviz", default_value="false")', self.source)

    def test_navigation_lifecycle_can_be_deferred_without_delaying_map_or_collision(self):
        spec = importlib.util.spec_from_file_location("nav2_core_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        launch_description = module.generate_launch_description()

        context = LaunchContext()
        context.launch_configurations["autostart"] = "true"
        context.launch_configurations["navigation_autostart"] = "false"
        context.launch_configurations["use_sim_time"] = "false"

        map_manager = next(
            entity for entity in launch_description.entities
            if getattr(entity, "_Node__node_name", None)
            == "lifecycle_manager_map_server"
        )
        map_parameters = evaluate_parameters(
            context, map_manager._Node__parameters
        )[0]
        self.assertTrue(map_parameters["autostart"])

        all_entities = list(launch_description.entities)
        for entity in launch_description.entities:
            if isinstance(entity, GroupAction):
                all_entities.extend(entity._GroupAction__actions)
        navigation_include = next(
            entity for entity in all_entities
            if type(entity).__name__ == "IncludeLaunchDescription"
        )
        navigation_arguments = dict(navigation_include.launch_arguments)
        navigation_autostart = perform_substitutions(
            context, [navigation_arguments["autostart"]]
        )
        self.assertEqual("false", navigation_autostart)

    def test_nav2_child_launch_is_scoped_to_preserve_parent_autostart(self):
        spec = importlib.util.spec_from_file_location("nav2_core_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        launch_description = module.generate_launch_description()

        navigation_group = next(
            entity for entity in launch_description.entities
            if isinstance(entity, GroupAction)
            and any(
                isinstance(action, IncludeLaunchDescription)
                for action in entity._GroupAction__actions
            )
        )
        self.assertTrue(navigation_group._GroupAction__scoped)

    def test_collision_monitor_and_its_manager_follow_bypass_switch(self):
        spec = importlib.util.spec_from_file_location("nav2_core_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        for enabled in ("true", "false"):
            context = LaunchContext()
            context.launch_configurations["use_collision_monitor"] = enabled
            for name in ("collision_monitor", "lifecycle_manager_collision_monitor"):
                node = next(entity for entity in description.entities
                            if getattr(entity, "_Node__node_name", None) == name)
                self.assertIsNotNone(node.condition)
                self.assertEqual(node.condition.evaluate(context), enabled == "true")

    def test_starts_map_server_and_navigation_without_localization_or_mapping(self):
        self.assertIn('package="nav2_map_server"', self.source)
        self.assertIn('executable="map_server"', self.source)
        self.assertIn('package="nav2_lifecycle_manager"', self.source)
        self.assertIn('navigation_launch.py', self.source)
        self.assertIn('package="nav2_collision_monitor"', self.source)
        self.assertIn('executable="collision_monitor"', self.source)
        self.assertIn('name="collision_monitor"', self.source)
        self.assertIn('name="lifecycle_manager_collision_monitor"', self.source)
        self.assertIn('"node_names": ["collision_monitor"]', self.source)
        for forbidden in (
                "bringup_launch.py", "localization.launch.py", "mapping.launch.py",
                "fastlio_mapping", "mapping_backend_node", "localization_node",
                "nav2_amcl", "go2_driver", "livox_ros_driver2", "static_transform_publisher"):
            self.assertNotIn(forbidden, self.source)

    def test_uses_parameter_rewrites_and_an_explicit_collision_monitor_chain(self):
        self.assertIn('"yaml_filename": map_yaml', self.source)
        self.assertIn('"default_nav_to_pose_bt_xml": bt_xml', self.source)
        self.assertIn('"params_file": rewritten_params', self.source)
        self.assertNotIn("SetRemap", self.source)
        self.assertNotIn("('/odom'", self.source)

    def test_effective_navigator_tree_waits_before_motion_recovery(self):
        # Exercise launch argument resolution and the actual parameter rewrite:
        # an unused XML file must not make this test pass.
        spec = importlib.util.spec_from_file_location("nav2_core_launch", LAUNCH_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        context = LaunchContext()
        context.launch_configurations.update({
            "map": "/tmp/a2w_test_map.yaml",
            "params_file": str(PACKAGE_ROOT / "config" / "nav2_params.yaml"),
        })
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        navigation_group = next(
            entity for entity in description.entities if isinstance(entity, GroupAction)
        )
        include = next(action for action in navigation_group._GroupAction__actions
                       if isinstance(action, IncludeLaunchDescription))
        rewritten = dict(include.launch_arguments)["params_file"].perform(context)
        try:
            parameters = yaml.safe_load(pathlib.Path(rewritten).read_text())
        finally:
            pathlib.Path(rewritten).unlink()
        tree_path = pathlib.Path(parameters["bt_navigator"]["ros__parameters"]
                                 ["default_nav_to_pose_bt_xml"])
        tree = ET.parse(tree_path).getroot()
        recovery = tree.find(".//RoundRobin[@name='RecoveryActions']")
        self.assertEqual(recovery[0].tag, "Wait",
                         "temporary blockage must wait before Spin/BackUp")
        self.assertEqual(float(recovery[0].attrib["wait_duration"]), 5.0)
        self.assertEqual([node.tag for node in recovery],
                         ["Wait", "Sequence", "Spin", "BackUp"])
        self.assertEqual(tree.find(".//RateController").attrib["hz"], "1.0")
        self.assertEqual(tree.find(".//RecoveryNode[@name='NavigateRecovery']")
                         .attrib["number_of_retries"], "6")
        follow = tree.find(".//RecoveryNode[@name='FollowPath']")
        self.assertEqual(follow.attrib["number_of_retries"], "1")
        self.assertEqual([node.tag for node in follow],
                         ["FollowPath", "ClearEntireCostmap"])
        self.assertEqual(float(recovery.find("Spin").attrib["spin_dist"]), 1.57)
        self.assertEqual(float(recovery.find("BackUp").attrib["backup_dist"]), 0.30)

    def test_nav2_python_expression_receives_a_python_boolean_literal(self):
        self.assertIn('"use_composition": "False"', self.source)


if __name__ == "__main__":
    unittest.main()
