#!/usr/bin/env python3

import pathlib
import unittest

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "nav2_params.yaml"


def parameters(document, node, nested=None):
    value = document[node]
    if nested is not None:
        value = value[nested]
    return value["ros__parameters"]


class Nav2ConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with CONFIG_PATH.open("r", encoding="utf-8") as stream:
            cls.document = yaml.safe_load(stream)

    def test_frames_and_odometry_match_the_existing_a2w_contract(self):
        bt = parameters(self.document, "bt_navigator")
        controller = parameters(self.document, "controller_server")
        local = parameters(self.document, "local_costmap", "local_costmap")
        global_costmap = parameters(self.document, "global_costmap", "global_costmap")
        behavior = parameters(self.document, "behavior_server")
        self.assertEqual((bt["global_frame"], bt["robot_base_frame"], bt["odom_topic"]),
                         ("map", "base_link", "/Odometry"))
        self.assertEqual(bt["default_nav_to_pose_bt_xml"], "")
        self.assertEqual(controller["odom_topic"], "/Odometry")
        self.assertEqual((local["global_frame"], local["robot_base_frame"]),
                         ("camera_init", "base_link"))
        self.assertEqual((global_costmap["global_frame"],
                          global_costmap["robot_base_frame"]), ("map", "base_link"))
        self.assertEqual((behavior["global_frame"], behavior["robot_base_frame"]),
                         ("camera_init", "base_link"))

    def test_selected_plugins_are_smac_mppi_omni_and_simple_smoother(self):
        controller = parameters(self.document, "controller_server")
        planner = parameters(self.document, "planner_server")
        smoother = parameters(self.document, "smoother_server")
        self.assertEqual(controller["controller_plugins"], ["FollowPath"])
        self.assertEqual(controller["FollowPath"]["plugin"],
                         "nav2_mppi_controller::MPPIController")
        self.assertEqual(controller["FollowPath"]["motion_model"], "Omni")
        self.assertEqual(planner["GridBased"]["plugin"],
                         "nav2_smac_planner/SmacPlanner2D")
        self.assertEqual(smoother["simple_smoother"]["plugin"],
                         "nav2_smoother::SimpleSmoother")
        self.assertGreaterEqual(
            controller["controller_frequency"],
            1.0 / controller["FollowPath"]["model_dt"],
        )

    def test_mppi_restores_the_go2_sampling_and_critic_profile_at_a2w_limits(self):
        follow = parameters(self.document, "controller_server")["FollowPath"]
        self.assertEqual(follow["motion_model"], "Omni")
        self.assertEqual(follow["time_steps"], 36)
        self.assertEqual(follow["batch_size"], 1000)
        self.assertEqual(follow["iteration_count"], 2)
        self.assertEqual((follow["vx_std"], follow["vy_std"], follow["wz_std"]),
                         (0.20, 0.10, 0.30))
        self.assertEqual((follow["vx_max"], follow["vy_max"], follow["wz_max"]),
                         (0.60, 0.15, 0.50))
        self.assertEqual(follow["prune_distance"], 1.7)
        self.assertTrue(follow["visualize"])
        self.assertEqual((follow["trajectory_step"], follow["time_step"]), (5, 3))

        expected_critics = {
            "ConstraintCritic": (10.0, None),
            "GoalCritic": (5.0, 1.4),
            "GoalAngleCritic": (3.0, 0.5),
            "PreferForwardCritic": (5.0, 0.5),
            "CostCritic": (25.0, None),
            "PathAlignCritic": (4.0, 0.5),
            "PathFollowCritic": (5.0, 1.4),
            "PathAngleCritic": (2.0, 0.5),
        }
        self.assertEqual(set(follow["critics"]), set(expected_critics))
        for name, (weight, threshold) in expected_critics.items():
            with self.subTest(critic=name):
                critic = follow[name]
                self.assertTrue(critic["enabled"])
                self.assertEqual(critic["cost_power"], 1)
                self.assertEqual(critic["cost_weight"], weight)
                if threshold is not None:
                    self.assertEqual(critic["threshold_to_consider"], threshold)
        self.assertTrue(follow["CostCritic"]["consider_footprint"])
        self.assertEqual(follow["CostCritic"]["collision_cost"], 1000000.0)
        # Humble derives "reversing allowed" from vx_min; it is not a critic
        # parameter. Keep limited reverse recovery while preferring a turn.
        self.assertEqual(follow["vx_min"], -0.05)
        self.assertNotIn("reversing_allowed", follow["PathAngleCritic"])
        self.assertTrue(follow["PathAngleCritic"]["forward_preference"])
        self.assertNotIn("mode", follow["PathAngleCritic"])

    def test_turning_counts_as_progress_without_relaxing_stall_timeout(self):
        controller = parameters(self.document, "controller_server")
        checker_names = controller["progress_checker_plugins"]
        self.assertEqual(checker_names, ["progress_checker"])
        checker = controller[checker_names[0]]
        self.assertEqual(checker["plugin"], "nav2_controller::PoseProgressChecker")
        self.assertEqual(checker["required_movement_angle"], 0.15)
        self.assertEqual(checker["required_movement_radius"], 0.10)
        self.assertEqual(checker["movement_time_allowance"], 15.0)

    def test_path_align_can_activate_inside_current_local_path_horizon(self):
        follow = parameters(self.document, "controller_server")["FollowPath"]
        offset = follow["PathAlignCritic"]["offset_from_furthest"]
        self.assertEqual(offset, 5)
        self.assertLess(offset * 0.20, follow["prune_distance"])

    def test_transient_control_failure_uses_bounded_nav2_zero_command_retry(self):
        controller = parameters(self.document, "controller_server")
        # Native Nav2 controller retries invalid control while publishing zero,
        # but must not wait indefinitely or relax the progress watchdog.
        self.assertEqual(controller["failure_tolerance"], 1.0)
        self.assertLess(controller["failure_tolerance"],
                        controller["progress_checker"]["movement_time_allowance"])

    def test_normal_stance_rectangle_encloses_captured_geometry_with_one_cm_margin(self):
        controller = parameters(self.document, "controller_server")
        local = parameters(self.document, "local_costmap", "local_costmap")
        global_costmap = parameters(self.document, "global_costmap", "global_costmap")
        self.assertEqual(local["footprint"], global_costmap["footprint"])
        points = yaml.safe_load(local["footprint"])
        self.assertEqual(len(points), 4)
        xs, ys = {p[0] for p in points}, {p[1] for p in points}
        self.assertEqual((len(xs), len(ys)), (2, 2))
        self.assertEqual({tuple(p) for p in points}, {(x, y) for x in xs for y in ys})
        # Independent FK bounds of 30 real /lowstate samples, tick
        # 6522755..6528570; see A2W_MODEL_INTEGRATION.md. In particular, the
        # hypothetical +/-0.233 m rectangle must not cut through this stance.
        clearances = (
            -0.4877449328 - min(xs), max(xs) - 0.3872475922,
            -0.2375757166 - min(ys), max(ys) - 0.2377351196,
        )
        for clearance in clearances:
            self.assertGreaterEqual(clearance, 0.010)
            self.assertLessEqual(clearance, 0.011)  # outward millimetre rounding
        self.assertEqual(local["footprint_padding"], 0.0)
        self.assertEqual(global_costmap["footprint_padding"], 0.0)
        self.assertNotIn("robot_radius", local)
        self.assertNotIn("robot_radius", global_costmap)
        self.assertIn("CostCritic", controller["FollowPath"])
        self.assertTrue(controller["FollowPath"]["CostCritic"]["consider_footprint"])

    def test_compact_inflation_keeps_obstacle_layers_and_footprint_checks(self):
        local = parameters(self.document, "local_costmap", "local_costmap")
        global_costmap = parameters(self.document, "global_costmap", "global_costmap")
        # User-requested compact trial; do not shrink the physical polygon or
        # disable live obstacle marking/clearing to obtain a smaller band.
        self.assertEqual(local["inflation_layer"]["inflation_radius"], 0.30)
        self.assertEqual(global_costmap["inflation_layer"]["inflation_radius"], 0.25)
        self.assertEqual(local["inflation_layer"]["cost_scaling_factor"], 5.0)
        self.assertEqual(global_costmap["inflation_layer"]["cost_scaling_factor"], 5.0)
        self.assertIn("obstacle_layer", local["plugins"])
        self.assertIn("static_layer", global_costmap["plugins"])
        source = local["obstacle_layer"]["jt128_scan_cloud"]
        self.assertTrue(source["marking"] and source["clearing"])
        follow = parameters(self.document, "controller_server")["FollowPath"]
        self.assertTrue(follow["CostCritic"]["consider_footprint"])

    def test_local_costmap_uses_scan_derived_cloud_for_marking_and_clearing(self):
        local = parameters(self.document, "local_costmap", "local_costmap")
        self.assertIsInstance(local["width"], int)
        self.assertIsInstance(local["height"], int)
        self.assertEqual(local["plugins"], ["obstacle_layer", "inflation_layer"])
        obstacle = local["obstacle_layer"]
        self.assertEqual(obstacle["plugin"], "nav2_costmap_2d::ObstacleLayer")
        self.assertTrue(obstacle["footprint_clearing_enabled"])
        self.assertEqual(obstacle["observation_sources"], "jt128_scan_cloud")
        source = obstacle["jt128_scan_cloud"]
        self.assertEqual(source["topic"], "/a2w/scan_cloud")
        self.assertEqual(source["data_type"], "PointCloud2")
        self.assertTrue(source["marking"])
        self.assertTrue(source["clearing"])
        self.assertNotIn("inf_is_valid", source)
        self.assertEqual(source.get("min_obstacle_height"), -0.20)
        self.assertEqual(source.get("max_obstacle_height"), 1.50)
        self.assertLessEqual(source["obstacle_max_range"], 4.0)
        self.assertEqual(source["raytrace_max_range"], 5.0)
        self.assertEqual(local["update_frequency"], 10.0)
        self.assertEqual(local["publish_frequency"], 10.0)
        self.assertEqual(source["observation_persistence"], 0.0)
        text = CONFIG_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("livox", text)

    def test_global_planner_must_not_route_through_unknown_space(self):
        planner = parameters(self.document, "planner_server")["GridBased"]
        self.assertFalse(planner["allow_unknown"])

    def test_forward_speed_increase_preserves_other_omni_and_acceleration_limits(self):
        behavior = parameters(self.document, "behavior_server")
        follow = parameters(self.document, "controller_server")["FollowPath"]
        smoother = parameters(self.document, "velocity_smoother")
        self.assertEqual(follow["vx_max"], 0.60)
        self.assertEqual(follow["vy_max"], 0.15)
        self.assertEqual(follow["wz_max"], 0.50)
        self.assertEqual(follow["ax_max"], 0.25)
        self.assertEqual(follow["ax_min"], -0.25)
        self.assertEqual(follow["ay_max"], 0.20)
        self.assertEqual(follow["az_max"], 0.80)
        self.assertEqual(smoother["feedback"], "OPEN_LOOP")
        self.assertEqual(smoother["odom_topic"], "/Odometry")
        self.assertEqual(smoother["max_velocity"], [0.60, 0.15, 0.50])
        self.assertEqual(smoother["min_velocity"], [-0.05, -0.15, -0.50])
        self.assertEqual(smoother["max_accel"], [0.25, 0.20, 0.80])
        self.assertEqual(smoother["max_decel"], [-0.25, -0.20, -0.80])
        self.assertEqual(behavior["max_rotational_vel"], 0.50)
        self.assertEqual(behavior["rotational_acc_lim"], 0.80)

    def test_collision_monitor_is_the_last_nav2_safety_filter(self):
        monitor = parameters(self.document, "collision_monitor")
        self.assertEqual((monitor["base_frame_id"], monitor["odom_frame_id"]),
                         ("base_link", "camera_init"))
        self.assertEqual((monitor["cmd_vel_in_topic"], monitor["cmd_vel_out_topic"]),
                         ("/cmd_vel", "/cmd_vel_safe"))
        self.assertFalse(monitor["base_shift_correction"])
        self.assertEqual(monitor["source_timeout"], 0.50)
        self.assertEqual(monitor["observation_sources"], ["jt128"])
        self.assertEqual(monitor["jt128"]["type"], "scan")
        self.assertEqual(monitor["jt128"]["topic"], "/a2w/scan")
        self.assertTrue(monitor["jt128"]["enabled"])
        self.assertEqual(monitor["polygons"], ["SlowZone", "StopZone"])
        self.assertEqual(monitor["SlowZone"]["action_type"], "slowdown")
        self.assertEqual(monitor["SlowZone"]["radius"], 1.4)
        self.assertEqual(monitor["SlowZone"]["max_points"], 3)
        self.assertEqual(monitor["StopZone"]["action_type"], "stop")
        self.assertEqual(monitor["StopZone"]["radius"], 0.6)
        self.assertEqual(monitor["StopZone"]["max_points"], 3)

    def test_amcl_and_unpublished_odometry_frames_are_absent(self):
        source = CONFIG_PATH.read_text(encoding="utf-8")
        self.assertNotIn("amcl:", source.lower())
        self.assertNotIn("base_footprint", source)
        self.assertNotIn("livox_frame", source)
        self.assertNotIn("global_frame: odom", source)
        self.assertNotIn("/localization/odom", source)

    def test_behavior_tree_uses_only_standard_single_goal_nodes(self):
        import xml.etree.ElementTree as ET
        path = PACKAGE_ROOT / "behavior_trees" / "navigate_to_pose_guarded.xml"
        root = ET.parse(path).getroot()
        self.assertEqual(root.attrib["main_tree_to_execute"], "MainTree")
        tags = {element.tag for element in root.iter()}
        self.assertTrue({
            "ComputePathToPose", "FollowPath", "ClearEntireCostmap",
            "GoalUpdated", "Spin", "Wait", "BackUp",
        } <= tags)
        self.assertNotIn("NavigateThroughPoses", tags)
        source = path.read_text(encoding="utf-8")
        self.assertIn('planner_id="GridBased"', source)
        self.assertIn('controller_id="FollowPath"', source)

    def test_rviz_uses_a2w_frames_and_topics(self):
        path = PACKAGE_ROOT / "rviz" / "navigation.rviz"
        source = path.read_text(encoding="utf-8")
        self.assertIn("Fixed Frame: map", source)
        self.assertIn("Value: /cloud_registered_body", source)
        self.assertIn("Value: /localization/path", source)
        self.assertIn("Value: /global_costmap/costmap", source)
        self.assertIn("Value: /local_costmap/costmap", source)
        self.assertGreaterEqual(source.count("Color Scheme: costmap"), 2)
        self.assertIn("Class: rviz_default_plugins/RobotModel", source)
        self.assertIn("Value: /robot_description", source)
        self.assertIn("Value: /plan", source)
        self.assertIn("Value: /transformed_global_plan", source)
        self.assertIn("Value: /trajectories", source)
        self.assertIn("Value: /local_costmap/published_footprint", source)
        self.assertIn("Value: /global_costmap/published_footprint", source)
        self.assertNotIn("/scan", source)
        self.assertNotIn("base_link", source)
        self.assertNotIn("livox", source.lower())
        self.assertNotIn("particle_cloud", source)


if __name__ == "__main__":
    unittest.main()
