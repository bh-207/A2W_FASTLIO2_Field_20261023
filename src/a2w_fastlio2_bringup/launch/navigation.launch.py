"""Compose the A2W model, Localization, Nav2, and fail-closed motion gates."""

import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("a2w_fastlio2_bringup")
    description_share = get_package_share_directory("a2w_description")
    navigation_share = get_package_share_directory("a2w_navigation")

    robot_description_path = os.path.join(
        description_share, "urdf", "a2w_navigation.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as model_file:
        robot_description = model_file.read()

    map_bundle_path = LaunchConfiguration("map_bundle_path")
    navigation_map_yaml = LaunchConfiguration("navigation_map_yaml")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_enable_motion = LaunchConfiguration("auto_enable_motion")
    runtime_diagnostics = LaunchConfiguration("runtime_diagnostics")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    motion_vx_min_effective = LaunchConfiguration("motion_vx_min_effective")
    motion_vy_min_effective = LaunchConfiguration("motion_vy_min_effective")
    motion_wz_min_effective = LaunchConfiguration("motion_wz_min_effective")
    motion_gait_type = LaunchConfiguration("motion_gait_type")
    motion_speed_level = LaunchConfiguration("motion_speed_level")
    axis_diagnostic_mode = LaunchConfiguration("axis_diagnostic_mode", default="false")
    navigation_lateral_limit = LaunchConfiguration("navigation_lateral_limit")
    lateral_limit = ParameterValue(PythonExpression([
        "0.30 if '", axis_diagnostic_mode, "'.lower() in ('true', '1') else ",
        navigation_lateral_limit,
    ]), value_type=float)
    guarded_input = PythonExpression([
        "'/a2w/axis_test/cmd_vel' if '", axis_diagnostic_mode,
        "'.lower() in ('true', '1') else ('/cmd_vel_safe' if '", use_collision_monitor,
        "'.lower() in ('true', '1') else '/cmd_vel')",
    ])

    arguments = [
        DeclareLaunchArgument(
            "map_bundle_path",
            description="Absolute path to the Localization Map Bundle",
        ),
        DeclareLaunchArgument(
            "navigation_map_yaml",
            description="Absolute path to the matching Nav2 map YAML",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(navigation_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("auto_enable_motion", default_value="false"),
        DeclareLaunchArgument("start_motion_bridge", default_value="true",
                              description="Set false for remote-controlled waypoint recording; no Sport output."),
        DeclareLaunchArgument("runtime_diagnostics", default_value="true"),
        DeclareLaunchArgument("use_collision_monitor", default_value="true"),
        DeclareLaunchArgument("motion_vx_min_effective", default_value="0.0"),
        DeclareLaunchArgument("motion_vy_min_effective", default_value="0.0"),
        DeclareLaunchArgument("navigation_lateral_limit", default_value="0.15",
                              description="Final guard/bridge lateral cap; opt in to 0.30 for terminal vy compensation"),
        DeclareLaunchArgument("motion_wz_min_effective", default_value="0.0"),
        DeclareLaunchArgument("motion_vx_deadband", default_value="0.01"),
        DeclareLaunchArgument("motion_vy_deadband", default_value="0.01"),
        DeclareLaunchArgument("motion_wz_deadband", default_value="0.02"),
        DeclareLaunchArgument("axis_diagnostic_mode", default_value="false",
            description="Route only the opt-in axis probe through LocalizationGuard; Nav2 velocities cannot move the robot"),
        DeclareLaunchArgument("motion_gait_type", default_value="0"),
        DeclareLaunchArgument("motion_speed_level", default_value="0"),
    ]

    # TF ownership is intentionally split: Localization owns map->camera_init,
    # FAST-LIO owns camera_init->body, and this publisher owns only the URDF
    # descendants below body. It must never publish a parent transform for body.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    # Isolate the child launch configurations. In ROS 2 launch, an included
    # launch file otherwise leaves its arguments in the parent context; the
    # child's rviz=false would then overwrite the rviz=true intended for Nav2.
    localization = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "localization.launch.py")
            ),
            launch_arguments={
                "map_bundle_path": map_bundle_path,
                "rviz": "false",
                "use_sim_time": use_sim_time,
            }.items(),
        )],
    )

    # Preserve the proven Scan + ObstacleLayer clearing behavior while keeping
    # JT128 /cloud_registered_body as the only live obstacle sensor. Empty
    # angular bins become finite max-range rays for dynamic-obstacle clearing.
    jt128_scan_projection = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="jt128_pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            ("scan", "/a2w/scan"),
        ],
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "target_frame": "body",
            "transform_tolerance": 0.05,
            "min_height": -0.35,
            "max_height": 2.0,
            "angle_min": -3.141592653589793,
            "angle_max": 3.141592653589793,
            "angle_increment": 0.008726646259971648,
            "scan_time": 0.1,
            "range_min": 0.20,
            "range_max": 5.0,
            "use_inf": False,
            "inf_epsilon": -0.001,
        }],
    )

    # Humble's ObstacleLayer adds a fixed 50 ms TF tolerance only to its
    # LaserScan message filter. Convert the finite Scan back to PointCloud2 so
    # the layer receives the same 720 marking/clearing rays without that drop.
    jt128_scan_cloud_projection = Node(
        package="pointcloud_to_laserscan",
        executable="laserscan_to_pointcloud_node",
        name="jt128_laserscan_to_pointcloud",
        output="screen",
        remappings=[
            ("scan_in", "/a2w/scan"),
            ("cloud", "/a2w/scan_cloud"),
        ],
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "target_frame": "body",
            "transform_tolerance": 0.01,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "nav2_core.launch.py")
        ),
        launch_arguments={
            "map": navigation_map_yaml,
            "params_file": nav2_params_file,
            "rviz": rviz,
            "autostart": autostart,
            # Map server and Collision Monitor may start immediately, but the
            # navigation lifecycle waits for a confirmed Localization pose.
            "navigation_autostart": "false",
            "use_sim_time": use_sim_time,
            "use_collision_monitor": use_collision_monitor,
        }.items(),
    )

    localization_guard = Node(
        package="a2w_navigation",
        executable="localization_guard",
        name="localization_guard",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "localization_status_topic": "/localization/status",
            "nav2_ready_topic": "/navigation/nav2_ready",
            "input_topic": ParameterValue(guarded_input, value_type=str),
            "output_topic": "/cmd_vel_guarded",
            "status_timeout_s": 1.0,
            "command_keepalive_s": 0.1,
            "max_forward": 0.60,
            "max_reverse": 0.05,
            "max_lateral": lateral_limit,
            "max_yaw": 0.50,
            "auto_arm_on_first_localized": ParameterValue(
                auto_enable_motion, value_type=bool),
        }],
    )

    nav2_activation_gate = Node(
        package="a2w_navigation",
        executable="nav2_activation_gate",
        name="nav2_activation_gate",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "enabled": ParameterValue(autostart, value_type=bool),
            "localization_status_topic": "/localization/status",
            "localization_pose_topic": "/localization/pose",
            "lifecycle_manager_service":
                "/lifecycle_manager_navigation/manage_nodes",
            "retry_period_s": 0.5,
        }],
    )

    runtime_monitor = Node(
        package="a2w_navigation",
        executable="navigation_runtime_monitor",
        name="navigation_runtime_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "report_interval_s": 2.0,
            "use_collision_monitor": ParameterValue(use_collision_monitor, value_type=bool),
        }],
        condition=IfCondition(runtime_diagnostics),
    )

    # Even when prepared at launch, hardware motion remains blocked by the
    # LocalizationGuard until the first fresh LOCALIZED state arrives.
    motion_bridge = Node(
        package="a2w_motion_bridge",
        executable="motion_bridge",
        name="a2w_motion_bridge",
        condition=IfCondition(LaunchConfiguration("start_motion_bridge", default="true")),
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "input_topic": "/cmd_vel_guarded",
            "output_topic": "/api/sport/request",
            "max_forward": 0.60,
            "max_reverse": 0.05,
            "max_lateral": lateral_limit,
            "max_yaw": 0.50,
            "min_effective_forward": ParameterValue(
                motion_vx_min_effective, value_type=float),
            "min_effective_lateral": ParameterValue(
                motion_vy_min_effective, value_type=float),
            "min_effective_yaw": ParameterValue(
                motion_wz_min_effective, value_type=float),
            "deadband_forward": ParameterValue(LaunchConfiguration(
                "motion_vx_deadband", default="0.01"), value_type=float),
            "deadband_lateral": ParameterValue(LaunchConfiguration(
                "motion_vy_deadband", default="0.01"), value_type=float),
            "deadband_yaw": ParameterValue(LaunchConfiguration(
                "motion_wz_deadband", default="0.02"), value_type=float),
            "configure_motion_profile": True,
            "gait_type": ParameterValue(motion_gait_type, value_type=int),
            "speed_level": ParameterValue(motion_speed_level, value_type=int),
            "command_timeout_s": 0.30,
            "start_enabled": ParameterValue(auto_enable_motion, value_type=bool),
        }],
    )

    return LaunchDescription([
        *arguments,
        robot_state_publisher,
        localization,
        jt128_scan_projection,
        jt128_scan_cloud_projection,
        nav2,
        nav2_activation_gate,
        localization_guard,
        motion_bridge,
        runtime_monitor,
    ])
