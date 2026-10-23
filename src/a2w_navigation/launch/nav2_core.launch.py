import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    navigation_share = get_package_share_directory("a2w_navigation")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    bt_xml = LaunchConfiguration("bt_xml")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    navigation_autostart = LaunchConfiguration("navigation_autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
            "default_nav_to_pose_bt_xml": bt_xml,
        },
        convert_types=True,
    )
    configured_params = ParameterFile(rewritten_params, allow_substs=True)

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Absolute navigation map YAML path"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(navigation_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "bt_xml",
            default_value=os.path.join(
                navigation_share, "behavior_trees",
                "navigate_to_pose_wait_first.xml"),
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(navigation_share, "rviz", "navigation.rviz"),
        ),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("navigation_autostart", default_value=autostart),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_collision_monitor", default_value="true"),
        LogInfo(msg=["Nav2 单点导航行为树：", bt_xml,
                     "（直接 FollowPath 采样路线不经过此树）"]),
        LogInfo(
            msg="Collision Monitor 已旁路：速度由 LocalizationGuard 限制，独立碰撞停车层未启用。",
            condition=UnlessCondition(use_collision_monitor),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }],
        ),
        GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": navigation_autostart,
                    "params_file": rewritten_params,
                    # Humble's Nav2 launch evaluates this with PythonExpression
                    # ("not <value>"), so it must be a Python boolean literal.
                    "use_composition": "False",
                    "use_respawn": "false",
                }.items(),
            )],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[configured_params],
            condition=IfCondition(use_collision_monitor),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["collision_monitor"],
            }],
            condition=IfCondition(use_collision_monitor),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="a2w_navigation_rviz",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(rviz),
        ),
    ])
