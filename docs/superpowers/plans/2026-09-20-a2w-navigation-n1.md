# A2W Navigation Phase N1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create an installable, offline-verified `a2w_navigation` ROS 2 package containing an A2W-native Nav2 skeleton without starting Localization, Mapping, or any robot command transport.

**Architecture:** Phase N1 owns only standard Nav2 composition assets: package metadata, one conservative parameter profile, one project-owned NavigateToPose behavior tree, one RViz profile, and one core launch file. The launch starts Nav2 navigation and map-server processes with `autostart:=false`; later phases validate robot geometry, add a navigation map generator, connect JT128 obstacles, and implement localization/motion safety gates.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, ament_cmake, Python 3, launch/launch_ros, Nav2 SmacPlanner2D, Nav2 MPPI, PyYAML, unittest/ament_cmake_pytest.

**Spec:** `docs/superpowers/specs/2026-09-20-a2w-navigation-design.md`

## Global Constraints

- Do not modify `src/FAST_LIO_Hesai`, `src/a2w_fastlio2_bringup/config/jt128.yaml`, `src/a2w_fastlio_mapping`, `src/a2w_fastlio_localization`, `src/a2w_fastlio_map`, or `maps`.
- Do not copy `robot_navigation2`, its maps, Go2/Livox drivers, AMCL/ICP, fake TF broadcasters, Go2 footprint values, or Go2 dynamics values.
- Preserve the only valid frame chain: `map -> camera_init -> body`.
- Use `/Odometry` only as the declared local odometry Topic; do not fabricate its missing twist.
- Do not connect `/cmd_vel`, `/cmd_vel_smoothed`, or another velocity Topic to a robot consumer in N1.
- Do not configure a live obstacle source in N1; direct `/cloud_registered_body` integration belongs to N4.
- Do not invent an A2W footprint in N1; footprint evidence belongs to N2.
- Default `autostart` must be `false` because LocalizationGuard does not exist until N6.
- Every production file is preceded by a test that fails for the expected missing behavior.
- End N1 with local commits only. Do not push or modify a remote branch without a new explicit request.
- Report hardware status exactly as: `Offline implementation and verification complete; JT128 hardware validation pending.`

## File structure

```text
src/a2w_navigation/
├── CMakeLists.txt                         # package install and offline test registration
├── package.xml                           # explicit Nav2 runtime dependencies
├── launch/
│   └── nav2_core.launch.py              # map server + standard navigation, safe inactive default
├── config/
│   └── nav2_params.yaml                 # map/camera_init/body Nav2 skeleton
├── behavior_trees/
│   └── navigate_to_pose_guarded.xml     # project-owned standard NavigateToPose tree
├── rviz/
│   └── navigation.rviz                  # map/TF/cloud/path/costmap displays
└── test/
    ├── test_package_contract.py          # metadata, dependencies, forbidden coupling, docs
    ├── test_nav2_configuration.py        # YAML, frame, plugin, safety boundaries, BT/RViz
    └── test_launch_contract.py            # arguments, safe defaults, included processes

docs/NAVIGATION_DEVELOPMENT.md                 # phase status and non-runtime usage boundary
tests/run_tests.sh                              # invokes the new repository-level contract suite
tests/test_navigation_contract.py               # package graph and protected boundary checks
README.md                                       # links to the navigation development status
```

## Review Focus

- A launch invoked without `autostart:=true` must not activate Nav2 or create a robot command path; Task 4 tests the declared default and absence of motion packages.
- A config accidentally reintroducing `odom`, `base_link`, `livox_frame`, `/scan`, or AMCL must fail; Task 2 recursively checks frame/topic/plugin values.
- A future dependency edit that makes navigation depend on Mapping/Localization/Map must fail; Tasks 1 and 5 assert the internal dependency graph.
- A copied Go2 footprint or dynamics profile must not hide in YAML; Task 2 asserts no footprint/radius and caps every provisional velocity/acceleration value.
- A launch-level broad remap such as `/odom -> /Odometry` could corrupt unrelated publishers; Task 4 rejects `SetRemap` and requires node-local parameters only.

---

### Task 1: Package metadata and dependency boundary

**Files:**

- Create: `src/a2w_navigation/test/test_package_contract.py`
- Create: `src/a2w_navigation/package.xml`
- Create: `src/a2w_navigation/CMakeLists.txt`

**Interfaces:**

- Consumes: ROS 2 Humble package index and `a2w_fastlio_msgs` as the only internal dependency.
- Produces: an ament package named `a2w_navigation` with explicit standard Nav2 runtime dependencies and pytest registration.

- [ ] **Step 1: Write the failing package contract test**

Create `src/a2w_navigation/test/test_package_contract.py`:

```python
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
            "nav2_velocity_smoother", "nav2_waypoint_follower", "rviz2",
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


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run RED and confirm the missing manifest is the reason**

Run:

```bash
/usr/bin/python3 -m unittest \
  src/a2w_navigation/test/test_package_contract.py -v
```

Expected: ERROR with `FileNotFoundError` for `src/a2w_navigation/package.xml`, proving the package does not yet exist.

- [ ] **Step 3: Add the minimal package manifest**

Create `src/a2w_navigation/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>a2w_navigation</name>
  <version>0.1.0</version>
  <description>A2W-native ROS 2 Humble Nav2 configuration and safety orchestration.</description>
  <maintainer email="AaronYanC@users.noreply.github.com">AaronYanC</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>a2w_fastlio_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_common</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_costmap_2d</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_mppi_controller</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_smac_planner</exec_depend>
  <exec_depend>nav2_smoother</exec_depend>
  <exec_depend>nav2_behaviors</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>nav2_velocity_smoother</exec_depend>
  <exec_depend>nav2_waypoint_follower</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Add the minimal CMake test registration**

Create `src/a2w_navigation/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(a2w_navigation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_package_contract test/test_package_contract.py)
endif()

ament_package()
```

- [ ] **Step 5: Run GREEN**

Run the Step 2 command. Expected: 2 tests pass.

- [ ] **Step 6: Commit the package boundary**

```bash
git add src/a2w_navigation/CMakeLists.txt \
  src/a2w_navigation/package.xml \
  src/a2w_navigation/test/test_package_contract.py
git commit -m "build(navigation): add A2W Nav2 package boundary"
```

### Task 2: Conservative Nav2 configuration contract

**Files:**

- Create: `src/a2w_navigation/test/test_nav2_configuration.py`
- Create: `src/a2w_navigation/config/nav2_params.yaml`
- Modify: `src/a2w_navigation/CMakeLists.txt`

**Interfaces:**

- Consumes: established frames `map`, `camera_init`, `body` and odometry Topic `/Odometry`.
- Produces: node parameters for Map Server, BT Navigator, SmacPlanner2D, MPPI Omni, SimpleSmoother, standard behaviors, waypoint follower, velocity smoother, and both costmaps.
- Does not produce: footprint, live obstacle source, collision monitor, localization guard, or robot command transport.

- [ ] **Step 1: Write failing YAML contract tests**

Create `src/a2w_navigation/test/test_nav2_configuration.py`:

```python
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
                         ("map", "body", "/Odometry"))
        self.assertEqual(bt["default_nav_to_pose_bt_xml"], "")
        self.assertEqual(controller["odom_topic"], "/Odometry")
        self.assertEqual((local["global_frame"], local["robot_base_frame"]),
                         ("camera_init", "body"))
        self.assertEqual((global_costmap["global_frame"],
                          global_costmap["robot_base_frame"]), ("map", "body"))
        self.assertEqual((behavior["global_frame"], behavior["robot_base_frame"]),
                         ("camera_init", "body"))

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

    def test_n1_does_not_invent_footprint_or_live_obstacle_input(self):
        local = parameters(self.document, "local_costmap", "local_costmap")
        global_costmap = parameters(self.document, "global_costmap", "global_costmap")
        for costmap in (local, global_costmap):
            self.assertNotIn("footprint", costmap)
            self.assertNotIn("robot_radius", costmap)
            self.assertNotIn("observation_sources", costmap)
        self.assertEqual(local["plugins"], ["inflation_layer"])
        self.assertEqual(global_costmap["plugins"], ["static_layer", "inflation_layer"])
        source = CONFIG_PATH.read_text(encoding="utf-8")
        self.assertNotIn("/scan", source)
        self.assertNotIn("collision_monitor", source)

    def test_provisional_motion_limits_are_low_and_open_loop(self):
        follow = parameters(self.document, "controller_server")["FollowPath"]
        smoother = parameters(self.document, "velocity_smoother")
        self.assertLessEqual(follow["vx_max"], 0.20)
        self.assertLessEqual(follow["vy_max"], 0.10)
        self.assertLessEqual(follow["wz_max"], 0.30)
        self.assertLessEqual(follow["ax_max"], 0.25)
        self.assertLessEqual(follow["ay_max"], 0.20)
        self.assertEqual(smoother["feedback"], "OPEN_LOOP")
        self.assertEqual(smoother["odom_topic"], "/Odometry")
        self.assertEqual(smoother["max_velocity"], [0.20, 0.10, 0.30])

    def test_amcl_and_legacy_frames_are_absent(self):
        source = CONFIG_PATH.read_text(encoding="utf-8")
        self.assertNotIn("amcl:", source.lower())
        self.assertNotIn("base_link", source)
        self.assertNotIn("base_footprint", source)
        self.assertNotIn("livox_frame", source)
        self.assertNotIn("global_frame: odom", source)
        self.assertNotIn("/localization/odom", source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run RED and confirm the missing YAML is the reason**

```bash
/usr/bin/python3 -m unittest \
  src/a2w_navigation/test/test_nav2_configuration.py -v
```

Expected: ERROR with `FileNotFoundError` for `config/nav2_params.yaml`.

- [ ] **Step 3: Add the conservative Nav2 profile**

Create `src/a2w_navigation/config/nav2_params.yaml`:

```yaml
map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: ""

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: body
    odom_topic: /Odometry
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: ""

controller_server:
  ros__parameters:
    use_sim_time: false
    odom_topic: /Odometry
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.0
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: nav2_controller::SimpleProgressChecker
      required_movement_radius: 0.20
      movement_time_allowance: 15.0
    goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      stateful: true
      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.20
    FollowPath:
      plugin: nav2_mppi_controller::MPPIController
      motion_model: Omni
      time_steps: 32
      model_dt: 0.05
      batch_size: 500
      iteration_count: 1
      vx_std: 0.10
      vy_std: 0.05
      wz_std: 0.10
      vx_max: 0.20
      vx_min: -0.10
      vy_max: 0.10
      wz_max: 0.30
      ax_max: 0.25
      ax_min: -0.25
      ay_max: 0.20
      az_max: 0.40
      prune_distance: 1.0
      transform_tolerance: 0.20
      temperature: 0.30
      gamma: 0.015
      visualize: false
      regenerate_noises: true
      critics:
        - ConstraintCritic
        - CostCritic
        - GoalCritic
        - GoalAngleCritic
        - PathAlignCritic
        - PathFollowCritic
        - PathAngleCritic
        - PreferForwardCritic

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: camera_init
      robot_base_frame: body
      rolling_window: true
      width: 6.0
      height: 6.0
      resolution: 0.05
      update_frequency: 5.0
      publish_frequency: 2.0
      transform_tolerance: 0.20
      plugins: ["inflation_layer"]
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.50
        cost_scaling_factor: 5.0
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: map
      robot_base_frame: body
      resolution: 0.05
      update_frequency: 1.0
      publish_frequency: 1.0
      track_unknown_space: true
      transform_tolerance: 0.20
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.50
        cost_scaling_factor: 5.0
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: nav2_smac_planner/SmacPlanner2D
      tolerance: 0.20
      downsampling_factor: 1
      allow_unknown: true
      cost_travel_multiplier: 2.0
      smooth_path: true
      motion_model_for_search: 2D

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: nav2_smoother::SimpleSmoother
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    spin:
      plugin: nav2_behaviors/Spin
    backup:
      plugin: nav2_behaviors/BackUp
    drive_on_heading:
      plugin: nav2_behaviors/DriveOnHeading
    wait:
      plugin: nav2_behaviors/Wait
    assisted_teleop:
      plugin: nav2_behaviors/AssistedTeleop
    global_frame: camera_init
    robot_base_frame: body
    transform_tolerance: 0.20
    simulate_ahead_time: 1.0
    max_rotational_vel: 0.30
    min_rotational_vel: 0.05
    rotational_acc_lim: 0.40

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 10
    stop_on_failure: true
    waypoint_task_executor_plugin: wait_at_waypoint
    wait_at_waypoint:
      plugin: nav2_waypoint_follower::WaitAtWaypoint
      enabled: true
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: OPEN_LOOP
    max_velocity: [0.20, 0.10, 0.30]
    min_velocity: [-0.10, -0.10, -0.30]
    max_accel: [0.25, 0.20, 0.40]
    max_decel: [-0.25, -0.20, -0.40]
    odom_topic: /Odometry
    odom_duration: 0.10
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 0.50
```

- [ ] **Step 4: Register and install the configuration**

Add this before the existing test block in `src/a2w_navigation/CMakeLists.txt`:

```cmake
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
```

Keep one `if(BUILD_TESTING)` block and replace it with this exact content:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_package_contract test/test_package_contract.py)
  ament_add_pytest_test(test_nav2_configuration test/test_nav2_configuration.py)
endif()
```

- [ ] **Step 5: Run GREEN**

Run the Step 2 command. Expected: 5 tests pass.

- [ ] **Step 6: Commit the configuration contract**

```bash
git add src/a2w_navigation/CMakeLists.txt \
  src/a2w_navigation/config/nav2_params.yaml \
  src/a2w_navigation/test/test_nav2_configuration.py
git commit -m "feat(navigation): add conservative Nav2 profile"
```

### Task 3: Project-owned behavior tree and RViz profile

**Files:**

- Modify: `src/a2w_navigation/test/test_nav2_configuration.py`
- Create: `src/a2w_navigation/behavior_trees/navigate_to_pose_guarded.xml`
- Create: `src/a2w_navigation/rviz/navigation.rviz`
- Modify: `src/a2w_navigation/CMakeLists.txt`

**Interfaces:**

- Consumes: planner ID `GridBased`, controller ID `FollowPath`, global/local costmap service names.
- Produces: `MainTree` NavigateToPose BT and an RViz view fixed in `map`.
- Guard semantics: the external LocalizationGuard added in N6 controls admission and command output; the N1 XML contains only standard Humble BT nodes.

- [ ] **Step 1: Extend the configuration test before adding assets**

Append these methods to `Nav2ConfigurationTests`:

```python
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
        self.assertNotIn("/scan", source)
        self.assertNotIn("base_link", source)
        self.assertNotIn("livox", source.lower())
        self.assertNotIn("particle_cloud", source)
```

- [ ] **Step 2: Run RED and confirm both asset paths are missing**

Run the Task 2 Step 2 command. Expected: two errors for missing BT/RViz files while the existing five tests remain green.

- [ ] **Step 3: Add the standard single-goal behavior tree**

Create `src/a2w_navigation/behavior_trees/navigate_to_pose_guarded.xml`:

```xml
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmapContext"
              service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmapContext"
            service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearCostmaps">
            <ClearEntireCostmap name="ClearLocalCostmap"
              service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap"
              service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="0.52"/>
          <Wait wait_duration="2"/>
          <BackUp backup_dist="0.10" backup_speed="0.03"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

- [ ] **Step 4: Add the minimal A2W RViz profile**

Create `src/a2w_navigation/rviz/navigation.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Plane: XY
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Navigation Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        Value: /map
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: JT128 Body Cloud
      Size (Pixels): 2
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
        Value: /cloud_registered_body
    - Class: rviz_default_plugins/Path
      Enabled: true
      Name: Localization Path
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        Value: /localization/path
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Global Costmap
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        Value: /global_costmap/costmap
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Local Costmap
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Reliability Policy: Reliable
        Value: /local_costmap/costmap
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/Measure
    - Class: nav2_rviz_plugins/GoalTool
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 20
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 1.0
      Yaw: 0.8
Window Geometry:
  Height: 900
  Width: 1440
```

- [ ] **Step 5: Install both asset directories**

Replace the Task 2 install line in `CMakeLists.txt` with:

```cmake
install(
  DIRECTORY behavior_trees config rviz
  DESTINATION share/${PROJECT_NAME}
)
```

- [ ] **Step 6: Run GREEN**

Run the Task 2 Step 2 command. Expected: 7 tests pass.

- [ ] **Step 7: Commit the project-owned assets**

```bash
git add src/a2w_navigation/CMakeLists.txt \
  src/a2w_navigation/behavior_trees/navigate_to_pose_guarded.xml \
  src/a2w_navigation/rviz/navigation.rviz \
  src/a2w_navigation/test/test_nav2_configuration.py
git commit -m "feat(navigation): add BT and RViz assets"
```

### Task 4: Safe inactive Nav2 core launch

**Files:**

- Create: `src/a2w_navigation/test/test_launch_contract.py`
- Create: `src/a2w_navigation/launch/nav2_core.launch.py`
- Modify: `src/a2w_navigation/CMakeLists.txt`

**Interfaces:**

- Launch arguments: `map`, `params_file`, `bt_xml`, `rviz`, `rviz_config`, `autostart`, `use_sim_time`.
- Starts: `nav2_map_server/map_server`, a map-server lifecycle manager, standard Humble `navigation_launch.py`, and optional RViz.
- Does not start: FAST-LIO, Mapping, Localization, AMCL, collision monitor, robot driver, motion bridge, or a TF publisher.
- Safety default: `autostart:=false` and no broad `SetRemap` action.

- [ ] **Step 1: Write the failing launch contract test**

Create `src/a2w_navigation/test/test_launch_contract.py`:

```python
#!/usr/bin/env python3

import pathlib
import unittest


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
                "autostart", "use_sim_time"):
            self.assertIn(f'DeclareLaunchArgument("{name}"', self.source)
        self.assertIn('DeclareLaunchArgument("autostart", default_value="false")',
                      self.source)
        self.assertIn('DeclareLaunchArgument("rviz", default_value="false")', self.source)

    def test_starts_map_server_and_navigation_without_localization_or_mapping(self):
        self.assertIn('package="nav2_map_server"', self.source)
        self.assertIn('executable="map_server"', self.source)
        self.assertIn('package="nav2_lifecycle_manager"', self.source)
        self.assertIn('navigation_launch.py', self.source)
        for forbidden in (
                "bringup_launch.py", "localization.launch.py", "mapping.launch.py",
                "fastlio_mapping", "mapping_backend_node", "localization_node",
                "nav2_amcl", "go2_driver", "livox_ros_driver2", "static_transform_publisher"):
            self.assertNotIn(forbidden, self.source)

    def test_uses_parameter_rewrites_not_broad_topic_remaps(self):
        self.assertIn('"yaml_filename": map_yaml', self.source)
        self.assertIn('"default_nav_to_pose_bt_xml": bt_xml', self.source)
        self.assertIn('"params_file": rewritten_params', self.source)
        self.assertNotIn("SetRemap", self.source)
        self.assertNotIn("/cmd_vel_safe", self.source)
        self.assertNotIn("('/odom'", self.source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run RED and confirm the launch file is missing**

```bash
/usr/bin/python3 -m unittest \
  src/a2w_navigation/test/test_launch_contract.py -v
```

Expected: ERROR with `FileNotFoundError` for `launch/nav2_core.launch.py`.

- [ ] **Step 3: Implement the minimal safe launch**

Create `src/a2w_navigation/launch/nav2_core.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    use_sim_time = LaunchConfiguration("use_sim_time")

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
                navigation_share, "behavior_trees", "navigate_to_pose_guarded.xml"),
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(navigation_share, "rviz", "navigation.rviz"),
        ),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": rewritten_params,
                "use_composition": "false",
                "use_respawn": "false",
            }.items(),
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
```

- [ ] **Step 4: Register and install the launch contract**

Change the CMake install list to:

```cmake
install(
  DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
```

Add inside the existing `if(BUILD_TESTING)` block:

```cmake
ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
```

- [ ] **Step 5: Run GREEN and syntax-check with ROS Python**

```bash
/usr/bin/python3 -m unittest \
  src/a2w_navigation/test/test_launch_contract.py -v
/usr/bin/python3 -m py_compile \
  src/a2w_navigation/launch/nav2_core.launch.py
```

Expected: 3 tests pass and `py_compile` exits 0.

- [ ] **Step 6: Commit the launch boundary**

```bash
git add src/a2w_navigation/CMakeLists.txt \
  src/a2w_navigation/launch/nav2_core.launch.py \
  src/a2w_navigation/test/test_launch_contract.py
git commit -m "feat(navigation): add safe inactive Nav2 launch"
```

### Task 5: Repository contracts and N1 documentation

**Files:**

- Create: `tests/test_navigation_contract.py`
- Modify: `tests/run_tests.sh`
- Create: `docs/NAVIGATION_DEVELOPMENT.md`
- Modify: `README.md`
- Modify: `src/a2w_navigation/test/test_package_contract.py`

**Interfaces:**

- Produces: repository-wide dependency/protected-boundary checks and operator-facing N1 status.
- Documents: N1 is not a runnable A2W navigation system and accepts no hardware validation claim.

- [ ] **Step 1: Add failing repository and documentation assertions**

Create `tests/test_navigation_contract.py`:

```python
#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src" / "a2w_navigation"


class NavigationRepositoryContractTests(unittest.TestCase):
    def test_navigation_has_only_msgs_as_an_internal_dependency(self):
        packages = {}
        for package_xml in (REPOSITORY_ROOT / "src").glob("*/package.xml"):
            root = ET.parse(package_xml).getroot()
            packages[root.findtext("name")] = {
                element.text
                for tag in ("depend", "build_depend", "exec_depend")
                for element in root.findall(tag)
                if element.text
            }
        internal = set(packages)
        self.assertEqual(packages["a2w_navigation"] & internal, {"a2w_fastlio_msgs"})

    def test_navigation_tree_contains_no_legacy_or_machine_paths(self):
        forbidden = (
            "/home/" "dndx", "Go2_Nav", "go2_driver", "go2_description",
            "livox_ros_driver2", "livox_frame", "base_link", "base_footprint",
            "icp_registration", "nav2_amcl",
        )
        violations = []
        for path in PACKAGE_ROOT.rglob("*"):
            if (path.is_file() and "test" not in path.parts and
                    "__pycache__" not in path.parts):
                text = path.read_text(encoding="utf-8")
                for token in forbidden:
                    if token in text:
                        violations.append(f"{path.relative_to(REPOSITORY_ROOT)}:{token}")
        self.assertEqual(violations, [])

    def test_navigation_does_not_modify_existing_slam_packages(self):
        launch_source = (PACKAGE_ROOT / "launch" / "nav2_core.launch.py").read_text(
            encoding="utf-8")
        self.assertNotIn("a2w_fastlio_mapping", launch_source)
        self.assertNotIn("a2w_fastlio_localization", launch_source)
        self.assertNotIn("fast_lio", launch_source)


if __name__ == "__main__":
    unittest.main()
```

Append to `PackageContractTests` in `src/a2w_navigation/test/test_package_contract.py`:

```python
    def test_n1_documentation_marks_runtime_and_hardware_as_pending(self):
        repository_root = PACKAGE_ROOT.parents[1]
        guide = (repository_root / "docs" / "NAVIGATION_DEVELOPMENT.md").read_text(
            encoding="utf-8")
        self.assertIn("Phase N1", guide)
        self.assertIn("autostart:=false", guide)
        self.assertIn("not connected to an A2W motion interface", guide)
        self.assertIn(
            "Offline implementation and verification complete; "
            "JT128 hardware validation pending.", guide)
```

- [ ] **Step 2: Run RED**

```bash
/usr/bin/python3 -m unittest tests/test_navigation_contract.py -v
/usr/bin/python3 -m unittest \
  src/a2w_navigation/test/test_package_contract.py -v
```

Expected: repository contract tests pass against the package, while the package documentation test fails because `docs/NAVIGATION_DEVELOPMENT.md` does not exist.

- [ ] **Step 3: Add the N1 development guide**

Create `docs/NAVIGATION_DEVELOPMENT.md`:

````markdown
# A2W Navigation Development Status

## Phase N1

Phase N1 installs the `a2w_navigation` package with an A2W-native Nav2 configuration, behavior
tree, RViz profile, and `nav2_core.launch.py`. It is an offline configuration milestone, not a
runnable robot-navigation release.

The launch defaults to `autostart:=false`. It does not start FAST-LIO, Mapping, Localization,
AMCL, a robot driver, LocalizationGuard, collision monitoring, or a motion bridge. Its command
output is not connected to an A2W motion interface.

The following gates remain:

1. N2 validates frame, odometry, twist, and A2W footprint contracts.
2. N3 creates the Map Bundle to 2D navigation-map converter.
3. N4 connects `/cloud_registered_body` to the local costmap.
4. N5 verifies or implements the A2W command interface.
5. N6 adds LocalizationGuard and command stopping.
6. N7 adds integrated Navigation bringup.
7. N8 completes offline integration verification.
8. N9 performs controlled low-speed hardware acceptance.

The N1 launch is available only for configuration inspection after building:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2w_navigation nav2_core.launch.py \
  map:=/absolute/path/to/a/navigation/map.yaml \
  autostart:=false \
  rviz:=false
```

Do not set `autostart:=true` or connect a command consumer before N2 through N6 are complete and
reviewed.

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```
````

- [ ] **Step 4: Add the guide link to the root README**

Insert after the existing runtime-guide paragraph in `README.md`:

```markdown
导航迁移目前处于不接底盘的 Phase N1 离线配置阶段；架构、阶段门槛和未验证项请见：
[A2W Navigation 开发状态](docs/NAVIGATION_DEVELOPMENT.md)。在 N2～N6 完成前不要将
`a2w_navigation` 的输出连接到机器人运动接口。
```

- [ ] **Step 5: Register the repository test**

Add this line after the existing `test_stage_contracts.py` invocation in `tests/run_tests.sh`:

```bash
python3 -m unittest "$repository_root/tests/test_navigation_contract.py" -v
```

- [ ] **Step 6: Run GREEN**

```bash
/usr/bin/python3 -m unittest tests/test_navigation_contract.py -v
/usr/bin/python3 -m unittest \
  src/a2w_navigation/test/test_package_contract.py -v
```

Expected: all repository and documentation contract tests pass.

- [ ] **Step 7: Commit documentation and repository contracts**

```bash
git add README.md docs/NAVIGATION_DEVELOPMENT.md \
  tests/run_tests.sh tests/test_navigation_contract.py \
  src/a2w_navigation/test/test_package_contract.py
git commit -m "test(navigation): enforce Phase N1 boundaries"
```

### Task 6: Build, full verification, and N1 checkpoint

**Files:**

- Modify only if verification exposes a defect: files already introduced by Tasks 1–5, with a new failing regression test before each fix.

**Interfaces:**

- Produces: an installed `a2w_navigation` package and reproducible evidence that N1 did not alter the stable SLAM/localization chain.

- [ ] **Step 1: Verify the package in isolation**

```bash
./scripts/build.sh --packages-select a2w_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select a2w_navigation \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose
```

Expected: package builds and all three ament pytest suites pass with zero errors/failures.

- [ ] **Step 2: Verify installation and launch argument visibility without starting nodes**

```bash
test -f install/a2w_navigation/share/a2w_navigation/config/nav2_params.yaml
test -f install/a2w_navigation/share/a2w_navigation/behavior_trees/navigate_to_pose_guarded.xml
test -f install/a2w_navigation/share/a2w_navigation/rviz/navigation.rviz
test -f install/a2w_navigation/share/a2w_navigation/launch/nav2_core.launch.py
ros2 launch a2w_navigation nav2_core.launch.py --show-args
```

Expected: all files exist and the command lists `map`, `params_file`, `bt_xml`, `rviz`,
`rviz_config`, `autostart`, and `use_sim_time`; no node is launched by `--show-args`.

- [ ] **Step 3: Run repository tests**

```bash
./tests/run_tests.sh
```

Expected: every repository test passes, including `test_navigation_contract.py`.

- [ ] **Step 4: Run the complete workspace build and test suite**

```bash
./scripts/build.sh
source install/setup.bash
colcon test --executor sequential \
  --packages-ignore teaserpp pmc \
  --return-code-on-test-failure
colcon test-result --verbose
```

Expected: all workspace packages build; all executed tests pass. `teaserpp` and `pmc` remain ignored
because the repository build intentionally excludes those vendor package wrappers.

- [ ] **Step 5: Verify protected paths and forbidden coupling**

```bash
git diff main...HEAD -- \
  src/FAST_LIO_Hesai \
  src/a2w_fastlio2_bringup/config/jt128.yaml \
  src/a2w_fastlio_mapping \
  src/a2w_fastlio_localization \
  src/a2w_fastlio_map \
  maps

rg -n '/home/'"dndx"'|Go2_Nav|go2_driver|go2_description|livox_ros_driver2|livox_frame|base_link|base_footprint|icp_registration|nav2_amcl' \
  src/a2w_navigation --glob '!test/**'

git diff --check
git status --short --branch
```

Expected: protected-path diff is empty; forbidden scan returns exit code 1 with no matches;
`git diff --check` is clean; only intentional plan checkbox or verification-document changes, if
any, remain visible.

- [ ] **Step 6: Record the checkpoint without claiming hardware validation**

If Tasks 1–5 already produced clean commits and Step 5 shows no remaining source changes, do not
create an empty commit. Report the current commit list with:

```bash
git log --oneline --decorate main..HEAD
```

The completion report must include:

```text
Phase N1 offline package/configuration verification complete.
Navigation is not connected to an A2W motion interface and autostart remains false.
Offline implementation and verification complete; JT128 hardware validation pending.
```

Stop after N1. Do not begin N2 until the N1 diff and verification evidence are reviewed.
