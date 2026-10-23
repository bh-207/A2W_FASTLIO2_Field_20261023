# A2W Navigation Migration Design

**Date:** 2026-09-20

**Status:** Approved for Phase N1 planning on 2026-09-20. No navigation implementation has started.

**Audit:** `docs/GO2_NAV_MIGRATION_AUDIT.md`

## 1. Goal and completion language

Add ROS 2 Humble navigation to the existing A2W + Hesai JT128 project without importing the old
Go2/Livox SLAM or localization stack:

```text
Hesai JT128 + IMU
  -> FAST_LIO_Hesai
  -> a2w_fastlio_localization
  -> Nav2
  -> guarded A2W velocity command
```

The first delivered navigation capability is a low-speed single `NavigateToPose` goal from RViz.
Waypoint, patrol, mission sequencing, and smooth-path playback are explicitly deferred until the
single-goal hardware path passes.

Offline work and hardware acceptance remain distinct. Before Phase N9, completion language is:

```text
Offline implementation and verification complete; JT128 hardware validation pending.
```

Offline tests are not evidence that JT128 Topic rates, QoS, frames, timestamps, obstacle filtering,
controller timing, A2W motion commands, or stopping behavior work on hardware.

## 2. Immutable boundaries

The following components remain authoritative and are not replaced or refactored by navigation:

- `src/FAST_LIO_Hesai` is the only LIO frontend.
- `a2w_fastlio_mapping` remains the only Mapping backend.
- `a2w_fastlio_map` remains the only Map Bundle owner.
- `a2w_fastlio_localization` remains the only global localization and relocalization implementation.
- Navigation mode never starts Mapping.
- Navigation does not modify the Localization state machine to solve cold start.
- No robot, network, JT128 Driver, ESIKF, ikd-tree, or robot-side service is changed by offline work.

The real frame contract remains:

```text
map --(Localization)--> camera_init --(FAST_LIO_Hesai)--> body
```

Navigation must not synthesize `map -> odom -> base_link`, copy Livox extrinsics, or rebroadcast
FAST-LIO pose under renamed frames.

The following Go2 components are forbidden:

```text
Go2_Nav/src/lio
Go2_Nav/src/mapper
Go2_Nav/src/registration
Go2_Nav/src/livox_ros_driver2
Go2 driver and description
Go2 maps
AMCL and ICP localization
Go2 obstacle-avoid SDK bridge
```

## 3. Considered approaches

### Approach A: A2W-native modular packages — selected

Create `a2w_navigation` for Nav2 configuration and safety orchestration, `a2w_nav_map` for the
offline Map Bundle conversion, and add `a2w_motion_bridge` only if the actual A2W command contract
requires conversion.

Advantages:

- preserves the existing SLAM/localization ownership boundaries;
- makes map conversion, navigation policy, and robot command transport independently testable;
- avoids Go2/Livox transitive dependencies;
- allows the motion bridge to remain absent if A2W already consumes standard `cmd_vel`;
- keeps later waypoint work outside the safety-critical first delivery.

Cost: more packages and explicit contracts than a copied launch file, but each boundary corresponds
to a different lifecycle, artifact, or hardware responsibility.

### Approach B: Rename and edit `robot_navigation2` — rejected

This looks fast but retains hidden AMCL startup, Go2 driver/description dependencies, fake TF,
inconsistent footprints, absolute paths, and a broken `/cmd_vel_safe` chain. Removing those concerns
would effectively rewrite the package while preserving misleading history and names.

### Approach C: Put all navigation behavior in bringup — rejected

A single launch/config package could start Nav2, perform map conversion, gate localization, and call
the A2 API. This would couple offline artifacts, runtime safety, and hardware transport, make unit
testing difficult, and turn bringup into an algorithm package. Bringup remains composition-only.

## 4. Package architecture and dependency direction

The target tree is:

```text
src/
├── FAST_LIO_Hesai/
├── a2w_fastlio_common/
├── a2w_fastlio_msgs/
├── a2w_fastlio_map/
├── a2w_fastlio_mapping/
├── a2w_fastlio_localization/
├── a2w_nav_map/
├── a2w_navigation/
├── a2w_motion_bridge/          # conditional on the verified A2W command contract
└── a2w_fastlio2_bringup/
```

Required dependency edges are:

```text
a2w_nav_map             -> a2w_fastlio_map + a2w_fastlio_common
a2w_navigation          -> a2w_fastlio_msgs + standard Nav2/ROS interfaces
a2w_motion_bridge       -> geometry_msgs + verified Unitree A2 interface
a2w_fastlio2_bringup    -> a2w_fastlio_localization + a2w_navigation
a2w_fastlio2_bringup    -> a2w_motion_bridge only when that package is required
```

Additional rules:

- `a2w_navigation` does not depend on Mapping, concrete registration algorithms, or Map Bundle I/O.
- `a2w_nav_map` does not depend on Nav2 runtime nodes or Localization.
- `a2w_motion_bridge` does not depend on Nav2 implementation classes or Localization internals.
- Safety decisions cross package boundaries using ROS messages/actions and explicit command topics.
- No dependency cycle is permitted.
- `a2w_fastlio2_bringup` contains no navigation algorithm, map conversion, or command conversion.

## 5. Phase N1 package boundary

Phase N1 creates only `a2w_navigation` and its offline validation surface. It does not connect a
motion sink or claim a runnable robot navigation loop.

The package structure is:

```text
a2w_navigation/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── nav2_core.launch.py
├── config/
│   └── nav2_params.yaml
├── behavior_trees/
│   └── navigate_to_pose_guarded.xml
├── rviz/
│   └── navigation.rviz
└── test/
    ├── test_package_contract.py
    ├── test_nav2_configuration.py
    └── test_launch_contract.py
```

`nav2_core.launch.py` starts only the robot-independent Nav2 runtime needed for static-map
single-goal navigation. It does not include Localization, FAST-LIO, Mapping, a robot driver, or a
motion bridge. Full composition is deferred to the bringup launch in N7.

N1 configuration uses the final frame and topic names where they are already established, but
robot-specific numeric values that lack evidence remain conservative, visibly provisional, and
blocked from hardware use until N2/N9. No Go2 numeric value is used as an A2W default.

## 6. Nav2 component selection

V1 uses standard ROS 2 Humble components:

| Responsibility | Component |
|---|---|
| Global planner | `nav2_smac_planner/SmacPlanner2D` |
| Local controller | `nav2_mppi_controller::MPPIController` |
| Motion model | `Omni` |
| Path smoother | `nav2_smoother::SimpleSmoother` |
| Progress check | `nav2_controller::SimpleProgressChecker` |
| Goal check | `nav2_controller::SimpleGoalChecker` |
| Behaviors | Spin, BackUp, DriveOnHeading, Wait, AssistedTeleop |
| Command smoothing | `nav2_velocity_smoother`, initially `OPEN_LOOP` |
| Navigation action | `NavigateToPose` |

MPPI/Omni is retained because the audited active Go2 configuration actually uses it and A2W is an
omnidirectional quadruped. This selection does not approve the Go2 dynamics values. Velocity,
acceleration, angular rate, critic weights, tolerances, and recovery behavior receive A2W-specific
offline defaults and final hardware tuning.

No custom planner or controller plugin is introduced. The historical `my_planner` entries have no
matching source in the audited package and are not migrated.

## 7. Frame, odometry, and time contracts

Initial Nav2 contract:

```yaml
global_frame: map
local_frame: camera_init
robot_base_frame: body
odom_topic: /Odometry
```

`/Odometry` currently provides:

- pose in `camera_init`, child `body`;
- LiDAR end timestamp shared with `/cloud_registered_body`;
- no assigned twist, so the default zero twist is not a measured robot velocity.

Nav2 must never use `/localization/odom` as local odometry. That Topic is a `map -> body` global
expression.

N2 must determine whether the MPPI controller and any closed-loop smoother configuration require a
measured body velocity. The allowed outcomes are:

1. use an existing, verified A2W velocity/odometry Topic;
2. keep components that do not require measured feedback in open-loop mode and document the
   limitation;
3. design a separately reviewed estimator if needed.

Differentiating pose or filling FAST-LIO twist without a reviewed interface is not allowed.

## 8. Map artifact design

Nav2 requires a 2D occupancy map, while the authoritative A2W artifact is Map Bundle V1:

```text
Map Bundle/global_map.pcd
  -> height and finite-point filtering
  -> ground/obstacle classification
  -> XY rasterization
  -> unknown/free/occupied classification
  -> navigation/nav_map.pgm
  -> navigation/nav_map.yaml
```

`a2w_nav_map` is an offline artifact generator, not a continuous runtime mapper. It must:

- validate `manifest.sha256` and Bundle metadata before conversion;
- require `metadata.map_frame == map` unless an explicit expected-frame parameter says otherwise;
- preserve every retained point's `map` coordinates;
- calculate the raster origin from the selected map-coordinate bounds and resolution;
- record conversion parameters and source Bundle UUID next to the output;
- write atomically so an interrupted conversion does not replace a valid navigation map;
- reject empty, non-finite, corrupt, or frame-inconsistent inputs;
- never treat `/mapping/optimized_map_preview` as authoritative input.

The planned command contract is:

```bash
ros2 run a2w_nav_map pcd_to_occupancy \
  --map-bundle /absolute/path/to/maps/factory_map \
  --output /absolute/path/to/maps/factory_map/navigation
```

The launch interface may resolve paths from project-relative arguments, but the executable receives
resolved paths and never embeds a machine-specific user-home path.

## 9. Costmaps and obstacle flow

Global costmap:

```text
frame: map
base: body
map_server -> StaticLayer -> InflationLayer
```

Local costmap:

```text
frame: camera_init
base: body
/cloud_registered_body -> PointCloud2 obstacle layer -> InflationLayer
```

PointCloud2 is selected over LaserScan for V1 because the current frontend already publishes the
body-frame cloud and standard costmap layers can consume it. A scan adapter is added only if a
specific validated component requires LaserScan or hardware performance proves the direct cloud
path unsuitable.

The following remain parameters and require hardware validation:

- QoS reliability, durability, depth, and queue behavior;
- min/max obstacle height;
- marking and clearing ranges;
- observation persistence and expected update rate;
- voxel resolution and point density limits;
- costmap update/publish rates;
- inflation radius and scaling;
- transform tolerance.

The global and local costmaps use one shared A2W footprint source. They may not silently diverge.

## 10. Footprint and dynamics policy

No Go2 footprint or dynamics value is copied. Before hardware activation, N2 requires a documented
A2W footprint derived from one of these sources, in priority order:

1. an A2W URDF or official dimension source that matches the deployed configuration;
2. a measured conservative body envelope recorded in project documentation;
3. a temporary conservative test footprint explicitly marked as not production-ready.

The same rule applies to vx, vy, wz, acceleration, deceleration, collision slowdown, and stop
zones. Offline defaults must prevent accidental high-speed operation and N9 begins below the final
target speed.

## 11. LocalizationGuard design

`a2w_navigation` owns a `localization_guard` node. It subscribes to
`a2w_fastlio_msgs/msg/LocalizationStatus` and controls goal acceptance, lifecycle activation, and
the downstream velocity gate without modifying Localization.

Policy V1:

| Localization state | Navigation behavior |
|---|---|
| `INITIALIZING` | Nav2 remains inactive or goal admission is closed; output is forced to zero |
| `LOCALIZED` | Goals may be accepted and guarded velocity may pass |
| `DEGRADED` | Configurable `warn`, `limit`, or `stop`; default is `stop` until hardware policy approval |
| `LOST` | Cancel active goal, close admission, force zero command |
| `RELOCALIZING` | Keep admission closed and output zero |
| Return to `LOCALIZED` | Do not resume an old path; require a new plan/action decision |

The guard also treats a missing, malformed, or stale status as not localized. A process that starts
and remains `INITIALIZING` logs exactly:

```text
navigation blocked: localization not ready
```

The guard owns no localization recovery algorithm. Cold-start global initialization remains outside
this navigation migration.

## 12. Velocity and motion bridge design

The logical command chain is:

```text
controller_server
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> safety limiter / LocalizationGuard
  -> /cmd_vel_guarded
  -> verified A2W command consumer
```

Topic names are parameters. There is exactly one downstream command owner. No launch-level remap
may unintentionally redirect an odometry publisher or command consumer outside its intended node.

N5 first performs a read-only hardware/interface audit. If A2W natively and safely consumes the
guarded standard Twist Topic, no bridge is built. Otherwise `a2w_motion_bridge` converts
`geometry_msgs/msg/Twist` to the verified A2W API and provides:

- vx/vy/wz finite-value validation and saturation;
- acceleration/deceleration limiting;
- command timeout;
- immediate zero/StopMove on guard closure;
- zero/StopMove during orderly shutdown;
- explicit network/interface and command-owner configuration;
- observable health and rejection reason;
- no automatic gait, stand, obstacle-avoid, or control-mode changes without separate approval.

The local Unitree SDK2 A2 `SportClient::Move/StopMove` API is a candidate, not yet the accepted
runtime contract. Go2 JSON/API IDs and `ObstaclesAvoidClient` are not reused.

## 13. Launch composition

N7 adds:

```text
a2w_fastlio2_bringup/launch/navigation.launch.py
```

It accepts at least:

```text
map_bundle_path
navigation_map_yaml
nav2_params_file
rviz
autostart
use_sim_time
```

Its composition is:

```text
navigation.launch.py
  ├── localization.launch.py
  │   ├── FAST_LIO_Hesai
  │   └── a2w_fastlio_localization
  ├── Nav2 map/planner/controller/smoother/behavior/BT nodes
  ├── LocalizationGuard
  ├── collision monitor or equivalent configured safety stage
  ├── optional a2w_motion_bridge
  └── optional RViz
```

It never starts Mapping, AMCL, ICP, a Go2/Livox component, or a second global TF publisher.
`autostart` does not override localization readiness: nodes being alive is not permission to move.

The corresponding `scripts/run_a2w_navigation.sh` resolves the repository from its own location,
uses existing environment templates, validates required artifacts before launch, and contains no
user-home absolute path.

## 14. Error handling and safe defaults

Navigation stays stopped and reports a specific reason when any of these holds:

- Map Bundle or navigation map is missing, corrupt, or frame-inconsistent;
- Localization status is absent, stale, `INITIALIZING`, `LOST`, or `RELOCALIZING`;
- required TF `map -> camera_init -> body` is unavailable or stale;
- odometry/cloud timestamps or frames violate the contract;
- the configured A2W motion consumer is unavailable;
- command input contains NaN/Inf or violates configured limits;
- a safety/guard process exits unexpectedly.

Failure must not fall back to an unguarded `/cmd_vel` connection. Shutdown order closes the command
gate before stopping Nav2 and Localization.

## 15. Test strategy

Each phase is an independent commit and must pass repository-wide build/test checks without
modifying protected frontend files.

Offline coverage includes:

- package dependency graph and absence of dependency cycles;
- launch syntax and launch composition;
- YAML parsing and installed Nav2 plugin names;
- no Go2/Livox/AMCL/ICP dependencies or hardcoded machine-specific user-home paths;
- `map/camera_init/body` frame consistency across Nav2, map metadata, and TF contracts;
- `/Odometry` and `/cloud_registered_body` topic/QoS/timestamp contracts;
- Map Bundle conversion with synthetic ground, obstacle, empty, corrupt, and non-finite clouds;
- navigation map origin/resolution and Bundle frame consistency;
- PointCloud2 local obstacle configuration;
- every LocalizationGuard state, stale/malformed/missing status, goal cancellation, and no old-path
  resume;
- velocity finite checks, saturation, acceleration limit, timeout, LOST/shutdown zero command;
- fake odometry/cloud/status and mock motion sink launch integration;
- Mapping absent and exactly one `map -> camera_init` publisher in Navigation mode.

The final hardware validation is sequential and low speed:

1. observe Topic/QoS/frame/timestamp and a trustworthy velocity source;
2. verify the A2W command/stop contract without Nav2 autonomous motion;
3. wait for real `LOCALIZED`;
4. activate navigation and send a short RViz goal in a controlled area;
5. verify plan, guarded command, obstacle response, stop behavior, and arrival;
6. separately test `DEGRADED`, `LOST`, `RELOCALIZING`, and return to `LOCALIZED`;
7. record CPU, RAM, latency, command rate, stopping latency, and observed failure behavior.

## 16. Phase sequence and acceptance gates

### N1 — A2W Nav2 package skeleton

Create `a2w_navigation`, standard component configuration, project-owned BT/RViz/launch, and
offline package/config/launch tests. No robot command output is connected.

### N2 — Frame, odometry, Topic, and footprint contract

Validate and harden the N1 `map/camera_init/body`, `/Odometry`, and `/localization/status`
configuration, then add an evidence-backed A2W footprint. Resolve or explicitly constrain the
missing twist problem.

### N3 — Map Bundle conversion

Create `a2w_nav_map` and reproducibly generate a 2D navigation map in the Bundle `map` frame.

### N4 — Costmaps and JT128 obstacle source

Connect global static map and direct PointCloud2 local obstacles, with offline contract tests and
hardware-pending parameter labels.

### N5 — A2W motion contract

Verify the real interface and either connect the guarded Twist Topic or implement the minimal A2W
bridge. No Go2 transport is imported.

### N6 — LocalizationGuard

Implement lifecycle/action/velocity gating and all state/staleness safety tests.

### N7 — Integrated bringup

Add `navigation.launch.py` and the portable run script. Ensure Localization starts, Mapping and AMCL
do not, and the safety gate is closed until readiness.

### N8 — Offline integration verification

Run fake-input launch tests and repository-wide build/tests; verify protected paths and forbidden
dependency/path scans.

### N9 — Hardware validation

Perform the controlled low-speed acceptance sequence and only then tune robot-dependent values.

Waypoint and mission work begins only after N9 single-goal acceptance and receives a separate
design review.

## 17. Non-goals

This design does not:

- alter FAST-LIO, Mapping, Localization, Map Bundle V1, or global relocalization algorithms;
- add automatic cold-start global localization;
- implement a persistent submap manager;
- import or maintain Go2/Livox drivers or maps;
- implement waypoint, patrol, mission, or UI workflows;
- claim hardware readiness from synthetic or offline tests;
- push code, modify a remote branch, or change robot-side software without an explicit later step.

## 18. Design approval result required

Approval of this document authorizes creation of a detailed implementation plan beginning with N1.
It does not by itself authorize skipping phase gates, connecting unverified motion commands, or
claiming hardware validation.
