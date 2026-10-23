# A2W Navigation N2-N8 Implementation Plan

**Spec:** `docs/superpowers/specs/2026-09-20-a2w-navigation-design.md`

**Goal:** Deliver the smallest guarded ROS 2 Humble chain for a first 1-2 m A2-W Nav2 goal with
`factory_map_v2`, while preserving `map -> camera_init -> body` and the existing Localization.

**Global constraints:** Do not modify FAST_LIO_Hesai, Localization algorithms, Mapping, Map Bundle
V1, JT128 configuration, or robot-side software. Hardware output starts disabled. Use only
`/Odometry`, `/cloud_registered_body`, `/localization/status`, and the verified Unitree
`/api/sport/request` Move/StopMove contract. Tests precede production changes.

## Task 1 — N2 frame, footprint, and low-speed contract

- Extend `a2w_navigation` tests first to require the real frames/topics, a shared conservative A2-W
  footprint derived from `X2-0807urdf`, open-loop odometry handling, and first-run limits
  `vx<=0.20`, `|wz|<=0.30`, `vy=0`.
- Update `nav2_params.yaml` minimally until those tests pass.
- Verify with the package's Python configuration tests.

## Task 2 — N3 Map Bundle to OccupancyGrid artifact

- Create `a2w_nav_map` with pure conversion tests for binary PCD parsing, map-frame validation,
  finite/height filtering, raster origin/scale, occupancy output, and corrupt input rejection.
- Implement `pcd_to_occupancy`, write `nav_map.pgm`, `nav_map.yaml`, and conversion metadata through
  temporary files followed by atomic replacement.
- Build/test and generate `maps/factory_map_v2/navigation/nav_map.yaml`.

## Task 3 — N4 JT128 local obstacles

- Add failing configuration tests for a PointCloud2 voxel source on `/cloud_registered_body` and
  matching global/local footprint values.
- Configure VoxelLayer + InflationLayer with conservative, parameterized first-run ranges and no
  LaserScan/Livox input.

## Task 4 — N5 verified A2-W bridge

- Create `a2w_motion_bridge` tests for finite validation, saturation, `vy=0`, timeout, disabled
  output, Move API 1008 JSON, and StopMove API 1003.
- Implement the ROS node publishing `unitree_api/msg/Request` to `/api/sport/request`; provide an
  explicit SetBool hardware enable service; remain disabled on startup and send StopMove on disable,
  timeout, and shutdown.

## Task 5 — N6 LocalizationGuard

- Add tests for every Localization state, stale/missing status, explicit arming, command limits,
  latch closure, zero output, and LOST goal cancellation request.
- Implement `localization_guard`: `/cmd_vel` to `/cmd_vel_guarded`, SetBool motion enable service,
  zero on non-LOCALIZED/stale state, cancel `navigate_to_pose` on LOST, and require re-arm after
  recovery.

## Task 6 — N7 integrated bringup

- Add launch contract tests first.
- Add `a2w_fastlio2_bringup/launch/navigation.launch.py` and
  `scripts/run_a2w_navigation.sh`, composing Localization, map server/Nav2, guard, bridge, and
  optional RViz. Mapping and AMCL must be absent; hardware output remains disabled.

## Task 7 — N8 focused offline verification and operator handoff

- Build only `a2w_nav_map`, `a2w_navigation`, `a2w_motion_bridge`, and bringup plus required message
  packages; run their tests and launch `--show-args`/short inactive smoke checks.
- Confirm protected paths are unchanged from the branch base.
- Update `docs/NAVIGATION_DEVELOPMENT.md` and runtime commands, preserving the status:
  `Offline implementation and verification complete; JT128 hardware validation pending.`

## Review focus

- No command can reach `/api/sport/request` before both LocalizationGuard and hardware bridge are
  explicitly enabled.
- Timeout/shutdown/LOST semantics always emit StopMove or zero and never resume an old command.
- Occupancy map pixels retain the Bundle's `map` coordinate origin, direction, and metric scale.
- Nav2 owns no fake TF and uses exactly `map -> camera_init -> body`.
