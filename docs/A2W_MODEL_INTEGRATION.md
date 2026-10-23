# A2W model and Nav2 geometry integration

This document records the model assumptions used by the first-motion A2W
navigation profile.  It does not claim a calibrated JT128/IMU installation.

## Sources and coordinate contract

- Kinematic source: the supplied `a2_wheel.urdf`, normalized into
  `src/X2-0807urdf/urdf/a2w_navigation.urdf`.
- Shape cross-check: `src/A2轮足版简化模型-20251125(1).stp`.
- FAST-LIO tracking tree remains `map -> camera_init -> body`.
- The supplied URDF root was `base_link`.  The navigation model preserves that
  link and adds one fixed joint `body -> base_link`.  This is the only
  bridge between the FAST-LIO tracking frame and the CAD tree, and is published
  by `robot_state_publisher`.
- Since 2026-09-22: `body -> base_link = xyz(0.33767,0,-0.08134), rpy(0,0,pi)`.
  Three single-axis hardware tests confirmed physical forward=-body X,
  physical left=-body Y, counterclockwise=positive body yaw. The nominal yaw
  agrees with the official front-lidar mounting composed with the unchanged
  164 driver normalization (details in `A2W_AXIS_DIAGNOSTICS.md`).
  **Translation is CAD-DERIVED / NOT IMU-ORIGIN CALIBRATED**: it assumes the
  aligned IMU and front-lidar origins coincide. Full six-DoF calibration is
  still pending, not implied by the direction tests.

TF ownership is intentionally non-overlapping:

```text
Localization       map -> camera_init
FAST_LIO_Hesai     camera_init -> body
robot_state_publisher
                   body -> base_link -> A2W wheel-leg links
```

The supplied URDF/STP contains no named JT128 or IMU mounting frame and no
sensor extrinsic.  No `jt128_link` or `imu_link` was invented.  FAST-LIO already
publishes `/cloud_registered_body` with `frame_id=body`, so the local costmap
does not require a raw-lidar-frame transform.  The raw JT128/IMU mounting
extrinsic remains a hardware-calibration item.

The original ROS1 display/Gazebo launch files and raw exporter CSV/URDF remain
in the source directory only as provenance.  The ROS2 package installs only
`a2w_navigation.urdf` and `meshes/`; legacy `base_footprint` publishers cannot
enter the navigation runtime.

## Current footprint: measured joint stance, 2026-09-23

The operator confirmed that the robot was in its normal navigation standing
posture. A read-only DDS subscriber collected 30 `unitree_hg/msg/LowState`
samples from `/lowstate` over 6 seconds (ticks 6522755 through 6528570).
Sport state was mode 3 (`DEFAULT_MODE`), gait 0; its body-height and foot-position
fields were zero and were NOT used as geometry measurements. No motion or joint
commands were published, and no robot settings were modified.

Leg motor slots are FR=0..2, FL=3..5, RR=6..8, RL=9..11 (hip/thigh/calf).
The ordering and inward-hip signs were cross-checked against the official A2
[deployment mapping](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/deploy/robots/a2/config/policy/velocity/v0/params/deploy.yaml)
and [joint model](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/src/assets/robots/unitree_a2/xmls/a2.xml).
These references are for the leg mapping, not a replacement for the supplied
wheel-robot mesh. Representative measured joint angles, in radians:

| Leg | Hip | Thigh | Calf |
| --- | ---: | ---: | ---: |
| FR | 0.130431 | 0.806797 | -1.588858 |
| FL | -0.135016 | 0.803729 | -1.588346 |
| RR | 0.125164 | 0.791082 | -1.589318 |
| RL | -0.124849 | 0.796537 | -1.591840 |

Forward kinematics applies URDF joint origin followed by joint-axis rotation,
then transforms each STL into base_link. Wheel meshes use the support over all
rotations about their local Y axes, avoiding an assumption about wheel encoder
slot order or instantaneous tread phase. For axis row `r` of the wheel rotation,
each vertex contributes `r_y*y +/- hypot(r_x,r_z)*hypot(x,z)` plus translation.
The union over samples gives the visual solid-envelope bounds (metres):

```text
x_min = -0.487744933   x_max = 0.387247592
y_min = -0.237575717   y_max = 0.237735120
length = 0.874992525   width = 0.475310836
```

The front is set by the body mesh, rear by the rear leg geometry, and lateral
limits by wheels. Relative to the operator's approximate 0.87 x 0.44 m manual
measurement, model length differs by about 5 mm and width by 35 mm. Do not
compress the mesh laterally to force agreement; a remaining mesh/physical-size
or measurement discrepancy is not additional safety padding.

With the requested 10 mm margin on EACH side, rounded outwards to millimetres:

```yaml
footprint: "[[0.398, 0.248], [0.398, -0.248], [-0.498, -0.248], [-0.498, 0.248]]"
footprint_padding: 0.0
```

Local/global costmaps share this 0.896 x 0.496 m rectangle. The earlier
hypothetical pose-A rectangle (Y +/-0.233) is superseded: it would clip the
currently observed model stance. The rectangle centre is X=-0.050, not base_link.

This is hardware joint sampling plus model-based geometry, NOT independent
measurement of the real wheel surfaces or the maximum walking/turning envelope.
Dynamic stance, payload, precise body-to-base translation, braking distance and
navigation clearance remain hardware-validation items. A 10 mm geometric margin
is not a braking allowance or validation of navigation at 0.60 m/s. Configuration
is installed offline; the operator must restart navigation to load it.

## Historical STEP footprint measurement (superseded)

The STEP file was imported with Gmsh/OpenCASCADE and its solid surfaces were
triangulated.  Surface vertices were projected to XY, STEP X was reversed to
match the URDF X direction, and a convex hull was computed.  This avoids the
invalid oversized bounds produced by STEP curve/control points.

Measured normal-stance rigid envelope before margin:

```text
X min/max: -0.443263 / +0.387252 m
Y min/max: -0.277090 / +0.277202 m
Length:     0.830515 m
Width:      0.554292 m
```

For comparison, the URDF zero-joint mesh projection is approximately
`0.741546 m x 0.554766 m`.  Width agrees within 1 mm; the STEP assembly is about
89 mm longer because it captures the supplied articulated wheel-leg stance.
The STEP stance therefore drives the navigation footprint.

A 50 mm outward margin is applied to the convex hull.  The resulting Nav2
envelope is:

```text
X min/max: -0.493 / +0.437 m
Y min/max: -0.327 / +0.327 m
Length:     0.930 m
Width:      0.654 m
```

```yaml
footprint: "[[-0.493, -0.221], [-0.491, -0.236], [-0.485, -0.253], [-0.433, -0.303], [-0.411, -0.324], [-0.390, -0.327], [0.294, -0.327], [0.314, -0.324], [0.345, -0.295], [0.437, -0.039], [0.437, 0.034], [0.436, 0.046], [0.345, 0.295], [0.314, 0.324], [0.294, 0.327], [-0.390, 0.327], [-0.411, 0.324], [-0.433, 0.303], [-0.485, 0.253], [-0.491, 0.236], [-0.493, 0.221]]"
```

No manipulator or future payload motion envelope is included: neither appears
as a kinematic chain in the supplied URDF.  Recompute the footprint whenever
the normal wheel-leg stance or fixed external payload changes.

## Navigation parameters

- Local/global inflation radii are `0.45 m` and `0.35 m`; scaling remains `5.0`.
- `/cloud_registered_body` is projected to the finite body-frame `/a2w/scan`.
  The local ObstacleLayer consumes `/a2w/scan_cloud`, while Collision Monitor
  consumes `/a2w/scan`; both therefore use the same obstacle geometry without
  making Collision Monitor process the full registered cloud.
- Collision Monitor uses `base_link` and the fixed URDF transform from the
  body-frame scan. Current-time base-shift correction stays disabled and its
  `0.50 s` source timeout remains unchanged.
- The first-motion `StopZone` radius is temporarily `0.60 m` (about `0.55 m`
  STEP circumscribed footprint plus a small margin); `SlowZone` remains
  `1.40 m`. Confirm both from JT128 hardware logs before production use.
- The local frame is `camera_init`; global frame is `map`; Nav2 base frame is
  `base_link`. FAST-LIO `/Odometry` and `/cloud_registered_body` remain in their
  original frames. No raw cloud labels or Map Bundle data are rewritten.
- The local Unitree B2W example exposes `Move(vx, vy, vyaw)` and explicitly
  demonstrates lateral `vy`, so MPPI remains `Omni`. Lateral motion is enabled
  end-to-end with `vy=[-0.15, 0.15] m/s`.
- Limits remain `vx=[-0.05, 0.15] m/s`, `vy=[-0.15, 0.15] m/s`, `|wz|<=0.50 rad/s`,
  `ax=[-0.25, 0.25] m/s^2`; MPPI, velocity smoother, localization guard,
  behavior server, and A2W bridge use the same limits.

The command path is direct and unit-preserving:

```text
Nav2 velocity_smoother /cmd_vel (m/s, rad/s)
  -> Collision Monitor /cmd_vel_safe
  -> LocalizationGuard /cmd_vel_guarded
  -> A2WMotionBridge
  -> /api/sport/request, API 1008, {x:vx, y:vy, z:wz}
```

Both safety nodes clamp values; they do not rescale or swap axes.  API 1003 is
used for `StopMove` on zero command, watchdog expiry, disable, and shutdown.
When Localization leaves `LOCALIZED`, the guard cancels the active Goal and the
activation gate pauses the Nav2 lifecycle. Recovery requires a fresh localized
pose and lifecycle resume; the canceled Goal is never resumed automatically.

## Still requiring hardware confirmation

1. Measure the physical IMU-to-CAD offset and precise mounting rotation.
   Direction signs are hardware checked; the nominal translation is not.
2. Add a verified A2W `/lowstate` to `/joint_states` adapter before depending on
   live wheel/leg visualization.  `robot_state_publisher` publishes fixed links
   immediately and movable links only when real joint states arrive.
3. Measure and encode JT128 and IMU mounting frames if raw sensor-frame TF is
   needed.  Do not derive those transforms from the current zero initial guess.
4. Recheck footprint clearance in the actual navigation stance and with any
   fixed payload, bumper change, or manipulator installation.
5. Validate voxel height/range filtering, inflation, and lateral motion with
   real obstacles at low speed.

## FAST-LIO extrinsic estimation is a different transform

`mapping.extrinsic_R/T` defines lidar -> IMU: `p_I = R_I_L p_L + t_I_L`.
`extrinsic_est_en=true` enables online estimation; the YAML values are initial
conditions, not necessarily the current filter estimates. The published
`camera_init -> body` is the IMU state pose, and `/cloud_registered_body` is
converted with that estimated lidar->IMU transform. None of these settings or
FAST-LIO code was changed for the base_link fix. A wrong lidar/IMU calibration
can still affect deskewing and mapping; the new URDF joint does not fix it.

Default navigation loads the bringup package's `config/jt128.yaml`, not the
FAST_LIO_Hesai source config directly. Both currently specify online estimation
with identity initial R and zero initial T; runtime parameter queries confirmed
that initialization configuration, not the evolved filter state.

This change does not populate the currently missing FAST-LIO odometry twist,
solve low-speed lateral actuator response, or validate navigation performance.
Do not relabel `/Odometry` as base_link odometry without transforming its pose
and twist. RViz `/localization/pose` still depicts body, not robot heading.
