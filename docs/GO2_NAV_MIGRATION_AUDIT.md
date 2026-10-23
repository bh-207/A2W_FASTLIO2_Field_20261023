# Go2 Nav → A2W Navigation 迁移审计（Phase N0）

> 审计日期：2026-09-20
>
> 审计性质：只读架构审计，未执行代码迁移，未修改机器人、网络、JT128 Driver 或 FAST-LIO 算法。
>
> 状态：**Phase N0 complete; migration design approval pending.**

## 1. 范围与结论

本次审计的两个工作区是：

- 旧导航来源：`<legacy-go2-workspace>`
- 未来唯一主工程：`<a2w-fastlio-workspace>`

结论：不应复制 `robot_navigation2` package，更不应合并整个 `Go2_Nav/src`。可复用的是标准 Nav2 组件选型、配置结构和部分调试思路；所有 frame、地图、障碍源、footprint、速度限制、定位门控和底盘接口都必须按 A2W 重新适配。

推荐保留的真实 TF 契约是：

```text
map --(a2w_fastlio_localization)--> camera_init
camera_init --(FAST_LIO_Hesai)--> body
```

新导航不得伪造 `odom` / `base_link` 或平移旧 Livox 外参。

## 2. `robot_navigation2` 实际 package tree

```text
robot_navigation2/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── navigation2.launch.py
├── config/
│   ├── nav2_params.yaml
│   ├── nav2_params-bak.yaml
│   ├── nav2_params-no-global-obstacle.yaml
│   ├── nav2_params_0419.yaml
│   ├── nav2_params_12272252.yaml
│   ├── nav2_params_1227_.yaml
│   ├── nav2_params_best_1228.yaml
│   ├── nav2_params_humble_mppi.yaml
│   └── nav2_params_orin(原始).yaml
├── maps/
│   ├── map.pgm
│   ├── map.yaml
│   ├── fac/{map.pgm,map.yaml,map0401.pgm,map0401.yaml}
│   └── lab/{map.pgm,map.yaml,map-1.pgm,map-1.yaml}
├── odom_tf_broadcaster.py
├── scan_stamp_fix.py
└── src/
    ├── cmdvel_to_go2_avoid.cpp
    └── enable_go2_obstacle_avoid.cpp
```

不存在 package 自有 BT XML、RViz 配置、planner plugin 源码或 controller plugin 源码。它使用已安装的 Nav2 标准组件。

`package.xml` 是未完成的占位元数据，未声明大部分实际运行依赖。`CMakeLists.txt` 则硬编码 `/usr/local/include`、`/usr/local/lib/libunitree_sdk2.a` 和 CycloneDDS C++ library，并强制检查 Go2 `obstacles_avoid_client.hpp`。因此 package 本身不具备可移植性。

## 3. Launch 真实行为

package 只有一个 launch：`launch/navigation2.launch.py`。默认使用 `config/nav2_params.yaml` 和 `maps/map.yaml`，并实际启动：

1. `go2_description/launch/robot.launch.py`；
2. `go2_driver/launch/go2_driver.launch.py`；
3. `odom_tf_broadcaster.py`，将 `/Odometry` 的 pose 再发为 `odom → body`；
4. `pointcloud_to_laserscan_node`，将 `/cloud_registered_body` 转成 `/scan`；
5. `base_link → livox_frame` 和 `body → base_link` 两个静态 TF；
6. Humble `nav2_bringup/bringup_launch.py`，参数 `slam=False`、`autostart=true`；
7. 可选的 Nav2 默认 RViz。

已定义但**未实际加入 LaunchDescription** 的节点：

- `scan_stamp_fix.py`；
- `nav2_collision_monitor/collision_monitor`；
- `cmdvel_to_go2_avoid` bridge（整段注释，且 target 也未编译）。

重要的 launch 副作用：

- 整个 bringup group 执行 `/odom → /Odometry` remap。这不仅影响 Nav2，还可能将 group 内 Go2 driver 的相对 `odom` publisher 映射到 `/Odometry`，与 FAST-LIO publisher 发生同名冲突。
- Go2 driver include 另被包在 `/cmd_vel → /cmd_vel_safe` remap group 内，所以它实际期待 `/cmd_vel_safe`。
- Humble `bringup_launch.py` 在 `slam=False` 时无条件 include `localization_launch.py`，而该 launch 固定创建 `map_server` 和 `amcl`。在自定义 YAML 中把 AMCL 从某个 lifecycle 列表注释掉，不会阻止 AMCL 节点被创建。
- Humble `navigation_launch.py` 的固定 lifecycle 列表不含 `collision_monitor`，而本 package 自己定义的 collision monitor 未被返回。YAML 里写了 `collision_monitor` 不等于它正在运行。

## 4. 所有 YAML 审计

Launch 默认仅选择 `nav2_params.yaml`。根目录启动脚本也没有通过 `params_file` 覆盖它。因此其他八份应视为历史/试验快照，不是当前默认运行配置。

| 文件 | 主要差异 | 审计处置 |
|---|---|---|
| `nav2_params.yaml` | 当前默认；SmacPlanner2D + MPPI Omni；局部 `/scan`；全局 static map | 仅作新 A2W 配置结构参考 |
| `nav2_params-no-global-obstacle.yaml` | MPPI Omni；局部/全局曾用 `/cloud_filtered` PointCloud2 | 仅用于比较历史方案 |
| `nav2_params_0419.yaml` | MPPI Omni；AMCL scan 名为 `/scan_livox`；局部 `/scan` | 禁止迁移 Livox 语义 |
| `nav2_params_best_1228.yaml` | MPPI DiffDrive；Navfn；局部和全局 `/scan` | 历史快照，不得代替当前实际选型 |
| `nav2_params_humble_mppi.yaml` | MPPI DiffDrive + Navfn | 历史快照 |
| `nav2_params_orin(原始).yaml` | DWB + Navfn，controller 10 Hz | 历史快照 |
| `nav2_params-bak.yaml` | DWB + Navfn；robot radius 0.15 m | 历史快照 |
| `nav2_params_12272252.yaml` | controller 填写 `my_planner::MyPlanner`，planner server 仍是 Navfn | 不自洽/无对应 plugin 源码，禁止迁移 |
| `nav2_params_1227_.yaml` | controller 填写 `my_planner/MyPlanner`，planner server Navfn | 不自洽/无对应 plugin 源码，禁止迁移 |

所有旧 map YAML 都是 Go2 地图，且 `image` 几乎都硬编码到 `<legacy-go2-workspace>/src/navigation/robot_navigation2/maps/map.pgm`。`fac` / `lab` YAML 的 origin 和 resolution 不同，却仍指向同一张根目录 PGM，不能作为 A2W 地图输入。

## 5. 当前 Nav2 组件（以默认 YAML 为准）

### 5.1 Planner

- Server plugin ID：`GridBased`
- 实际 plugin：`nav2_smac_planner/SmacPlanner2D`
- search motion model：`2D`
- resolution downsampling factor：1
- `allow_unknown: true`
- `smooth_path: true`

分类：**修改后迁移**。保留标准 SmacPlanner2D 选型，但基于 A2W 导航地图和运动验收重新调参。

### 5.2 Controller

- plugin ID：`FollowPath`
- 实际 plugin：`nav2_mppi_controller::MPPIController`
- motion model：`Omni`
- controller frequency：20 Hz
- 同时使用 `SimpleProgressChecker` 和 `SimpleGoalChecker`
- MPPI 限制包括 `vx_max=1.2`、`vy_max=0.35`、`wz_max=1.9`、`ax_max=5.0`、`ay_max=4.0`

分类：**修改后迁移**。保留 MPPI/Omni 作为首选候选，但 Go2 速度、加速度、critic、goal tolerance 和 footprint 数值不得直接复制到 A2W。

### 5.3 Behavior Tree 与 behavior server

无自定义 BT XML，直接引用 Humble 安装路径：

- `navigate_to_pose_w_replanning_and_recovery.xml`
- `navigate_through_poses_w_replanning_and_recovery.xml`

Behavior server 启用标准 `Spin`、`BackUp`、`DriveOnHeading`、`Wait`、`AssistedTeleop`。Smoother 是 `nav2_smoother::SimpleSmoother`，velocity smoother 是 open-loop 20 Hz。

分类：标准 BT 逻辑与组件选型可**直接复用概念**；新 package 应将实际使用的 BT XML 纳入工程或明确依赖，并加入 LocalizationGuard 语义。现有文件无可直接复制的自有 BT。

### 5.4 Costmaps

Global costmap：

- `global_frame: map`
- `robot_base_frame: base_link`
- resolution 0.05 m
- plugins：`StaticLayer + InflationLayer`
- update/publish 1 Hz
- track unknown space

Local costmap：

- `global_frame: odom`
- `robot_base_frame: base_link`
- rolling 6 m × 6 m，resolution 0.05 m
- update/publish 10 Hz
- plugins：`ObstacleLayer + InflationLayer`
- obstacle source：`/scan` / `LaserScan`
- obstacle/raytrace max range：7/8 m
- height range：-0.35..2.0 m

分类：**修改后迁移**其通用层级结构。A2W V1 建议 global `map/body`，local `camera_init/body`，且 local obstacle 首选直接使用 `/cloud_registered_body` PointCloud2，不为了旧配置保留 Livox/LaserScan 适配。

### 5.5 Collision monitor

YAML 意图是：

```text
/cmd_vel -> collision_monitor -> /cmd_vel_safe
```

障碍源为 `/cloud_registered_body` PointCloud2，包含 slowdown radius 1.4 m 和 stop radius 0.90 m。但当前 launch **没有启动 collision monitor**，因此这不是实际安全链。

分类：安全链思路属于**修改后迁移**；几何半径、高度和限速值必须用 A2W 尺寸和低速实机数据重新验证。

## 6. Map server 与导航地图

旧系统明确依赖 Nav2 `map_server` 的 `OccupancyGrid`。默认 launch 的 `map` argument 会由 Nav2 `RewrittenYaml` 覆盖 params 中的 `yaml_filename`，但 params 仍存在不可移植的 `<legacy-go2-workspace>/map/map.yaml` 硬编码。

A2W Map Bundle 当前提供：

```text
metadata.yaml
global_map.pcd
optimized_poses.csv
keyframes/index.csv
keyframes/clouds/*.pcd
descriptors/scan_context.bin
config/*.yaml
manifest.sha256
```

Bundle metadata 已包含 `map_frame`（默认 `map`）、`odom_frame`（`camera_init`）和 `tracking_frame`（`body`）。但当前没有 Map Bundle → 2D OccupancyGrid 转换器。

结论：需要**A2W 重新实现** `a2w_nav_map`，从 `global_map.pcd` 执行高度过滤、地面/障碍处理和 2D 投影，输出与 Bundle 同一 `map` frame 的 PGM/PNG + YAML。不得重新估计或平移导航地图原点。

## 7. TF、Odom 与 frame 审计

### 7.1 旧 Go2 launch 期望的 tree

```text
map --(AMCL)--> odom --(odom_tf_broadcaster)--> body
body --(static)--> base_link --(static)--> livox_frame
```

该 tree 是通过重命名/复制 FAST-LIO pose 和旧 Livox 外参拼出的，不等于当前 A2W 真实契约。

分类：`odom_tf_broadcaster.py`、`body → base_link`、`base_link → livox_frame` 全部**禁止迁移**。

### 7.2 A2W 当前真实接口

- `/Odometry`：`nav_msgs/Odometry`，`header.frame_id=camera_init`，`child_frame_id=body`。
- timestamp：使用 FAST-LIO `lidar_end_time`；`/cloud_registered_body` 使用同一 `lidar_end_time`。
- pose：由 FAST-LIO state 填充。
- twist：FAST_LIO_Hesai `publish_odometry()` 中没有任何 twist 赋值，因此消息中 twist 保持默认零，不是可信的实时底盘速度。
- TF：FAST-LIO 直接发布 `camera_init → body`；Localization 发布 `map → camera_init`。
- `/localization/odom`：是 `map → body` 全局表达，不应代替 controller 的本地 odometry。

Nav2 中 odom topic 优先设为 `/Odometry`，但在 N2/N5 之前必须明确 controller/velocity feedback 如何获得可信 twist。不得用 pose 差分或常数悄悄伪造速度。当前本地 A2W 主工程内没有另一个已接入的底盘 velocity/odometry topic。

## 8. Obstacle source

旧默认链路是：

```text
/cloud_registered_body
  -> pointcloud_to_laserscan (target_frame=body, best-effort input)
  -> /scan
  -> local ObstacleLayer
```

但 collision monitor YAML 已经证明 Nav2 组件可直接消费 `/cloud_registered_body` PointCloud2。A2W 建议首选 PointCloud2 `VoxelLayer`/`ObstacleLayer`，并将 QoS、min/max height、range、clearing/marking 全部参数化。只有在实际插件或性能验证证明必需时，才增加 PointCloud2 → LaserScan adapter。

## 9. Footprint / robot radius

默认 YAML 的两个 footprint 还不一致：

- local：`[[0.30,0.175],[0.30,-0.175],[-0.30,-0.175],[-0.30,0.175]]`
- global：`[[0.30,0.175],[0.30,-0.175],[-0.38,-0.175],[-0.38,0.175]]`
- `robot_radius: 0.0`，依赖显式 footprint；padding 0.10 m。

这些是 Go2 数值，分类为**禁止按数值迁移**。N2 必须从 A2W URDF/官方尺寸/真实机体确认统一 footprint，再通过低速实机验收确定 padding。

## 10. `/cmd_vel` 与 Go2/A2W 底盘链路

### 10.1 旧 Go2 真实代码链

Humble 标准 `navigation_launch.py` 执行：

```text
controller_server
  -> /cmd_vel_nav
  -> velocity_smoother (OPEN_LOOP)
  -> /cmd_vel
```

`go2_driver` 原本订阅相对 topic `cmd_vel`，回调将 Twist 转为 JSON `{x,y,z}`，设置 Go2 Move API ID，发布 `unitree_api/Request` 到 `api/sport/request`。但在 `navigation2.launch.py` 中它被 remap 为订阅 `/cmd_vel_safe`。

由于 collision monitor 实际未启动，当前默认 launch 的实际链是：

```text
controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> [无 /cmd_vel_safe publisher]
                                                        Go2 driver waits /cmd_vel_safe
```

即默认链路不完整。

`cmdvel_to_go2_avoid.cpp` 是另一个直连 Go2 `ObstaclesAvoidClient::Move()` 的试验 bridge，具有 50 Hz 发送、0.2 s timeout 和限幅；但它在 CMake 中未构建，在 launch 中未启动，不属于当前运行链。

### 10.2 A2W 现状与缺口

- `fast_lio_ros2_ws` 中当前没有 `/cmd_vel` consumer、A2W motion bridge 或 `unitree_api` 依赖。
- 本机 Unitree SDK2 含 A2 `unitree::robot::a2::SportClient`，官方本地 example 支持 `Move(vx,vy,vyaw)` 和 `StopMove()`。这证明存在候选 API，但不证明当前 A2W 机器人系统已原生订阅标准 `/cmd_vel`。
- 由于本次无实机连接，A2W 的最终 command topic/service、QoS、控制权模式和安全停车语义均未验证。

因此 N5 在真实接口确认前不应猜测。若 A2W 无原生 `/cmd_vel` consumer，则**A2W 重新实现** `a2w_motion_bridge`，至少包含 vx/vy/wz 限幅、加速度限制、command timeout、LOST 零速、shutdown 零速和单一 command owner。Go2 driver 与 Go2 obstacle-avoid bridge **禁止迁移**。

## 11. Waypoint / mission 逻辑

直接相关脚本的实际行为：

| 脚本 | 接口 | 问题/定位 | 分类 |
|---|---|---|---|
| `waypoint_follower.py` | `NavigateToPose` action，`map` goal，读同目录 `pose.txt` | 自动开始；GO/BACK 延时语义；其中一处注释说 20 s 但 timer 是 10 s | 基础导航之后重新设计 |
| `multi_nav_new.py` | `NavigateToPose` action，`map` goal，`pose.txt` | 与上述功能高度重复，含非标准 `oneshot=True` timer 用法风险 | 基础导航之后重新设计 |
| `follow_smooth_path.py` | `FollowPath` action，读 `pose_D3.txt`，默认 `/odom`，发布调试 Path | 自行密化/滑窗平滑，与 Nav2 planner/smoother 职责重叠 | 后续 mission layer 需求审核后再实现 |
| `icp_seed_amcl.py` | 查 `map → base_link` TF，发 `/initialpose` | 属于旧 ICP → AMCL 定位链 | **禁止迁移** |

Phase N1–N9 不迁移巡航/mission；先完成 RViz 单点 Goal 闭环，再单独设计 waypoint package/API。

## 12. 根目录脚本审计

- `nav_new.sh`：依次启动 Livox driver、旧 FAST-LIO、pointcloud-to-scan、ICP、`robot_navigation2` 和 RViz。除“从一个入口启动系统”的概念外，实际链路全部禁止照搬。
- `nav_new_A.sh`：导航启动行均已注释，只启动 RViz；硬编码 domain 30 和 `<user-home>/cyclonedds/cyclonedds.xml`，不可迁移。
- `nav2_40m_debug.sh`：其“记录 topic/TF/Hz/BW/params/bag 检查点”思路可修改后迁移，但所有 `odom/base_link/livox_frame/scan_raw` 都必须换成 A2W 契约，输出路径必须便携。
- `mapping.sh`：启动 Livox + 旧 FAST-LIO，**禁止迁移**。
- `close_all.sh`：通过 GUI 工具和宽泛 `pgrep/pkill` 关闭窗口、节点乃至所有 `ros2 launch`，破坏范围过大，**禁止迁移**。

## 13. 复用/迁移/禁止矩阵

| 内容 | 分类 | 理由 |
|---|---|---|
| SmacPlanner2D、MPPI Omni、SimpleSmoother、标准 behaviors | 直接复用标准组件，参数重做 | 机器人无关 plugin，但数值是 Go2 特定 |
| Nav2 default BT 行为概念 | 修改后迁移 | 需定位门控和项目内可审计路径 |
| Global static + inflation / Local obstacle + inflation 结构 | 修改后迁移 | frame、source、QoS、range、footprint 需重做 |
| `/cloud_registered_body` 作为障碍源 | 直接复用 A2W 现有 topic | 当前 FAST-LIO 已输出 body-frame PointCloud2；实机 QoS/高度待验证 |
| `nav2_40m_debug.sh` 检查项思路 | 修改后迁移 | 不复制旧 frame/路径 |
| `navigation2.launch.py` | 不复制，A2W 重新实现 | 启动 AMCL、Go2 driver、伪 TF，且 cmd 链不完整 |
| `nav2_params.yaml` | 只作结构参考 | 含 Go2 frame/footprint/动力学/绝对路径 |
| Go2 maps 及 map YAML | 禁止迁移 | 不是 JT128 Map Bundle 地图 |
| `odom_tf_broadcaster.py` / `scan_stamp_fix.py` | 禁止迁移 | 前者伪造 frame；后者改写传感器时间戳 |
| Go2 driver / description / Unitree Go2 message chain | 禁止迁移 | 机型和 API 不同 |
| `cmdvel_to_go2_avoid.cpp` / `enable_go2_obstacle_avoid.cpp` | 禁止迁移 | Go2-only SDK API 与硬编码 NIC |
| Livox driver、旧 FAST-LIO、ICP、AMCL | 禁止迁移 | 违反当前 A2W 唯一定位链 |
| waypoint / smooth-path 脚本 | 延后，需求驱动重新实现 | 不阻塞单点导航，现有实现重复且不便携 |

结论：**没有一个 `robot_navigation2` 自有文件适合 byte-for-byte 直接拷贝到新 package**。“直接复用”指重新声明 ROS 2 Humble 已安装的标准 Nav2 plugin，以及消费 A2W 已有 topic，而不是复制 Go2 文件。

## 14. A2W 当前缺失的导航接口

1. Map Bundle → 2D navigation map 转换器。
2. A2W 真实 footprint/外形边界与安全 padding 来源。
3. `/Odometry.twist` 可信速度，或另一个经证实的底盘 velocity/odometry topic。
4. A2W 的实际 velocity command contract：topic/service/API、QoS、单位、取值范围、控制权和 stop acknowledgement。
5. `a2w_motion_bridge`（仅当 A2W 不原生消费标准 `/cmd_vel` 时需要）。
6. `LocalizationGuard`：将 `/localization/status` 状态映射为 Nav2 lifecycle/action/cmd 安全门。
7. cold-start 启动门：长期 `INITIALIZING` 时必须保持停止并拒绝 goal。
8. 与 `map/camera_init/body` 一致的 Nav2 config、launch、RViz 和 contract tests。
9. 可便携的 `scripts/run_a2w_navigation.sh` 和导航专用验收/调试脚本。

## 15. 推荐最终 ROS2 package tree

```text
src/
├── FAST_LIO_Hesai/                 # 保持当前唯一前端
├── a2w_fastlio_common/
├── a2w_fastlio_msgs/
├── a2w_fastlio_map/
├── a2w_fastlio_mapping/
├── a2w_fastlio_localization/
├── a2w_nav_map/                   # Bundle PCD -> 2D map + contract checks
├── a2w_navigation/
│   ├── launch/
│   ├── config/
│   ├── behavior_trees/
│   ├── rviz/
│   ├── include/a2w_navigation/
│   ├── src/                         # LocalizationGuard
│   └── test/
├── a2w_motion_bridge/             # 条件新增；A2W 接口确认后决定
└── a2w_fastlio2_bringup/
    └── launch/navigation.launch.py # 只启 Localization + Nav2，不启 Mapping
```

依赖方向应保持单向：

```text
a2w_navigation -> a2w_fastlio_msgs (+ standard Nav2 interfaces)
a2w_nav_map -> a2w_fastlio_map
a2w_fastlio2_bringup -> localization + navigation + nav_map (+ optional motion_bridge)
```

`a2w_navigation` 不应依赖 mapping backend；`a2w_motion_bridge` 不应依赖 Nav2 实现类型；禁止循环依赖。

## 16. 推荐运行架构

```text
JT128 + IMU
  -> FAST_LIO_Hesai
     -> /Odometry (camera_init -> body; pose/timestamp valid, twist not established)
     -> /cloud_registered_body (body)

Map Bundle/global_map.pcd
  -> a2w_nav_map (offline)
  -> navigation/nav_map.yaml + image (same map frame/origin)
  -> Nav2 map_server

a2w_fastlio_localization
  -> map -> camera_init
  -> /localization/status

Nav2: map/body global + camera_init/body local
  -> /cmd_vel_nav
  -> velocity_smoother / safety limiter
  -> LocalizationGuard
  -> optional a2w_motion_bridge
  -> verified A2W motion interface
```

同一时刻仍只允许 Localization 发布 `map → camera_init`；Navigation 模式绝不启动 Mapping。

## 17. Phase N1–N9 实施计划

### N1：Nav2 package skeleton，不接底盘

- 新建 `a2w_navigation`，显式声明 Humble Nav2 依赖。
- 重建 config/launch/BT/RViz，不复制 Go2 文件。
- 移除 AMCL、Go2 driver/description、Livox、ICP、旧 map。
- 测试 package dependency、launch syntax、YAML schema、plugin availability 和零 Go2/Livox 字符串。

### N2：frame / odom / topic 契约

- Nav2 frames 设为 `map/camera_init/body`，odom topic 设为 `/Odometry`。
- 增加 timestamp/frame/QoS/TF contract tests。
- 明确 controller 对 twist 的需求，确认 A2W 可信 velocity 源；不修改 FAST-LIO 来伪造数据。
- 从 A2W 机体资料确认 footprint，不延用 Go2 数值。

### N3：Map Bundle → 2D Navigation Map

- 新建 `a2w_nav_map`，读 Bundle/manifest/metadata/global map。
- 实现高度过滤、地面/障碍分类、2D rasterization 和 PGM/PNG + YAML 输出。
- 保留 `map` 坐标，使用格网 origin 表达 PCD 边界，不改变点的 map 坐标。
- 添加 synthetic PCD、ground/obstacle、origin/resolution、portable path 和 round-trip tests。

### N4：Global/Local Costmap 与 JT128 obstacle source

- Global costmap 连 map server static layer + inflation。
- Local costmap 首选直接订阅 `/cloud_registered_body` PointCloud2。
- 所有 QoS、height/range、frequency、clearing/marking 参数化。
- 增加离线 PointCloud2/costmap config contract tests；不声称 JT128 高度阈值已实机验证。

### N5：A2W motion interface / bridge

- 先在实机可用时只读确认 A2W 已有 command topic/service/API 和控制权。
- 若已原生消费标准 `/cmd_vel`，不新建重复 bridge。
- 否则新建 `a2w_motion_bridge`，实现 limit/acceleration/timeout/LOST/shutdown zero command。
- 通过 mock A2W sink 完成离线 timeout、clamp、rate limit、zero-on-state 测试。

### N6：LocalizationGuard

- 订阅 `/localization/status`，不修改 Localization 状态机。
- `INITIALIZING` 禁止激活/接受 goal；`LOCALIZED` 允许；`DEGRADED` 策略参数化；`LOST/RELOCALIZING` cancel goal + zero command。
- 重新 `LOCALIZED` 后要求重规划，不继续旧 path。
- 测试所有状态迁移、stale status、goal cancellation 和 cold-start block。

### N7：统一 bringup 和可便携脚本

- 新增 `a2w_fastlio2_bringup/launch/navigation.launch.py`。
- 参数至少包含 `map_bundle_path`、`navigation_map_yaml`、`nav2_params_file`、`rviz`、`autostart`、`use_sim_time`。
- 确保启动 Localization，不启 Mapping，不启 AMCL。
- 新增从脚本自身路径解析项目根的 `scripts/run_a2w_navigation.sh`。

### N8：离线集成验证

- launch/config/plugin/TF/frame/topic/QoS/map contract tests。
- synthetic map + fake localization status + fake odometry/cloud + mock motion sink。
- 验证 Mapping 未启动、Localization 启动、单一 `map → camera_init` owner、零 Go2/Livox/绝对路径。
- 验证 LOST/RELOCALIZING 无非零 command，以及 map/costmap frame 一致。

### N9：集中实机低速验收

1. 先验证 `/Odometry`、PointCloud2 QoS/frame/timestamp、可信 velocity 和 A2W command contract。
2. 等 Localization 真实进入 `LOCALIZED`，否则导航必须保持 blocked。
3. 在安全区域以低速发送短距离单点 goal，确认 plan、controller、safety chain、A2W 运动和到达。
4. 再独立验证 `DEGRADED/LOST/RELOCALIZING/LOCALIZED` 期间的暂停、停止和重规划。
5. 根据实机结果调整 footprint、速度、加速度、costmap 高度/距离、MPPI 和 collision monitor。

N9 之前的成果只能标记为 offline implementation/verification，不得宣称 Topic/QoS/Frame/实时性/运动安全已通过 A2W/JT128 实机验证。

## 18. 设计确认前的决策点

Phase N1 开始前建议确认以下设计：

1. 接受“不复制 `robot_navigation2`，重建 A2W-native Nav2 package”。
2. 接受 SmacPlanner2D + MPPI Omni 作为 V1 候选，参数全部从安全低值重新标定。
3. 接受 local costmap 首选 PointCloud2，不默认增加 LaserScan adapter。
4. 接受独立 `a2w_nav_map` package 作为 Map Bundle → OccupancyGrid artifact 生成器。
5. 接受 motion bridge 为“条件新增”；在 A2W 真实接口确认前不猜测实现。
6. 接受 cold-start 限制：未 `LOCALIZED` 不激活/不接受导航 goal，不在本次导航迁移中偷改 Global Relocalization。

设计确认前不进入 N1，不创建 navigation package，不接底盘，不修改任何机器人端状态。
