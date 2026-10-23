# 完整运行与使用手册

先按 README 执行 `./setup.sh --install-system`、`./build.sh`，每个操作终端加载
`source ./env.sh`。所有路径以克隆后的项目根目录为基准。

## 模式和坐标

Mapping 与 Localization 不能同时运行，`map → camera_init → body` 由后端与前端分别发布；
`body→base_link` 由现有 URDF fixed joint 和 robot_state_publisher 发布。
Nav2 消费 `map`、`camera_init`、`base_link` 与 `/Odometry`。
不要根据 frame 名称推断其物理朝向，外参需沿用对应机器人已确认的配置。

## 雷达和时间

机器人端先按已有 Hesai 驱动流程启动 `/lidar_points` 和 `/lidar_imu`。
PC/板载使用路由自动选网卡；必要时配置 `A2W_NETWORK_INTERFACE`、`A2W_PC_IP`、
`A2W_DDS_PEER`。`./scripts/check_lidar_topics.sh` 读取输入类型、QoS 和发布者。
先检查传感器、TF 与系统时间域一致，再启动导航。NTP 已同步不保证 LiDAR 硬件时间已同步；
不要用调大 TF 容差或重写时间戳掩盖百秒级偏差。脚本不会自动改系统时钟。

```bash
./scripts/check_lidar_topics.sh
./scripts/run_fastlio_jt128_pc.sh rviz:=true
```

## Mapping 与保存

```bash
./scripts/run_a2w_mapping.sh rviz:=true
```

终端在真正进入候选验证、接受/拒绝回环及图优化完成时输出中文提示。
回环成功只说明对应约束已接受，不保证整个地图几何正确。
后端统计 received、processed、pending；保存会冻结接收范围并排空队列。

```bash
# 另一个终端，保持 Mapping 继续运行。
source ./env.sh
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
ros2 service call /mapping/save_map_bundle \
  a2w_fastlio_msgs/srv/SaveMapBundle '{output_path: factory_map}'
ros2 run a2w_fastlio_map map_bundle_inspect maps/factory_map
```

只有服务返回 success、received==processed 且 pending/in-flight 清空后才算保存成功。
不要在 Draining 期间 Ctrl+C 或 SIGKILL。输出包括 metadata、manifest、局部关键帧、
原始/优化位姿、Scan Context 描述子和 `global_map.pcd`。
`/mapping/optimized_map_preview` 是 RViz 预览，不是完整总图；
配准详情见 `/mapping/registration_status`。

## 二维地图

使用 [一键二维地图工具](2D_MAP.md)。先明确竖直参考；没有可靠参考时不假定地图 Z。
确认原 map Z 合适后：

```bash
./scripts/generate_2d_map.sh --map-bundle maps/factory_map \
  --config config/map_2d.yaml --confirm-map-z --output maps/factory_map/navigation
```

输入只读，默认拒绝覆盖既有输出。Nav2 的 PGM/YAML 必须和 Localization 使用的坐标一致。
如果投影改变坐标轴，应先处理 `conversion.yaml` 中的 frame/变换，再接入导航；
不能只旋转 PGM 而保持定位坐标不变。地图局部变形无法靠二维投影校正。

## Localization

```bash
./scripts/run_a2w_localization.sh maps/factory_map
```

当前冷启动使用 GlobalRelocalizer（Scan Context Top-K → Quatro → Nano-GICP → 多帧确认）。
配置 `initialization.mode=fixed_region` 默认限制在起点附近 3×3m；更换地图或启动位置时
须调整 `relocalization.yaml` 中的初始化区域，或显式选择 `global`。
LOST / RELOCALIZING 使用全局重定位；不会因为机器人位于 B 点就从 map(0,0,0)传播。
LOCALIZED 的后台 normal match 使用 prediction + Local Map + fine-only；失败后按现有
状态机升级 full registration。FAST-LIO pose propagation 与配准分离。

```bash
source ./env.sh
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
ros2 topic echo /localization/status
ros2 topic echo /localization/registration_status
ros2 run tf2_ros tf2_echo map base_link
```

状态为 INITIALIZING / LOCALIZED / DEGRADED / LOST / RELOCALIZING。
correction 的 source stamp、接受时间和匹配耗时含义不同；地图/时钟/外参变化都需要重新检查。
手动重定位 Service：当前未实现。停止任务后可按现有全局恢复或重启定位流程处理。
当前未实现的能力不能因为 RViz 中看见点云就被当作已经验证。

## Nav2、采点和连续路线

```bash
./scripts/run_a2w_navigation.sh maps/factory_map \
  maps/factory_map/navigation/nav_map.yaml auto_enable_motion:=false
```

确认 LOCALIZED、新鲜 TF、costmap 和障碍正常。脚本默认 RViz 自动打开；
真正执行时可不传 `auto_enable_motion:=false`，沿用现有自动准备门和故障关闭逻辑。
不会自动发送目标。Collision Monitor 默认按当前已确认配置旁路；Local Costmap + MPPI
仍承担局部避障，旁路保护状态会在终端摘要显示。

```bash
# 手控采点模式：不启动运动桥，避免与遥控命令竞争。
./scripts/run_a2w_navigation.sh maps/factory_map \
  maps/factory_map/navigation/nav_map.yaml start_motion_bridge:=false auto_enable_motion:=false

# 另一个终端。
source ./env.sh
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
ros2 run a2w_waypoint_tools waypoint_recorder --ros-args \
  -p output_prefix:="$PWD/maps/factory_map/routes/ab"
```

S/s 或空格记录 `map→base_link`；Ctrl+C 保存 YAML/CSV。使用遥控器返回 A，
停止采点启动终端，再启动完整导航。连续轨迹仅加密原始锚点，默认不做移动平均。

```bash
./scripts/run_a2w_navigation.sh maps/factory_map maps/factory_map/navigation/nav_map.yaml

# 预览，确认后 Ctrl+C，再加 execute:=true。
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/factory_map/routes/ab.yaml"
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/factory_map/routes/ab.yaml" -p execute:=true
```

FollowPath 使用现有 MPPI 和 local costmap；避障可以偏离参考轨迹，之后继续跟踪后续路径。
直接 FollowPath 不具备 NavigateToPose BT 的完整全局重规划/Recovery；持续受阻时会失败，
客户端不会自动恢复旧目标。剩余路径长度为零不代表已到达，最终看 Action 和终点 XY/yaw。
导航状态异常时取消任务和零速度，恢复 LOCALIZED 后需显式重新下发任务。

```bash
./scripts/a2w_stop_now.sh
```

停止脚本依次关闭最终运动桥、导航门并取消 NavigateToPose/FollowPath。
停止软件本身请在启动终端 Ctrl+C。实体急停仍由现场操作人员掌握。

## 实机验收和结果边界

```bash
./scripts/run_jt128_full_validation.sh --help
```

验收入口收集 Topic/QoS/Frame/timestamp、匹配、回环、TF、地图及资源信息。
离线实现状态可记录为：

`Offline implementation and verification complete; JT128 hardware validation pending.`

这不是对所有后续改动的实机认证。当前设备、外参、场地地图、时间源与负载均影响结果。
本轮仓库整理只验证源码、构建和离线测试，不重新采图或宣称新实机验收完成。
