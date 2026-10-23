# A2W Navigation 当前实现

原 N1 配置阶段之后，已接通定位、地图、JT128 scan/costmap、MPPI Omni、
velocity_smoother、LocalizationGuard、MotionBridge、RViz 和连续 FollowPath。
当前使用 `map→camera_init→body→base_link`，不引入 AMCL、Go2 前端或第二套定位。
Unitree 接口定义已随仓库提供；驱动和运动模式仍依赖现场机器人。

完整启动入口为 `scripts/run_a2w_navigation.sh`；配置位于
`src/a2w_navigation/config/nav2_params.yaml`，RViz 位于同包 `rviz/navigation.rviz`。
默认速度上限保持 vx=0.60m/s、vy=0.15m/s、wz=0.50rad/s。
当前 local/global inflation 分别为 0.30/0.25m，保留已确认的 A2W footprint。
本轮仓库整理不重新调整上述运动和定位参数。

当前运动链为 MPPI → velocity_smoother → cmd_vel →（可配置 Collision Monitor）
→ LocalizationGuard → MotionBridge → Unitree Sport request。
导航入口默认旁路 Collision Monitor，并在摘要中明确显示；local costmap/MPPI 仍参与避障。
LOCALIZED、新鲜 pose/TF 和 Nav2 就绪才允许授权。其他状态取消任务并零速度，
不自动恢复旧任务。请按 [运行手册](RUNTIME_USER_GUIDE.md) 操作。

二维地图使用 [a2w_nav_map](2D_MAP.md)，不得把未知物理竖直的原地图直接假定为平面。
路线采集/FollowPath 见 `a2w_waypoint_tools`；直接 FollowPath 不包含完整 NavigateToPose BT recovery。

离线构建和测试、已有局部实机经历、当前机器人整机验收是不同证据。
新部署设备与所有改动组合仍需按现场时钟、坐标、地图、障碍和运动接口验收。

`Offline implementation and verification complete; JT128 hardware validation pending.`
