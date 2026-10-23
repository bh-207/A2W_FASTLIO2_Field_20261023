# A2W 路径采点和连续跟踪

在项目根目录编译后加载 `source ./env.sh`。使用与导航相同的 DDS 配置：

```bash
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
ros2 run a2w_waypoint_tools waypoint_recorder --ros-args \
  -p output_prefix:="$PWD/maps/factory_map/routes/ab"
```

LOCALIZED 且 TF 可用时，按 S/s 或空格记录 `map→base_link`。
Ctrl+C 保存 YAML、CSV 并退出。采点时使用导航的
`start_motion_bridge:=false auto_enable_motion:=false` 模式，手控移动。
勿同时让运动桥和遥控器控制机器人。

```bash
# 默认仅预览；确认完整参考路线后，再明确执行。
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/factory_map/routes/ab.yaml"
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/factory_map/routes/ab.yaml" -p execute:=true
./scripts/a2w_stop_now.sh
```

每个原始 XY 采样点作为固定锚点，间距默认加密到不超过 0.20m。
默认不做移动平均；显式开启平滑时只移动插入点，最大偏离 0.03m。
中间朝向沿轨迹切线，B 点保留记录朝向。只发送一次 `/follow_path`，中间点不停留。
MPPI 和 local costmap 可使机器人为避障偏离参考线，之后继续跟踪后续路径。
直接 FollowPath 不包含 NavigateToPose BT 的完整全局重规划/Recovery；持续受阻会失败。

发送前检查 LOCALIZED、Nav2 READY、两道运动门、新鲜 TF、距 A 距离、中心线和 footprint
通行性。状态或 TF 异常取消本任务，不自动重发。沿线剩余为零不等于到达；
终点日志另给出真实 map TF 的 XY/yaw 估计误差和轨迹偏差。

| 参数 | 默认 | 用途 |
|---|---:|---|
| `execute` | false | 显式 true 才执行 |
| `spacing_m` | 0.20 | 加密点间距 |
| `smooth_window` / `smooth_passes` | 5 / 0 | 默认只加密、不均值平滑 |
| `start_tolerance_m` | 0.5 | 需先靠近 A，不自动导航到 A |
| `tf_max_age_s` | 1.0 | TF 时效检查，不能掩盖时钟不同步 |
| `pose_jump_translation_m` / `pose_jump_yaw_rad` | 0.60 / 0.20 | 连续性修正余量，另计运动上界×时间 |
| `pose_jump_max_speed_mps` / `pose_jump_max_yaw_rate_rps` | 0.80 / 0.75 | 连续性检查使用的运动上界 |
| `rejoin_segment_index` | -1 | 显式重接原路线段 i→i+1；-1 从 A 开始 |
| `wait_timeout_s` / `request_timeout_s` | 120 / 5 | 等待就绪、单次服务超时 |
| `global_frame` / `robot_frame` | map / base_link | 必须与记录一致 |
| `controller_id` / `goal_checker_id` | FollowPath / goal_checker | Nav2 插件 ID |

重接时需要重新确认定位并授权，所选段、当前位置连接段和剩余路径均重新检查。
旧进程不自动挑最近分支，也不恢复旧目标。`route_path.py` 生成/加密轨迹，
`waypoint_recorder.py` 保存采样点，`route_follower.py` 提交 Action 并记录反馈。

终点横移补偿沿用现有默认关闭策略；必要时通过导航参数显式开启，必须实机验证。
它依赖新鲜的 `/route_follower/terminal_error_body`，不更改 MPPI 算法。
停止和授权接口详见项目 [运行手册](../../docs/RUNTIME_USER_GUIDE.md)。
