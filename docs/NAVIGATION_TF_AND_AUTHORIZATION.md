# 全局 TF 发布与导航授权修正（2026-09-23）

## 已确认的现场证据

- FAST-LIO odom、`camera_init → body` 和 Localization pose 约 10 Hz；
  `map → camera_init` 受 owner heartbeat 限制，仅 4 Hz。
- 关闭运动输出的采样中，控制器原始 `/cmd_vel_nav` 约 8 Hz，
  smoother `/cmd_vel` 仍为 20 Hz，后者不代表控制器达标。
- 独立 TF 查询复现中，141 次查询有 41 次超过 50 ms；
  map/camera_init 指定时间查询 p95 约 142 ms、最大约 162 ms。
  这确认了 TF 等待贡献，不代表已排除 MPPI、costmap 的其他开销。
- 定位从 DEGRADED 恢复后，旧版 Nav2 可 READY，但 Guard 仍为
  `localized_waiting_for_arm`，导致接收 Goal 后零速度、随后进展超时。

## 最小修改

1. Localization 新增 `global_tf_publish_period_ms: 50`，独立于
   `owner_heartbeat_ms: 250`。只重发最近有效 correction，保留有效性和
   TF 唯一发布者检查；不运行额外配准，不改变 FAST-LIO 或定位算法。
2. `/navigation/safety_status` 增加 `motion_authorized`（操作者授权）；
   原 `motion_enabled` 表示实际速度门已放行。授权可先于 Nav2 激活，
   实际放行仍需定位健康和 Nav2 就绪，避免互相等待。
3. 无授权、非 LOCALIZED 或安全心跳丢失时暂停 Nav2。恢复定位不恢复旧授权，
   也不自动恢复旧 Goal；需重新授权，再发送新 Goal。
4. 授权心跳超时参数 `authorization_timeout_s` 默认 1 秒。
   `/navigation/nav2_ready` 为 transient-local 状态，仅状态变化时发布，
   避免与 safety_status 构成通知反馈环。
5. 原有低频导航摘要增加 `motion=... reason=...`，区分“定位好了”与“可运动”。

## 隔离版本与现场复测

新代码隔离构建到 `build_tf_gate` / `install_tf_gate`，不覆盖现场运行版本。
先用遥控器确保静止，执行 `./scripts/a2w_stop_now.sh`，在原导航启动终端
Ctrl+C，确认旧导航已退出后再启动。不要同时启动两套定位/导航。

```bash
cd ~/fast_lio_ros2_ws
conda deactivate 2>/dev/null || true
A2W_NETWORK_INTERFACE=enp49s0 A2W_INSTALL_SETUP="$PWD/install_tf_gate/setup.bash" ./scripts/run_a2w_navigation.sh maps/factory_large_v3 auto_enable_motion:=false
```

该命令保持 Guard 和 MotionBridge 输出关闭。等待 LOCALIZED 后，在另一终端
加载相同 ROS/DDS 环境，只授权 Nav2 计算，**不启用 MotionBridge**：

```bash
cd ~/fast_lio_ros2_ws
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source install_tf_gate/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
ros2 topic echo --once /a2w_motion_bridge/status
ros2 service call /navigation/enable_motion std_srvs/srv/SetBool '{data: true}'
```

先确认 bridge 的 `hardware_output_enabled=false`，再在 RViz 发短距离 Goal。
此时 Guard 可放行到 bridge 输入，但 bridge 不向机器人输出非零运动命令。
静止机器人最终触发进展超时是此测试的预期限制；只短时采样，不据此评估跟踪效果。

```bash
timeout 15 ros2 topic hz /cmd_vel_nav --window 50
ros2 topic echo --once /navigation/safety_status
```

应在新版本复测全局 TF 分支频率、TF 等待及控制器周期后，再决定是否还需
分析 MPPI/costmap 开销。`/tf` 总频率不能当作 map/camera_init 分支频率。
本修改不把 controller_frequency 从 20 Hz 改成 10 Hz，也不放宽安全阈值。

## 验证范围

相关包编译与离线测试包含：TF 发布独立于 owner 心跳、原 Localization 两个
launch 集成场景、授权/暂停/重新授权、授权心跳断开和无状态通知反馈环。
导航联动测试使用真实 Guard/Gate 节点和模拟生命周期服务，不接实机运动接口；
不等价于真实 Nav2 所有生命周期/Action 行为验收。

新版尚未完成 JT128 实机复测，不能据此声称控制器已经稳定达到 20 Hz。
