# Collision Monitor 旁路试验

2026-09-21：按现场要求，一键脚本 `run_a2w_navigation.sh` 默认传入
`use_collision_monitor:=false`。底层 launch 默认仍为 `true`，显式参数优先于脚本默认值。

旁路时不启动 `collision_monitor` 及其 lifecycle manager，速度链为：

```text
MPPI / Recovery → velocity_smoother → /cmd_vel
→ LocalizationGuard → /cmd_vel_guarded → a2w_motion_bridge
```

局部障碍层、膨胀层、MPPI footprint 碰撞检查继续启用。速度限制、定位安全门、
运动桥 watchdog 和停止脚本保持原行为。旁路意味着取消独立的 SlowZone/StopZone
减速和停车保护，不代表任何障碍都可以绕行。

启动（同时包含 Localization，不应另开一个定位实例）：

```bash
A2W_NETWORK_INTERFACE=enp49s0 ./scripts/run_a2w_navigation.sh maps/factory_map_final maps/factory_map_final/navigation/map.yaml
```

终端显示 `safe=BYPASSED(已旁路)`；`/cmd_vel_safe` 在此模式下不在运动链中。
恢复 Collision Monitor 需要重启，并在上面的命令末尾加：

```text
use_collision_monitor:=true
```

立即停止仍使用 `./scripts/a2w_stop_now.sh`。

## 这轮现场日志的边界

21:31 这一轮不仅有 Collision Monitor 减速，还出现了两轮
`DEGRADED → LOST → RELOCALIZING`。此时 Nav2 暂停、取消 Goal 和零速度输出
来自定位安全门，旁路 Collision Monitor 不会消除这些停车。

恢复 LOCALIZED 后，安全门不会自动恢复旧 Goal。需确认状态和位置正常后重新启用
`/navigation/enable_motion`（`std_srvs/srv/SetBool`，`data: true`），再重新发 Goal。
单独看到 `nav2=READY` 不等于 `motion_enabled=true`。

`Control loop missed its desired rate of 20.0000Hz` 表示计算/调度没有赶上每次 50 ms
的周期，并不是输出高于 20 Hz。此项旁路未修改控制频率、MPPI 或定位算法，也不承诺
消除超时警告。旧的 raw/safe 摘要是最近一次收到的数值，不证明暂停期间还在发新速度。

改动验证：launch 参数与节点条件测试覆盖启用/旁路两种模式，并确认旁路仍经过
定位安全门。实机旁路后的掉头和避障效果待下一次现场测试。
