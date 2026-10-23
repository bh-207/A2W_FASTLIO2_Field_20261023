# Global → fine-only 同帧诊断

本探针不调整算法、阈值、Local Map、Map Bundle 或 Nav2。默认关闭。
仅在初始全局定位确认之后，对一帧新的、严格同 stamp 的 cloud + odom 执行一次：

- A：当前 fine-only 管线。
- B：当前 Quatro → Nano-GICP full 管线。

两者使用同一个 source、同一个 prediction、相同的关键帧 ID 和同一个
已构建的 Local Map 对象，串行运行；只有 A 按现有逻辑交给 Manager，
B 只记录，绝不替代 A 或回写 correction。该帧的 normal match 总耗时包含
A/B 诊断开销，不应作为正常运行性能统计；分别看 A、B 的 match_latency_ms。

仅用于机器人静止、Nav2 未运行的定位诊断，不是导航启动配置：

```bash
cd ~/fast_lio_ros2_ws
A2W_NETWORK_INTERFACE=enp49s0 ./scripts/run_a2w_localization.sh maps/factory_map_final rviz:=false handoff_ab:=true
```

出现 `[handoff_ab]` 摘要后可 Ctrl+C 停止此次 PC 诊断。日志自动保存在
项目的 `log/fast_lio_runtime/localization_node_*.log`。不启动或停止机器人端服务。
正常运行省略 `handoff_ab:=true` 即可。

## 时间与变换语义

- correction_stamp 是已接受配准的 source stamp，不是接收时刻。
- 修正前 correction_age 是最新输入 stamp 减 correction_source_stamp。
  修正后 `/localization/status.correction_age_ns` 与 monitor 的 stale 比较，
  都使用 `steady_now - correction_accepted_steady_stamp`，即接受后的时长。
  Manager 在实际接受 normal/global correction 的锁内记录该时刻；
  失败、跳变拒绝、过期任务和高频 propagate 均不会刷新该时刻。
- source_age_ms：诊断开始时 ROS now 减本次 source stamp；时钟不同步也会影响它。
- match_latency_ms：各次配准的 steady-clock 耗时，不含等待队列时间。
- correction_accepted_age_ms：诊断开始距离上次接受 correction 的 steady-clock 时长
  （上一轮 A/B 的历史日志字段名为 correction_validation_age_ms）。
- correction_source_age_ms：本次 source stamp 减已接受 correction 的 source stamp。

`correction_stale_after_ms=3000` 不变。既有 monitor 在连续匹配失败达到门限后
才降级；age 影响 reason 标签，不会单独触发降级。修正不引入新的状态转换或阈值，
不调整 GlobalRelocalizer、任何配准算法、Nav2 或 Map Bundle。

Global 的 correction 计算使用该任务自带的同 stamp odom：
`T_map_camera_init = T_map_body(source) × inverse(T_camera_init_body(source))`。
跟踪预测：`T_map_body(current) = T_map_camera_init × T_camera_init_body(current)`。
这个 body → map 初值经过 RegistrationPipeline 传入 `solver.align`，Nano-GICP
内部的初始优化位姿 x0 使用的就是该矩阵。Quatro 的现有适配器不使用传入的
初值，B 的 GICP 初值取自 Quatro 输出；这正是 A/B 要比较的差异。

摘要给出实际初值 xyz/quaternion、独立 correction × odom 的误差、关键帧 ID、
点数和两条管线的结果。不收敛时 overlap/correspondences 可能是未经几何证据
计算的默认零值，不能据此直接宣称真实点云零重叠。

## 2026-09-21 22:11 现场一次性结果

日志：`log/fast_lio_runtime/localization_node_256680_1789999880270.log`。
仅启动 PC FAST-LIO + Localization，未启动 Nav2，采集后两个 PC 进程均已正常退出。

- source stamp：1789999934299807071 ns，body 点云 9922 点。
- correction source stamp：1789999896799818992 ns。
- source_age：82.130 ms；correction_source_age：37499.988 ms；
  correction_validation_age：468.631 ms（均在 A/B 开始前采样）。
- selected IDs `[0, 6, 1]`；两条路径共享 990 点 target。
- prediction 与独立 correction × odom 对比：平移、旋转误差均打印为 0。
- 实际初值 xyz `[-0.055157, -0.010584, -0.003951]`，
  quaternion xyzw `[-0.021134, -0.001715, -0.098332, 0.994928]`。
- A：accepted/converged，1074.602 ms，fitness 0.142629，overlap 0.966640，9591 对应点。
- B：accepted/converged，2958.432 ms，fitness 0.142628，overlap 0.966640，9591 对应点。
- 之后 7 次在线 fine-only 全部 accepted，耗时 858–1495 ms。

本次证明旧 source 的 correction 可以正确传播，也验证了实际初值传递；
未复现上一轮不收敛，不得据此声称所有现场问题已解决，或认定一定是异步 bug。
无 Nav2 的短时采样不能排除原运行场景的输入/局部地图差异和计算资源竞争。

验证：本次相关两个 package 的定向构建/安装成功；Localization 单包 CTest
8 组核心测试与 2 个 ROS2 launch 测试均通过。未进行全工作区回归。

## 在线 fine-only 失败诊断（无需打开 A/B）

每个失败任务只输出一条 `[fine_only_failure] 在线精配准失败摘要` 中文日志。
成功 fine-only 仅保存一份小型元数据记录，不持有点云，不重跑算法，不输出新周期日志。
摘要包括当前和最近一次**被 Manager 接受**的 fine-only 记录：

- source/odom stamp、source age（匹配开始与结束），frame pair 在 adapter 中严格同 stamp；
- 当前 correction source stamp / accepted age，以及预测实际使用的 correction source stamp / accepted age；
- FAST-LIO odom、预测 map pose、实际 Nano-GICP initial guess（xyz + quaternion）；
- 所选关键帧 ID、optimized pose 到预测位置的 3D 距离；
- selection 所属任务 stamp 与完成任务 stamp 是否一致；
- 匹配耗时、Nano-GICP 耗时、点数、失败原因和 stale-result 标志；
- 与上一成功帧的预测及 FAST-LIO 位姿平移/旋转差。

Global/full 成功和过期任务不会冒充“上一成功 fine-only”；没有上一成功时明确写“无”。
这里的位姿差只是观测值，不新增“跳变”阈值或自动修正。正常每次匹配的旧控制台
metrics 改为 DEBUG；`/localization/registration_status` 的发布保持不变。

本次 accepted-age 及失败摘要修正属于离线验证，不能代替后续故障现场的复现结论。
定向构建通过；Localization 单包 9 组核心测试及 2 个 ROS2 launch 测试通过。
