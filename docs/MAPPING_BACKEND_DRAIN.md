# 大地图 Mapping 后端处理与完整保存

## 改动及边界

后端接收同步的 keyframe cloud/odom 后，立即存入现有 KeyFrameProvider。
处理游标顺序消费这些关键帧；积压不再经过容量 32 的 LoopPipeline 队列。
`loop_pipeline.queue_capacity` 保留兼容独立 LoopPipeline 调用者，后端不使用该队列。
这避免队列满导致后续关键帧被拒收，且不再为积压建立另一份点云副本。

正常处理仍执行原 Scan Context、回环配准、验证和 iSAM2 更新。
只更新 optimized path/odom、correction 和用于预览的位姿快照，不拼接完整地图。
回环算法、参数以及最终点云保存精度没有调整。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `preview_period_ms` | 5000 | 独立回调低频构建 RViz 预览；无新位姿时不重建 |
| `preview_voxel_leaf_m` | 0.5 | 只影响预览 |
| `preview_max_points` | 100000 | 预览发布点数上限 |
| `full_map_voxel_leaf_m` | 0.0 | 保存时完整地图的体素参数，0 表示不降采样 |
| `status_period_ms` | 5000 | 后端进度日志及 registration_status 摘要周期 |
| `mapping_shutdown_grace_s`（launch 参数） | 600 | launch 在 SIGINT 后给后端排空的宽限时间 |

预览读取不可变位姿快照，不占用配准工作线程或图优化互斥锁。
保存/退出时取消预览；取消检查覆盖点云拼接阶段及滤波前后。
PCL 的单次滤波、排序和正在执行的配准不能强制抢占。

## 状态统计

每个统计周期输出类似：

```text
[MappingBackend] received=391 processed=320 pending=71 active=320 state=receiving ...
```

相同内容通过既有 `/mapping/registration_status` 的 `stage=backend` 发布。
`pending=received-processed` 包含正在处理的那一帧。
`last_completed_stamp_ns` 是最近一次处理完成的节点时钟时间；
`last_completed_age_ms` 和 `processing_ms` 使用单调时钟。
`processing_ms` 包含该帧的回环检测/配准、图更新和轻量输出，
`preview_ms` 单独记录最近一次预览耗时。
`full_map_builds`、`preview_map_builds` 显示构建次数。
回环接受时另记 current/candidate ID 和 factor 数，便于日志核实。

`received` 指后端已接收的有效、同步、不重复关键帧，**不是前端发布计数**。
DDS 丢失或 ExactTime 未配对的消息不在此计数内，现场必须对比 ingress 日志与 received。
冻结后到达的消息记入 `ignored_after_freeze`，不会进入本次地图。
无效 frame/点云及后端处理异常会进入 fault，禁止把缺帧快照保存为完整地图。

## 保存与退出顺序

### 中文回环事件提示（2026-09-23）

候选实际进入处理时输出 `[回环检测]`，通过候选预检查后、运行配准前输出
`[回环验证]`。配准/验证拒绝输出 `[回环失败]`，包含中文原因与原始原因码；
候选歧义提前拒绝时不声称运行过 Quatro/Nano-GICP。无候选和冷却期不新增中文事件日志。

`[回环成功]` 表示配准与约束检查通过、约束已入队；紧接着的 `[图优化]` /
`[图优化完成]` 才代表该回环的图更新开始/成功提交。失败只输出 `[图优化失败]`。
位置/旋转修正是当前关键帧相对本次图更新前输入位姿的变化，包含图优化的综合影响，
不是 fitness 的米制转换，也不是所有关键帧的最大修正。fitness 保持算法原始数值。

程序不能仅凭回到起点认定“最终回环”，也不会保证绕一圈必然通过回环验证。
结束意图由现有保存服务明确给出，无需新增结束服务或改变算法。

保存提示严格按实际阶段输出：

```text
[结束采集] 已冻结后端接收边界
[保存等待] 正在排空建图队列……（仅有积压时，跟随低频进度统计）
[队列排空] received=N processed=N pending=0 active=none
[最终优化] 正在执行最终位姿图更新
[最终优化完成] 位姿已提交（尚未写盘）
[地图同步] 正在按最终位姿构建完整地图
[地图就绪] 位姿、点云、描述子已同步，正在写入 Map Bundle
[保存成功] 输出路径与关键帧数，可以关闭建图程序
```

回环处理和图更新都在同一关键帧处理任务中同步执行，没有另外的回环队列或优化线程。
因此冻结后 `active=none`、`received==processed`，再持后端锁成功完成最终图更新，
才是无待处理任务/因子的依据；不会用瞬时 `pending=0` 提前宣布地图同步完成。
`[地图就绪]` 只在完整 Bundle 内存快照构建完成后输出，**此时仍不能关闭程序**。
必须等 `[保存成功]` 和服务 `success: true`。失败不输出成功，可修正写盘问题后重试。
没有已提交回环时另外警告“尚无已提交的回环”，仍允许保存现有里程计地图。

本轮 Release 构建及 Mapping 包全部 12 组 CTest（9 组单测、3 组 ROS2 集成测试）通过。
新增测试验证回环通知发生在配准前/图提交后、歧义拒绝不谎报配准、通知异常不改变结果，
并验证 391 帧冻结排空保存的中文阶段顺序、未排空/失败不提示保存成功。
这是隔离 ROS Domain 的离线合成验证，尚未完成本轮 JT128 实机回环验收。

### 本轮现场命令

先停止 Navigation / Localization 及旧 FAST-LIO，确认只运行一套前端。
只需停止机器人移动即可等待回环处理，不要提前 Ctrl-C 建图。启动前核对 PC 与机器人
时间及 JT128 Topic；本次日志修改不会改动驱动或自动校时。

终端 1（此版本 Mapping 安装在主 `install`，不需要导航专用 overlay）：

```bash
cd ~/fast_lio_ros2_ws
conda deactivate 2>/dev/null || true
export PATH="/usr/bin:/bin:$PATH"
export ROS_DOMAIN_ID=0
export A2W_NETWORK_INTERFACE=enp49s0
export A2W_INSTALL_SETUP="$PWD/install/setup.bash"
./scripts/check_lidar_topics.sh
./scripts/run_a2w_mapping.sh rviz:=true
```

低速手控采集；最后回到起点附近，尽量重访相同结构与视角，观察回环事件。
当前配置排除最近 30 个关键帧、Top-K=5、成功回环后间隔至少 10 帧，这些参数未改。
停住机器人后让前端/入口把末段数据发完，核对入口计数与后端 received，再发起保存。
`received` 不能证明 DDS 传输前的关键帧没有丢失。

终端 2（使用时间戳新目录，不覆盖旧地图）：

```bash
cd ~/fast_lio_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
MAP_NAME="factory_large_$(date +%Y%m%d_%H%M%S)"
echo "本次保存目录：$PWD/maps/$MAP_NAME"
ros2 service call /mapping/save_map_bundle a2w_fastlio_msgs/srv/SaveMapBundle "{output_path: '$MAP_NAME'}"
ls -lh "maps/$MAP_NAME/global_map.pcd" "maps/$MAP_NAME/metadata.yaml"
```

保存服务等待不加短 timeout。`success: true` 后才在终端 1 按 Ctrl-C。
最终优化点云为 `maps/<本次目录>/global_map.pcd`；RViz preview 不是完整保存地图。

机器人停止采集路线后，保持 Mapping 运行，在已 source 工作区的终端调用原服务：

```bash
ros2 service call /mapping/save_map_bundle a2w_fastlio_msgs/srv/SaveMapBundle "{output_path: 'factory_large_v2'}"
```

请使用未存在的目录名。保存服务依次执行：

1. 检查输出路径，冻结后端接收边界。已进入接收回调的帧纳入本次边界。
2. 等待 `received == processed`、`pending == 0`、`active=none`。
3. 对既有因子进行最后一次 iSAM2 update，核对图节点数与接收数一致。
4. 构建一次完整 global_map，生成关键帧、描述子及元数据，交给原 Map Bundle writer。
5. writer 成功后才返回 `success: true` 和完整 keyframe_count。

排空期间服务保持等待，终端持续显示进度，不要用短 `timeout` 终止服务调用。
保存成功后接收保持冻结，本次建图结束；重新采集需重新启动 Mapping。
保存失败也不自动恢复接收，可以修正路径/磁盘问题后重试；处理错误不能靠重试绕过。
并发保存请求返回 `save_busy`。

**先等保存成功，再在启动终端 Ctrl-C。** SIGINT/SIGTERM 会冻结接收并排空待处理帧，
不新增全图构建，也不自动选择保存路径。没有提前保存就直接退出会失去内存中的地图。
保存期间收到退出信号，会等待这次保存结束。
launch 默认给后端 600 秒排空，再经过 SIGTERM 的 30 秒宽限；这不是配准的硬超时。
积压严重或求解器卡住仍可能超过宽限，需要观察 active 与最近完成时间。

## 验证及剩余风险

离线回归使用 391 个同步合成关键帧，故意降低 worker 处理频率并将旧队列容量设为 2，
验证无队列丢帧、积压时不能提前保存、冻结边界、并发保存拒绝、最终点数和构建次数。
另覆盖 SIGINT/SIGTERM 积压退出、错误输入拒绝保存以及预览取消。
2026-09-22 离线验证：391 帧全部处理并写入 Bundle，`pending=0`，
正常采集期间完整地图构建次数为 0，保存时为 1；冻结后注入的 5 帧被计数并排除。
SIGINT 排空 20 帧退出；SIGTERM 排空 20 帧并完成已发起的保存后退出，退出码均为 0。
这组点数可手算的合成测试用于验证完整性，不是密集 JT128 点云性能基准。
这些测试验证流水线和保存完整性，不代表 JT128 大厂区吞吐已经实机验证。

回环 Top-K 的串行 Quatro/Nano-GICP 仍可能主导耗时，本轮没有修改它们。
所有关键帧仍存于 RAM；大地图应监控内存。预览临时拼接和最终输出也会使用额外内存。
内存耗尽、系统强杀/断电或 DDS 接收前丢失无法由 `received==processed` 修复。
本次改动也无法找回旧 factory_large_v1 未落盘的后续关键帧。

## Release 与候选计算剖析

`a2w_fastlio_common` 和 `a2w_fastlio_mapping` 在调用者未指定构建类型时默认使用
`Release`。显式指定 `Debug` 或 `RelWithDebInfo` 时保持调用者选择不变。
Quatro、Nano-GICP、公共配准库、LoopPipeline 和 Mapping Backend 因此使用同一构建类型；
现场仍须通过 `ldd` 确认运行时没有被已 source 的旧工作区动态库覆盖。

候选处理日志以 `[LoopProfile]` 开头，包含：候选数量、source/target 点数、
Scan Context、两侧 LocalMap、Quatro、Nano-GICP、几何证据、Validator、GTSAM、
完整 registration 和整关键帧耗时。数据集工具 `candidate_chain_benchmark` 与
`mapping_replay_benchmark` 只在 `BUILD_TESTING=ON` 时构建，不注册为自动测试，且只读 Map Bundle。

2026-09-22 在单核、低优先级、`OMP_NUM_THREADS=1` 条件下用已有 Bundle 对比：

- 成功回环整帧：Debug/O0 约 6.99 s，Release 约 0.376 s；两者均接受。
- Nano-GICP 不收敛候选：Debug/O0 约 79.6 s，Release 约 1.33 s；两者均拒绝。
- fitness/overlap 不满足候选：Debug/O0 约 51.4 s，Release 约 2.48 s；两者均拒绝。

配准内部存在非确定性，重复运行的 fitness/overlap 数值和最终拒绝原因可能不同，
因此回归要求接受/拒绝结论一致，不声称浮点结果逐位一致。

原候选循环在 descriptor best/second 差值已小于既有阈值时，仍会对全部候选执行配准；
该条件与候选几何结果无关，且现有 Validator 必然拒绝，因此现在只把这一项检查提前。
fitness、overlap、位姿跳变和具体配准结果仍在配准后判断。已有 59 帧 Bundle 顺序回放中，
50 个候选因此跳过无效配准；单核 Release 总耗时约 2.40 s，保留 1 个已接受回环。
对相同的 31～58 帧逐帧独立候选集合（不使用 cooldown）进行前后对照时，86 个候选中
85 个在既有 best/second 差值规则下必然拒绝；总耗时由约 145.3 s 降至约 2.73 s，
唯一原本接受的候选仍接受。该对照用于量化剪枝收益，不代表真实连续运行中的候选分布。

当前侧 LocalMap 本来就只构建一次。Quatro 的当前侧 FPFH 与 Nano-GICP 的当前侧
KD-tree/协方差仍会在多个实际配准候选间重复计算；target LocalMap 也会按候选构建。
已有剖析中 LocalMap 和 Validator 占比很小，复用 FPFH/KD-tree 需要扩展算法准备阶段接口并
严格按点云身份和图优化 revision 失效，本轮未实施，避免引入过期缓存或修改第三方算法行为。

上述结果是已有小型 Bundle 的离线回放，不是 339 帧大厂区实机吞吐验收。
只有新版本实机运行时持续满足 `processed` 速率不低于 `received` 速率、pending 不增长，
并在冻结后 `received == processed`、无 active 任务且 Map Bundle 保存成功，才能确认跟上前端。
