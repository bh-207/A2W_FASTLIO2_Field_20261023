# 受控采集全程真实状态日志（默认关闭）

本扩展仅观测，不改变时间、滤波、重力、bias 或外参。原 v9 的 source/build/RED/GREEN 和结果文件保留。当前构建入口为 `frontend_init_v4_tools.py prepare/build`，使用全新 `controlled_v3`；`controlled_v1` 是记录器 INIT 尚未立即 flush 的历史构建，`controlled_v2` 是 metadata close/时间精度两项说明补齐之前的历史构建，不作为最终采集版本。

## 参数与输出

独立受控入口显式设置以下参数；未启用时不创建 writer、线程或文件：

```text
frontend_init_v4.runtime_state_log_enabled = true   # 默认 false
frontend_init_v4.runtime_state_log_path = 新的绝对 CSV 路径   # 默认空
frontend_init_v4.runtime_state_log_capacity = 1024  # 固定容量 ring，1..1048576
```

provenance 复用 `frontend_init_v4.source_provenance`，启用时必须是 `real` 或 `synthetic`。严格采集还应启用既有质量门/明确单位接口，且保存其完整参数与证据旁车；runtime 日志本身不能证明驱动单位正确。

CSV：`seq,stage,scan_begin_ns,scan_end_ns,state_stamp_ns`，随后实际 `pos_O_I[xyz],vel_O_I[xyz],q_O_I[xyzw],g_O[xyz],bg_I[xyz],ba_I[xyz],t_I_L[xyz],q_I_L[xyzw]`，以及 `anti_g_O_to_plus_Z_O_rad,accepted_window_last_imu_ns,group_last_imu_ns,init_stamp_consistent,lidar_effective_features`。数组在 CSV 中展开为各轴列。

O=`camera_init`（不是已测绘地图系 M），I=`body`，L=LiDAR。位置/平移 m，速度 m/s，g/ba m/s²，bg rad/s，四元数 xyzw，角度 rad；`anti_g` 角为 `angle(-g_O,+Z_O)`，不假设 O 的 Z 已与物理重力对齐。`gravity_alignment=false`。

- `INIT_COMMIT`：真实 `Process` 返回时检测初始化 false→true，立即读取 actual KF state。严格门使用 `stats.last_stamp_ns`（同源接受窗口最后一个 IMU），另记录本组末 IMU stamp 并验证一致；末尾异常组不会提交。未经严格窗口验证的 legacy 初始化记录 `init_stamp_consistent=0`，不得作为严格 ready。INIT 是初始化状态，不声称已完成激光测量更新。
- `POST_LIDAR_UPDATE`：真实 `kf.update_iterated_dyn_share_modified` 返回、`state_point=kf.get_x()` 后立即复制，先于发布/地图更新；`state_stamp_ns=该次 Measures.lidar_end_time`。`lidar_effective_features` 表明该次有效特征计数；仅阶段名不是收敛、充分约束或独立真值精度的证明。

scan 时间来自 FAST-LIO 当前实际 MeasureGroup 的 double 秒转纳秒（llround），不重写输入。INIT 接受窗口 stamp 保留原整数 ns。double 源时间本身的浮点精度不能凭输出纳秒列恢复；不存在伪造的高精度原始时间。

## 有界 writer、错误与关闭

producer 只做固定大小 state 拷贝、计数、try-lock 和固定容量 ring 入队；磁盘/CSV 格式化在专用 worker。队列满或短暂锁争用采用 drop-new，不阻塞等待队列空；首次 `QUEUE_DROP` 报警，所有丢失均计数。CSV 行是值拷贝，不持有 KF/状态指针。

worker 对 INIT 立即 fflush（ready 无须等关闭可见）；其它行每16条 fflush。真实磁盘写/flush 错误产生 `WRITE_ERROR`，最终 `flush_ok=false`。生产者首次报警使用 stderr；这不是对任意系统/文件系统阻塞给出的硬实时保证。

析构/正常 SIGINT spin 返回后，node 析构主动 `close()`：停止入队、worker drain、join、CSV fflush/fclose、写 `.status.json`，最后打印 `CLOSED`。该顺序不在 signal handler 内操作文件或 mutex。没有 fsync 持久化保证；SIGKILL/崩溃/掉电不能保证 flush。

`.metadata.json` 记录 schema/frame/unit/provenance/阶段与 flush 语义。`.status.json` 启动时独占预留、正常 close 后填充，记录 attempted/enqueued/written/dropped/write_errors/maximum_pending/closed/flush_ok；空文件或缺文件不是关闭成功。`written` 表示格式化/stdio 接受行数，必须联合 `flush_ok` 才能确认完整 graceful flush，不是 fsync 落盘计数。所有输出 O_EXCL 禁覆盖（包括符号链接）；状态文件写失败也报警，不能只凭旧 status 当成功。

## 根代理协调的实际节点 smoke

脚本默认不会运行 ROS，必须由根代理协调本机独占 domain 并显式授权：

```bash
source /opt/ros/humble/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=170 /usr/bin/python3 scripts/diagnostics/frontend_init_v4_runtime_smoke.py artifacts/loop_v4_20261021/frontend/controlled_v3 artifacts/loop_v4_20261021/frontend/frozen_raw_prefix_v2 artifacts/loop_v4_20261021/frontend/runtime_node_smoke_v1 --domain 170 --allow-localhost-ros
```

170 仅示例，需先协调空闲 domain。外部 discovery server/static peers/DDS profile 配置存在时脚本拒绝。输入是 hash 冻结的 synthetic30 scan timed cloud/IMU，局部独有 topics，不连接设备、不发运动命令。脚本实际启动隔离 laserMapping、通过 localhost DDS 喂数据，核对 POST 与同源时间 Odometry pos/q、INIT 与 actual Process quality 日志真实 state/window、有效激光特征、SIGINT exit0 及零 drop/error 的完整关闭；保留输入/binary/patch身份、节点日志和失败结果。`1ns` Odometry 对比容差只针对 epoch1000 合成夹具中原 `get_ros_time` 截断与日志 llround 的差；不是改输入时间。

此前 `Process` harness 不等于此真实节点更新路径。只有上述实际运行结果 PASS 才能宣布该 smoke 通过，仍不能替代真实164噪声/单位/运动/定位验收。脚本在本子任务内只做离线语法/输入/安全测试，实际 ROS 由 root 执行。

## 打包

实际 prepare/build 的最小新增输入仍为完整 `0001`、`frontend_init_v4_tools.py`、`frontend_init_v4_harness.cpp` 和干净上游/嵌套 submodule；header 已嵌在 patch 中。补丁再生成额外需要 `frontend_init_v4_runtime_patchgen.py`、原 state/v8 generator、`frontend_runtime_state_v4.hpp`、原 state/core_v8 header。新测试在 `test_frontend_runtime_state_v4.cpp`、`test_frontend_runtime_state_v4.py`、`test_frontend_runtime_smoke_v4.py`。完整 ROS smoke 另需已发布冻结 raw-prefix 数据，不能声称 ignored artifacts 自带于 fresh clone。

## 已验证版本边界

`controlled_v3` 完整 node+harness 编译通过，原25+新增10测试通过，9个实际Process机制场景通过。根代理实际在 localhost domain218 运行 `artifacts/mapping_capture_20261021/frontend_node_smoke_v2`：1条 INIT 与 actual Process提交匹配，17条 POST 与同scan实际Odometry匹配（16条有有效激光特征，不能把全部17条都称有有效约束）；SIGINT exit0，writer18行、零drop/error、closed/flush均通过。仍仅synthetic/no-robot验证。最早 smoke_v1 在 rclpy.init 默认HOME日志只读处失败、未启动estimator，已保留；脚本现将 ROS_LOG_DIR/ROS_HOME 均放入独立输出会话。
