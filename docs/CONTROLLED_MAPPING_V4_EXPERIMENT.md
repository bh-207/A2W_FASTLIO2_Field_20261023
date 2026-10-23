# 现有未验证初值：v4 静止复测与手控短闭环

仅用于本次获授权的受控实验。使用本地已构建的独立 v4 前端和保守后端，保留 GTSAM
依赖修复，不拉取旧代码、不重建工作区。生产入口仍默认拒绝未验证初值；实验入口必须显式
传入授权开关和说明。所有命令在笔记本执行，不在 164 上启动第二套驱动。

会话明确记录 **沿用现有配置的未验证实验初值**：`R=I、T=0、offset=0、extrinsic_est_en=true`。
配置文件及 SHA256 已绑定，物理标定未完成。驱动已输出 `rad/s、m/s²`，frame 为
`hesai_lidar_aligned / hesai_imu_aligned`；点云、角速度、加速度各由驱动执行一次既有旋转。
FAST-LIO 不重复缩放，不复制驱动旋转矩阵。`body → base_link` 安装变换不是 LiDAR—IMU 外参。

当前没有确认 `/tf_static` 发布者，之前静止录包实际只有五个话题。下面仍请求原有六话题，
不发布假 TF 补数。缺少该话题必须在交付时保留说明，不能宣称六话题齐全。

## 启动条件

- 机器人保持静止且无人遥控行走。关闭自己拥有的其他 mapping/localization/录包任务；有重名
  节点时入口拒绝启动，不能用 `pkill` 清理不明任务。
- 已核实驱动 `ROS_DOMAIN_ID=0`。有线 DDS peer 为 `192.168.123.164`，SSH 地址为
  `<LOCAL_NETWORK_ADDRESS>`；SSH 连通不能代替 DDS 数据持续接收。
- 只读执行 `timedatectl timesync-status` 和 `ip -4 route get 192.168.123.164`。
  本次笔记本跟随机器人内部时间源 `192.168.123.161`，主机时间差不填入固定 offset。
  如需校时，先停止所有采图进程，校时后重新检查传感器 stamp。
- `captures/jt128_interface.json` 必须保留已审计的驱动单位/frame 来源，不用模板覆盖。
  `check` 的 `errors/contract_errors/build_errors` 必须为空；`start` 还会检查同会话录包身份、
  实际已写入 bag 的两路 raw、stamp、字段、QoS 和数据新鲜度。
- 先做下面的静止流程。仅当静止录包、保存和最终检查合格后，另建新会话做手控短闭环。
  128 MiB 缓存是本次复测的唯一采集参数调整，双缓冲可占约 256 MiB；不改变生产默认 16 MiB。
  无缓存丢失告警也不等于证明上游/DDS/存储无丢失。

## 终端 1：新会话、DDS、先录包

```bash
cd "$HOME/A2W_FastLio2_controlled" || exit
unset LD_PRELOAD A2W_FROZEN_MATCHER_TIME
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
SESSION_NAME="v4_trial_$(date +%Y%m%d_%H%M%S)"
SESSION="$PWD/captures/$SESSION_NAME"
./scripts/mapping_capture_experiment.sh init "$SESSION_NAME" \
  --interface "$PWD/captures/jt128_interface.json" \
  --base-config "$PWD/src/a2w_fastlio2_bringup/config/jt128.yaml" \
  --authorize-existing-unverified-initials \
  --authorization-note '沿用现有配置的未验证实验初值；R=I、T=0、offset=0、extrinsic_est_en=true；物理标定未完成；先静止检查，只有ready通过才手控小闭环，禁止自动运动。' || exit
printf '%s\n' "$SESSION" > captures/current_experimental_session.path
source scripts/lib/a2w_common.sh
export PYTHON_BIN=/usr/bin/python3 A2W_DDS_PEER=192.168.123.164
unset A2W_PC_IP A2W_NETWORK_INTERFACE
a2w_resolve_network "$A2W_DDS_PEER" || exit
/usr/bin/python3 scripts/generate_cyclonedds_config.py \
  --template config/cyclonedds_unitree_a2.xml.in --output "$SESSION/cyclonedds.xml" \
  --pc-address "$A2W_PC_IP" --peer-address "$A2W_DDS_PEER" || exit
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$SESSION/cyclonedds.xml"
./scripts/mapping_capture_experiment.sh check "$SESSION" || exit
./scripts/mapping_capture_experiment.sh record "$SESSION" --compression none --cache-size-mib 128
```

终端保持运行，不提前 Ctrl+C。录制 `/lidar_points /lidar_imu /Odometry /tf /tf_static
/mapping/registration_status`，512 MiB 或 300 秒分段。原始输入建立后再执行终端 2。
若有环境自定义构建目录/依赖前缀，先确认与当前本地构建身份一致，不能指向其他工作区。

## 终端 3：先开真实提交监视，再启动终端 2

```bash
cd "$HOME/A2W_FastLio2_controlled" || exit
SESSION="$(cat captures/current_experimental_session.path)"
unset LD_PRELOAD A2W_FROZEN_MATCHER_TIME
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$SESSION/cyclonedds.xml"
./scripts/mapping_capture_experiment.sh shell "$SESSION" -- \
  /usr/bin/python3 "$PWD/scripts/mapping_committed_loops.py" --csv "$SESSION/committed_loops.csv"
```

仅精确 `mapping_backend_node` 的 `[LoopCommitted]` 记录会提示“真实回环已提交”。候选、
配准 accepted、等待提交、地图保存均不算回环。无提交不强制加边。传感器到墙钟延迟仍标为 unknown。

## 终端 2：真实 v4、初始化门

```bash
cd "$HOME/A2W_FastLio2_controlled" || exit
SESSION="$(cat captures/current_experimental_session.path)"
unset LD_PRELOAD A2W_FROZEN_MATCHER_TIME
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$SESSION/cyclonedds.xml"
./scripts/mapping_capture_experiment.sh start "$SESSION"
./scripts/mapping_capture_experiment.sh ready "$SESSION" --timeout 90
```

保持静止，只有退出码 0 且 `ready=true`、真实 `INIT_COMMIT` 和状态检查均通过，才算本次初始化
合格。`start=true` 或出现 Odometry 不能代替 ready。门限保持至少 200 个样本、1 秒窗口、
最大 IMU 间隔 20 ms；拒绝/重置时读 `init_quality.csv`，不降低门限。

静止复测应覆盖至少 150 秒录包和多次分段后再保存。手控短闭环使用另一个全新会话，仍须通过
自己的 ready；行走前可再次执行 ready，期间出现输入/录包异常应由现场人员停止行走。
所有运动由现场手控；本入口不发布运动命令，不允许全厂采图或自动导航。

实际参数在 `snapshots/actual_*.yaml`。每次激光更新后的重力、偏置、姿态和外参在
`runtime_state.csv`，`INIT_COMMIT` 单独留精确 accepted-window/group/state stamp。
POST 的 scan stamp 继承前端 double 秒回转精度。`camera_init` 的重力夹角不是机体 roll/pitch，
不代表地面已水平；`map_gravity_alignment=false`。

## 终端 2：停止行走、冻结排空、保存、正常退出

以下每一步确认成功后再执行下一步；终端 2 如果已经关闭，先重新执行上面的 cd、SESSION 和环境设置。

```bash
./scripts/mapping_capture_experiment.sh save "$SESSION" --timeout 600
```

必须看到服务 `success=true`、Bundle manifest 有效，并且 drain 为 `received==processed`、
`pending=0`、`active=none`。回环工作与关键帧同步执行，此时无在途关键帧任务。
若超时，保留进程并读 `logs/backend.log`，底层可能仍在排空，不重复保存、不强杀。
即使录包降级也应先保全地图；`capture_degraded=true` 时不能算数据集验收通过。

```bash
./scripts/mapping_capture_experiment.sh stop "$SESSION"
```

确认终端 1 正常返回、`record_closed.json` 退出码 0 且 metadata 存在；终端 3 按 Ctrl+C 结束监视。

```bash
./scripts/mapping_capture_experiment.sh verify "$SESSION"
```

`verification.json` 必须检查实际 topic 数量、raw 间隙/覆盖、recorder 告警、Bundle 和状态关账。
已发现 `Cache buffers lost messages` 或非零 `Total lost` 会让最终 verify 退出 2 并标记降级；
同时会阻止后续 ready，将保存诊断标为降级。旧会话应以补充该检查后的验证结果为准。
状态日志要求 `closed && flush_ok`，`dropped==write_errors==0`，
`attempted==enqueued==written==CSV行数`，且 CLOSED 汇总一致。
缺 `/tf_static` 且从未见到发布者时会明确列在 `missing_topics`，不会伪造 TF；即使其余检查通过，
也只能说明实际五话题记录通过相应检查，不能称六话题完整。

地图在 `bundles/map_bundle/`；必须连同 `bundles/experimental_initial_conditions.json` 和会话来源
一起保管，不能把此实验图冒充已标定图。原图及导航地图不变。若无成功保存且明确放弃本次地图，
可正常停止自己的进程并保留 raw：

```bash
./scripts/mapping_capture_experiment.sh stop "$SESSION" --discard-unsaved
```

## 检查失败时

- `build/runtime library mismatch` 或 `undefined symbol`：停止启动流程，检查 `.deps/install`
  与构建清单的 GTSAM 路径/SHA，不通过算法修改规避，不替换系统库。
- `input report stale` / QoS / stamp / fields：检查有线 DDS 与真实原始输入，不能用 SSH 成功代替。
  主机若恢复公共 NTP，先停采图再按内部时间源校时。
- `imu_gap` / 方差 / 方向不稳定：保持静止检查输入连续性，窗口会重新累计；不伪造 INIT。
- 缓存丢失：先保存退出，保留日志；128 MiB 也失败时具体存储停顿/负载原因仍需定位，不能仅因
  增大缓存而宣称已修复。不要把 probe 差值或两次告警总数相加成精确端到端丢包统计。
- 此流程没有验收物理标定、行走漂移、地图精度或实地回环；增强回环、修平地图和切导航地图均不在范围内。
