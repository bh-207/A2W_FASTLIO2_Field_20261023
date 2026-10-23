# FAST-LIO v4：隔离初始化质量门与单位/时间契约

仅实验实现，不是生产配置变更。FAST_LIO_Hesai 原工作树基线为 `16e97cdcc4260b9ba6241518a19ad8384224ba48`，嵌套 ikd-Tree 为 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`。新开关默认均关闭。本次不连接机器人、不启动 ROS、不发布消息、不修改主 install；完整节点只编译，运行的是无 ROS 节点/执行器的进程内实际处理路径测试。

## 实现范围与边界

根仓库补丁：`patches/fast_lio_v4/0001-frontend-init-quality-units.patch`。补丁在隔离副本中修改真实 `laserMapping.cpp::imu_cbk`、`ImuProcess::Process`，以及实验构建选项；不是只在外部统计“好窗口”。`frontend_init_v4_patchgen.py` 按唯一上下文生成补丁；`frontend_init_v4_tools.py prepare` 要求原子模块和嵌套子模块干净、版本准确，并通过 `patch --fuzz=0` 检查。绝不在原子模块直接应用补丁。

质量门真正阻止旧初始化状态写入，直至整个同步 IMU 组检查完成。最低样本数与连续时长必须同时满足；严格递增时间戳、最大间隙、单样本/均值角速度范数、逐轴总体方差、加速度范数及相对窗口首样本的方向稳定性共同约束。运动或时间异常清空窗口，之后重新累计；就绪不在组内锁存，同组后续坏样本会撤销就绪。原始接口拒绝 NaN/时间/frame 等输入时另投递失效通知，Process 清窗并保守跳过下一整个同步组，不让“丢弃坏后缀后留下的好前缀”绕过门；该通知不是精确的原始组归属推断。仅初始化期生效，初始化后不自动重置正常行驶状态。

提交状态严格来自该窗口的同一批成员：`g_W = S2(-mean_acc / |mean_acc| * 9.81)`、`bg_I = mean_gyro`，外参继续采用配置 `T_I_L`。不估计新 `ba`，不以地面法向替代重力，不把姿态强制旋转到世界 Z。W=`camera_init`、I=`body`，L=LiDAR；输入 `frame_id` 必须与明确声明一致，严格接口不做轴重映射。S2 类型固定长度实际为 9.809，保留上游实现，不把它误写成实测重力标量。

静止倾斜可通过。低方差也可能来自匀速运动/恒定线加速度，IMU-only 门不能证明物理静止、不能独立区分加速度 bias 与重力，不能替代充分激励的外参/时间标定。当前阈值是可重复实验默认值，未作为 164 实机验收阈值验证；风扇振动、温漂、真实启动过程需要独立评估。

## 默认关闭参数与量纲

| 参数 `frontend_init_v4.*` | 默认 | 含义 |
|---|---:|---|
| `input_interface_enabled` | false | 严格原始单位/frame/offset声明入口 |
| `quality_gate_enabled` | false | 真正阻止初始化，启用时要求严格接口 |
| `input_gyro_unit` / `input_accel_unit` / `input_imu_frame` | unknown | 只接受明确原始声明，不按数值猜测 |
| `source_provenance` | unknown | 严格入口启用时必须明确 synthetic/real |
| `gyro_unit_evidence` / `accel_unit_evidence` / `time_offset_evidence` | 空 | 调用者提供可追溯来源；字符串存在不等于设备已独立验证 |
| `minimum_samples` | 200 | 精确进入窗口的 IMU 成员数 |
| `minimum_duration_s` | 1.0 | 末成员时间减首成员时间，和样本数同时满足 |
| `maximum_gap_s` | 0.02 | 连续已观察 IMU 最大间隙 |
| `maximum_gyro_norm_rad_s` | 0.05 | 单成员角速度范数 |
| `maximum_mean_gyro_norm_rad_s` | 0.02 | 窗口均值角速度范数 |
| `gyro_variance_limit` | 1e-5 | 每轴总体方差，(rad/s)^2 |
| `accel_variance_limit` | 0.04 | 每轴总体方差，(m/s^2)^2 |
| `minimum_accel_norm_m_s2` / `maximum_accel_norm_m_s2` | 9.0 / 10.6 | 单成员 SI 加速度范数 |
| `maximum_direction_change_rad` | 0.02 | 相对首成员加速度方向，非相对世界 Z |
| `input_trace_path` / `quality_trace_path` | 隔离源码 Log 下 CSV | 默认关闭时不创建；开启时已有同名文件拒绝覆盖 |

内部窗口最多 20000 成员，避免不合适的时长/样本配置导致无界内存。每轴方差采用 Welford 算法；日志区分精确窗口计数与旧初始化内部 N，关闭场景不会假造旧内部计数。

## v9：状态一致性检查后才释放建图输入

样本质量 PASS 不再等于初始化 PASS。仅启用 quality gate 时，在任何实际 KF state/P 更新之前检查：完整原状态有限、两四元数单位/旋转正规、原始外参矩阵正规（先于 SO3 转换）、原 P 有限/对称/正定、四类预测噪声方差有限且正。候选仍按原公式和原 P 数值在局部副本构建，检查其结构/P、保留的 pos/vel/ba/rot、配置外参以及真实预测域静止残差。

本启动契约要求现存 R_W_I 在数值容差内为 identity。Reset 复用非identity KF（包括仅yaw旋转）明确拒绝 `state_requires_fresh_identity_rotation_restart`，应从一致状态明确重启；不替用户旋转旧世界系、不清零 ba、不改成 gZ。静止倾斜是 qI、body 测到的重力方向倾斜，仍可通过。有限的原 pos/vel 不盲改。

残差是 `R_W_I*(a_I*9.81/|mean_acc|-ba_I)+g_W`，使用原去畸变的内部归一化域，不将 raw SI 输入与内部 ba 域混用；分别记录窗口均值和末样本实际残差。状态检查失败时 KF/P 逐字段不变，初始化仍关闭并清窗。enabled 初始化入口在空 IMU 早退之前清输出云，覆盖等待、拒绝、成功初始化当帧，防止 caller 复用旧非空 cloud 绕过门；该帧不释放旧点，随后新帧实际去畸变才可进入建图。disabled 路径不新增这些拒绝或清云行为。

新增参数均前缀 `frontend_init_v4.state_`，只在 quality gate 启用时约束提交。以下为数值/实验模板，**未经164实测，不是硬件噪声验收值**：

| 后缀 | 默认 | 含义 |
|---|---:|---|
| `quaternion_norm_tolerance` | 1e-6 | 四元数范数数值容差 |
| `rotation_orthogonality_tolerance` / `rotation_determinant_tolerance` | 1e-6 | 正规 SO3 数值容差 |
| `initial_rotation_identity_tolerance` | 1e-8 | 启动 R 与 I 的矩阵范数容差 |
| `covariance_symmetry_tolerance` | 1e-10 | P 对称数值容差 |
| `minimum_covariance_pivot` | 0 | LDLT 最小 pivot 必须严格大于此值，不修补/投影 P |
| `maximum_mean_accel_residual_m_s2` | 0.05 | 均值静止残差范数，m/s² |
| `maximum_last_accel_residual_m_s2` | 0.25 | 末样本静止残差范数，m/s² |
| `gravity_norm_tolerance_m_s2` | 1e-6 | S2 实际长度9.809数值容差 |
| `candidate_preservation_tolerance` | 1e-12 | 候选保留字段数值容差 |

另写 `.state_checks.csv` 与 `.state_checks.metadata.json`：phase、明确 reason、窗口边界、mean/last 是否实际计算、残差xyz/norm、原/候选 P 指标、原始外参指标、原状态/候选完整向量/四元数和所有生效阈值。未计算残差留空，不填0或PASS。`preconditions`/`candidate` 阶段 PASS 不能冒充实际提交，`committed` 才记录已写实际 KF 的状态。

旧 v8 的实际 RED 已存 `state_RED_v1`/`state_RED_v2`：18种非法/不一致状态错误初始化，以及等待/拒绝/提交边界残留旧云3种反例；源/旧补丁/测试输入/二进制身份均保存。v8及更早 RESULT/final_evidence 不能作为具有状态门的最终证据。最终 v9 完整编译通过、原25 unittest/9机制场景通过，新增实际 Process 状态/输出/配置验收27/27通过：`state_GREEN_v2/suite.json`。三个原成功场景（含静止倾斜）的 `actual_states.csv` 与 v8 逐字节一致。最终入口 `RESULT_v4_zh.txt`/`final_evidence_v4.json`；首次宽松配置夹具遗漏last约束导致26/27，保留 `state_GREEN_v1`，不掩盖负结果。宽松配置用例仅证明参数确实生效，不能将其1m/s²残差容许值推荐为硬件配置。

## 单位与时间的唯一应用点

严格入口允许原始 gyro=`rad/s` 或 `deg/s`、accel=`m/s^2` 或 `g`。已修正 SI 输入比例为 1，不再额外乘 9.81；明确声明的 deg/s/g 才转换一次，方向轴不变。`common.imu_gyr_unit` 必须与声明相符；`auto` 和同时开启 `common.time_sync_en` 被拒绝，避免第二个自动时间/单位路径。

固定偏移定义为同一物理事件的 `delta = t_IMU_raw - t_LiDAR`，因此 `t_IMU_aligned = t_IMU_raw - delta`。正负偏移均有测试；严格分支绕过旧转换/自同步块，仅应用一次，整数纳秒检查非负、溢出及 ROS 秒范围。ROS `Imu` 没有单位/转换次数字段，因此不能凭此接口识别“上游已转换却错误声明仍为 deg/s”的物理错配；核心重复调用测试依赖内部标记，设备层仍必须审核驱动代码/版本/配置和记录 provenance。

保留原 FAST-LIO 去畸变中的 `9.81 / |mean_acc|` 内部标度归一化。这不是本补丁重复单位转换，也不意味着可以省略原始单位证据。加速度门在该内部标度之前使用明确 SI 输入。

## 本地单位证据：不要把模拟桥当 164

用户已明确 164 驱动单位修正完成；本次不将其重新列为主因，不连接设备验证，不重新推断/缩放历史数据。定向查到的 stage4 `imu_safe*` 是模拟 TruthBuffer 的右侧差分支持修正，不是实机单位修正证据：`sensors/imu_safe_sensor_bridge.py:2`、`sensors/truth_buffer.py:34`（位置二阶差分与 yaw 一阶差分）、`:45`（加 9.81）；`external_sensor_bridge.py:229` 把模拟量写入消息。配对试验快照 `fastlio_effective.yaml:16` 的 rad 只证明该模拟运行参数。当前定向记录中未找到能绑定真实 164 修正驱动版本/实际输出记录的证据，因此机器表中的真实源 gyro/acc 单位仍记 unknown，而不是否认用户前提。

已知 SI 的冻结输入来自 `terrain_model.py:22,59` 的物理构造、`freeze_inputs.py:32` 保存的原始加速度/角速度。它们明确是 synthetic，不是 1016 原始数据。1016 Bundle XYZI 缺失原始 IMU/逐点时间，不能拿来重放前端初始化或评估 LI-Init。

## 可执行隔离命令（从工作区根目录）

使用全新输出目录；以下命令不启动/发布 ROS。`build` 只在指定隔离目录构建，甚至不执行 install。

```bash
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_tools.py prepare artifacts/loop_v4_20261021/frontend/my_isolation
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_tools.py build artifacts/loop_v4_20261021/frontend/my_isolation
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_fixture.py artifacts/loop_v4_20261021/frontend/my_raw_prefix
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_run.py artifacts/loop_v4_20261021/frontend/my_isolation artifacts/loop_v4_20261021/frontend/my_raw_prefix artifacts/loop_v4_20261021/frontend/my_mechanism
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_state_tests.py artifacts/loop_v4_20261021/frontend/my_isolation artifacts/loop_v4_20261021/frontend/my_state_checks green
/usr/bin/python3 -m unittest discover -s scripts/diagnostics -p 'test_frontend_init_v4*.py' -v
```

测试默认使用状态门修正版 `isolation_v9/build/frontend_init_v4_harness`；可设 `FRONTEND_INIT_V4_ISOLATION` 指向新副本。本机优先 `/usr/bin/python3`，避免 Conda 抢 ROS 构建解释器。编译失败可显式 `build PATH --resume`，每次新 attempt 日志保留；源码/补丁 hash 不符则拒绝继续，须新 prepare。前两次环境失败（Python/嵌套子模块）及后续编译负结果都保留，不能只留成功运行。冻结机制场景限定已核验 flat fixture、epoch1000、30 scans；短 prefix 导出只供离线检查，不是可完成机制场景，runner 会提前拒绝。

实际测试调用 patched `ImuProcess::Process`、真实 ESKF prediction、真实 Hesai `Preprocess` 和去畸变，不调用激光测量/地图更新；输出 `actual_states.csv`、同步组成员边界、全精度 `init_quality.csv`、结果 JSON 和源码/二进制/输入 hash。冻结场景外参取生成器已知安装配置，仅验证配置在真实状态中保持一致；不加载 p/R 轨迹真值数组，不估计外参，不是盲标定或轨迹精度验收。它是机制验证，不等于实际 ROS 回调/传输/整个建图/实机定位已通过。关闭质量门的运动场景作为负对照，只验证旧路径首组即初始化，不把它叫安全结果。

每次启用 gate 还写 `init_quality.csv.metadata.json`：实际生效的所有 GateConfig、InputConfig、来源证据、provenance、base commit、坐标和时间规则；不是仅记录默认值。独立审查曾复现旧实验版本“adapter 丢弃组末 NaN/负时间，Process 用219个好前缀误初始化”的真实RED，见 `review_red_v1/`，故早期 v5/mechanism_v3 不是最终修正版证据。新坏输入场景调用实际 adapter + 失效通知 + Process；ROS 回调本身只编译未执行。

另一个入口级时间反例：Process 最后只见 `.995`，callback 缓存先收到 `1.000` 再收到正数 `.999`，原 buffer-clear 会丢掉较新样本，Process 随后无法发现 raw 回退。因此 strict-enabled 的公共 loopback 清 buffer 同样投递失效通知；默认 legacy 不变。`positive_backtrack_buffered` 验证实际 adapter+同缓冲语义+Process 的新行为，`positive_backtrack_legacy_drop` 是显式保留的旧丢弃语义 RED 控制，不能当安全通过场景。

## LI-Init 独立评估和结果入口

官方项目支持运动激励下的时间/外参/重力与 bias 初始化；要求 gyro 为 rad/s，建议初始静止和充分多轴激励。它依赖 ROS1/catkin、Livox 驱动和 Ceres，不能直接等同当前 ROS2 JT128 节点的可运行插件。[官方 README](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/README.md)

官方结果文件先写 Initialization，完成在线细化后再写 Refinement；矩阵方向是 LiDAR→IMU，最终时间值为硬时差加估计 lag。导入最后一次完整 Refinement 的 4×4 矩阵与总时间，避免复用仅 lag 或按不明确 Euler 顺序重建；上游文件只有六位小数，严格旋转验证失败时保留拒绝结果，不静默正交化。[官方结果/时间代码](https://github.com/hku-mars/LiDAR_IMU_Init/blob/main/src/laserMapping.cpp)

本机没有 `roscore`/`catkin_make`，本次禁止发布 ROS，且缺实际 1016 raw 和经过审核的激励标定结果。因此上游算法状态为 NOT_RUN，不伪造收敛/精度/实机标定通过。官方 main 当前未固定到本地构建 commit，独立入口要求将来运行前记录精确 SHA。静止质量门不是 LI-Init 替代品，LI-Init 的世界重力也不能不经坐标链审核直接导入当前世界系。

```bash
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_tools.py external-plan artifacts/loop_v4_20261021/frontend/my_li_plan
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_tools.py external-import PATH/Initialization_result.txt artifacts/loop_v4_20261021/frontend/imported_UNVERIFIED.json
# 独立审核后补充 provenance=synthetic/real、unit/frame/激励/时钟稳定证据；verified 是调用者签认，不是工具认证。
/usr/bin/python3 scripts/diagnostics/frontend_init_v4_tools.py external-review PATH/reviewed_result.json artifacts/loop_v4_20261021/frontend/my_li_review
```

导入结果默认 unknown/UNVERIFIED，直接导出会拒绝；review 检查有限值/正规旋转/方向/offset 约定和证据字段，输出始终 `production_accepted=false`、两新开关仍 false。可审计实验配置永不自动写入/加载生产。要启用隔离输入契约仍需填写真实 input frame 和三个来源字段；结构检查不代替重复标定、留出段残差、固定安装审核和跨上电时钟稳定性验证。
