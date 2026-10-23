# 状态一致性提交门：v8真实RED与v9验证记录（原pending设计）

状态：最初在full paced冻结期仅准备文本；完整回放结束、root明确授权后，已先在v8执行实际RED（state_RED_v1/v2），再整合正式0001/header/generator并新建isolation_v9。v9完整编译通过、原25测试/9机制场景通过、state_GREEN_v2新增27/27实际Process状态/输出/配置检查通过。所有旧证据保留；文件名PENDING仅保留历史命名，最终生效补丁为0001-frontend-init-quality-units.patch，不是另存的历史PENDING.patch。

## 已核实缺口与可手算反例

`frontend_init_v4_patchgen.py:104–121` 在 ready 后直接写 `kf_state.change_x/change_P` 并关闭 `imu_need_init_`。没有状态 finite、正规 SO3、协方差、静止动力学残差的硬检查。原 `src/IMU_Processing.hpp:118–133` 外参 setter 原样收矩阵/平移；`:103–115` Reset 只重置 ImuProcess，不重置复用的 KF rot/ba/pos/vel/P。

静止 mean_a=(0,0,9.81)，mean_gyro=0，现存 R_W_I=R_y(+90°)、ba=0：样本质量门完全通过，当前候选 g_W=(0,0,-9.809)。真实预测式使用 R*(a*9.81/|mean_a|-ba)+g，故静止残差=(9.81,0,-9.809)，norm约13.87m/s²，仍会标记 initialized。若 R=I、保留 ba=(1,0,0)，残差约1m/s²。非法外参或 NaN position/ba 等也未被样本质量门覆盖；原 P 中 NaN/负值会被 `setIdentity()` 静默掩盖。

## 最小 default-off 行为

不增加生产默认开关：仅现有 `quality_gate_enabled=true` 路径执行状态门；gate-off 路径逐字保留。保留原 mean_acc→gravity 候选公式和初始 P 数值，不强制 R=I、不 gZ、不盲估/清零 ba、不自动修补非法旋转或协方差。按 root 明确决定，非identity的KF rotation一律显式拒绝并说明从一致状态重启（连绕g轴、残差偶然为零的yaw也不豁免）；这是本启动世界系契约，不是声称所有合法非identity物理状态都错误。静止倾斜指 qI/body测得g倾斜，可以通过；不要求自动支持复用任意旧世界系。

在任何 `change_x/change_P`、`imu_need_init_=false`、传播边界/noise 更新之前：

1. 检查完整现存 state 的 pos/vel/bg/ba/g/ext_t 与两四元数 finite；四元数 norm接近1，转换矩阵 R^T R≈I、det≈+1。现存 R_W_I 必须在数值容差内为identity，否则 `state_requires_fresh_identity_rotation_restart`，不暗改R。检查原始 `Lidar_R_wrt_IMU` 同样正规，先于其转换为 SO3，避免构造器掩盖错误；原始 ext_T finite。
2. 检查现存 KF P finite、对称、正定（Eigen LDLT/LLT；拒绝而不投影），以及用于后续实际预测的 acc/gyr/bias 方差 finite 且 >0。构建候选 P 后再次检查 finite/对称/正定；不是只检查被 setIdentity 掩盖后的 P。
3. 在局部变量构造当前候选 state 和 P。核验候选完整 finite/proper SO3、S2 gravity norm≈9.809、configured ext准确一致、bg来自同一精确窗口。不要修改保留的 ba/R/pos/vel。
4. 用与实际 UndistortPcl/get_f 一致的域计算 `a_I_scaled=mean_acc*(9.81/|mean_acc|)`、`r_mean=R_W_I*(a_I_scaled-ba_I)+g_W`，而非未缩放 raw acc。也检查用于 acc_s_last 的 last IMU 残差，所有中间量 finite。先提议均值阈值0.05m/s²、last阈值0.25m/s²，必须写 metadata、明确实验阈值并用原 frozen fresh场景检验，不当164硬件校准值。精确阈值由 root 决定；fresh S2 norm差带来的约0.001m/s²必须容许。
5. 全检查 PASS 后才调用 change_x/P、更新 noise/传播边界，并最终 `imu_need_init_=false`；失败保持实际 KF state/P逐字段不变、仍需初始化、输出云为空。记录 `state_check_rejected`、精确 reason、残差xyz/norm、quat/R/协方差检查量、原状态/候选状态及 threshold；可清窗重新累计，但不得将不一致状态静默“修好”。

有效配置 sidecar 应区分 sample_quality_pass/state_consistency_pass/map_release，记清 state guard 版本与实际阈值。已有 sample trace 的 `initialized` 不能代替 state硬检查 PASS。

## 文本测试与未来验证顺序

已实际编译执行的真实 Process 测试包含：实际 Reset 复用90°pitch/yaw旋转、保留 ba 导致大残差、质量窗允许但末acc残差过大的样本、NaN pos/ba/ext_T、非单位 quaternion、缩放/反射 raw ext_R、NaN/非对称/非正定/奇异原 P、NaN/负噪声方差；均验证不 init、不释放云、实际 state/P没有部分修改。`fresh_unchanged` / `off_unchanged` 是兼容正例，手写固定期望 g=(0,0,-9.809)、ba/bg=0、R=I、ext保持配置；`off_reused_rotated_legacy`验证关闭路径不新增此拒绝，不把其旧不一致行为称安全通过。没有用被测 helper 推导期望。

最终实现：`frontend_init_state_v4.hpp` 与 `scripts/diagnostics/frontend_init_v4_state_patchgen.py`，由正式generator生成完整0001；旧v8 generator/core已另存以复现RED，新增header纳入prepare/build哈希。阈值是可配置实验模板、未经164实测；数值SO3/identity容差另设合理上限，防通过极大容差关闭硬条件。

执行顺序已完成：full完成后先在全新隔离输出编译测试针对v8观察named RED（不是unknown-case/缺API），保存原patch/source/binary/input hash及逐例exit/log；随后整合正式提交门，在全新isolation_v9完整构建，全部原25 tests/9机制场景与27状态/输出/配置场景GREEN。全部旧证据保留。

新增 `include/frontend_init_state_v4.hpp` 已纳入prepare/build的patched_hashes。新测试每个非法用例观察实际init/云/原state/P：18种状态反例在旧v8均真实错误initialized后断言失败，3种sentinel反例证实输出旧云可绕门；新v9均阻止。不是helper缺失或unknown-case故障。额外宽松配置均值测试首轮遗漏last约束，被正确拒绝；修复测试夹具后27/27通过，26/27首轮负结果完整保留，无生产修补。

局限：此检查仍不证明物理静止/绝对世界对齐或标定准确；恒速及恒定线加速度可不可观。不存在独立真值时不能把硬门 PASS 描述成真实标定通过。非零位置/速度只检查 finite，不凭IMU静止代理盲清零或推定其绝对值。
