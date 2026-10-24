# 固定地面高程/纵坡离线评价

只操作本专项 ground_eval 目录，不启动 ROS、雷达、机器人，不回放 Bag，不编译。
新完整原算法与联合初始化必须使用同一次 baseline 冻结的 scan/point/cell/参考方向。

## 已确认

- 历史 raw828 与旧 optimized828 八 ROI 已实际计算；历史模式仅重变换同一旧 body 云。
- 原始八 ROI 点身份和 floor 标签保留；历史 cell 在 raw828 坐标中首次冻结，legacy_cell 也保留。
- 旧 KF 精确匹配 runtime POST end，再匹配原 Bag header，816/828 成功，最大量化差 128ns。
- 12 个旧 KF 的源 scan 未保存在 Bag：21,38,55,134,164,193,202,207,221,240,248,727。
  距最近实际 Bag header 约 100ms，不能当浮点误差。新 baseline 不用邻近 scan 冒认身份。
- 上述缺失涉及 ROI Y=-75 的 heldout KF193、Y=-125 的 train KF240。
- 17 项 unittest 通过，包括真实缓坡、固定 up、不以质心替代固定位置、低支持、
  凸包外移、源时间量化、缺失旧 scan、sidecar 身份、指纹变更闭锁、结构固定邻居、
  缺失 cell 不缩小分母，以及完整现代 CSV/NPZ 接口的合成集成测试。

历史 raw 固定 up 下，Y=-25 → -315 的 train 地面锚高差约 -2.4163m；
它是**估计几何**，不是实测场地高差。中段先升后降可观察，不能据此判真实地形或漂移。
历史后端中后段部分点移出了固定锚支持，因此保留 UNKNOWN，未扩大外推门限。
没有独立测量时，absolute_accuracy 和 grade_accuracy 始终 UNKNOWN。

## 输入与身份约定

post_index.csv:

```
source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,cloud,point_ids,tx,ty,tz,qx,qy,qz,qw
```

T_O_body，四元数 xyzw；cloud/point_ids 相对 index 所在目录，也接受绝对路径。
body 云支持 binary/ASCII PCD；现代 PCD 输入必须带 sidecar，不能把排序后记录号当 ID。
point_ids 是每点 uint32 little-endian sidecar，必须为
0..N-1 的完整排列；ID 是固定 preprocess 返回序号，随 UndistortPcl curvature 排序携带。
它不是 LiDAR 原始 packet record 身份。读取时按 ID 关联，不能按候选 PCD 当前行号关联。
NPZ 测试/独立导出接口含 body[N,3] 和 point_id[N]。

preprocess_identity.csv:

```
source_scan_id,header_ns,point_count,xyz_intensity_curvature_fnv1a64
```

候选必须与冻结 baseline 的**全部**预处理记录具有相同 header/count/fingerprint；
本几何工具不把该检查冒充完整 IMU 供数或原速验收，后者必须引用独立 input ledger。

identity_contract.json 的字段见 example。stable_across_runs 只有检查了实际实现后才能设 true。
本专项导出机制在 offline 模式用未被估计过程读取的 normal_x 携带 preprocess 序号，
然后分别写 XYZI body 与 uint32 sidecar；需要检查处理链没有删点/覆盖此槽。
implementation_evidence 应指向真实源文件/检查记录；preprocessing_identity_sha256
保存实际预处理/身份实现与相关固定配置的指纹。两次运行共用同一份 contract。

--trace 当前读取标准 runtime_state.csv 中 INIT_COMMIT 的 g_O_x/g_O_y/g_O_z。
使用该次原算法 INIT 的 -g/|g|，候选无法重定方向。

## 复现历史参考（普通离线终端）

```bash
cd /home/bh/A2W_FastLio2_controlled
python3 -m unittest discover -s experiments/field_frontend_z_init_20261024/ground_eval -p 'test_*.py' -v
python3 experiments/field_frontend_z_init_20261024/ground_eval/ground_eval.py freeze-historical
python3 experiments/field_frontend_z_init_20261024/ground_eval/ground_eval.py evaluate \
  --frozen captures/field_frontend_z_init_20261024_001/ground_eval/historical_frozen \
  --index captures/field_fixed_sessions/fixed_short_20261023_001/bundles/map_bundle/keyframes/index.csv \
  --out captures/field_frontend_z_init_20261024_001/ground_eval/historical_raw828
python3 experiments/field_frontend_z_init_20261024/ground_eval/ground_eval.py evaluate \
  --frozen captures/field_frontend_z_init_20261024_001/ground_eval/historical_frozen \
  --index captures/field_fixed_sessions/fixed_short_20261023_001/bundles/map_bundle/optimized_poses.csv \
  --out captures/field_frontend_z_init_20261024_001/ground_eval/historical_optimized828
```

## 新完整 baseline → joint

等 run 结束、文件关闭、完整供数 ledger 核对完毕，再将以下变量设为实际输出目录。
不要在 baseline 仍写文件时冻结。identity_contract.example.json 是 fail-closed 模板，
不能直接当已证明身份的 contract。

```bash
BASE=/absolute/completed_original_run
JOINT=/absolute/completed_joint_run
CONTRACT=/absolute/verified_identity_contract.json
python3 experiments/field_frontend_z_init_20261024/ground_eval/ground_eval.py freeze-baseline \
  --index "$BASE/post_index.csv" --trace "$BASE/runtime_state.csv" \
  --timing captures/field_frontend_z_init_20261024_001/input_audit/cloud_timing.csv \
  --contract "$CONTRACT" --preprocess "$BASE/preprocess_identity.csv" \
  --out captures/field_frontend_z_init_20261024_001/ground_eval/baseline_frozen
python3 experiments/field_frontend_z_init_20261024/ground_eval/ground_eval.py evaluate \
  --frozen captures/field_frontend_z_init_20261024_001/ground_eval/baseline_frozen \
  --index "$BASE/post_index.csv" --contract "$CONTRACT" --preprocess "$BASE/preprocess_identity.csv" \
  --out captures/field_frontend_z_init_20261024_001/ground_eval/complete_original
python3 experiments/field_frontend_z_init_20261024/ground_eval/ground_eval.py evaluate \
  --frozen captures/field_frontend_z_init_20261024_001/ground_eval/baseline_frozen \
  --index "$JOINT/post_index.csv" --contract "$CONTRACT" --preprocess "$JOINT/preprocess_identity.csv" \
  --out captures/field_frontend_z_init_20261024_001/ground_eval/joint
```

预期：八行 evaluated，进程 exit=0。exit=0 表示分析成功，不代表候选通过。
固定输入/cache SHA 改变、header/fingerprint/point ID 不一致立即非零退出。
无地面支持或固定参考点在支持范围外时保留 UNKNOWN；不换锚、不选最平旋转。

## 输出检查与限制

manifest.json 包含 up/basis/纵向 axis、固定锚位置、固定 station、全部 scan/point 身份、
baseline 支持 cell、原始源文件与 NPZ hash，候选读取计数在冻结时为 0。
report.json 和每 ROI JSON 保留所有 baseline 支持 cell，包括候选失败项。
cells.csv 含有符号高程、相对 baseline 改变量、局部纵坡、法向变化。
ground_profile.png 显示固定锚高程及相邻段有符号纵坡，坐标轴标明高度/距离比例不同。
结构 NPZ 保留 baseline query mask、固定近邻索引、候选支持及原始距离。

主剖面是固定位置高度和段间高差；局部法向给出该位置自由地面纵坡。
结构保护残差是墙/设备的几何代理，非 Z 误差；设备物理身份未经人工确认仍 unknown。
结构/接缝保护有失败即 FAIL，缺支持为 UNKNOWN；“更平”或“更低”没有 PASS 规则。
没有测量真值时不能把原始前端、优化图或 FAST-LIO 地面法向作为独立真值。

Ctrl-C 可停止此离线 Python 进程，不需要停止机器人节点。当前没有 ARM64/ROS/现场/运动验证。
