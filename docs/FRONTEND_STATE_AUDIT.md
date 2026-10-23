# 离线前端状态审计（无 ROS）

`scripts/diagnostics/frontend_state_audit.py` 只读解析已落盘日志，使用 Python 标准库；不启动 ROS、不连接机器人、不改变 FAST-LIO/配置/Bundle。输出目录必须不存在，且不能包含输入路径。输入格式坏列、非有限值、异常四元数将报错，不静默跳行；时间倒退/重复、缺 phase/缺传感器记录等负面结果保留在摘要。

## 来源与坐标约定

每次必须指定 `--provenance synthetic|real` 和 `--provenance-note`，这是调用者的来源声明，不是工具验证出的真值。摘要包含 schema、工具/输入/可选生产者 patch 的 SHA256、生产者 build 标识（未知则警告）。不得把 `artifacts/terrain_extrinsic_1016_v1/runs/` 合成日志标为真实 1016 历史。

固定约定：W=`camera_init`，I=`body`（IMU，而不是 `base_link`），L=LiDAR。pose 为 `T_W_I`，extrinsic 为 `T_I_L`，四元数 xyzw；g_W 以 m/s² 表示，派生 `g_I=R_W_I^-1*g_W`。bg 为 I 系 rad/s；ba 是 FAST-LIO 经 `9.81/initial_mean_acc_norm` 归一化加速度域中的状态，不应直接与原始 driver 加速度比较。g_W 不默认沿世界负 Z。

原始 `sensor_audit` 单位默认 **unknown**。只有存在生成器/driver/配置等可靠证据时才显式传 `--gyro-unit rad/s|deg/s` 和 `--accel-unit m/s^2|g`。未知 gyro 单位时不生成 SI gyro norm 检查；未知 accel 单位时不臆造 SI 统计。`--gyro-mean-limit-rad-s` 默认 0.5 仅用于复核现有初始化 warning 对应的均值范数，不是静止验收门。

## 可直接执行的已有合成日志解析

以下只读现有文件，将结果写入新目录；示例故意保留原始加速度单位 unknown。原始 gyro 也可保持 unknown，不传单位即可。

```sh
python3 scripts/diagnostics/frontend_state_audit.py \
  --trace artifacts/terrain_extrinsic_1016_v1/runs/flat_correct_true/dbg.txt \
  --sensor-audit artifacts/terrain_extrinsic_1016_v1/runs/flat_correct_true/sensor_audit.txt \
  --producer-patch artifacts/terrain_extrinsic_1016_v1/experiment_instrumentation_v4.patch \
  --provenance synthetic \
  --provenance-note 'terrain_extrinsic_1016_v1 isolated v4 flat_correct_true; REPORT_zh.txt:48-64; not real1016' \
  --output artifacts/loop_recall_v3_20261020/state_audit/flat_correct_true
```

生产者 build id 若有真实二进制 SHA，可额外传 `--producer-build-id SHA256:...`；不可拿当前 install 二进制 hash 替代曾经生成日志的二进制。用于额外配置/生成器证明的文件应随 run 归档。工具不会通过日志数值倒推出单位或真值来源。

输出 `states.csv` 含预测/更新完整状态和派生 g_I/norm/RPY；`gravity.svg`、`bias.svg`、`extrinsic.svg` 显示时间曲线（更新实线、预测虚线）。`summary.json`、`WARNINGS.txt` 保存缺失与限制。不同 phase 分开连线，不把预测和更新混成轨迹真值。SVG 不做曲线平滑。四元数在允许的 .01 norm 误差内归一化用于计算，超出即拒绝。

启动统计输出 `startup_imu_proxy.csv` 以及 summary 的 `startup_imu_proxy`：仅按 sensor timestamp 截取“截至最早可见 predicted 的 scan-end”的原始 IMU 前缀，给样本数、跨度、原始均值、总体方差(ddof=0)、重复/倒退数及最大排序间隔。日志写入顺序、回调到达顺序、sync 组和实际内部初始化筛选不是同一事物。内部准确初始化成员/计数N/质量始终标记 missing，不用 proxy 充当内部样本集合，也不生成静止质量通过结论。没有绝对 trace cutoff 或没有 sensor 文件时，统计保留 missing。

## 原生 mat_out 可选导入

```sh
python3 scripts/diagnostics/frontend_state_audit.py \
  --native-mat-out /path/to/captured/mat_out.txt \
  --provenance real --provenance-note 'run UUID + capture location; real assertion only' \
  --output /tmp/new_native_state_audit
```

严格只接受26列 post 格式，不接受旧姿态有问题的 mat_pre。RPY 数值除 **57.3** 恢复 rad，再构造 ZYX 四元数；不是精确标准度。输出 `native_states.csv` 和 `native_*.svg`，与 v4 时间轴分开。原生日志约6位有效精度，时间是相对 scan **BEGIN** 标签、状态却位于 scan **END**；不会合成绝对历史时间，不与 sensor 做伪精确匹配。velocity 列不用于本工具的重力/bias/外参审计。原生 pos_log 的零 omega/acc 占位列不是实测 IMU，本工具不导入它。

## 复用已有隔离 v4 仪器，而不是改生产源码

已有 `artifacts/terrain_extrinsic_1016_v1/isolated_frontend_source_v4/` 和 `experiment_instrumentation_v4.patch`：原 cloud data 字节 FNV-1a64、原始 IMU、sync begin/end/组内stamp，以及 predicted/updated 的17位精度姿态、g/bg/ba/extrinsics。cloud FNV 不是 SHA256 完整消息 hash；v4 时间是 double 秒，不是无损整数 sec/nanosec。v4 并未记录完整内部初始化质量窗口或P。

优先只解析已保存日志。若确需未来隔离诊断，先人工审查源码/patch，复制现有隔离源码到独立临时工作区，不在生产包上直接应用：

```sh
frontend_audit_root=$(mktemp -d /tmp/a2w_frontend_audit.XXXXXX)
cp -a artifacts/terrain_extrinsic_1016_v1/isolated_frontend_source_v4 \
  "$frontend_audit_root/FAST_LIO_Hesai"
```

这不是启动或编译命令。本工具不执行它。未来独立构建要使用不同 source/build/install/log 路径，不能 source/覆盖主 install；ROOT_DIR 是编译时路径，原生日志会截断同名文件。执行任何 ROS 数据回放/采集需另行隔离与授权，避免机器人运动/网络接入。

特别注意：patch 第65–68行把隔离订阅改为 **RELIABLE + KEEP_ALL**，因此这不是可原样植入现网的“仅加日志”补丁。其目的是隔离配对实验消除输入丢包混淆；不代表生产 QoS、安全或实时验收。不要默默复制该 QoS 到生产。

验证：

```sh
python3 -m unittest discover -s scripts/diagnostics -p test_frontend_state_audit.py -v
```
