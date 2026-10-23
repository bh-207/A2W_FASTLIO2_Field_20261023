# Mapping 回环位姿保护：最小修复与离线证据

## 范围

只修复 Mapping 侧的预测位姿验证、优化后坐标一致性和事件日志。
未修改 FAST-LIO、Quatro、Nano-GICP、Nav2、Map Bundle 格式或任何算法参数。
位置/旋转门限仍为 **5 m / 1 rad**；fitness、overlap 和候选歧义门限不变。
未重新启动实机建图，未覆盖已有地图。

## 坐标和方向

以下 `T_A_B` 表示把 B 系的点变换到 A 系；默认 frame 名为
`map / camera_init / body`，实际日志使用节点参数中的名称，不涉及 `base_link`。

- `O_i = T_camera_init_body_i`：与第 i 帧点云同步的原始 FAST-LIO 位姿。
- `P_(i-1)`：上一关键帧最近一次已提交的优化位姿 `T_map_body_(i-1)`。
- `P_i = P_(i-1) * inverse(O_(i-1)) * O_i`：当前帧在 map 系的预测位姿。
  第一帧以原始位姿初始化图的 map 基准。
- Local Map 中每个点为 `p_map = P_k * p_body_k`。历史帧采用当前图版本的优化位姿，
  新帧采用 `P_i`，不再把新帧的 camera_init 数值直接拼进优化后的 map 点云。
- 源 Local Map、目标 Local Map 都在 map 系。
  Quatro 给 Nano-GICP 的初值以及最终结果 `D` 都是
  **源 map 点云 → 目标 map 点云** 的配准修正，不是机器人绝对位姿。
- 回环隐含的当前机器人位姿 `L_i = D * P_i`。
- 真正交给现有验证器比较的是 `result.transform=L_i` 和
  `context.predicted_transform=P_i`，两者均为同一当前帧的 `T_map_body_i`。
- 位置差 `d = norm(t(L_i) - t(P_i))`；旋转差为
  `angle(inverse(R(P_i)) * R(L_i))`。日志 RPY 是该相对旋转的 ZYX 分解，
  不是两个绝对欧拉角逐项相减。
- 通过后才构造 GTSAM 因子 `Z_ij = inverse(L_i) * P_j`，方向为当前 i 到历史 j。

当前侧点云是附近关键帧拼接结果，而非单张原始 scan；其中 body 点云不改变。
循环内仍使用同一批优化位姿，GTSAM 在候选处理后同步更新，再统一写回历史位姿。

## 修复前的两个缺口

1. `LoopPipeline` 原来只传 best/second descriptor distance，未设置
   `ValidationContext.predicted_transform`，所以已有 5 m / 1 rad 检查根本不执行。
   现在在入图前复用**同一个验证器实例/同一参数**重新验证机器人位姿；不直接拿
   `D.translation` 当作机器人位移，也不将 `D` 与绝对 `P_i` 混比。
2. 旧实现接收的新关键帧 `optimized_pose=odom_pose`，而历史关键帧已经优化到 map，
   确实存在 raw/optimized 混用。改为以上里程计增量传播；原始 odom 仍用于里程计因子，
   不修改 FAST-LIO 数据。

## 47→16 实际顺序回放

输入：`maps/factory_large_20260923_212543`，415 帧。
从原始 odom 重放 0～47，不使用已经被坏回环污染的已保存优化位姿。
离线工具的默认算法配置逐项核对过 Bundle 的 registration/scan_context 配置。

| 项目 | 实测 |
| --- | --- |
| 修复前 | 在线日志 ACCEPT；旧流程离线重放也 ACCEPT |
| 预测位置 map XYZ | `[-7.833103, 36.061057, 0.665671]` m |
| 回环纠正位置 map XYZ | `[-6.663141, 6.618708, 0.463442]` m |
| 位置差 | **29.466280 m → REJECT**（5 m） |
| 旋转差 | **0.065843 rad / 3.772545° → PASS**（1 rad） |
| 相对 roll/pitch/yaw | `[-0.500368, 3.737578, 0.095913]` ° |
| map 系 Z 差 | `-0.202229 m` |
| fitness / overlap | `0.248392 / 0.667694`，原有质量门限仍通过 |
| 修复后 | **REJECT：translation_jump_above_limit**，未加回环因子 |

同帧其他四个候选也未被接受；0～47 共48关键帧、回环因子0。
保护不是保证以后不会有坏回环，而是明确挡住本次约29.47m错误纠正。
原坏回环入图造成后续轨迹整体倾斜；配准时3.77°不等于最终全图倾角。

回放命令（不启动 ROS 节点，不写地图）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
build/a2w_fastlio_mapping/mapping_replay_benchmark maps/factory_large_20260923_212543 47
```

证据：`log/loop_47_diagnosis/fixed_pipeline_results.txt`；旧流程证据：
`log/loop_47_diagnosis/pose_guard_results.txt`。

## 正常例和数据边界

- 单元正常例：小幅平移回环仍接受；绕远处 map 原点产生 `|D.t|>5m`、
  但机器人实际原地转0.1rad的正常回环仍接受，避免比较错对象造成误杀。
- 坐标回归：成功回环后下一帧源点云，使用上一帧优化位姿和原始 odom 增量，
  测试在旧实现失败、修复后通过。
- 完整算法正对照：取 `factory_large_v3` 的真实点云，构造已知
  0.5m/0.2m 平移、0.1rad yaw 里程计偏差的回环序列，沿正式 LoopPipeline 执行
  Scan Context、Quatro、Nano-GICP、验证、GTSAM。结果位置纠正0.538245m、
  旋转0.100181rad、fitness0.045012、overlap1.0，**ACCEPT**，32节点/33因子。
  这是**合成正对照，不是旧实机正常回环复现**。
  源码和结果在 `log/loop_47_diagnosis/normal_control.cpp`、`normal_control_results.txt`。
- 旧日志正常回环31→0、66→33属于460帧批次，时间约17900626xx。
  两份异常 Bundle 分别466/415帧，source stamp 从1790168554/1790169377起，
  与该旧批次不一致。当前未找到460帧数据，**不能声称这两个历史正常回环已经回归**。
- v3的真实返回候选300→8等，仍被原有SC歧义门限提前拒绝，根本未进入新增位姿检查；
  没有为凑正常例绕过或调整歧义门限。

## 验证和后续

- Common + Mapping 的 Release 构建通过（实际 `-O3 -DNDEBUG`）。
- 两包 CTest 共22组全部通过，其中 Mapping 含3个ROS2 launch测试；
  新的回环保护/日志测试包含在18项 LoopPipeline 单测中。
- ROS测试使用 localhost-only、独立Domain187，不接管机器人。
- `[回环验证]`仅在实际处理候选配准后输出位置、四元数、角差、相对RPY、
  fitness/overlap、两项PASS/REJECT和最终结果；不可用配准显示N/A，不伪造PASS。

本次5m门限足以拒绝47→16，不需要为了这个样本额外修改旋转/Z门限。
针对近距离错误回环，roll/pitch/Z独立保护可作为后续防线，但需要正常坡道、
姿态和噪声数据确定门限，不能把厂区强行假设为绝对平面。本轮未新增这些限制。
历史正常实机回环回归仍待对应原始Bundle；新版本实机建图尚未验证。
