# 全局初始化：候选判定与多帧确认

本次修改仅涉及 `a2w_fastlio_localization`。串行 Top-K、Quatro、Nano-GICP、
Scan Context、Map Bundle 及各项验证阈值保持原值。

## 时间预算

`global_relocalization.evaluation_time_budget_ms` 现在是完成一次评估后的性能告警线。
它不再覆盖 `success`、`strong` 或原始 `reason`。它不是可取消配准的硬超时，
因此不能保证初始化耗时低于此数值。所有候选依然经过原有几何验证。

## 位姿假设与歧义

每个候选得到最终 `map→body` 后，按原有综合分数排序。比较其与最佳候选的
最终三维平移和旋转，复用以下已有会话参数：

- `global_relocalization.maximum_translation_difference_m`
- `global_relocalization.maximum_rotation_difference_rad`

与最佳位姿一致的候选视为同一假设。原有 `minimum_score_margin` 用于最佳假设
与最佳不同位置/姿态假设之间的比较。关键帧 ID 和关键帧原点不作为合并依据；
每个候选直接与最佳位姿比较，避免通过相邻关系传递合并不同位置。

## 多帧确认

初始化仍强制 `confirmation_count` 次一致证据，即便单帧达到 strong 条件也不跳过。
一致性比较使用每帧配准结果与该帧 FAST-LIO odom 求得的 `map→camera_init`，
允许最佳关键帧 ID 改变。以本轮确认的第一帧 correction 为锚，避免逐帧小漂移累积。
匹配失败或位姿不一致会清零或重启连续计数。

会话计时使用 PC 单调时钟；传感器时间戳继续用于位姿和 correction 新鲜度检查。
只对有效且一致的本次配准扣除其实际计算时间，剩余等待时间仍受原有
`monitor.relocalization_timeout_ms` 限制。有效进展刷新等待起点。
失败计算和超过等待窗口的输入间断仍可导致 `session_timeout`。
会话在收到结果时检查超时；没有输入时不会发布已确认定位。
外部会话启用时，LocalizationMonitor 使用会话的超时结论，避免另一个总时长
计时器提前否决已经完成的确认。

恢复 correction 前继续检查传感器时间戳的单调性，旧结果不得覆盖新 correction。
新初始化结果仍可能有较高 correction age，需要后续 fine-only 更新；性能告警并
不表示数据更新及时，也不表示可以直接启动导航。

## 诊断与验证

每轮初始化中文摘要保留候选及配准指标，并增加同位姿候选数量、不同假设的
次佳分数、确认计数及独立耗时告警。候选配准成功不等于已完成全局定位。

定向回归覆盖同位置重复候选、不同位置歧义、候选 ID 变化、慢配准三帧确认、
失败/等待超时、位姿旋转变化以及连续小漂移。ROS2 集成测试使用独立 DDS 域，
以低于正常评估耗时的测试预算验证原始结果不会被性能告警覆盖。

## 2026-09-21 本轮结果

- Localization 包编译通过；8 组核心测试和两个 ROS2 launch 测试通过，
  `colcon test-result` 汇总 45 tests、0 failures。
- JT128 实机短测约 85 秒，使用 `factory_map_final`，未启动运动桥。
- 三轮候选评估分别耗时 16.21、17.64、21.40 秒，确认按 1/3、2/3、3/3 推进；
  最佳候选从 ID 7 改为 ID 5，约启动后 56 秒进入 `LOCALIZED`。
- 后续记录 8 次 fine-only 匹配，均 accepted，耗时 0.812～3.892 秒。
  已采集状态没有出现 DEGRADED/LOST。这是本次短测结果，不是长时稳定性或导航验收。
- 由于现有 worker 的待处理帧排队，首次确认时 correction age 约 38.9 秒；
  首次 fine-only 更新后约 1.2 秒。排队策略本轮未修改，启动时应继续保持静止，
  检查后续 `tracking_accepted` 和地图位置后再单独进行导航验证。
- 5 秒预算继续报告告警，初始化耗时未降低；程序在定时短测结束后已停止。

日志保存在工作区 `log/localization_initialization_fix/`。
