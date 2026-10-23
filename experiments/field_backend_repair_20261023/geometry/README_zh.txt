固定点身份的八 ROI 离线几何复评

适用范围
本工具只读已经保存的 fixed_short_20261023_001，不启动 ROS、雷达、TF 或机器人。
只在本目录与 captures/field_backend_repair_20261023_001/geometry 写评估结果。
输入为 828 帧原始 body PCD、原始 keyframes/index.csv、旧 optimized_poses.csv，
以及候选 poses CSV。候选必须包含同一套 id=0..827 与 tx,ty,tz,qx,qy,qz,qw。
姿态采用 T_map_body，四元数顺序 xyzw。候选第一帧必须保持保存地图 gauge。

先冻结，再读候选
首次运行先写 frozen/acceptance_thresholds.json，再生成八 ROI 的选点缓存。
选择规则逐项保持原 geometry_protocol.json：原优化坐标选 ROI/range/voxel，
保留每个 body PCD 的原始记录 index、keyframe id、floor 标签与 0.75m cell 标签。
每次运行检查冻结缓存与输入的 SHA256，不因候选改变选点或删除失败 ROI。
INIT_COMMIT 重力经保存 map gauge 变换得到固定 up，不追随优化姿态重新定义重力。

两种指标必须同时看
legacy 完全重现旧方法：每个版本独立筛有效 cell，参考位置取其 train/return
平面质心均值。已用旧报告验证 16 组 median/P95 最大差异为 2.22e-16m。
common 使用固定 cell 中心 X0=0.75*(kx+.5)*ex+0.75*(ky+.5)*ey，
对 raw/old/candidate 使用共同通过全部门限的 cell 集合。平面坡度自由，
没有地面压平、Z=常数约束或局部 ICP。给定平面 n·(X-c)=0，
沿 up 通过 X0 的线交平面高度为：
    h = X0·up - n·(X0-c)/(n·up)
return/train 高度在同一水平位置相减；normal 角直接比较向上定向法向量。
plane 门限原样保留：>=8点、第二特征值>=0.0025、lambda0/lambda1<=.08、
P95 平面残差<=5cm、与 up 夹角<=35度，三组法向差<=10度。
summary.csv 同时包含 legacy、全候选共同集合及每候选 raw/old/candidate 三元组。
候选验收使用三元组共同集合，repeat 另用同一 family 全成员共同集合。

墙面保护
非 floor 标签的 query/train 原始点身份固定。各版本都以相同规则重建最近12邻域：
最远邻居<=.8m、lambda1>=.0005、lambda0/lambda1<=.08、|n·up|<.5。
保护指标只在 raw/old/candidate 共同支持的 query id 上比较点到局部平面绝对距离。
单版本支持数量、支持丢失/新增、共同 query 邻域身份变化数量都保留；原始 mask、
距离和邻域索引存于 *_wall_support.npz，可通过 frozen 对应 group 的 ~floor 标签
还原 query/train 的 (keyframe id, point_index)。物理表面身份仍未知。
Y=-175 的原始/旧单版本支持仅7/6点且交集0；Y=-315均0点，墙保护为 UNKNOWN。

验收规则
thresholds 已在读取新候选前冻结，不根据候选调阈值。
主区域 Y=-75/-125/-175/-225 至少3处 median 比旧优化改善>=5cm；
四处各自 median 不高于 raw+3cm、P95 不高于 raw+5cm。
回环邻域外区域 median/P95 不高于旧优化+1cm/+2cm。
有共同支持的墙面 median/P95 不高于旧优化+2cm/+5cm。
floor 共同 cell<8、墙面共同 query<25 时保留 UNKNOWN，不当作通过。
独立重复优化运行通过 --repeat 指定，按 ROI 的共同 cell 上 absolute-height median
max-min 量化重复包络，主区域至少3处改善量还须严格超过包络+1cm。
不将同程 heldout 散布称为运行重复包络；report 也保留重复轨迹位置/旋转差。
PASS/FAIL/UNKNOWN 只评价离线几何代理；不证明绝对精度，不是部署或运动授权。

可复现命令（单个普通终端；完成后进程自动退出）
cd /home/bh/A2W_FastLio2_controlled
python3 -m unittest discover -s experiments/field_backend_repair_20261023/geometry -p 'test_*.py' -v
python3 experiments/field_backend_repair_20261023/geometry/evaluate.py \
  --out captures/field_backend_repair_20261023_001/geometry/baseline

候选示例（将 CSV 占位路径替换为实际输出，标签必须唯一）：
python3 experiments/field_backend_repair_20261023/geometry/evaluate.py \
  --candidate candidate=/absolute/candidate_poses.csv \
  --candidate repeat2=/absolute/repeat2_poses.csv \
  --candidate repeat3=/absolute/repeat3_poses.csv \
  --repeat candidate_family=candidate,repeat2,repeat3 \
  --out captures/field_backend_repair_20261023_001/geometry/candidates

预期与检查
输出八行 evaluated 后打印候选 acceptance，成功进程 exit=0。
report.json 为完整结论；每 ROI JSON 保留失败 cells/支持数量；summary.csv 便于列表；
每版本 sections.png 全景使用相同横纵比例；floor_sections.png 使用统一放大的垂直轴，
须保留其 vertical axis magnified 标记。绘图只抽稀显示，度量使用全部冻结点。
若 input/cache SHA256 改变立即停止，不覆盖冻结文件重选；检查是否拿错会话或候选。
若 candidate differs from fixed first-pose map gauge，先检查导出 gauge；不得每 ROI 配准。
若支持不足则查看 failed_cell_reasons / wall mask，保留未知，不通过调选点凑通过。
Ctrl-C 可终止本离线工具；不需要停止任何机器人节点。

验证状态
已确认：六项几何/输入测试通过，其中四项覆盖合成斜坡/固定up/同水平位置/低支持门限；旧指标数值复现通过。
已确认：固定 up、点身份、cell 标签与接受阈值在首次候选读取前冻结。
待验证：候选优化结果与真实重复优化包络；真实场地物理表面身份与独立真值未知。
ROS集成/ARM64编译/硬件/运动不属于本工具验证范围。

新增因果几何 guard 模块（后续实验，默认关闭）
源码只存在于隔离副本 experiments/field_backend_repair_20261023/source/src/a2w_fastlio_mapping：
  include/a2w_fastlio_mapping/causal_geometry_guard.hpp
  src/causal_geometry_guard.cpp
  test/test_causal_geometry_guard.cpp
主工程已有文件未由几何子任务修改。模块不更改点云、外参、重力或前端算法。
调用方必须使用事务化 trial graph，guard 通过后才提交；UNKNOWN 与 REJECT 不允许提交。
assess 的输入为历史 KeyFrame map、before/trial snapshots、当前/候选 id、注册使用帧 id 集合。
只使用 id<current 的历史帧，排除确切注册所用 source/target 邻域。
选择128帧以内、每帧4096原始记录以内，body系1.5m三维voxel中自由拟合平面，
每帧64patch以内，最多256framepair/512patchrecord。全部是确定性有界采样，非全图真值。
跨时间>=30s、原始观察位置<=2m、patch质心<=.75m、有限切向footprint相交，
原始平面法向差<=10deg、相同水平位置的有符号间隔<=3cm，才作为几何代理anchor。
至少2个framepair、总共8个patchrecord才有可靠支持；不足明确UNKNOWN/reject。
同一原始point index集与固定raw reference/normal用于before/trial；斜坡保持自由。
逐pair与总体检查absolute seam median/P95退化<=3cm/5cm、normal P95退化<=1deg。
JSONL ledger保留所有source ids、point indices、支持数量、失败原因与work cap flags。
物理表面身份始终unknown；guard通过仅说明被检有界独立代理未明显变坏。
最初有界遍历对早期framepair优先，达到pair/record cap会截断后段覆盖；必须查看ledger，
不能用此代理通过替代八ROI独立验收。进一步覆盖策略由主任务审查，阈值不得事后调优。
独立Werror编译+7项C++单测通过，XML在captures/.../geometry/causal_guard_tests.xml。

覆盖修复 v2（几何门限不变）
第一版真实三回环流全部放行，而外部八ROI显示失败，属于失败候选。其源码、协议保留在
captures/.../geometry/guard_v1_source；首次 guarded_run* 输出由因子重放子任务保留。
v2 枚举所有合格历史帧对，最多524288次枚举检查，超过则明确UNKNOWN；候选帧对按排序
索引均匀抽样且包含首末项，超过128个eligible帧时最多64pair，确保<=128unique帧。
每pair预留 floor(512/selected_pair_count) 条patch记录，对通过raw支持的patch匹配再均匀
抽样；每帧合格patch同样均匀截取最多64，避免早期静止点耗尽后段记录预算。
不足最低支持预算则UNKNOWN。Ledger保留总/未采样eligible帧对数量、采样时间与原始XYZ跨度。
这仍是有界几何代理抽样，不能声称覆盖全部物理路线。外部八ROI验收仍须独立通过。
v2_protocol.json 在真实v2输出读取前冻结，validation.json保存旧版晚期异常漏检的红灯
及v2通过的8项测试，-Wall -Wextra -Werror独立编译exit0。
