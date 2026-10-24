# 固定地面高程和坡度评价

本工具只读已有数据，不执行 ROS、Bag 回放、编译或机器人操作。
主比较为完整供数原算法 baseline 与联合初始化候选。旧 828 KF 仅作历史参考。

## 冻结与几何定义

1. 复用旧八 ROI 的扫描时间分组，不将 heldout 称为未查看的独立验收集。
2. historical 模式保留旧 body PCD record 身份和 floor 标签；它不能冒充新前端原始点身份。
   旧 cell 来自 old optimized 坐标，raw 点已可能移动出其 footprint，因此保留 legacy_cell，
   在首次候选前按 raw828 坐标重新冻结本评价的 cell，避免扩大外推门限掩盖缺失支持。
3. 新 baseline 模式按精确原始 scan id/header stamp 连接；完整未体素 deskew body 云
   需要保留跨运行稳定的 preprocess record 身份。仅点数相同不是身份证明。
4. up 固定为**该次完整原算法** INIT_COMMIT 的反重力方向。历史模式单独保留旧 INIT up。
   不给候选重选方向，不把 FAST-LIO 地面法向视为独立 IMU 真值。
5. ground 标签仅在 baseline 冻结：12m range、6m 固定 ROI、原方法 body-relative
   [-2.5,-0.15]m 地面候选带、0.12m 代表点、0.75m cell。候选不重选点、不裁 Z。
   自由拟合平面要求 >=8点、lambda1>=.0025、lambda0/lambda1<=.08、
   残差 P95<=.05m、与固定 up 夹角<=35度。候选失败/缺点必须保留 UNKNOWN。
6. 每 ROI 主锚为离 baseline region center 最近、三组均可靠且固定 cell 中心落在
   各自平面支持凸包 10cm 外扩内的 cell。候选不可更换锚或丢掉失败区域。
7. 固定纵轴为起点锚到远端锚在 up 水平面内的单位方向。每个固定 cell 中心
   沿 up 与自由平面求交获得有符号高程。段间高差和纵坡使用固定锚及固定段长。
   不以全云 Z 跨度评分，不减趋势、不强迫地面水平、不做局部 ICP。
8. 墙/设备保护使用冻结非 floor 点与 baseline 固定近邻的局部平面残差、法向和
   支持状态；这些只是结构代理，物理设备/同面身份无人工确认时保持 unknown。
   往返高差作为额外接缝保护；它不替代纵向地面剖面。

高程为固定 gauge 下的推定几何量，缺少独立测量时 absolute_accuracy=unknown。
不依据 raw 基准更低、更平或往返更重合宣称物理精度改善。

## 候选接口

index CSV: source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,cloud,
tx,ty,tz,qx,qy,qz,qw。T_O_body，四元数 xyzw；cloud 支持 ASCII/binary PCD
或含 body[N,3]/point_id[N] 的 NPZ。PCD 可有显式 point_id；若只有 PointXYZI，
必须提供双方相同且有实现证据的 identity contract，声明跨运行稳定预处理与排序。
每源 scan 的 header、点数、point ids/identity hash 均检查；不能用 NN 构造同点。

先运行 freeze 子命令生成不可变 manifest/NPZ，再评候选。冻结输入和输出均 SHA256。
任何缓存/来源 hash 改变立即停止。候选只读取冻结 cell 身份，不重新做选点。

## 验证

合成真缓坡保留；固定水平位置避免质心偏差；固定 up 不自动消除候选倾斜；
缺点/不平面/支持外移保持未知；精确 scan 身份与旧 double 量化连接有独立测试。
真实旧828历史基准实际运行。新baseline/joint待输出后运行，不能预先宣称通过。

当前方案已向主任务回报并获同意。实现位置仅限本 ground_eval 和新 O/ground_eval。
