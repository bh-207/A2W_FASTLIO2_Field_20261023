公开交付边界及配置准备请先看PUBLIC_EXPORT_ZH.md；本机真实baseline.yaml/接口档案与运行数据未上传。

完整输入、Z来源和联合初始化实验（2026-10-24）

结论边界
本次完成了顺序供数入口、18D联合候选→23D FAST-LIO状态/协方差真实写入实现、首次去畸变边界处理及完整重建。
真实150–215s候选未通过独立留出预测和重力不确定性门，所以真实C001/C002从头回退原初始化；没有获得已验收的联合初始化地图。
不得把这些回退地图、独立重力坐标投影或接缝重复性称为高程精度改善。

保持的输入契约
FAST-LIO2+关键帧+保守LoopPipeline+iSAM2；R=I，T=[-0.00862,0.00914,0.03916]，extrinsic_est_en=false、offset=0。
该R/T是用户指定配置，物理标定未经认证。单位rad/s、m/s²和既有驱动旋转不重复处理。P/R/J、新重力软约束与增强回环关闭，Q未改。
联合初始化参数 frontend_init_v4.joint_initialization_enabled 默认false；只能在显式 offline-test/域87/localhost-only的顺序输入模式中开启。
保留live分支端点身份/运行生命周期代码；顺序入口没有源DDS订阅、没有重复录包，不声称验证了现场DDS或原速。

入口和构建
唯一应用源码：本目录source/frontend和source/src（不是嵌套的历史field_source_snapshot）。
构建：python3 /home/bh/A2W_FastLio2_controlled/experiments/field_frontend_z_init_20261024/build.py --build /tmp/NEW_EXPERIMENT/build
构建使用本地已有ROS2 Humble及工作区.deps/install；不安装依赖、不改变主install。初次必须全包构建，增量有源码覆盖检查。
新建可用至少12GiB的输出根。本模式无重复原始Bag，不能套用旧实时录包入口的35GiB容量估计；不自动删除任何文件。

一键重新执行（会编译、完整回放三次、独立ICP/联合估计、真实后端/Bundle和固定几何评价）
python3 /home/bh/A2W_FastLio2_controlled/experiments/field_frontend_z_init_20261024/reproduce.py --root /tmp/a2w_z_init_reproduce_20261024_new
要求root不存在；拒绝覆盖。依赖本轮只读导出的input_audit/imu_raw.csv、cloud_timing.csv与原Bag。
若审核CSV需重导出，使用 input_audit/extract_input_audit.py --bag 原始Bag绝对路径 --output 新目录；不得覆盖现有证据。
命令退出0只表示实验链执行完成，必须查看reproduction_summary.json、候选accepted/receipt和几何报告，不能当精度通过。
本轮这一聚合入口只做语法/帮助检查；其全部实际步骤已分别执行并保存证据，不冒称另跑了一次整链。

单次完整运行（已有独立build时）
python3 /home/bh/A2W_FastLio2_controlled/experiments/field_frontend_z_init_20261024/run_consumption.py NEW_RUN_ID --root /tmp/a2w_frontend_z_init_20261024_001 --export-seconds 230
不传--seconds表示必须消费全部166023条IMU、8188点云。--seconds仅用于明确短程调试，不能通过全长验收。
候选组增加 --candidate 候选JSON绝对路径；必须有独立LiDAR/原始IMU证据hash。accepted=false会保存明确回退receipt。
--debug只用于本地调试，不能以gdb本身退出0替代运行/完整性结果。

保存与检查
前端运行通过EOF停止新输入，排完可覆盖scan，保留并逐ID说明最后扫描之后20条IMU，关闭状态日志和ROS。
input_audit/audit_consumption_stages.py对原Bag导出→读取→callback/adapt/buffer/sync→POST逐整数时间戳和重复重数核算。
8188同步scan里13条合法非POST：首帧1、初始化待定10、实际INIT_COMMIT1、建树1。不得减少expected数量来包装通过。
field_consumption_backend复用真实KFM/LoopPipeline/iSAM2/MapBundleWriter/Reader，逐POST消费，检查received=processed、pending=0，精确回环账本和Bundle逐点/位姿回读。
backend可执行文件由mapping包CMake原生构建；不依赖历史link.txt、对象或私有二进制。

实际交付
持久目录 /home/bh/A2W_FastLio2_controlled/captures/field_frontend_z_init_20261024_001
maps/B_009/map_bundle：完整原算法控制组；maps/C_001、C_002：请求联合但拒绝后回退组。
各map_bundle/global_map.pcd是保守后端地图；同层frontend_native_raw.pcd是相同关键帧body点在raw前端位姿下的原生地图，无过滤/压平。
各runs/*/reconstruction保存全部POST原始身份body云、sidecar、阶段状态与整数输入账本，session保存完整状态及正常退出证据。
REPORT_ZH.html为结论；map_views/common_points_3d.html是离线内嵌JS的同身份点交互图。
原/tmp输出完整保留，持久副本记录在persistent_copy.json。不得自动清理/tmp原证据或旧Map Bundle。

当前未通过/未完成
真实joint commit=0，几何改善没有通过，不推荐切换生产或现场开启新初始化。
独立高度/坡度真值缺失；部分非地面保护支撑不足、107/143评价扫描没有精确后端KF节点，保持UNKNOWN，不插值冒充。
保守后端实际LoopCommitted在重复中为4/3/4，optimized差异与前端收益分开，不用回环数证明正确性。
18项mapping纯单测通过；3个继承launch测试因缺少受控field会话/trace环境在准入处失败，失败日志保留，未放宽guard。
原速性能、真实DDS、Localization/Nav2均未验收。本次未连接164、未发布运动、未改导航地图或原始数据。
