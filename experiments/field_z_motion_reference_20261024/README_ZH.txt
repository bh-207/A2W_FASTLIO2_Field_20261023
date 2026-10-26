Z中段形状与独立运动参考专项（基于ebaebae60138，2026-10-24）

实际结论
本轮识别了被拒绝联合候选中“GLS隐含条件姿态修正与原始LiDAR提交均值混用”的问题，
增加了同一惯性均值拟合/准备提交的独立候选实现，并处理共同前缀协方差、共享原始IMU插值噪声及估计/直接传播交叉项。
实际真实窗口重估全部拒绝。没有真实JOINT_INIT_COMMIT，没有新前端地图，也没有重复生成拒绝后的回退地图。
不能把反事实154m→7.198m或一致积分154m→2.343m写成B_009地图精度改善；它们都是失败初始化的预测失配。
B_009的固定up地面仍为入口至中段+2.530502m、中段至远端-3.597845m；真实坡度与累计误差未被独立高程区分。

保持
R=I、T=[-0.00862,0.00914,0.03916]m、extrinsic_est_en=false、offset=0、alpha=1.0136393576435327。
R/T是用户指定配置，不是物理标定证明。没有改单位、驱动旋转、时间语义、Q、Huber前端、P/R/J或回环guard。
所有当前工具只读PCD/CSV/NPZ；不连接164，不启动驱动，不发布运动，不改主install/原地图/导航。
新方法不是生产默认：独立拟合器通过并不等于前端提交，仍必须经过原C++候选装载、实际INIT_COMMIT和全量/几何验收。

现有输入与输出
原控制：/home/bh/A2W_FastLio2_controlled/captures/field_frontend_z_init_20261024_001
同次原始预处理云使用B_008预先导出文件（不是FAST-LIO已去畸变body云）；B_009仍是完整输入控制地图。
工具优先读取/tmp/a2w_frontend_z_init_20261024_001/runs/B_008/reconstruction/preprocess_index.csv；若不存在，读取持久副本同名路径。
仅有前230秒的原始预处理云。后续新可信候选如需全长，仍从原830.667秒Bag重新消费，而不是拼接旧PCD。
本轮结果：/home/bh/A2W_FastLio2_controlled/captures/field_z_motion_reference_20261024_001
实际一键复现：/home/bh/A2W_FastLio2_controlled/captures/field_z_motion_reference_reproduce_20261024_001
主要文件：REPORT_ZH.html、FINAL_RESULT.json、REPRODUCTION_CHECK.json、SOURCE_PRESERVATION_CHECK.json、final_verified/、ground_shape_v3/。
SUPERSEDED_DIAGNOSTICS.json保留修正共同前缀协方差前的失败诊断；它们不参加最终候选验收。

可复制复现（分析需要>=3GiB；新目录必须不存在；不会删除/覆盖数据）
cd /home/bh/A2W_FastLio2_controlled
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 experiments/field_z_motion_reference_20261024/reproduce.py --output /home/bh/A2W_FastLio2_controlled/captures/field_z_motion_reference_reproduce_20261024_002

该入口已另建_001实际运行，exit0；三组独立参考重算、四组联合拟合均与主实验一致，maps_generated=0、frontend_commits=0。
入口允许全频原相邻ICP按真实拒绝结束；这不是完整轨迹，也不算通过。其余完整子图参考仍进入联合一致性验证。
所有候选拒绝时不会调用前端、后端或生成Map Bundle。若未来修改产生通过候选，才会独立构建原已发布前/后端源码，
准备一次训练首帧→B_009锚点SE3，进行全长重放，并检查真实committed receipt后才允许后端Bundle保存与自身重复。
本轮通过分支没有执行，保持NOT_RUN；几何指标必须另审，不因为脚本exit0就通过地图精度或实时验收。
prepare_candidate.py遇到accepted=false直接拒绝，不会改写为回退候选；旧prepare入口不要用于本轮训练首帧坐标定义。

独立构建与回归
python3 experiments/field_z_motion_reference_20261024/build_reference.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s experiments/field_z_motion_reference_20261024 -p 'test_*.py' -v
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s experiments/field_z_motion_reference_20261024/joint_init -p 'test_*.py' -v
实际：17项新增回归、17项继承联合回归通过。C++积分核用g++/Eigen独立编译，无ROS运行节点，无安装动作；build/identity.json记录命令/编译器/源码与.so哈希。
依赖本机已有numpy/scipy/matplotlib/Eigen/g++；Open3D仅用于已失败的GICP对照和对应回归，不作为默认运动参考。
协方差有限差分通过只证明既定一阶iid噪声模型的实现，不证明真实噪声、共享子图系统误差或物理外参已标定。

补充实验命令
MPLCONFIGDIR=/tmp/a2w_z_motion_mpl OPENBLAS_NUM_THREADS=1 python3 experiments/field_z_motion_reference_20261024/make_report.py /home/bh/A2W_FastLio2_controlled/captures/field_z_motion_reference_20261024_001
该命令只更新本轮报告，不重放、不改地图。完整XYZ数值使用未缩放点；3D展示各轴显示比例独立并明确标注。
训练选择已冻结在captures/field_z_motion_reference_20261024_001/training_window_selection_frozen_v2.json；不使用215～230秒验证误差选窗。
分段原始整数stamp、源scan ID、每次参考失败、目标历史scan ID与补偿阶段均保留在registration.json和相关CSV/NPZ。

未完成与最低补测
真实可信联合初始化、B_009中段隆起改善、新候选全长地图与几何验收未完成；当前不推荐现场开启新初始化。
窗口自由g/ba模型仍有径向g/近竖直ba弱方向；已知|g|使S2形式rank5不代表模型正确。训练chi2及独立预测明确失败，不能只归因激励不足。
至少现场绑定入口E、中段M(Y约-175m)、远端F(Y约-315m)三个可复找实体地面点，独立测两段高差，
比较+2.530502m与-3.597845m。要进一步区分角度参考/IMU/固定模型，仍需独立角度基准或可靠多方向结构运动片段。
绝对高程精度、原速、真实DDS、Localization/Nav2均不在本轮通过范围。新源码目前仅保存在本机，未自动上传Github。
