公开配置和数据边界请先阅读 PUBLIC_EXPORT_ZH.md。实际baseline.yaml/接口档案及运行数据不上传。

本轮只对新Bag完整重建及分析，不引入新滤波算法。
输入：/home/bh/A2W_FastLio2/bags/factory_live_20261024_195303_YOI7fO
完整输入控制：355087 IMU、17778点云；读取到EOF，原始时间与逐点时间不修改。
源码独立复制自 experiments/field_frontend_z_init_20261024/source。
C++适配：离线完整性计数增加由当前Bag元数据绑定的expected参数；保留旧默认值和严格相等检查。
本Bag预计约192万条分阶段账本，独立副本的有界诊断记录容量由100万增加到250万；不是DDS队列或算法缓冲修改，溢出仍为失败。
FAST-LIO滤波、去畸变、初始化质量门、固定R/T、Q及保守后端均保持。
R=I,T=[-0.00862,0.00914,0.03916],extrinsic_est_en=false，用户指定，物理标定未认证。
联合初始化、P/R/J、增强回环、重力软约束均关闭。
ROS_DOMAIN_ID=87,ROS_LOCALHOST_ONLY=1，顺序供数；无原始数据再次录制、无设备连接或运动发布。
这证明离线算法输入完整性，不认证现场DDS原速或绝对高程精度。
输出位于 captures/factory_live_195303_20261025_001，已有输出不覆盖。
