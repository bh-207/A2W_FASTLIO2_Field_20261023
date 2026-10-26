# 2026-10-25 新真实运动Bag完整回放工具（2026-10-26上传）

这是FAST-LIO2＋实际关键帧管理＋保守回环＋iSAM2的完整离线入口，不是LIO-SAM或新的地面约束算法。
实际1777.762秒输入：355087条IMU、17778点云，17765次POST，1841KF，15452713点，3次LoopCommitted。
整数时间戳/重复重数分账、完整输入、Map Bundle回读及正常退出已核对。几何结果有改善也有退步，不认证高程精度或生产可用性。

## 适配和保留

- 本机真实C++源码及Python工具按字节保留。唯一C++差异为当前Bag元数据expected计数绑定（仍严格相等）及有界诊断账本100万→250万条。后者不是DDS队列修改，不用它声称实时通过。
- 原预处理、逐点时间、去畸变、初始化质量门和滤波方程保留；Q/Huber未调。R=I，T=[-0.00862,0.00914,0.03916]，extrinsic_est_en=false，是用户指定而非认证标定。
- 联合初始化、P/R/J、增强回环、新重力软约束关闭。顺序供数没有源DDS发布/订阅，不认证现场DDS或原速性能。
- 保留启动失败证据在本机；最终成功会话B_003。ROS_LOG_DIR/ROS_HOME隔离，域87，ROS_LOCALHOST_ONLY=1，禁多播、仅127.0.0.1对端；RMW自行选择lo，XML不重复指定。

## 公开文件与本机资料

Bag、PCD、Map Bundle、原始/运行CSV、NPZ、日志、构建产物、报告图片、驱动接口档案和实际baseline.yaml不上传。
baseline.example.yaml故意禁用输入并清空证据。需要操作者将本地已审核配置另存为config/baseline.yaml；不能编造证据、关闭质量门或减少expected来启动。
source保留实际编译依赖闭包，第三方TEASER-plusplus/pmc以相对符号链接复用根目录固定子模块。
保留代码中的本机Bag/配置路径约定；原数据和参考Bundle仅配置快照需要由操作者提供，不声明新克隆无数据即可复现。

```bash
python3 experiments/factory_live_195303_20261025/build.py --build captures/factory_live_195303_20261025_001/build
# 先准备已审核的本地baseline.yaml、原Bag及参考Bundle；新ID不得已存在。
python3 experiments/factory_live_195303_20261025/run.py --id B_004
python3 experiments/factory_live_195303_20261025/test_counts.py
```

analyze.py、analyze_z_sources.py、local_examples.py、report.py保存本次B_003的分析方式，依赖该会话实际产物；不是自动接受任意新候选的评分入口。
全部8区旧数据规则没有迁移为新Bag物理真值。自动低面/重访的物理身份为UNKNOWN，NN尾部混合遮挡和覆盖，不能冒充绝对精度。
本机静态PNG已检查；浏览器file://策略阻止自动打开，3D交互浏览器验收未完成。
