# 828KF后端保护与工程修复（独立入口）

实际源码：source/frontend、source/src；构建：build.py。原仓库其余入口与field_source_snapshot保留历史身份，不会自动升级或切生产。

已修复必要端点身份/STOPPING、ROS covariance发布、实际wait退出及日志关闭、后端错误返回0、完整性核算首尾缺样与部分构建身份遗漏。新增精确因子账本与默认关闭事务化几何保护。

独立构建通过；最终20s原速回归4000 IMU、189云完整，保存/退出通过。冻结828KF三回环8组合×3重算；中段四低面中位从10.97/12.71/14.09/10.38cm降至0.66/1.37/1.31/1.20cm，但wall125 P95从41.11升至55.75cm，整体几何FAIL。历史精确因子缺失，新测量不冒充历史回环，不判定具体误回环。

两轮全长原速回放均输入/录包完整性FAIL；013有并行大Bag I/O干扰。最终补丁只完成短程回归，不能称整程或部署通过。所有失败数据保留本机，不上传。

```bash
python3 experiments/field_backend_repair_20261023/build.py
python3 experiments/field_backend_repair_20261023/reproduce.py my_fresh_repro_001
```

先按根目录说明初始化固定子模块及ROS/GTSAM等依赖。源码包不会提供Bag、地图、运行日志、冻结评估缓存或现场接口档案。离线/现场实验需操作者提供本地原数据与可信输入契约（source/config/field_input_contract.json）；未提供则拒绝启动，不伪造证据。保留原数据路径限制，不宣称新克隆硬件开箱即用。

source/third_party/TEASER-plusplus和pmc使用相对符号链接复用仓库固定子模块。所有发布源码的C++/Python字节保持本轮实现；不附带私有二进制或历史link.txt。

[按终端完整命令](RUNBOOK_ZH.txt)。guard默认关闭；false/T=[-.00862,.00914,.03916]/R=I为用户指定、物理精度未认证。无P/R/J、无新增重力约束、无运动或导航切换。
