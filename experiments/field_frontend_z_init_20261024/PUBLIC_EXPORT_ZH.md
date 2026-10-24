# 2026-10-24 完整输入重建与联合初始化源码交付

本目录保存本轮实际应用源码、独立CMake构建入口、顺序Bag供数、联合g/ba/bg辨识、真实Process写入/协方差/首次去畸变回归、固定位置高程坡度评价和完整后端导出工具。

## 结果必须如实保留

- 同一830.667秒Bag：166023 IMU、8188点云完整消费；8175 POST，845KF。三次全长前端/保守后端/Bundle保存与自身重复已完成。
- 新联合初始化默认关闭。真实150–215秒候选未通过独立留出预测和不确定性门，C1/C2实际从头回退原算法；没有通过几何改善验收的新前端，不得部署或把回退曲线冒充优化效果。
- 数学17项、地面评价22项、整数重数3项及实际Process/Bundle回归有本机证据；18个后端纯单测通过，3个继承ROS launch测试仍因受控会话/trace环境在guard处失败。原速/真实DDS/导航未验收。
- 固定R=I、T=[-0.00862,0.00914,0.03916]、extrinsic_est_en=false为用户指定配置，未经物理标定认证。Depth100/BEST_EFFORT、初始化门、原尺度/时间、Q保留。未连接164，未发布运动，未切换生产地图。

## 源码与数据边界

本机原应用C++/Python内容不为上传修改；省略不参与此入口构建的导航包/嵌套历史快照。source/frontend及source/src四包是实际应用源码，不依赖旧对象或link.txt。上轮稳定化工程修改已包含在这些源码中，历史field_backend_repair入口仍保持历史版本。
source/third_party/TEASER-plusplus与pmc以相对符号链接复用根目录固定子模块；先按仓库根说明初始化子模块和ROS/GTSAM依赖。新的构建必须生成新SHA，不能冒用本机历史身份。

公开排除：Bag、地图、现场点云、运行CSV/日志、报告图片、冻结评估缓存、二进制、编译缓存、凭据、现场接口档案。仅复用仓库已公开的3个原有配准测试PCD，不增加新现场数据。
实际config/baseline.yaml中嵌有驱动/单位/frame审核档案，因此不上传；source/config/field_input_contract.json同样排除。baseline.example.yaml仅保留算法参数和空证据，input_interface_enabled=false、source_provenance=unknown，不伪造有效证据，也不允许直接启动。

## 在已有本机数据环境复现

1. 阅读本目录README_ZH.txt及根目录依赖说明；原Bag、参考Bundle配置、只读IMU/时间审核CSV和冻结评估集合须由操作者另行提供。当前脚本保留原绝对路径边界，不宣称新克隆开箱即用。
2. 将已在本地审核的真实运行配置保存为config/baseline.yaml。若从baseline.example.yaml复制，必须补充真实SI/frame/时间证据并明确启用输入；不能把unknown或空字符串改成虚假认证。质量门保持开启，联合初始化默认关闭。
3. 构建并按本目录README_ZH.txt的独立入口执行。缺原Bag/配置/证据时失败是预期保护，不降低expected、不补造数据。

```bash
python3 experiments/field_frontend_z_init_20261024/build.py --build /tmp/NEW_Z_REPRO/build
python3 experiments/field_frontend_z_init_20261024/reproduce.py --root /tmp/ANOTHER_NEW_Z_REPRO
```

聚合包装器做过语法/帮助检查，其各组件均已分别完整执行；不冒称额外整链重跑。编译仅证明构建成功，退出0只证明执行完成，均不等于几何改善或原速通过。
