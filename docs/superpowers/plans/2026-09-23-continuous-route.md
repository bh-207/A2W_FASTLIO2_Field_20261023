# 采点到连续 A→B 跟踪实施记录

目标：按用户确认，连续跟踪采样路线，只在 B 点停止；暂停 20 Hz 性能优化。
沿用当前分支和现有未提交修改，不做全仓回归、提交、自动运动或现场重启。

## 接口与边界

- 输入：现有 waypoint_recorder 的 map/base_link YAML。
- route_path.py：复用 GO2 弧长加密、移动平均、端点保持；不声称是避障优化器。
- route_follower.py：默认仅发布预览；显式 execute 后检查定位、安全门、TF、起点、
  global costmap 和 IsPathValid，再提交一次 FollowPath（FollowPath/goal_checker）。
- 定位失效/门关闭/Ctrl+C：取消本任务，失败不重发、不自动恢复。
- 手控采点：navigation.launch.py 增加 start_motion_bridge=false，保留定位/模型/地图。
- 不修改 Localization、MPPI、速度参数、FAST-LIO 或 Map Bundle。

## 执行与验证

- [x] route_path 11 项单测 RED→GREEN：格式、平滑、端点、无默认目标、地图中心线检查。
- [x] ROS 隔离域集成测试先 RED：预览无 Goal；一次整条路径；取消/恢复不重发；障碍拒绝。
- [x] 实现 route_follower 及安装入口，6 个真实 ROS 客户端/模拟服务集成测试 GREEN。
- [x] 采点启动参数和 RViz 路径显示；启动条件测试 RED→GREEN。
- [x] 隔离构建 waypoint_tools/bringup/navigation；包内 25 项 pytest 通过；不重启当前节点。
- [x] README 记录可执行现场命令、限制及回退停止方式。

补充：停止脚本增加 FollowPath 取消；桩 CLI 实际执行验证先关闭硬件门，再关闭安全门，
再取消两类 Action。与 bringup 测试合计 19 项通过。
本轮未做全仓回归、分支提交和真实运动。现有 20 Hz 问题保持暂停。

重点风险：平滑切角、旧 body 坐标采点、过期 TF、安全状态中断、Goal 尚未接受时取消。
这些由路径验证、frame 校验、TF 时效检查及取消测试覆盖；真实控制器到点/避障留待现场。
