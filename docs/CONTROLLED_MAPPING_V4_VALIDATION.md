# 受控 v4 采图交付验证（2026-10-21）

现场完整顺序见 [受控采图操作指令](CONTROLLED_MAPPING_V4.txt)。旧生产入口、主 install、原 Bundle 不切换。

## 本次真实执行的软件验证

- 独立 Release 编译了当前补丁前端及配套后端。最终前端在 `.controlled_mapping/frontend/build/fastlio_mapping`；未复用旧主 install。最终运行库核查没有 Conda 路径或缺失依赖。
- 后端 CTest：17/17 通过，包含 14 个 GTest 目标及 3 个实际 ROS 启动/保存/排空测试。首次未 source 独立环境的运行失败，正确 source 后复跑通过；未改测试掩盖。
- 会话/安全/录包工具：58 项通过；构建隔离 4 项、前端机制 25 项、运行状态 10 项、联测安全 8 项、回环提交监视 7 项通过。项目原测试套件通过。
- 完整 localhost 合成输入联测 v5 通过：实际 ROS 前端、ingress、后端、rosbag2、服务保存和正常退出，不是无 ROS harness 替代。

| 项目 | 实际结果 |
|---|---|
| 初始化 | 真实 `INIT_COMMIT`，窗口末 IMU/state/group stamp 一致 |
| 激光更新 | 68 次 POST，68 次均有有效激光特征 |
| 状态关账 | attempted=enqueued=written=69，dropped=write_errors=0，closed/flush_ok=true |
| 保存队列 | received=processed=4，pending=0，active=none，state=saved |
| 地图构建 | full_map_builds=1；新 Bundle 4 个关键帧，清单检查通过 |
| 最小 bag | points=106、IMU=2123、Odometry=68、TF=104、静态 TF=1、配准状态=13 |
| 静态 TF | 起录前已发布，晚订阅实际录到；不是事后补造 |
| 退出 | recorder、三个节点和合成发布者正常退出，cleanup_errors=[] |

联测墙钟 18.63 秒；该小型合成数据的耗时/内存不是 JT128 全厂实机录制开销验收。
采样点、IMU、逐点时间、frame 保留原输入；只为合成回放统一平移其时钟，变换及输入 hash 独立保存。
这些结果不证明无丢帧、不证明首次实机回环或地图几何精度。

本机证据目录为项目根目录下 `artifacts/mapping_capture_20261021/`，不上传 GitHub。
联测结果在 `full_capture_smoke_v5/result.json`；各自完整会话包含 bag、状态、参数、版本和新 Bundle。
v1～v4 的失败记录保留：Humble 嵌套命令参数解析、SQLite 边录边读锁冲突、录包节点身份判断，以及联测脚本静态 TF 字段不一致。修复均增加回归；最终复跑通过。

## 关键实现边界

- SQLite 使用 resilient/WAL，实际确认本会话 bag 已提交两路 raw 后才允许前端启动。
- 当前 Humble rosbag2_py 0.15.16 不能通过 Python 指定唯一 recorder 节点名。采取启动前独占检查、两路新增固定 GID 绑定、自有 PID 身份和本会话 SQL 联合核验；不冒称密码学 PID—DDS 绑定。任何额外/漂移/丢失身份永久拒绝。
- 启动 summary 记录实际二进制/库 SHA、配置、单位/时间来源、实际启动环境与日志路径；init 声明和当前环境分开。
- 初始化质量门开启，但地图重力对齐未启用。保留真实 g 和 `angle(-g_camera_init,+Z_camera_init)`，不把单位姿态或地面法向冒充物理水平。
- 只在实际 GTSAM 提交完成后发 `[LoopCommitted]`。当前小型合成联测未产生实机回环，提交监视仅通过自动化事件测试。
- POST 时间继承 FAST-LIO 内部 double scan-end 后取整，不冒充输入 header 精确整数。INIT 额外保留实际接受窗口整数时间。

## 当前实机缺项

164 驱动已修正是已知前提，但本地 `maps/` 未找到其可审核配置来源，两条只读 SSH 连接均超时。
本机已经保存 `captures/jt128_interface.json` 和 `captures/driver_source_status_20261021.md`。
未知的真实单位、frame、偏移及来源 hash 没有猜测；来源未补齐时可以先录 raw，但严格前端拒绝 start。
请使用实际已标定配置及已确认的驱动 domain，不能使用合成 fixture 或默认单位阵替代。

因此交付结论是：**受控软件全流程已离线实跑；164 实机输入绑定、静止门限、短闭环和录制负载仍待现场验收。**
