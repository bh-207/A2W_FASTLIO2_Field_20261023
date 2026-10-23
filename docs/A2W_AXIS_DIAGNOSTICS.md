# A2W 单轴方向核对（2026-09-22）

## 本轮边界与证据

先完成单轴测量，再启用 PC 外参和 Nav2 base_link；不直接翻转 cmd_vel。
164、FAST-LIO、Localization 算法、Map Bundle、Nav2 controller/planner 参数不变。

只读核对的官方来源：
https://github.com/unitreerobotics/unitree_ros/blob/master/robots/a2_description/urdf/a2.urdf

官方给的是 `T_base_front_lidar`：

* xyz = [0.33767, 0, 0.08134] m
* rpy = [1.5708, 0, 1.5708] rad
* 后雷达的 xyz 是相对于 **front_lidar_link**，不是 base_link，不能直接拿来当后雷达在 base 下的平移。

164 的只读证据文件：
机器人 Hesai 工作区的 `src/HesaiLidar_ROS_2.0/config/config.yaml`（安装位置由现场确认）。
安装目录的 config.yaml 是指向该文件的符号链接。运行中的驱动来自 hesai_ws。
该配置 source IP 为 192.168.124.20，通用 transform_flag=false；但另有
`ros.frame_normalization.enable=true`，两者不能混为一谈。

`R_aligned_raw = [[0,0,-1],[-1,0,0],[0,1,0]]`。
`src/manager/source_driver_ros2.hpp` 对点云 xyz、IMU 加速度和角速度应用相同旋转，
分别发布 hesai_lidar_aligned / hesai_imu_aligned。保持此驱动和 R 原样。
配置中的文字注释不能代替实际矩阵乘法。

令 R_B_L 为官方安装旋转，R_A_L 为驱动矩阵：

```
R_B_A = R_B_L * transpose(R_A_L)
      ≈ diag(-1,-1,+1)
T_A_B = inverse(T_B_A)
```

在“当前输入确为前雷达、body 与 aligned IMU 轴一致，IMU 与雷达原点暂视为重合”的前提下，
PC 候选 `body -> base_link` 为：

```
xyz = [0.3376697012, -0.0000012403, -0.0813412403]
rpy = [0.0000000000135, -0.0000036732, 3.1415889804]
```

近似为 `[0.33767,0,-0.08134] / [0,0,pi]`，来自组合外参而不是凭反向运动猜测。
原点平移仍须核对真实 IMU 相对前雷达的偏置；三次脉冲不能独立标定完整六自由度外参。
2026-09-22 已完成单轴方向检查并由操作者确认：正vx真实前进、body X约-0.128m；
正vy=0.30真实左移、body Y约-0.093m；正wz真实逆时针、body yaw约+0.098rad。
因此 PC URDF 已采用名义 `[0.33767,0,-0.08134] / [0,0,pi]`，Nav2 基座改为 base_link。
这是方向证据支持的名义安装值，不是精确六自由度标定；平移保留未标定标记。
修改只在重新启动 PC 进程后生效，不在运行时热换 TF。

## 已实现的速度策略

MotionBridge 限幅后执行 deadband：vx=0.01 m/s、vy=0.01 m/s、wz=0.02 rad/s。
绝对值小于阈值置零；达到阈值后原值通过。不交换轴、不改符号、不抬升小速度。
旧 min_effective_* 参数仍可解析但不再生效；启用旧值会警告。
全部归零立即 StopMove；现有 watchdog 和 disable 保持有效。
已有 velocity_smoother 的加速度限制不变，不叠加新的斜率限制。

`/a2w_motion_bridge/request_audit` 是实际构造并发布的请求的本地审计，包含 request_id、api_id、
parameter、ROS/单调时钟时间。它**不是机器人执行确认**。诊断工具另记录
`/api/sport/request` 及 `/api/sport/response`，便于核对其他控制源。
本机 Humble 的 Python 回调只收到消息，不提供每条消息的发布源 GID；记录中该字段为 null，
不能用 graph 上发现的 GID 冒充实际来源。运动期间将请求编号、API、完整参数与 Bridge 审计关联，
允许跨 topic 到达顺序不同；150ms 内无法关联的 Move/StopMove 将中止测试。
这是竞争控制源检查，不是 DDS 身份认证，也不是机器人执行确认。

## 操作

空旷场地、遥控急停在手。先停止之前的 PC 导航实例，不要同时启动两份。

终端 1（只准备定位和隔离控制链，尚不运动）：

```bash
cd ~/fast_lio_ros2_ws
A2W_NETWORK_INTERFACE=enp49s0 ./scripts/run_a2w_navigation.sh "$PWD/maps/factory_large_v3" axis_diagnostic_mode:=true auto_enable_motion:=false
```

诊断模式仅改变 Guard 的输入为 `/a2w/axis_test/cmd_vel`。Nav2 仍可计算路径，
但其 /cmd_vel 不进入硬件输出。正常模式仍按原 Collision Monitor 配置连接。
**单轴诊断入口不经过 Collision Monitor**，只保留定位/就绪门和硬件看门狗；不能把此模式用于导航。

终端 2：

```bash
cd ~/fast_lio_ros2_ws
source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"

# 默认只读，可先验证记录工具，不会启用/关闭运动接口
ros2 run a2w_motion_bridge axis_probe --record-seconds 5

# 等待 LOCALIZED / Nav2 就绪后，一次执行一条；每条还需输入轴名确认
ros2 run a2w_motion_bridge axis_probe --axis vx --enable-motion
ros2 run a2w_motion_bridge axis_probe --axis vy --enable-motion
ros2 run a2w_motion_bridge axis_probe --axis wz --enable-motion
```

速度固定为 +0.10 m/s、+0.10 m/s、+0.20 rad/s；默认每轴 1 秒，duration 最大 2 秒。
横移速度可显式指定 `--vy-speed 0.30`（最大0.30），X和转向速度不受影响。
诊断模式两道安全门的横移上限为0.30；普通导航仍为0.15。
若上次启动在本次限幅更新之前，必须先停止并重启终端1的 PC 诊断启动程序；
不需要修改或重启164驱动。工具会读取两道门的运行参数，限幅不足时在启用运动前拒绝测试。

```bash
ros2 run a2w_motion_bridge axis_probe --axis vy --vy-speed 0.30 --duration 1 --enable-motion
```

工具取消已有 NavigateToPose Goal、验证隔离输入、唯一 Bridge 发布源和健康数据，
才临时开启原有两道安全门。结束后立即零速并关闭硬件输出及 Guard。
数据过期、Goal 再次激活或 Move/StopMove 无法及时关联 Bridge 审计会中止本次脉冲。
Ctrl+C 保留已记录文件并尝试软件停车。服务超时必须使用遥控急停。

输出：`log/axis_diagnostics/<axis>_<timestamp>/events.jsonl` 和成功测量的 `summary.json`。
body/base_link 位移均在各自**起始帧**表达，yaw 使用相对四元数计算；不能把 map 位移直接当机体位移。
旧日志中的单位外参使 body/base_link 结果相同，不构成独立标定证据。
加载新 URDF 后，两者会按固定旋转和杆臂转换；它们仍来自同一份里程计，不是独立传感器测量。
请同时记下真实“前/后、左/右、顺/逆时针”；可使用现有 AprilTag axis-test 提供独立测量，
但不得同时运行两个运动发布程序。

停止命令（不是物理急停替代品）：

```bash
./scripts/a2w_stop_now.sh
```

## 验收和后续顺序

三轴方向已核对，PC 使用上述名义外参，Nav2 基座已切换为 base_link，STEP footprint 保持不变。
首次重启保持运动关闭，先确认 TF / RViz 机头 / 障碍位置，再做隔离单轴复测。
随后单独验证 TF 发布时序、控制耗时和 odometry twist，再考虑标准 odom 适配层。
不因旧日志的数秒配准耗时重写已经存在的独立 worker；不先降低20Hz、放宽 progress checker。
改到标准 map->odom->base_link 时，原始 body TF 须隔离，避免双父节点；Map Bundle 不重命名。

三轴符号已有本次 JT128 实机证据；新 TF 下的导航效果与精确外参仍待现场验收。

离线验证：相关129项测试通过，包含独立 robot_state_publisher 进程实际发布固定TF、
接收TF后将负body X/Y位移转换成正base_link X/Y的检查。a2w_description 与
a2w_navigation 编译通过；未自动重启实机进程或发送运动命令。
隔离测试使用 ROS_DOMAIN_ID=87、lo 接口、禁用组播，并显式设置
CycloneDDS Discovery/ParticipantIndex=auto，以保证两个测试进程能通过单播互相发现。
