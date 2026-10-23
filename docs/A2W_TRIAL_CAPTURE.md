# 定位与导航试验采集

`./scripts/record_a2w_trial.sh`记录已有ROS图并离线评估，不启动驱动、定位、Nav2或MotionBridge，不设置网络、发送Goal、授权运动或调用底盘接口。操作者使用已有运行流程完成试验；录制器只订阅和写文件。`evaluate`只读SQLite/CDR，不播放bag、不创建ROS节点。

## 先固定试验条件

每种场景至少做3次独立试验，每次使用新的输出目录。重复之间保持同一地图Bundle、同一原始map目标/路线、同一速度配置、同一实体起点/终点参考和载荷。记录实际配置快照、参考点标识、障碍位置、试验序号和人工干预；不能把不同地图或不同目标的结果合成提速/精度结论。

路线须为`map/base_link`，且`--route-file`末点应对应当次实际目标。保存的65点路线位于`maps/factory_large_guard_20260923_222120/routes/ab_20260923_235028.yaml`，长度124.827713m；完整执行通常超过120s。下面120s是命令示例：短场景使用与真实短段目标一致的已保存路线，完整65点试验应覆盖启动、到达或明确终止及停止后的状态。不要把短段结束位置与65点路线终点比较后声称到达误差。

现场参考位置必须可重复识别，例如已有固定地面标记。需要物理精度结论时另存独立测量方法、原始测量和时间对应；`map→base_link`、odom及同一定位系统产生的坐标不构成独立真值。

## 预览与录制

在工作区根目录运行。下面变量会展开为绝对路径；替换成当次实际地图、路线和新输出目录：

```bash
trial_map_bundle="$PWD/maps/factory_large_guard_20260923_222120"
trial_route_file="$trial_map_bundle/routes/ab_20260923_235028.yaml"
trial_output_root="$PWD/log/a2w_trials"

./scripts/record_a2w_trial.sh record --scenario straight --duration 120 \
  --map-bundle "$trial_map_bundle" --route-file "$trial_route_file" \
  --output "$trial_output_root/straight_01" \
  --note "repeat=1 start_ref=A goal_ref=B speed_ref=unchanged_config independent_truth=none" \
  --dry-run
```

`--dry-run`只输出topic/QoS/录制命令计划，不录制、不创建试验目录，也不验证活跃publisher或归档文件的实际hash。确认路径和场景后，以同样参数去掉`--dry-run`录制：

```bash
./scripts/record_a2w_trial.sh record --scenario straight --duration 120 \
  --map-bundle "$trial_map_bundle" --route-file "$trial_route_file" \
  --output "$trial_output_root/straight_01" \
  --note "repeat=1 start_ref=A goal_ref=B speed_ref=unchanged_config independent_truth=none"

./scripts/record_a2w_trial.sh record --scenario turn --duration 120 \
  --map-bundle "$trial_map_bundle" --route-file "$trial_route_file" \
  --output "$trial_output_root/turn_01" \
  --note "repeat=1 start_ref=T_A goal_ref=T_B turn_ref=fixed_corner speed_ref=unchanged_config"

./scripts/record_a2w_trial.sh record --scenario lateral --duration 120 \
  --map-bundle "$trial_map_bundle" --route-file "$trial_route_file" \
  --output "$trial_output_root/lateral_01" \
  --note "repeat=1 start_ref=L_A goal_ref=L_B heading_ref=fixed speed_ref=unchanged_config"

./scripts/record_a2w_trial.sh record --scenario avoidance --duration 120 \
  --map-bundle "$trial_map_bundle" --route-file "$trial_route_file" \
  --output "$trial_output_root/avoidance_01" \
  --note "repeat=1 start_ref=A goal_ref=B obstacle_ref=fixed_position speed_ref=unchanged_config"
```

这些命令不会选择或执行路线；`--scenario`只是试验分类，`--route-file`只是归档及评估输入。四场景应分别填写对应实际路线。每次重复更换`_02`、`_03`目录和note序号，独立完成同一试验，不直接对已存在目录重录。

指定`--map-bundle`后，工具默认归档并校验**manifest.sha256列出的全部文件，包括关键帧PCD**，同时归档manifest、路线和当前相关YAML配置，记录SHA256。不会仅保留子地图或配置冒充完整输入；未列入manifest的额外文件不属于Bundle归档。原始输入读入时和复制后都校验hash，缺失/变化会报错。`--map-bundle`和`--route-file`在CLI中可选，但完整可重放试验应提供这两项。归档源码YAML不能自动证明当前运行节点的有效参数；若有命令行覆盖或独立install版本，应另存实际启动命令/只读参数快照，并在note注明速度依据。

时限结束或Ctrl+C只停止录制进程，**不会取消机器人导航或替操作者停车**。随后工具自动评估`bag/`。退出0要求录制退出正常且`capture_complete=true`；退出2须检查`recorder.log`与`evaluation/summary.json`，不能当完整采集成功。

## 必须收齐的数据

| 数据组 | 完整性要求 |
|---|---|
| 原始传感器 | `/lidar_points`、`/lidar_imu`均有实际消息；占位空消息不支持传感器重放结论 |
| 同帧前端输入 | `/Odometry`为camera_init→body，`/cloud_registered_body`为body；保留source stamp |
| 定位 | `/localization/pose`、`/localization/status`、`/localization/registration_status`；至少有normal stage=`localization`的注册记录 |
| 完整TF | `/tf`及latched `/tf_static`；能找到map→camera_init、camera_init→body和body→base_link三条边 |
| 导航评估 | 原始/发送/执行路径、body终点误差、Nav2 ready和安全状态、`/cmd_vel_nav`、`/cmd_vel_guarded`、local costmap，以及Action状态/反馈 |
| 事件与审计 | bridge状态、request_audit、碰撞监测、rosout阶段耗时、必要时的`/a2w_validation/events`及操作者时间记录 |

录制计划包含hidden Action topics、传感器QoS和transient-local TF/路径QoS；topic被列入计划不等于收到消息。评估的`capture_complete`只表示定位重放输入与合约完整，还应检查`navigation_data_complete`及其缺失项，才能讨论导航跟踪。存在decode/frame/nonfinite错误时不判完整。

## 四场景的重复验收

每次使用相同配置和参考条件，保存全部3次结果及失败试验，不只保留最好一次。至少对比normal接受率/拒绝原因、分阶段耗时p50/p95/max、状态转换、TF/pose时序及残差、路径偏差、body终点误差和完整原始记录。到达由当次Action结果和参数快照中的goal checker容差判定；真实位置误差另由独立测量判定。

| 场景 | 固定条件及可检验行为 |
|---|---|
| straight | 固定起终点、初始yaw和直线路径；比较沿线偏差与终点纵/横/yaw误差，确认没有错误跳变暂停 |
| turn | 固定转弯位置、角度、起终点和速度；确认连续转向时正常修正不会误暂停，终点yaw与目标一致 |
| lateral | 固定初始朝向和横移起终点；把body误差、vy审计和独立观察的横向方向对应起来，不用同一定位输出证明方向/精度 |
| avoidance | 固定原始目标/路线、障碍位置和场景时序；保留点云、costmap及障碍事件，确认暂时偏离后重回参考路径或明确安全失败；原始map目标不被替换 |

动态障碍“绕过”必须有可对应的障碍记录、轨迹和恢复路径。仅看到costmap变化、事件文本或定位输出收敛，不能证明实体净空、制动或机器狗避障成功。持续堵塞导致abort时保留失败，不自动重发Goal。

## LOST、位姿跳变与显式重接

录制器不会制造失定位、遮挡传感器或修改运动状态。若试验实际出现DEGRADED/LOST/RELOCALIZING或导航姿态跳变，记录发生时刻、之前是否已有真实非零运动、guarded零速、Action取消/结束、Nav2暂停及操作者干预。定位恢复本身不能恢复旧授权或旧Goal。

RouteFollower的默认连续性边界为`0.60m+0.80m/s×dt`及`0.20rad+0.75rad/s×dt`，source倒退/间隔过长也取消。取消时`remaining_m=None`，清零诊断progress不代表到达。操作者查明原因、确认LOCALIZED/新鲜TF并按既有流程重新授权后，才能启动新任务；本工具不执行授权。

新任务默认`rejoin_segment_index=-1`仍从A附近开始。中途重接必须由操作者明确指定**原始点i→i+1段**，不是加密后的索引，例如65点文件的`20`表示原点20→21：

```bash
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$trial_route_file" -p execute:=true \
  -p rejoin_segment_index:=20
```

这是操作者在现有导航流程内显式启动的新导航任务，**不是采集命令**。当前位姿距选段须≤`start_tolerance_m`，连接和剩余锚点仍必须通过global costmap中心线及Nav2 footprint检查；原始65点/raw终点不改。默认不自动选最近分支、不自动续跑旧Goal。仅预览仍显示原完整路线，执行就绪后才生成当前位姿的重接路径。

只有bag包含“故障前已运动、故障状态、零速输出、取消/暂停、恢复后仍未运动、新授权与新Goal”的完整时序，才可支持对应的控制链停止/恢复结论。当前评估`stop_latency_verified=false`，不能把消息序列先后顺序换成真实停车延迟。

## 离线评估和结果解释

传入实际SQLite bag目录（试验根下的`bag/`），或单个`.db3`；使用新评估目录保留此前结果：

```bash
./scripts/record_a2w_trial.sh evaluate "$trial_output_root/straight_01/bag" \
  --output "$trial_output_root/straight_01_eval" \
  --route-file "$trial_output_root/straight_01/inputs/route.yaml"
```

`summary.json`/`summary.txt`给出完整性、匹配阶段耗时、状态、TF一致性、估计路线/终点误差；`registration.json`保留每轮注册，`match_stages.csv`保留阶段数据，`events.json`保留事件。`trial.json`保存note、配置/地图/路线hash、录制命令、domain及退出状态；原始bag是主要证据。

命令退出0只表示评估过程执行成功，应实际查看`capture_complete`、`navigation_data_complete`、decode错误和缺失项。`true_endpoint_error_m=null`及`accuracy_validation=independent_ground_truth_required`是当前工具的明确边界：消息中的终点/路径误差和TF代数残差都是系统内部估计，不能写成真实物理精度。
