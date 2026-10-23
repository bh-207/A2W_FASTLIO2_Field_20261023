# A2W 建图、路径采点与导航操作手册

适用版本：`feature/a2w-navigation`，2026-10-17。

运行方式：PC 运行 FAST-LIO、Mapping / Localization 和 Nav2；164 运行 JT128 雷达驱动。
本手册不是 164 全板载部署命令。

流程：建图 → 保存 Map Bundle → 准备二维地图 → 手控采点 → 返回 A → 执行导航。

## 一、操作前须知

- 本文统一使用新地图名 `factory_map_new`。换地图时，各终端的地图名必须一致。
- 已有地图、路线不要混用；新地图的坐标系不一定与旧地图一致。
- Mapping、独立 Localization、完整 Navigation 三种启动方式不要同时运行。
- 完整 Navigation 已包含 FAST-LIO 和 Localization，不需要额外启动。
- 机器人运动时必须有人现场看护，并握住遥控急停。
- 当前最高前进速度仍为 0.60 m/s；换地图或首次运行，先验证短距离单点导航。
- 当前 Collision Monitor 默认旁路，MPPI 与 Local Costmap 仍承担局部避障，不能视为完整的独立停车保护。
- 系统时间、LiDAR 时间和 TF 时间必须一致。出现百秒级偏差或 TF 时间错误时，不要启动运动，也不要靠放宽超时掩盖。

## 二、首次编译

首次使用整理后的源码，或更新代码后，在 PC 执行一次：

```bash
cd ~/fast_lio_ros2_ws
conda deactivate 2>/dev/null || true
./build.sh
```

已有代码未变化时，不必每次运行都重新编译。

## 三、每个 PC 新终端的公共环境

**后续每个 PC 新终端，都先执行这一整段。**

```bash
cd ~/fast_lio_ros2_ws
conda deactivate 2>/dev/null || true
source ./env.sh

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export A2W_NETWORK_INTERFACE=enp49s0
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"

MAP_NAME=factory_map_new
```

`enp49s0` 是当前 PC 的机器人有线网卡。更换电脑或网卡后，应使用实际接口，不能原样照搬。
运行入口会生成运行时 DDS 配置；不要手工使用其他工作区遗留的 DDS 文件。

## 四、启动 164 上的 JT128 驱动

在已经登录 164 的终端执行：

```bash
source ~/unitree_ros2/setup.sh
cd ~/hesai_ws
source install/setup.bash
ros2 launch hesai_ros_driver start.py
```

如果驱动已经运行，不要重复启动。让这个终端保持运行。

## 五、开始建图

### 5.1 PC 终端一：检查输入并启动 Mapping

先加载第三节公共环境，再执行：

```bash
./scripts/check_lidar_topics.sh
./scripts/run_a2w_mapping.sh rviz:=true
```

使用遥控器移动机器人建图。尽量平稳行走，覆盖需要导航的区域，最后返回起点附近。

终端会在重要状态出现中文回环提示，包括候选、验证接受/拒绝、图优化及队列处理状态。
一次回环成功不等于整个地图几何已经验收；没有回环成功也不能直接认定保存失败。

### 5.2 PC 终端二：保存 Map Bundle

机器人停稳，保持终端一继续运行。新终端加载第三节公共环境，然后执行：

```bash
ros2 service call /mapping/save_map_bundle \
  a2w_fastlio_msgs/srv/SaveMapBundle \
  "{output_path: '$MAP_NAME'}"
```

`output_path` 使用地图名这个相对路径，不要填工作区绝对路径。

保存过程会冻结接收范围、等待待处理关键帧完成、执行最终优化，再构建和写入总图。
出现 `Draining mapping queue` 时继续等待，**不要 Ctrl+C 或强制结束进程**。

只有保存服务返回 `success: true` 才算本次保存成功。随后检查：

```bash
ros2 run a2w_fastlio_map map_bundle_inspect \
  "$PWD/maps/$MAP_NAME"
```

主要保存位置：

```text
maps/factory_map_new/
  global_map.pcd       完整地图点云
  metadata.yaml       Bundle 元数据
  manifest.sha256     文件校验清单
  ...                 关键帧、位姿、描述子等 Bundle 内容
```

`/mapping/optimized_map_preview` 只是 RViz 预览，不是最终完整总图。
已有同名 Bundle 时，请换新地图名，不要为重复保存删除原地图。

### 5.3 停止建图

保存和检查成功后，回终端一按 `Ctrl+C`。
之后才能启动独立 Localization 或 Navigation。

## 六、准备导航二维地图

### 6.1 使用恢复的一键工具

**只在独立确认地图 +Z 可以作为竖直方向后，才能使用 `--confirm-map-z`。**

加载公共环境，然后执行：

```bash
./scripts/generate_2d_map.sh \
  --map-bundle "maps/$MAP_NAME" \
  --config config/map_2d.yaml \
  --confirm-map-z \
  --output "maps/$MAP_NAME/navigation"
```

输出：

```text
maps/factory_map_new/navigation/
  nav_map.pgm
  nav_map.yaml
  conversion.yaml
```

默认不覆盖既有输出。参数及输入格式见 [二维地图工具说明](2D_MAP.md)。

### 6.2 坐标与倾斜限制

- 不能仅凭地图 frame 名为 `map` 就认定 Z 轴等于重力竖直。
- 有独立确认的上方向时，可使用工具的 `--up` 参数；改变投影坐标后，不能直接把输出当成原 map 导航。
- 不能只旋转或平移 PGM，而让定位、目标点和路线仍使用原坐标。
- 单一二维投影不能修复三维地图的局部倾斜、变形或分层。
- 当前 `factory_map_1016` 的局部倾斜及竖直参考仍待核实，本手册不授权直接确认它的 Z 轴。
- 也可以使用自己的二维生成程序，但必须保证输出与 Localization 的地图坐标一致。

下面命令按一键工具生成的 `navigation/nav_map.yaml` 编写。若使用其他文件名，应同步修改导航命令。

## 七、采集 A→B 路径点

### 7.1 PC 终端一：启动手控采点模式

机器人先放在建图起点附近，以便完成默认起点区域初始化。
确认 Mapping 已停止，加载公共环境，然后执行：

```bash
./scripts/run_a2w_navigation.sh "maps/$MAP_NAME" \
  "maps/$MAP_NAME/navigation/nav_map.yaml" \
  start_motion_bridge:=false \
  auto_enable_motion:=false
```

这个模式不启动运动桥，使用遥控器移动机器人，避免两个控制源竞争。
程序默认打开 RViz。

### 7.2 PC 终端二：确认定位并启动采点器

加载公共环境，然后执行：

```bash
ros2 topic echo /localization/status --once --field state_label
```

显示 `LOCALIZED` 后启动采点器：

```bash
ros2 run a2w_waypoint_tools waypoint_recorder --ros-args \
  -p output_prefix:="$PWD/maps/$MAP_NAME/routes/ab"
```

操作方式：

1. 到 A 点，按 `S`、`s` 或空格，记录起点。
2. 遥控沿预期路线移动，在转弯和关键位置记录点。
3. 到 B 点，调整好最终朝向，再记录最后一点。
4. 按 `Ctrl+C` 保存并退出。

保存位置：

```text
maps/factory_map_new/routes/ab.yaml
maps/factory_map_new/routes/ab.csv
```

记录的是 `map→base_link` 位姿。至少需要两个有效点；定位或 TF 不可用时不会记录。
**同名前缀会覆盖既有路线**，重复采集请将末尾 `ab` 换成其他名称。

## 八、执行正式导航

### 8.1 返回 A 并切换到完整导航

先用遥控器返回 A 点附近，距采样起点不超过 0.5 m。
然后在采点模式的启动终端按 `Ctrl+C`，不要让两套导航并行运行。

加载公共环境，启动完整导航：

```bash
./scripts/run_a2w_navigation.sh "maps/$MAP_NAME" \
  "maps/$MAP_NAME/navigation/nav_map.yaml"
```

这个入口已经启动 FAST-LIO、Localization、Nav2、RViz 和运动桥。
默认自动准备运动门，但不会自动发送目标；不要另开 FAST-LIO 或 Localization。

### 8.2 检查定位和导航就绪状态

另一个终端加载公共环境，然后执行：

```bash
ros2 topic echo /localization/status --once --field state_label
ros2 topic echo /navigation/nav2_ready --once
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

应满足：

- Localization 为 `LOCALIZED`。
- Nav2 就绪 Topic 为 `data: true`。
- TF 连续更新，没有持续的时间外推错误。
- RViz 中地图、机器人、局部障碍和代价层位置合理。
- 周围安全，遥控急停在手。

### 8.3 执行采集的 A→B 路线

```bash
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/$MAP_NAME/routes/ab.yaml" \
  -p execute:=true
```

中间采样点不停靠，只在 B 点结束。程序检查定位、两道运动门、TF、起点距离和路径通行性。
不会自动导航到 A；不满足检查时，不要通过关闭检查强行执行。

也可先只预览，不发送运动任务：

```bash
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/$MAP_NAME/routes/ab.yaml"
```

预览确认后 `Ctrl+C` 退出，再执行带 `execute:=true` 的命令。

### 8.4 RViz 单点导航

也可以在 RViz 中使用 **Nav2 Goal** 发送目标。
先测试前方约 1 m 的短距离目标，确认方向、避障和停止正常，再执行完整路线。
单点目标和路线跟踪不要同时下发。

### 8.5 当前速度和路线能力边界

| 项目 | 当前上限 |
|---|---:|
| 前进速度 | 0.60 m/s |
| 横移速度 | 0.15 m/s |
| 角速度 | 0.50 rad/s |

这些是速度上限，不是恒定巡航速度。

路线通过现有 FollowPath、MPPI 和 Local Costmap 跟踪，避障时可以偏离参考线，之后继续跟踪后续路径。
直接 FollowPath 没有 NavigateToPose BT 的完整全局重规划/Recovery，也不会失败后自动续跑。
持续受阻、定位失效或 TF/安全门异常时，任务可能取消。沿线剩余距离为零不等于已到达，仍须看 Action 结果和终点误差。

## 九、立即停止与退出

新终端执行：

```bash
cd ~/fast_lio_ros2_ws
./scripts/a2w_stop_now.sh
```

脚本会依次关闭运动桥、关闭导航运动门，并取消 NavigateToPose / FollowPath 任务。
随后在导航启动终端按 `Ctrl+C` 退出程序。

**软件停止不是实体急停。出现异常运动或软件停止无响应时，立即使用遥控/实体急停。**
停止后的任务不会自动恢复，重新执行前须重新确认定位、导航和运动门就绪。

## 十、文件位置速查

| 内容 | 本文示例位置 |
|---|---|
| Map Bundle | `maps/factory_map_new/` |
| 完整三维点云 | `maps/factory_map_new/global_map.pcd` |
| 二维图片 | `maps/factory_map_new/navigation/nav_map.pgm` |
| 二维地图 YAML | `maps/factory_map_new/navigation/nav_map.yaml` |
| 二维转换记录 | `maps/factory_map_new/navigation/conversion.yaml` |
| 路线 YAML | `maps/factory_map_new/routes/ab.yaml` |
| 路线 CSV | `maps/factory_map_new/routes/ab.csv` |
| Nav2 参数 | `src/a2w_navigation/config/nav2_params.yaml` |
| 二维转换参数 | `config/map_2d.yaml` |
| 立即停止入口 | `scripts/a2w_stop_now.sh` |

更多状态、接口及能力限制见 [完整运行手册](RUNTIME_USER_GUIDE.md)。
