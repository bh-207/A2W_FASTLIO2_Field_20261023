> **2026-10-24最新离线源码：** [完整输入重建、Z来源与联合初始化](experiments/field_frontend_z_init_20261024/PUBLIC_EXPORT_ZH.md)。完整同输入重建已完成；真实联合初始化候选被拒绝并回退，几何改善未通过，新功能默认关闭。

> **最新独立修复入口：** [828KF后端与退出修复](experiments/field_backend_repair_20261023/README.md)。工程短程回归通过；候选整体几何及全长原速完整性仍FAIL，未切生产。

> **2026-10-23源码快照：** 请先阅读 [导出内容、已知问题及构建边界](EXPORT_NOTES_ZH.md)。现场入口的停机异常与后端接缝退步尚未修复；本仓库不是导航验收通过版本。

# A2W Fast-LIO2 Mapping, Localization and Navigation

Unitree A2-W Pro + Hesai JT128 的 ROS 2 Humble 工作区。保留现有 `FAST_LIO_Hesai`
前端，组合共享 Scan Context / Quatro / Nano-GICP、GTSAM 回环后端、Map Bundle、
异步定位与全局重定位、Nav2 Omni 导航、路径采点和连续 FollowPath。

[完整运行与使用手册](docs/RUNTIME_USER_GUIDE.md) · [二维地图工具](docs/2D_MAP.md) ·
[依赖和复现说明](docs/BUILD_AND_DEPENDENCIES.md)

## v4 受控重新建图

[按终端划分的完整采图命令](docs/CONTROLLED_MAPPING_V4.txt)。新入口先录最小原始 bag，
再启动独立 v4 严格接口/初始化质量门和保守后端；保存整程真实激光更新状态、重力和偏置。
不启用尚未通过几何验收的增强回环，不覆盖主 install，不启动本机雷达驱动或机器人运动。

仅在操作者明确授权沿用 `R=I、T=0、offset=0、extrinsic_est_en=true` 的未验证实验初值时，
使用[独立实验入口与静止复测命令](docs/CONTROLLED_MAPPING_V4_EXPERIMENT.md)。授权默认关闭，
会话明确标记物理标定未完成，保留输入、时间和初始化检查；生产默认不变。

```bash
./setup.sh --install-system  # 已有依赖可先用 ./setup.sh --check
./scripts/build_controlled_mapping.sh
./scripts/mapping_capture.sh --help
```

首次需提供164修正后驱动的明确单位/frame/时间和已标定配置来源；未知契约允许先录原始数据，
但拒绝正式启动。不要把下面保留的旧 `run_a2w_mapping.sh` 当成已启用v4的入口。

## 支持环境

- Ubuntu 22.04，ROS 2 Humble，GCC 11，后端 C++17，Python 3.10。
- x86_64；aarch64/ARM64 使用同一源码和依赖脚本，本轮不代表 ARM 或 JT128 实机验收。
- PCL 1.12、Eigen 3.4、GTSAM 4.1.1、OpenMP、TBB、yaml-cpp、OpenSSL。
- 使用系统 ROS Python；不要用 Conda Python 编译 ROS 消息。
- 默认 Release、每个包最多 2 个编译任务，顺序构建，适合内存有限的开发机。

## 下载、安装、编译

```bash
git clone --branch feature/a2w-navigation --recurse-submodules \
  https://github.com/AaronYanC/A2W_FastLio2.git
cd A2W_FastLio2

# 全新 Ubuntu：安装 ROS、系统依赖，初始化固定版本子模块和缺失的 GTSAM。
./setup.sh --install-system
# 已有 ROS 和 rosdep 时也可使用 ./setup.sh。
./setup.sh --check
./build.sh
source ./env.sh
```

脚本从自身位置确定项目根目录，不绑定用户名、主目录或仓库克隆位置。
GTSAM 若没有匹配系统版本，则按固定提交从官方源码构建到本仓库 `.deps/install`，
不覆盖系统库。Unitree `unitree_api` / `unitree_go` 消息源码随仓库提供，无需复制某台
电脑的 `unitree_ros2/install`，也不替换系统 DDS 或机器人驱动。

```bash
# 可选：独立构建目录，不覆盖原来的安装。
A2W_BUILD_BASE=build_check A2W_INSTALL_BASE=install_check ./build.sh
A2W_INSTALL_BASE=install_check source ./env.sh

# 基础回归；完整 C++/ROS 测试见依赖文档。
bash tests/run_tests.sh
```

## 主要入口

机器人端 JT128 驱动须已运行。网络接口通过路由自动选择，也可配置
`A2W_NETWORK_INTERFACE`、`A2W_PC_IP`、`A2W_DDS_PEER`；默认 peer 为
`192.168.123.164`。不自动设置系统时钟或修改机器人网络。

```bash
./scripts/check_lidar_topics.sh
./scripts/run_fastlio_jt128_pc.sh rviz:=true
./scripts/run_a2w_mapping.sh rviz:=true

# 另一个已 source env.sh 的终端保存；等待服务返回成功后再停止 Mapping。
ros2 service call /mapping/save_map_bundle a2w_fastlio_msgs/srv/SaveMapBundle \
  '{output_path: factory_map}'

./scripts/run_a2w_localization.sh maps/factory_map
```

Mapping 与 Localization 互斥，只能有一个 `map→camera_init` 发布者。
现有 TF 为 `map → camera_init → body → base_link`，前端仍发布 `camera_init→body`；
本次整理不改变已配置的固定外参或 FAST-LIO 算法。
保存服务冻结接收范围、排空后端队列、优化并构建最终总图；不要在排空期间强制终止。

### 一键二维地图

已恢复原 Map Bundle 导出入口，并整合此前开发的并行 PMF 地面分割。
支持 Map Bundle 或独立 PCD，输出 `nav_map.pgm`、`nav_map.yaml`、`conversion.yaml`。

```bash
# 仅当独立确认地图 +Z 可作为竖直方向后，显式确认；保留 map 的 XY/原点/方向。
./scripts/generate_2d_map.sh --map-bundle maps/factory_map \
  --config config/map_2d.yaml --confirm-map-z --output maps/factory_map/navigation
```

未知重力方向时程序拒绝猜测。也可用 `--up ux uy uz` 指定已经核实的源坐标系上方向；
非单位投影会生成独立投影 frame，并记录正反变换，不能直接冒充原 `map` 用于导航。
PMF 使用局部地面相对高度，不会将三维地图压平；对局部变形地图，单一投影不能修复几何。
参数和坐标关系见 [二维地图说明](docs/2D_MAP.md)。

### 导航、采点与轨迹

```bash
# 初次检查可保持运动关闭，确认定位、TF、costmap、障碍与路径。
./scripts/run_a2w_navigation.sh maps/factory_map \
  maps/factory_map/navigation/nav_map.yaml auto_enable_motion:=false

# 默认使用现有运动策略；只在现场准备好实际运动时运行。
./scripts/run_a2w_navigation.sh maps/factory_map maps/factory_map/navigation/nav_map.yaml

source ./env.sh
export CYCLONEDDS_URI="file://$PWD/log/fast_lio_runtime/cyclonedds.xml"
ros2 run a2w_waypoint_tools waypoint_recorder --ros-args \
  -p output_prefix:="$PWD/maps/factory_map/routes/ab"
# S/s 或空格采点，Ctrl+C 保存 YAML/CSV。
ros2 run a2w_waypoint_tools route_follower --ros-args \
  -p route_file:="$PWD/maps/factory_map/routes/ab.yaml" -p execute:=true
./scripts/a2w_stop_now.sh
```

导航启动脚本沿用当前自动准备运动门的行为；有新鲜 LOCALIZED、TF 和 Nav2 就绪条件才放行。
不自动发送目标，定位或安全条件变化会取消任务；软件停止命令不等于实体急停。
当前默认 MPPI Omni，`vx_max=0.60 m/s`、`vy_max=0.15 m/s`、`wz_max=0.50 rad/s`，
不在本轮重新调参。采点时建议 `start_motion_bridge:=false auto_enable_motion:=false`，
使用遥控器移动，避免两个控制源竞争。

## 目录与依赖方向

```text
src/
  FAST_LIO_Hesai/             # 固定提交的唯一前端（含 ikd-Tree 子模块）
  a2w_fastlio_common/         # 算法接口与共享 SC / Quatro / Nano-GICP
  a2w_fastlio_msgs/           # ROS 接口
  a2w_fastlio_map/            # Bundle 存取、校验
  a2w_fastlio_mapping/        # KeyFrame、Loop、GTSAM、队列排空与保存
  a2w_fastlio_localization/   # 传播、后台匹配、状态监测、全局重定位
  a2w_nav_map/                # 一键二维栅格与 PGM/YAML
  a2w_pcd_filters/            # 复用的并行 PMF / 地面相对障碍滤波
  a2w_navigation/            # Nav2、Guard、就绪检查、RViz、AprilTag 适配
  a2w_motion_bridge/         # Twist → Unitree Sport 请求、安全限幅
  a2w_waypoint_tools/        # 采点、锚点保留、连续路线跟踪
  X2-0807urdf/               # a2w_description 与必要 STL
  unitree_api/, unitree_go/  # 官方消息定义，含固定版本及许可来源
third_party/                # 固定算法核心、许可和子模块
config/, scripts/, tests/   # 网络模板、构建/运行入口、回归
docs/                       # 架构、依赖、运行及坐标说明
```

`common` 不依赖上层；`map→common`；Mapping/Localization 依赖 `common+map+msgs`；
bringup 负责组合启动。Nav2 仅消费定位状态，不决定定位算法。
完整地图保存在 Bundle，DDS 只发布降采样 preview。

日志、build/install、地图和录包、实验输出、备份、CAD STEP 与个人相机图像均排除上传；
URDF/STL、配置、接口源码、算法依赖和约 200 KB 的必要配准回归样本保留。
没有附带厂区地图或 Teach 目标，需由使用者提供。AprilTag IPPE 核心已随包保留，
可选检测器依赖见 `requirements-apriltag.txt`，Teach 路径通过 `target_path` 配置。

本轮离线检查不能替代不同设备、地形、标定和时钟条件下的实机验收。
各算法的许可证分别见 [第三方说明](third_party/THIRD_PARTY_NOTICES.md)，
特别注意 Scan Context 等上游含非商业许可条件。
