# 环境、依赖与可复现构建

支持 Ubuntu 22.04 / ROS 2 Humble / Python 3.10 / GCC 11。
后端使用 C++17，FAST-LIO 保持上游构建配置。ARM64 使用同一依赖流程，
本轮 x86_64 构建通过不等于 ARM64 或机器人实时性验收。

## 全新主机

```bash
git clone --branch feature/a2w-navigation --recurse-submodules \
  https://github.com/AaronYanC/A2W_FastLio2.git
cd A2W_FastLio2
./setup.sh --install-system
./setup.sh --check
./build.sh
source ./env.sh
```

安装脚本需要网络和 sudo，通过 Ubuntu/ROS 官方源安装 ROS、rosdep、colcon、编译器、
Eigen、PCL、OpenMP、TBB、yaml-cpp、OpenSSL、Boost 与 Python NumPy/PyYAML/pytest。
rosdep 根据所有 package.xml 补齐 Nav2、TF、点云转激光、RViz 等依赖。
不自动修改机器人网络、时间同步、驱动或当前用户的 shell 启动文件。

已有 ROS 的主机可以执行 `./setup.sh`；只检查、不安装用 `./setup.sh --check`。
当前机的系统检查成功不能证明全新系统的 apt 安装已经实际执行；网络、源镜像和 sudo
仍是安装前提，检查/构建失败时应按实际错误处理。

## 依赖来源

| 依赖 | 来源和固定版本 |
|---|---|
| FAST_LIO_Hesai | 子模块 `16e97cdcc4260b9ba6241518a19ad8384224ba48` |
| ikd-Tree | 由 FAST-LIO 递归子模块固定 |
| SC / Quatro / Nano-GICP | `third_party/*/UPSTREAM.md` 记录固定算法核心 |
| TEASER++ / PMC | 固定 Git 子模块，由公共算法库编译需要的目标 |
| GTSAM | 4.1.1；系统匹配版本优先，缺少时源码提交 `69a3a75195b65356d6e56669e5199d325c7962c9` |
| Unitree API / Go messages | 官方提交 `3ff13ea08ec619496c2651fd21b172f7958dd5a5` 的接口源码，随仓库提供 |
| PMF filters | 复用此前 pcd2pgm 并行版本，来源和 BSD 声明在 `a2w_pcd_filters` |
| ROS / Nav2 | Humble apt 包；由 rosdep 声明，不使用旧工作区已编译文件 |

GTSAM 源码及构建缓存位于 `.deps/`，安装在 `.deps/install`，不写入系统库。
setup/build/env 脚本会加入对应 CMake 和动态库搜索路径。不会从本机复制 `.so`。
GTSAM 完整依赖构建可能较慢；`A2W_BUILD_JOBS` 控制并发，默认 2。
GTSAM 找不到时运行 `./scripts/install_gtsam.sh`，查看 `.deps/gtsam_probe.log`。
也可使用 `--from-source` 验证固定源码构建。此依赖关闭 GTSAM 内部 TBB 和 native CPU
优化以适配 Ubuntu 22.04 与不同 CPU；公共配准库仍使用自己的 OpenMP/TBB 配置。
许可证分别保留，见 `third_party/THIRD_PARTY_NOTICES.md` 和接口包 LICENSE。

可选 AprilTag 检测器不影响基本定位和 Nav2 构建：

```bash
sudo apt-get install python3-venv python3-opencv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements-apriltag.txt
```

已验证 IPPE 计算核心和相机内参随包保留；每台相机仍需匹配自己的标定和 Teach 数据。
`target_path` 必须显式指定，不附带某台机器主目录里的 Teach 文件或 SDK 控制脚本。

## 可配置构建与运行环境

| 变量 | 用途 |
|---|---|
| `A2W_ROS_SETUP` | ROS setup，默认 `/opt/ros/humble/setup.bash` |
| `A2W_BUILD_BASE` / `A2W_INSTALL_BASE` / `A2W_LOG_BASE` | 默认 build / install / log；可使用独立目录 |
| `A2W_PYTHON_EXECUTABLE` | ROS 兼容解释器，默认 `/usr/bin/python3` |
| `A2W_DEPS_ROOT` / `A2W_DEPS_PREFIX` | 源码依赖缓存和本地安装前缀 |
| `A2W_INSTALL_SETUP` | 运行脚本使用的工作区 setup |
| `A2W_NETWORK_INTERFACE` / `A2W_PC_IP` / `A2W_DDS_PEER` | DDS 网卡、源地址、机器人地址 |
| `A2W_UNITREE_ROS_SETUP` | 可选旧 Unitree overlay；一般无需设置 |
| `A2W_PCD_FILTER_BIN` | 可选 PMF CLI 路径；默认从 ament 索引查找 |

编译默认 Release；显式 CMake 选项可附在 `./build.sh` 后，例如 `-DBUILD_TESTING=OFF`。
正常构建会检查环境，`A2W_SKIP_ENV_CHECK=1` 仅用于已独立验证环境或测试工具替身。
当前构建默认仅发现 src，第三方求解器由 common 明确编译，不作为另一个 ROS 前端启动。
不要同时 source 历史 `install_tf_gate` 或旧 Unitree middleware 来掩盖依赖缺失。

```bash
A2W_BUILD_BASE=build_check A2W_INSTALL_BASE=install_check ./build.sh
export A2W_INSTALL_BASE=install_check
source ./env.sh
```

## 验证

```bash
bash tests/run_tests.sh
PYTHONPATH=src/a2w_nav_map /usr/bin/python3 -m unittest discover -s src/a2w_nav_map/test -v
source ./env.sh
colcon test --base-paths src --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

ROS 集成测试应在独立 ROS_DOMAIN_ID 和仅回环 DDS 配置下执行，避免连接现场运行图。
脚本的静态/fixture 测试不代替实际编译；构建成功也不代替实机验收。
真实配准回归样本仅约 200 KB，按 manifest 校验后随源码保留；大型地图、录包、缓存
和历史实验结果不上传。原地图由使用者通过 Map Bundle 另行提供。

### 本轮实际验证（2026-10-17）

- Ubuntu 22.04 x86_64 / GCC 11 / Humble：依赖检查通过。
- 将待提交源码及固定子模块导出到含空格路径，清空继承环境、不使用旧工作区 overlay：
  15 个包 Release 构建通过。默认编译入口也通过 PATH 中存在不兼容 colcon 的检查。
- 固定 GTSAM 4.1.1 官方提交完整源码构建、安装、Mapping 重新链接通过；
  确认运行时加载这份 GTSAM 和 Metis，再进行全包回归。
- 仅回环 DDS 下 colcon 汇总：509 tests，0 errors，0 failures，0 skipped。
  已补齐两个 Python 包的 pytest 注册，避免现有测试被当作零项跳过。
  这是 colcon 汇总口径，包含 CTest 外层与内部测试结果，不表示 509 个独立场景。
- 二维转换 11 项回归及恢复的 PMF 滤波测试通过；实际运行一键 shell 入口，
  已知水平地面/箱体导出 121×121 栅格、25 个障碍格，输入哈希保持不变。
- 原 `factory_map_1016` Bundle 的 manifest 校验通过，未生成其修复图或导航图。

没有在全新空白操作系统执行 apt 安装，也未完成 ARM64、相机或 JT128 实机验收。
本轮二维地图验证使用已知参考的合成数据，不代表厂区地图几何或导航安全已经验收。
