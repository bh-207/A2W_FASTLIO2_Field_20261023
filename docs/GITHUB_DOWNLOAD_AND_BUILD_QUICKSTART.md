# A2W GitHub 代码下载与一键构建

## 一、仓库与分支

- GitHub 仓库：[AaronYanC/A2W_FastLio2](https://github.com/AaronYanC/A2W_FastLio2)
- 最新功能分支：`feature/a2w-navigation`
- [分支代码入口](https://github.com/AaronYanC/A2W_FastLio2/tree/feature/a2w-navigation)

**请选择 `feature/a2w-navigation`，不要只下载默认的 `main` 分支。**

## 二、一键下载、安装依赖并编译

适用环境：**Ubuntu 22.04 + ROS 2 Humble**。

在新电脑终端复制执行：

```bash
cd ~ &&
sudo apt-get update &&
sudo apt-get install -y git &&
git clone --branch feature/a2w-navigation --recurse-submodules \
  https://github.com/AaronYanC/A2W_FastLio2.git &&
cd A2W_FastLio2 &&
./setup.sh --install-system &&
./setup.sh --check &&
./build.sh &&
source ./env.sh
```

代码下载到：

```text
~/A2W_FastLio2
```

说明：

- 不会覆盖原来的 `~/fast_lio_ros2_ws`。
- 如果 `~/A2W_FastLio2` 已存在且非空，克隆会停止；不要删除已有目录，按下一节更新已有克隆，或另选新目录。
- 安装需要联网和输入 sudo 密码，会安装系统依赖；`--check` 只检查环境。
- 构建默认使用 Release。首次需要源码编译 GTSAM 时可能较慢。
- 使用系统 ROS Python，不要使用 Conda Python 编译 ROS 消息。
- **请使用 Git 克隆，不建议下载 ZIP**：项目包含必要子模块，需要 `--recurse-submodules`。
- 全新空白系统的 apt 安装流程及 ARM64 整机运行仍需在目标设备实际验证；已有离线构建结果不能替代实机验收。

## 三、后续更新代码并重新编译

在已有克隆中执行：

```bash
cd ~/A2W_FastLio2 &&
git pull --ff-only &&
git submodule update --init --recursive &&
./build.sh &&
source ./env.sh
```

若有本地修改或 Git 报告冲突，请先保留修改并检查原因，不要使用强制重置覆盖自己的代码。

## 四、每次新开终端加载环境

```bash
cd ~/A2W_FastLio2
source ./env.sh
```

后续运行入口和参数见仓库 [README](../README.md) 与 [完整运行手册](RUNTIME_USER_GUIDE.md)。

## 五、地图与路线需要单独迁移

**GitHub 不包含厂区地图、采集路线和传感器录包。**

需要从原电脑单独复制对应的完整地图目录：

```text
maps/地图名称/
  global_map.pcd
  metadata.yaml
  manifest.sha256
  ...                  其余必要的 Map Bundle 文件
  navigation/          二维地图图片与 YAML
  routes/              采集的路线 YAML、CSV
```

不要只复制 `global_map.pcd` 就当作完整 Map Bundle。
二维地图、路线、目标点必须与该 Bundle 的地图坐标一致。
复制完成后，再按实际网卡、时间同步和机器人配置检查定位及导航，不要直接发送运动目标。
