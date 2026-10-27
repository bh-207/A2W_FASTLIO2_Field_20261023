# 195303真实数据：关键Bag片段与同点地图预览

这是公开的**精选数据样例**，不是50.90 GiB完整原始Bag或完整Map Bundle的备份。完整数据继续在本机保留。

对应代码：`20568dbaeb078f81ea1acef3248199cbfc593620`。算法为FAST-LIO2、实际关键帧管理、保守回环、GTSAM/iSAM2。
完整运行B_003读取355087条IMU、17778点云，17765次POST、1841KF、3次LoopCommitted；原生完整地图15452713点。
完整建图通过，几何结果有改善和退步，尚未认证绝对高程、整图相对精度、实时性能、硬件或导航；候选自身重复未运行。

## 真实Bag片段

均从 `factory_live_20261024_195303_YOI7fO` 只读提取，只含原始 `/lidar_points` 和 `/lidar_imu` 两个话题。
按**录包记录时间**相对原metadata起点选择，区间左闭右开；header时间、原始CDR载荷、点字段及逐点时间原字节保留。
不同窗口的消息到达批次可能不同，数量不能按标称频率反推。没有补造数据，没有使用新机器时间戳。

| 文件 | 原Bag相对记录时间 | IMU / 点云 | 用途 |
|---|---|---:|---|
| `bags/initialization.tar.gz` | 0.0-1.6秒 | 329 / 16 | 初始化长度的原始输入检查 |
| `bags/mid_route.tar.gz` | 800.0-800.25秒 | 71 / 3 | 固定中段输入格式/时间样例 |
| `bags/late_return.tar.gz` | 1650.0-1650.25秒 | 70 / 3 | 晚期返程时段样例，物理地点未独立认证 |

每个归档解压后包含MCAP及真实生成的 `metadata.yaml`，可用ROS2 Humble查看。
所有记录已回读，顺序、记录纳秒、载荷长度和SHA256与原始提取记录完全一致；完整证据见各 `*_identity.json`。
这些短片段不能复现639/830/1777秒完整地图，后两段也不含独立初始化历史。不得把它们拼接成连续运动数据或称为完整算法验收。

## 地图与轨迹

- `maps/raw_same_identity_preview.pcd.gz`：camera_init系。
- `maps/optimized_same_identity_preview.pcd.gz`：map系。
- 两张PCD均为154764个XYZ点，来自既有冻结同body点身份的整条路线显示集；没有重新配准、调平、裁Z或根据结果选点。
- 这是**覆盖全路线的稀疏显示预览**，不是原生15452713点PointXYZI地图。仅保留XYZ字段，float32序列化，最大舍入误差记录在provenance。
- 不将抽样PCD或此目录当作可加载的完整Map Bundle，不直接用于Localization/Nav2。完整body云、描述子、配置与原生PCD留在本机。
- 逐位置高程/坡度评估表和详细轨迹CSV不包含在本次公开数据中，相关明细及PDF保留本机。

固定外参R=I、T=[-0.00862,0.00914,0.03916]m、extrinsic_est_en=false：用户指定，非认证标定。
联合初始化、P/R/J、新重力软约束、增强回环关闭。没有对这些样例运行新建图或宣称精度改善。

## 校验与使用

```bash
cd data_samples/195303_20261027
sha256sum -c SHA256SUMS
# 输出目录必须不存在；脚本只解压和验证，不连接设备、不发布ROS话题。
python3 restore_samples.py --output /tmp/a2w_public_samples_20261027
source /opt/ros/humble/setup.bash
ros2 bag info /tmp/a2w_public_samples_20261027/bags/initialization
# 地图已解压为标准二进制XYZ PCD，可离线查看；不能用它替代完整地图验收。
```

完整记录与筛选规则在 `DATA_PROVENANCE.json`。最新中文PDF已在本机生成，未纳入本次公开提交。
生产环境、164驱动、原始Bag和已有地图均未修改。GitHub上的这些精选数据不替代独立备份。
