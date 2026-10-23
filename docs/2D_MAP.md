# 一键二维地图

原有实现已找到并复用：`a2w_nav_map` 首次进入仓库的提交为 `357c99f`；
其原入口为 `ros2 run a2w_nav_map pcd_to_occupancy`。此前独立 `pcd2pgm` 工作区的
并行 PMF 地面分割和 K=12 邻域地面相对高度滤波已整合到 `a2w_pcd_filters`。
未复制原目录里的绝对路径配置、地图、ROS 图形界面或调试输出。

## 输入和输出

输入可以是 Map Bundle V1（校验 manifest/metadata 后读 global_map.pcd），
或独立 PCD。height 方法支持 ASCII/binary XYZ 或 XYZI；PMF 使用 PCL 读取器，
也支持 binary_compressed。PGM 为 P5，YAML 为 Nav2 标准 trinary 格式，图片相对路径。

输出目录包含：

- `nav_map.pgm`：障碍=0、已观测地面=254、未知=205，障碍优先。
- `nav_map.yaml`：分辨率、图片相对路径、原点和占用阈值。
- `conversion.yaml`：输入 SHA256、Bundle UUID、有效参数、frame、投影正反变换和格数统计。

默认只将观测地面单元设为 free，不把整个包围盒未知空间自动标白。
这是与旧独立 pcd2pgm 输出的区别；稀疏地面可能保留未知孔洞，导入 Nav2 前需检查通行区域。
输入文件只读；默认拒绝覆盖这三个输出，明确需要时加 `--overwrite`。

## 明确竖直参考

FAST-LIO 的 camera_init/map Z 不能仅凭名称或截图当成物理竖直。
只能使用独立核实的方向：采集时重力经正确变换得到的方向、可靠标定或现场测量参考。
从倾斜地图拟合的地面法向不是独立重力真值。

`config/map_2d.yaml` 默认未确认参考，直接运行会拒绝导出。
如果已确认地图 +Z 可用，使用下面一条命令，坐标 XY、尺度和朝向保持原 map 不变：

```bash
./scripts/generate_2d_map.sh --map-bundle maps/factory_map \
  --config config/map_2d.yaml --confirm-map-z --output maps/factory_map/navigation
```

如果有可靠上方向，使用三个数指定它在输入 frame 中的表达。下面 `[0,0,1]` 仅是
已确认 map Z 的等价示例，不是对任何厂区地图的标定建议：

```bash
./scripts/generate_2d_map.sh --pcd maps/factory_map/global_map.pcd \
  --config config/map_2d.yaml --up 0 0 1 --output maps/factory_map/navigation
```

非单位参考使输出 frame 变为 `map_projection`。以投影后的原 map X 为水平 X，
`Y=up×X`，输入/输出原点相同，`p_projection=R_projection_source*p_source`。
`conversion.yaml` 保存 `projection_T_source` 和它的逆 `source_T_projection`。
不能把这种 PGM 直接按原 map XY 使用；需要使用记录的变换统一定位、costmap、目标和路线。
现有导航启动器保持原 map 契约，本工具不自动改变导航 TF 或发布静态 TF。
Nav2 map YAML 本身不保存 3D roll/pitch，这就是额外保留变换文件的原因。

当地图局部姿态变形、分层或真实地坪有起伏时，一个全局旋转不能修复三维几何。
本轮不为 factory_map_1016 猜测方向，不生成或覆盖它的导航地图。

## 两种处理方式

`--method pmf` 复用 PMF 地面分割，随后可选 SOR，再按附近地面平均高度提取障碍。
保留真实坡度；不将点的 Z 压平。先执行 `./build.sh` 并 `source ./env.sh`。

`--method height` 使用固定投影高度区间，只有 NumPy/PyYAML 依赖，无需编译 PMF：

```bash
./scripts/generate_2d_map.sh --map-bundle maps/factory_map \
  --confirm-map-z --method height --resolution 0.05 \
  --ground-min-z -0.85 --ground-max-z -0.25 \
  --obstacle-min-z -0.20 --obstacle-max-z 1.50 \
  --output maps/factory_map/navigation
```

这些高度是示例/旧默认值，必须对应输入坐标的地面高度和机器人可碰撞高度。
大范围坡地优先检查 PMF 的适用性，而不是扩大固定高度范围掩盖错误。

| 配置／CLI | 默认 | 含义 |
|---|---:|---|
| `resolution` | 0.05 m | 地图单元尺寸 |
| `padding` | 1 m | 包围盒留边，保持未知 |
| `voxel_leaf` | 0.10 m | PMF 前降采样；0 禁用 |
| `threads` | 4 | 并行查询线程，限制到可用 CPU |
| `max_window_size` | 32 | 复用原 PMF 窗口规则 |
| `slope` / `initial_distance` / `max_distance` | 1 / 0.20 / 2.5 | PMF 阈值参数 |
| `mean_k` / `stddev_mul` | 20 / 1 | 非地面 SOR；mean_k=0 禁用 |
| `h_min` / `h_max` | 0.10 / 1.50 m | 障碍物相对局部地面高度 |
| `min_obstacle_points` | 配置文件 3，CLI 默认 1 | 单元障碍点数门槛 |

参数 YAML 使用下划线，CLI 使用连字符，例如 `--min-obstacle-points 3`；CLI 优先。
地图最多 1 亿格，超限应增大分辨率或先选择合理区域。
PMF 会处理二维参考方向，不会自动验证物理高程、重访一致性或定位精度。

## 检查和加载

```bash
cat maps/factory_map/navigation/conversion.yaml
cat maps/factory_map/navigation/nav_map.yaml
./scripts/run_a2w_navigation.sh maps/factory_map \
  maps/factory_map/navigation/nav_map.yaml auto_enable_motion:=false
```

上面的导航命令仅适用于输出仍为原 `map` 的情况。先检查参考 frame、原点、已知结构
和目标位置一致；颜色/导出成功不是几何精度验收。
