> **2026-10-26最新源码：** [完整新Bag回放与PCD分析](experiments/factory_live_195303_20261025/PUBLIC_EXPORT_ZH.md)；[独立运动参考及拒绝候选](experiments/field_z_motion_reference_20261024/PUBLIC_EXPORT_ZH.md)。完整建图已完成；几何改善未整体验收，联合初始化继续关闭。原始数据不随源码上传。

> 2026-10-24新增离线源码交付见 experiments/field_frontend_z_init_20261024/PUBLIC_EXPORT_ZH.md；真实联合候选未启用，未认证几何改善或原速。现场档案与数据仍排除。下文保留历史快照边界。

> 新增2026-10-23独立修复入口见 experiments/field_backend_repair_20261023/README.md；下文为原现场快照历史边界，旧入口未被原位覆盖。现场接口档案、Bag、地图与运行日志继续排除。

# 2026-10-23 受控源码快照

目标仓库：bh-207/A2W_FASTLIO2_Field_20261023。此导出保留当前本地源码与未提交修改，不覆盖原工作区、不切换生产环境。

## 本次内容

- 原工程的 FAST-LIO2 v4 质量门补丁、关键帧、保守 Scan Context/Quatro/Nano-GICP、iSAM2 后端、Map Bundle 及原有导航接口源码。
- `experiments/field_fixed_20261023`：Depth100/BEST_EFFORT、真实DDS端点握手、128MiB最小录包、固定外参及运行身份检查。
- 固定参数：`extrinsic_est_en=false`，`R=I`，`T=[-0.00862,0.00914,0.03916]m`，offset=0。来源为用户指定，不是物理标定认证。
- `field_source_snapshot/frontend`：现场build_003实际编译的完整前端代码与头文件；backend目录为实际带B账本的两个翻译单元；patches保留修改来源。
- `tools/field_map_review`：地图/重力/局部往返几何评估及中文报告/PDF导出源码。分析输入数据不在本仓库。
- 原本已经跟踪、供现有配准回归使用的3个小型PCD样本保留；未添加任何本次现场点云或地图。

P、R、J、新增重力软约束和未经几何验收的增强回环均未作为现场功能启用；不引入LIO-SAM。

## 已知问题必须保留

本次828KF地图的部分中段往返低面接缝在后端优化后变差；3条LoopCommitted不等于整条路线精度通过。
前端SIGINT后可能在严格输出端点检查处terminate，导致尾部状态/账本未正常关闭；目前没有修复此问题。
输入Bag和callback存在少量双向记录差异，不能宣称全链路无损。固定外参精度、绝对高程、Localization/Nav2验收仍未通过。

## 构建与复现边界

基础工程可按 `docs/BUILD_AND_DEPENDENCIES.md` 初始化固定子模块并独立构建：

```bash
git clone --recurse-submodules https://github.com/bh-207/A2W_FASTLIO2_Field_20261023.git
cd A2W_FASTLIO2_Field_20261023
./setup.sh --check
# 缺少依赖时由操作者决定执行 ./setup.sh --install-system
A2W_BUILD_BASE=build_review A2W_INSTALL_BASE=install_review ./build.sh
```

基础README沿用原工程的历史URL/分支；当前快照克隆以本说明为准。
`scripts/build_controlled_mapping.sh` 可从固定上游和v4补丁构建独立受控根目录，主install不变。

现场 `build.py` 原来为了避免重编译复用了本机冻结对象，`field.py` 绑定本机build_003和历史接口证据。
这些缓存、对象和远端驱动归档没有上传；不能将这两个脚本在新克隆中的直接执行声称为开箱即用。
为便于重新编译/审阅，本导出另存了实际前端完整源码、B后端翻译单元及全部相关头文件，而非只留私有二进制。
新主机仍需在独立构建树中配置这些源码、重新生成软件身份与真实输入证据，不能复用旧二进制SHA或伪造驱动证据。
公开导出明确排除现场接口档案、内部主机地址及驱动运行配置身份等运行环境元数据。输入契约证据由操作者在现场独立审核；不能伪造来源或借公开源码快照冒充硬件验收。
现场运行脚本和分析脚本保留原本机绝对路径限制，本导出不为上传而修改安全边界或编造可移植性测试通过。

分析脚本所需：NumPy、SciPy、Matplotlib、PyYAML；PDF导出另需ReportLab、Pillow、pypdf和中文字体。
原始Bag、地图、运行CSV及已有报告需由操作者单独提供。不要把缺失数据标成算法运行失败，也不要用合成结果冒充实测。

SHA256清单在 `MANIFEST.sha256`；`SOURCE_SNAPSHOT.json`记录源版本和实际现场二进制身份，仅用于追溯，不附带可执行产物。
第三方许可证、NOTICE与固定子模块引用均保留。
