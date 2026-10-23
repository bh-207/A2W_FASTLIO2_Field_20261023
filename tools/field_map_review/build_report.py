"""Generate a source-preserving Chinese review, with measured limitations."""
import os,sys,json,hashlib,csv,html
from pathlib import Path
os.environ['MPLCONFIGDIR']='/tmp/a2w_field_review_mpl'
sys.dont_write_bytecode=True
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
WS=OUT.parent.parent
S=WS/'captures/field_fixed_sessions/fixed_short_20261023_001'
B=S/'bundles/map_bundle'
R=json.loads((OUT/'analysis.json').read_text())
H=json.loads((WS/'captures/field_ground_height_20261023_001/result.json').read_text())
V=json.loads((OUT/'bundle_verification.json').read_text())
I=json.loads((OUT/'integrity/integrity_summary.json').read_text())
scores={(q['region'],q['mode']):q for q in R['geometry']}
regions=list(dict.fromkeys(q['region'] for q in R['geometry']))
labels=['Y -25','Y -75','Y -125','Y -175','Y -225','Y -275','Y -315','Return near loop']
fig,axs=plt.subplots(1,2,figsize=(15,5))
for mode,dx,color in [('raw',-.18,'tab:blue'),('optimized',.18,'tab:orange')]:
    median=[scores[n,mode]['floor']['absolute_return_height_difference_m']['median']*100 for n in regions]
    p95=[scores[n,mode]['floor']['absolute_return_height_difference_m']['p95']*100 for n in regions]
    normal=[scores[n,mode]['floor']['return_normal_angle_deg']['median'] for n in regions]
    x=np.arange(len(regions))+dx
    axs[0].bar(x,median,width=.34,color=color,label=mode)
    axs[0].scatter(x,p95,marker='_',s=95,color=color)
    axs[1].bar(x,normal,width=.34,color=color,label=mode)
for ax in axs:
    ax.set_xticks(range(len(labels)),labels,rotation=40,ha='right');ax.grid(axis='y',alpha=.3);ax.legend()
axs[0].set_ylabel('Absolute relative height difference (cm)');axs[0].set_title('Floor-like cross-pass seams: bars median, ticks P95')
axs[1].set_ylabel('Median cross-pass normal angle (deg)');axs[1].set_title('Free local planes; slopes are retained')
fig.suptitle('Same scan point identities, one global gauge, no local ICP; physical surface identity not surveyed')
fig.tight_layout();fig.savefig(OUT/'seam_before_after.png',dpi=160);plt.close(fig)

# Show the actual saved body-cloud geometry under raw and optimized poses.
sys.path.insert(0,str(WS/'experiments/offline_v4/gravity'))
import gravity_geometry as io
from scipy.spatial.transform import Rotation
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
raw=rows(B/'keyframes/index.csv');opt=rows(B/'optimized_poses.csv')
def pose(r):
    return Rotation.from_quat([float(r['q'+a]) for a in 'xyzw']).as_matrix(),np.array([float(r['t'+a]) for a in 'xyz'])
point_groups=[[],[]]
for a,b in zip(raw,opt):
    points=io.xyz(io.read_pcd(B/a['cloud'])[1])[::20]
    for out,r in zip(point_groups,[a,b]):
        rot,trans=pose(r);out.append(points@rot.T+trans)
points=[np.concatenate(a) for a in point_groups]
fig,axes=plt.subplots(2,2,figsize=(16,7))
for j,(q,name) in enumerate(zip(points,['Raw frontend','Saved optimized'])):
    axes[0,j].scatter(q[:,1],q[:,0],c=q[:,2],s=.1,vmin=-8,vmax=10,cmap='viridis',rasterized=True)
    axes[1,j].scatter(q[:,1],q[:,2],c=q[:,2],s=.1,vmin=-8,vmax=10,cmap='viridis',rasterized=True)
    axes[0,j].set_title(name+' - plan');axes[1,j].set_title(name+' - native YZ')
    axes[0,j].set_ylabel('X (m)');axes[1,j].set_ylabel('Z (m)')
    for ax in axes[:,j]:ax.set_xlim(-380,20);ax.set_xlabel('Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
    axes[0,j].set_ylim(-65,50);axes[1,j].set_ylim(-15,25)
fig.suptitle('Same body point indices (1/20 display sample), same axes and color; no registration or flattening')
fig.tight_layout();fig.savefig(OUT/'whole_map_same_points.png',dpi=160);plt.close(fig)

# Sanity check the free-plane same-horizontal-location height formula on slopes.
up=np.array([0.,0.,1.]);n=np.array([-.2,0.,1.]);n/=np.linalg.norm(n)
c=np.array([0.,0.,2.]);ref=np.array([3.,4.,9.])
height=lambda center: float(ref@up-n@(ref-center)/(n@up))
assert abs(height(c)-2.6)<1e-12
assert abs(height(c+np.array([0.,0.,.13]))-height(c)-.13)<1e-12
assert V['valid'] and V['keyframe_count']==828
assert len(R['loops_committed'])==3
assert R['state']['fixed_extrinsic_max_T_error_m']==0
assert R['state']['fixed_extrinsic_max_R_quaternion_error']==0

loop_table=[]
for q in R['loops_committed']:
    loop_table.append(f"| {q['source_id']}→{q['target_id']} | {q['source_relative_s']:.1f} | {q['time_separation_s']:.1f} | {q['fitness']:.4f} | {q['overlap']*100:.2f}% | {q['correspondences']} | {q['optimized_translation_m']*100:.2f} cm / {q['optimized_rotation_correction_deg']:.3f}° |")
floor_table=[];floor_html=[];wall_table=[]
for label,n in zip(labels,regions):
    a=scores[n,'raw'];b=scores[n,'optimized'];af=a['floor'];bf=b['floor']
    f=lambda g,k: g[k]['median']*100
    vals=(f(af,'absolute_return_height_difference_m'),f(bf,'absolute_return_height_difference_m'),af['absolute_return_height_difference_m']['p95']*100,bf['absolute_return_height_difference_m']['p95']*100)
    floor_table.append(f"| {label} | {af['normal_consistent_cells']} | {vals[0]:.2f} / {vals[2]:.2f} | {vals[1]:.2f} / {vals[3]:.2f} | {af['return_normal_angle_deg']['median']:.2f} / {bf['return_normal_angle_deg']['median']:.2f} |")
    floor_html.append('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in [label,af['normal_consistent_cells'],f'{vals[0]:.2f} / {vals[2]:.2f}',f'{vals[1]:.2f} / {vals[3]:.2f}',f"{af['return_normal_angle_deg']['median']:.2f} / {bf['return_normal_angle_deg']['median']:.2f}"])+'</tr>')
    aw=a['wall_return'];bw=b['wall_return'];na=aw.get('planar_wall_supported_count',0);nb=bw.get('planar_wall_supported_count',0)
    va=aw.get('point_to_local_plane_abs_m',{}).get('median');vb=bw.get('point_to_local_plane_abs_m',{}).get('median')
    wall_table.append(f"| {label} | {na} / {nb} | {va*100:.2f} / {vb*100:.2f} | {'支持少于50，unknown' if min(na,nb)<50 else '描述性几何指标，物理身份unknown'} |" if va is not None and vb is not None else f'| {label} | {na} / {nb} | 无 | unknown |')

txt=f'''当前固定外参现场地图复核（只读，2026-10-23）

结论：地图确实保存成功，828关键帧、6,512,118点，约794.80秒、700.90米XY路线；3条回环实际入图，其中1条是相隔580秒的长时间返程。部分局部几何较好，但最终优化地图不是全路段都优于原始前端：若干走廊区的往返低面高度接缝从约1厘米增大到10～14厘米。尚不能作为已验收的导航地图。

1. 实际入图回环（不是检测候选数）
| source→target | 源相对时间s | 间隔s | fitness | overlap | 对应点 | 当次源节点修正 |
|---|---:|---:|---:|---:|---:|---|
{chr(10).join(loop_table)}
31→0主要是静止位置重复观测；113→82是早期局部重叠；788→90是长时间返程重访。回环配准质量值不是绝对定位误差或真实性认证。3821条LoopProfile中3742次被时间/关键帧间隔保护拦截，不能当作3821次独立闭环尝试；几何阶段3次提交、2次失败，不据此宣称召回率或“60%成功率”。

2. 同点往返局部几何：后端有改善也有退步
以下是每个0.75m格中，自由低面在同一水平位置的高度差绝对值，厘米；不要求地面水平，不执行局部ICP。ROI先写入geometry_protocol.json，再计算指标。低面来源和语义没有现场测量认证，因此这些是相对几何证据，不是高程真值。保留全部8区、失败项和原始格结果。
| 区域 | 合格格数 | raw 中位/P95 cm | optimized 中位/P95 cm | 法向中位raw/opt ° |
|---|---:|---:|---:|---:|
{chr(10).join(floor_table)}
其中Y约-75、-125、-175、-225米，raw中位分别约0.68、1.42、1.32、1.17厘米，optimized约10.94、12.66、14.00、10.36厘米。同次首程留出扫描对训练扫描的差异中位约0.10～0.42厘米，所以上述10厘米量级变化明显大于本次局部扫描差异；这不是重复建图运行波动测试。
返程回环邻域之外的独立扫描组：地面中位1.51→0.64厘米、P95 2.87→1.94厘米；墙类点到面中位3.85→2.87厘米。该组排除了配置±5关键帧配准子图使用的扫描，但物理表面身份仍非测绘认证。
原始/最终几何只比较同一批body云点、同一全局坐标基准和相同尺度。它证明最终位姿图修正会改变沿途相对接缝，不能凭此把某一条回环直接认定为误闭环；需单因子移除/恢复图约束才能归因。无此次回放/消融。

墙类非地面保护指标（支持点不足的区域保留unknown；不能用墙法向当Z误差）：
| 区域 | raw/opt支持点 | 点到面中位raw/opt cm | 解释 |
|---|---:|---:|---|
{chr(10).join(wall_table)}
Y=-75、-125、-225区域非地面描述性指标也退步，但支持比例仅约3.6%～9.8%，对应结构未人工标定，不能外推到整段所有墙柱。Y=-175仅6～7点、Y=-315无支持，明确unknown。

3. 高度与倾斜：有坐标贡献，但剩余高度不能全判为漂移
同次INIT重力camera_init = {R['state']['init_gravity_camera_init']} m/s²；反重力相对原生Z倾角{R['state']['initial_anti_g_tilt_deg']:.6f}°。用首帧map←camera_init关系转换，固定up_map={R['state']['init_up_map']}，全过程使用这一固定轴；不是用逐帧变化的g旋转地图。
已有28,871个低面候选中心的原生Z跨度7.333米；沿同次初始重力竖直方向投影后跨度4.613米。轨迹对应为7.124米和4.381米。整张点云35.191米Z跨度还包括设备/屋顶/墙体，不能当作地面高度差。极值可能来自不同点，不能把两种跨度相减就叫“倾斜修复了多少漂移”。剩余量包含真实坡度、局部姿态/位移估计误差等，缺少高程真值无法分离其绝对比例。详见先前只读提取的field_ground_height_20261023_001/protocol.json及result.json，本轮未修改该结果。

4. 前端状态与固定外参
保存范围7821条POST中：g方向相对INIT最大变化1.431782°（约216.90秒），末值0.828483°；加计偏置变化最大0.608643m/s²，陀螺偏置变化最大0.000954rad/s。这些是滤波估计变化，不是外部姿态真值。
全部已有8048行INIT/POST中的固定外参保持R=I、T=[-0.00862,0.00914,0.03916]m，记录精度内变化0。关闭在线外参确实生效；不能据此证明用户指定值物理标定正确。roll约-3.05～3.58°、pitch约-6.95～2.41°包含真实行走/坡道调姿，不能称为姿态误差。
后端相对raw最大位置修正1.433米、最大Z修正0.633米、最大角度修正0.410°，并非统一刚体旋转。闭环当次源节点3.73厘米修正与最终全图最大修正是不同量。
停止点距开始点约16.10米；未确认实际同一物理位置，不能写成16米闭环误差。本次Bag/路线和此前A727不同，没有同输入控制实验，不能把g变化比旧会话小直接归因为关闭在线外参。

5. 保存完整性与结束异常
本轮重新核验Bundle的836个manifest条目SHA256、UUID、828连续ID、位姿和body云索引，全部通过；manifest SHA256={V['manifest_sha256']}。
保存前15条下游账本边完整：7821 POST→输出→ingress判定，828KF→backend→Bundle；POST与整数消息时间仅允许1000ns派生容差，实际最大128ns，其他边整数精确。不是旧POST0～47漏收问题。
但最终capture_degraded=true：/tf_static未录到；同区间Bag IMU163780、回调163784，有12条Bag-only、16条callback-only，不能把总数相减只说少4条。保存截止前仍有10/12条双向差异，并非只在停机尾部。原始IMU最大源间隔44.84ms、点云300.05ms。录包exit0且无cache-loss告警，不足以证明完整无损，不能称为164驱动丢包率。
前端收到SIGINT后抛出“Odometry ingress/observer/recorder edge disconnected”并terminate；runtime status为空、frontend handoff status缺失。已知runtime少6条、cloud返回账本少1条均在保存截止后约22.7～23.2秒；后者已被ingress接收。因此不能把这些尾缺当作已保存828KF缺失，但全程关闭/记录质量未通过。stop.json的SIGINT completed只代表PID消失，不代表正常退出。
高概率退出原因：SigHandle直接rclcpp::shutdown()，在途timer继续检查严格订阅数，可能将停机断连作为运行错误抛出，阻止日志正常close；缺当时DDS端点/异常栈，不能认定唯一因果。证据见integrity/REPORT_zh.txt及原始frontend.log:17-19、实际编译laserMapping.cpp:170-175和1396-1400。本轮不修改停机逻辑。

6. 本轮结论和下一步
已确认：固定外参生效；真实3次入图；有效828KF Bundle；保存前交接完整；选定中段地面接缝被最终位姿图放大，返程邻域局部改善。
高概率判断：当前最具体的可定位地图质量问题是沿途非统一后端修正损害部分往返一致性，而不是所有高度变化都来自前端Z漂移。
待验证：各回环约束单独贡献、物理对应、真实坡度/高程、固定外参精度、重复运行波动、Localization/Nav2使用效果。
建议先保留此Bundle及raw轨迹，针对3条实测约束做独立冻结图消融和中段非地面核对；下次现场前修复并回归正常退出与日志关闭。不要为降低高度跨度压平厂区，不切导航地图。

PCD：{B/'global_map.pcd'}
实际运行配置、输入、日志：{S}
本轮输出：{OUT}
复现：python3 -B {OUT/'analyze.py'} && python3 -B {OUT/'build_report.py'}
分析脚本、Bundle哈希和斜面高度公式自检通过；本轮未编译、未回放Bag、未连接机器人、未改原始数据或生产配置。数值评估/静态图已检查；HTML文件生成和本地引用完整性检查不等于浏览器交互或导航验收。
'''
(OUT/'REPORT_ZH.txt').write_text(txt)
figures=[('seam_before_after.png','主要结果：往返高度接缝，有改善也有退步'),('whole_map_same_points.png','同点同尺度整图：原始前端与保存优化图'),('local_return_sections.png','局部实际点云：首程训练/留出与返程'),('trajectory_and_backend.png','轨迹、高度及后端修正'),('state_diagnostics.png','重力、姿态、偏置随传感器时间变化')]
content=html.escape(txt)
links=' '.join(f'<a href="{html.escape(str(p))}">{title}</a>' for p,title in [(B/'global_map.pcd','原生PCD'),(OUT/'analysis.json','全部量化结果'),(OUT/'geometry_protocol.json','冻结评估规则'),(OUT/'integrity/REPORT_zh.txt','完整性审计')])
images=''.join(f'<section><h2>{title}</h2><img src="{name}" alt="{title}"></section>' for name,title in figures)
page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>现场828KF地图只读复核</title><style>body{{font-family:system-ui,sans-serif;max-width:1280px;margin:auto;padding:24px;background:#f6f7fa;color:#172135;line-height:1.65}}a{{color:#175cb2;margin-right:20px}}section{{background:white;padding:18px;margin:20px 0;border-radius:10px}}img{{max-width:100%;height:auto}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;font:15px/1.8 system-ui,sans-serif}}.lead{{border-left:6px solid #d18313;padding:18px;background:#fff1d6}}</style><h1>当前固定外参现场地图复核</h1><p class="lead">保存成功，828KF、约701米，3条回环实际入图。返程附近局部改善，但中段部分往返高度接缝被后端放大到10～14厘米；结束异常，尚未通过完整采集质量与导航验收。</p><p>{links}</p>{images}<section><h2>完整结论、数字与边界</h2><pre>{content}</pre></section></html>'''
(OUT/'REPORT_ZH.html').write_text(page)
sources=[S/'runtime_state.csv',S/'logs/backend.log',S/'logs/frontend.log',S/'field_verification.json',S/'save_success.json',B/'manifest.sha256',OUT/'analyze.py',OUT/'build_report.py',OUT/'geometry_protocol.json',WS/'captures/field_ground_height_20261023_001/result.json']
provenance={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
(OUT/'report_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(OUT/'analysis_checks.json').write_text(json.dumps(dict(free_sloped_plane_height_formula=True,same_slope_13cm_separation=True,bundle_manifest=V,actual_committed_loops=3,recorded_fixed_extrinsic_unchanged=True,generated_image_count=len(figures),source_session_modified=False,robot_connection=False,Bag_replay=False,SLAM_code_modified=False,geometry_ground_truth_certified=False),indent=2)+'\n')
for name,_ in figures:assert (OUT/name).is_file()
print('Chinese report, plots, provenance and offline sanity checks generated.')
