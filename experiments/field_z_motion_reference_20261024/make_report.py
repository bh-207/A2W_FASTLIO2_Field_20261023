"""Chinese evidence report for the actual motion-reference experiments."""
import argparse,csv,hashlib,html,json
from pathlib import Path
import numpy as np


def build(out):
    out=Path(out).resolve();W=Path(__file__).resolve().parents[2];old=W/'captures/field_frontend_z_init_20261024_001'
    source=W/'experiments/field_z_motion_reference_20261024';F=out/'final_verified'
    names={'original_reference':'一致均值＋原相邻ICP参考','local_submap':'全频率＋10扫描局部子图','no_deskew':'局部子图＋关闭参考去畸变','endpoint_trajectory_deskew':'局部子图＋端点轨迹二次去畸变','current_motion_iterated':'局部子图＋当前运动有界两次校正','selected_window':'训练激励选窗＋扩展留出至230秒'}
    results={k:json.loads((F/(k+'.json')).read_text())for k in names}
    shape=json.loads((out/'ground_shape_v2/shape.json').read_text());train=shape['groups']['train']
    original=json.loads((old/'candidate_motion_final.json').read_text());eiv=json.loads((out/'old_reference_eiv_diagnostic.json').read_text())
    counter=json.loads((out/'mean_component_counterfactual/counterfactual.json').read_text())
    angular=json.loads((out/'angular_original/angular_windows.json').read_text())
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    labels=['old GLS','consistent old','local map','no deskew','endpoint pass','current iter','selected window']
    pos=[original['heldout_validation']['max_position_error_m']]+[results[k]['heldout_validation']['max_position_error_m']for k in names]
    angle=[original['heldout_validation']['max_rotation_error_rad']*180/np.pi]+[results[k]['heldout_validation']['max_rotation_error_rad']*180/np.pi for k in names]
    chi=[original['residual_reduced_chi2']]+[results[k]['residual_reduced_chi2']for k in names]
    fig,axes=plt.subplots(3,1,figsize=(13,12),sharex=True)
    for ax,value,limit,label in zip(axes,[pos,angle,chi],[.5,.05*180/np.pi,3.],['heldout position mismatch (m)','heldout angle mismatch (deg)','training reduced chi-square']):
        ax.bar(np.arange(len(labels)),value,color=['#db7368' if v>limit else '#69a88b'for v in value]);ax.axhline(limit,color='k',ls='--',label='gate')
        for x,v in enumerate(value):ax.text(x,v,f'{v:.3f}',ha='center',va='bottom',fontsize=8)
        ax.set_ylabel(label);ax.grid(axis='y',alpha=.2);ax.legend()
    axes[0].set_yscale('log');axes[-1].set_xticks(np.arange(len(labels)),labels,rotation=15)
    fig.suptitle('Real motion data: initialization acceptance, NOT map accuracy\nAll candidates rejected; no JOINT_INIT_COMMIT or fallback map')
    fig.tight_layout();fig.savefig(out/'initialization_comparison.png',dpi=155);plt.close(fig)
    # Compare actual propagated means and observed poses in the SAME fixed
    # training coordinate chart; no local ICP or candidate-specific flattening.
    fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True)
    for k in ['original_reference','local_submap','current_motion_iterated']:
        d=results[k];checks=d['heldout_validation']['checks'];start=d['training_last_stamp_ns']
        t=[(x['stamp_ns']-start)*1e-9 for x in checks]
        axes[0].plot(t,[x['position_error_m']for x in checks],label=names[k])
        axes[1].plot(t,[x['rotation_error_rad']*180/np.pi for x in checks],label=names[k])
    for ax,limit,label in zip(axes,[.5,.05*180/np.pi],['position (m)','angle (deg)']):
        ax.axhline(limit,color='k',ls='--');ax.set_ylabel(label);ax.grid(alpha=.25)
    # Use English labels inside charts to avoid relying on host CJK Matplotlib.
    for ax in axes:ax.legend(['consistent original','local submap','current-motion deskew'],fontsize=9)
    axes[-1].set_xlabel('seconds after frozen training endpoint');fig.tight_layout();fig.savefig(out/'heldout_time_comparison.png',dpi=155);plt.close(fig)
    rows=[]
    for k,label in names.items():
        d=results[k];h=d['heldout_validation'];rows.append([label,f"{d['residual_reduced_chi2']:.6f}",f"{h['max_position_error_m']:.6f}",f"{h['max_rotation_error_rad']*180/np.pi:.6f}",f"{d['gravity_direction_sigma_degrees']:.4f}",'拒绝：'+d['reason']])
    refrows=[]
    for label,folder in [('原相邻ICP stride1','pair_stride1'),('原目标函数＋有界优化续算','pair_stride1_polish_v2'),('局部子图10扫描','submap_stride1_v2'),('关闭参考去畸变','submap_stride1_no_deskew'),('端点轨迹去畸变','submap_stride1_trajectory'),('当前运动两次校正','submap_current_iterated'),('扩展至230秒','submap_150_230'),('GICP stride3','gicp_pair_stride3'),('GICP stride1','gicp_pair_stride1')]:
        p=out/folder/'registration.json'
        if p.exists():
            d=json.loads(p.read_text());failure=d.get('failure');refrows.append([label,f"{d.get('completed_scans','?')}/{d.get('expected_scans','?')}",'完整参考（不等于准确）'if d.get('complete')else str(failure.get('reason')if failure else '未完成')])
        else:
            p=out/folder/'registration_failure.json';d=json.loads(p.read_text())if p.exists()else{}
            refrows.append([label,'部分扫描；已保留失败','scan '+str(d.get('scan_id','?'))+': '+str(d.get('reason','未完成'))])
    def table(head,rows):return '<table><thead><tr>'+''.join('<th>'+html.escape(str(v))+'</th>'for v in head)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>'for v in row)+'</tr>'for row in rows)+'</tbody></table>'
    def image(path,caption):return '<figure><a href="'+html.escape(path)+'"><img src="'+html.escape(path)+'"></a><figcaption>'+html.escape(caption)+'</figcaption></figure>'
    cvals={k:v['max_position_error_m']for k,v in counter['results'].items()}
    summary=dict(scope='Z shape and rejected motion initialization; original B009 control unchanged',base_commit='ebaebae601387190a0f59751764ccd07e8c034f7',
        actual_frontend_joint_commits=0,new_maps_generated=0,geometry_improvement_proven=False,
        fixed_up_shape=dict(entry_to_peak_m=train['entry_to_peak_m'],peak_to_far_m=train['peak_to_end_m'],maximum_chord_up_plane_deviation_m=max(map(abs,train['in_chord_up_plane_deviation_m'])),
            physical_slope_accuracy='UNKNOWN_WITHOUT_SAME_PHYSICAL_POINT_SURVEY'),
        old_rejected_holdout_m=154.01231658525214,counterfactual_holdout_m=cvals,
        final_candidates={k:dict(accepted=v['accepted'],reason=v['reason'],chi2=v['residual_reduced_chi2'],heldout_position_m=v['heldout_validation']['max_position_error_m'],heldout_rotation_rad=v['heldout_validation']['max_rotation_error_rad'])for k,v in results.items()},
        raw_imu_gap_in_window=dict(maximum_seconds=angular['raw_IMU_maximum_gap_s'],count_over20ms=angular['raw_IMU_gaps_above20ms']),
        observed='old GLS large latent pose pull mixed with raw LiDAR commit mean; counterfactual and consistent-mean replay support major software contribution to failed-candidate blow-up',
        unknown='physical IMU vs independent LiDAR motion vs fixed calibration/scale model residual; true factory terrain vs accumulated map error',
        source_default='new code isolated; original frontend and production defaults unchanged; no hardware or realtime acceptance')
    (out/'FINAL_RESULT.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    htmltext='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Z形状与运动参考专项验证</title><style>body{font:16px/1.8 system-ui,sans-serif;max-width:1150px;margin:28px auto;padding:0 24px;color:#183448;background:#fafcfe}h1,h2{color:#165c80}h1{font-size:29px}h2{margin-top:38px;border-bottom:1px solid #ccdde8;padding-bottom:6px}.lead{background:#fff1d6;border-left:5px solid #c78e32;padding:18px 22px}.note{color:#4c6579;font-size:14px}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #d3dfe7;padding:9px;vertical-align:top}th{background:#eaf2f7}tr:nth-child(even){background:#f1f6f9}img{max-width:100%;height:auto}figure{margin:26px 0}figcaption{font-size:14px;color:#4e6779}code,pre{background:#edf2f6;overflow-wrap:anywhere}pre{padding:14px;white-space:pre-wrap;font-size:13px}a{color:#14638e}.fail{color:#a84335}</style><body>'''
    htmltext+='<h1>Z中段形状、独立运动参考与联合初始化专项</h1><p class="note">2026-10-24｜基于本地核对的 ebaebae60138 快照｜B_009为完整输入冻结控制｜全部实验独立保存</p>'
    htmltext+='<div class="lead"><b>结论：定位并修正了失败联合初始化中的一组均值/协方差一致性问题，但尚未形成可信真实初始化，地图中段隆起没有获得算法改善证明。</b><br>本轮实际进行了真实预处理云配准、原始IMU短时积分、单因素反事实与联合重估。所有最终候选都拒绝；真实JOINT_INIT_COMMIT=0，新Map Bundle/PCD=0。没有重复生成拒绝后的回退地图，也没有改变B_009或生产。</div>'
    htmltext+='<h2>1. 已确认与不能推断的边界</h2><p>1.16m/s²属于被拒绝的150～215秒联合候选，不是B_009实际滤波状态；不能把它直接说成真实IMU偏置或本图隆起的根因。入口至远端约4.888m的坐标投影项继续保留为投影解释，不作为修复量。固定R=I、T=[-0.00862,0.00914,0.03916]、false及offset=0保持，来源仍为用户指定，非物理标定认证。单位、驱动旋转、时间语义、Q、后端guard均未更改。</p>'
    htmltext+='<h2>2. 统一旋转不改变三维形状，竖直投影仍依赖up</h2>'
    htmltext+=table(['同一冻结地面位置的诊断','数值','含义'],[['入口→中段峰值',f"+{train['entry_to_peak_m']:.6f}m",'固定初始up方向；未认证真实厂区高差'],['中段→远端',f"{train['peak_to_end_m']:.6f}m",'同一组锚点、点身份与评价方向'],['中段对首末连线在连线/up平面内的偏离',f"{max(train['in_chord_up_plane_deviation_m']):.6f}m",'三维形状诊断；不得作为拉平目标'],['首末XYZ弦长',f"{train['chord_length_m']:.6f}m",'同一SE3下弦距/三维夹角不变'],['全XYZ最大弦偏离',f"{max(train['chord_distance_m']):.6f}m",'包含入口转弯和横向路线弯曲，绝不能称Z误差']])
    htmltext+=image('ground_shape_v3/xyz_chord_and_height.png','三组冻结锚点的完整XYZ、固定up高度、弦垂直分量及三维分段角。3D展示各轴显示比例独立，全部指标仍使用未缩放XYZ；重复出现不等于独立真值。')
    htmltext+='<p>原生Z入口至远端约-5.954527m，固定up约-1.067342m，中段仍先升后降。长廊内部三维段间角约0.90°～1.92°，入口转角约32.19°，后者不是估计错误的证据。平面法向和高度均自由，没有局部ICP、地面共面化、重新选最平up、扣趋势或裁Z。</p>'
    htmltext+='<p><b>解释边界：</b>沿冻结路线，2.53m上升分布在约172m，3.60m下降分布在约140m，平均估计坡度约+1.47%和-2.56%；不能仅凭米级高差把它判为漂移。弦距不变量只能证明点列的三维弯曲不会因刚体旋转消失，不能独立证明哪个竖直方向正确，也不能排除任意统一参考误差。要确认这是否与真实缓坡一致，仍须同物理位置的独立高差。</p>'
    htmltext+='<h2>3. 1.16m/s²残余从哪里出现</h2><p>原始训练末后16.299986s留出可精确复现154.012317m最大位置失配。用原冻结g/ba和独立LiDAR姿态计算，水平残余在训练前缀已约0.96m/s²，训练和留出持续存在，留出均值约1.1669m/s²；不是到留出才突发。窗口内原始IMU最大间隔'+f"{angular['raw_IMU_maximum_gap_s']*1000:.3f}"+'ms，超过20ms为0次；本轮无DDS追赶输入路径。</p>'
    htmltext+='<p>旧协方差边缘化解释残差时，隐含了最大9.183934°、训练末7.069315°的LiDAR姿态条件修正，但原提交均值仍直接使用原LiDAR姿态。如此大的修正也说明一次线性化不能被视为可信的真实姿态。这个诊断支持均值不自洽问题，不能独立证明它解释了全部真实传感器误差。</p>'
    htmltext+=table(['同一真实IMU留出；单组均值改变','最大位置失配/m','用途'],[['全部冻结，精确复现',f"{cvals['original_frozen']:.6f}",'失败控制'],['仅训练末R条件线性修正',f"{cvals['only_commit_R_conditional_linear_pull']:.6f}",'仅因果敏感性；不可提交'],['仅g改为未加权种子',f"{cvals['only_g_unweighted_seed']:.6f}",'诊断，非物理重力真值'],['g/ba联合种子组',f"{cvals['g_ba_unweighted_seed_group']:.6f}",'诊断；仍未通过0.5m门']])
    htmltext+=image('mean_component_counterfactual/single_component_counterfactual.png','反事实只用于定位失败层，不产生候选、初始化或地图。')
    htmltext+=image('original_residual_audit/short_horizon_residuals.png','按0.5/1/2/5秒重置积分，分离力残余、陀螺残余和局部位移失配；LiDAR微分/插值不是独立真值。')
    htmltext+='<h2>4. 独立运动参考确有未解释的局部角度失配</h2><p>原参考与原IMU在约170.2、182.2、197.2秒附近，1秒旋转增量可相差约2.6°～3.1°，其中约197.2秒主要是yaw分量：LiDAR参考约19.266°，积分陀螺约16.185°。这不是3°的roll/pitch误差，也不能当作Z误差；它说明未建模运动/配准/固定模型不一致真实存在，需要与联合参数误估分开。</p>'
    htmltext+=image('angular_original/angular_interval_comparison.png','相同真实时间区间的LiDAR相对旋转与原始陀螺减已估bg。没有试写时间偏移、猜单位或改轴。')
    htmltext+='<h2>5. 实际完成的参考配准消融</h2>'+table(['实验','完成扫描','结果'],refrows)
    htmltext+='<p>原相邻ICP直接改stride1会因目标函数不下降而拒绝；有界续算也保留了失败，不能拿部分前缀宣称全频率优于stride3。有限子图保留最多10个历史扫描，目标只含当前扫描之前的数据；参数仍为0.35m voxel、0.8m关联距离、0.55 overlap和0.15m点面RMS门。为调查邻域切换导致的GN方向不下降，独立实验增加默认关闭的真实目标函数有界续算；没有调前端Huber/Q。</p><p>端点轨迹补偿只使用截至当前scan END的首轮LiDAR端点与分段插值；当前运动补偿则固定同一历史目标，最多两次重算当前原始采样点的去畸变，不插入中间结果。GICP是另列的方法对照，数值稳定性检查未通过，未替代原参考，也未提交。Open3D接口见<a href="https://open3d.org/docs/release/cpp_api/classopen3d_1_1pipelines_1_1registration_1_1_transformation_estimation_for_generalized_i_c_p.html">官方GICP文档</a>。</p>'
    htmltext+='<h2>6. 一致惯性均值、协方差修复后的真实重估</h2><p>新增独立直接积分似然：拟合与准备提交的是同一条陀螺传播的p/R/v，避免参数基于条件姿态、提交却换回原LiDAR姿态。g方向、ba、bg和初速联合辨识，固定R/T、offset和原加速度内部比例1.0136393576435327，不加方向硬先验，不截偏置。已恢复原输入provenance、SO3、协方差、可观性、自由重力模长、最少0.5秒留出检查；新独立训练一致性门为χ²≤3。现有前端C++准入未改，若有通过候选仍须经真实装载与INIT_COMMIT。</p><p>协方差实现显式处理：训练首帧重设坐标的完整Jacobian与交叉项；同一原始IMU样本在相邻插值中的共享噪声；拟合参数与后续直接传播之间的相关项；状态仍保持训练首帧的条件坐标，准备入口只做一次B_009锚点SE3放置。未用伪逆制造可提交协方差。先前未处理共同前缀的候选保留在SUPERSEDED_DIAGNOSTICS.json，不参加本表。</p>'
    htmltext+=table(['最终真实对照','训练χ²','留出最大位置/m','最大角度/°','形式g σ/°','结论'],rows)
    htmltext+=image('initialization_comparison.png','全部候选的训练、位置及角度门分开报告；绿色单项不意味着整个候选通过。')
    htmltext+=image('heldout_time_comparison.png','同一16.3秒冻结留出，修正均值不一致后病态失配显著下降，但均未通过全部门限。')
    htmltext+='<p>当前运动有界校正相对局部子图上一对运动补偿，将留出最大角度失配从约3.894°降到1.270°，训练χ²从12.178降至4.182；但位置从3.403m退步至5.336m，所以明确否决。关闭参考去畸变、端点二次补偿也未通过，不能靠训练曲线或单一角度指标选优。</p>'
    htmltext+='<p>扩展窗口仅依据训练段弱轴方向支持选窗，保留215～230秒未参与选择的新验证段。选中约160～190秒训练，验证一直延伸至约230秒；总留出最大位置失配约25.313m，依然拒绝。自由g/ba问题在当前相对阈值下为rank5，已知|g|后的S2问题为rank5：自由径向重力与沿近竖直体轴的加计偏置仍弱可分。已知模长可以形式上补足约束，但不能使错误运动模型变正确。</p>'
    htmltext+='<h2>7. 根因分层与本轮完成度</h2>'+table(['层级','已确认','仍未知/未通过'],[['失败联合候选','旧GLS大条件姿态修正与原LiDAR提交均值混用；单均值反事实、直接一致积分对照均能大幅改变失配','不能将它认证为唯一物理根因；不能直接修复B_009'],['独立运动参考','部分相邻/全频配准会拒绝；子图与去畸变改变训练和留出结果；1秒yaw增量存在数度失配','ICP系统误差、真实IMU/固定R或尺度模型贡献不能唯一分离'],['地图中段三维形状','+2.530502/-3.597845m和3.114450m弦/up平面偏离可重复计算','真实坡道与累计误差尚无独立高程认证'],['功能链/新地图','17项新增回归、17项继承联合回归、独立C++构建和真实数据运算已执行','真实可信JOINT_INIT_COMMIT未发生；完整新前端Map Bundle/PCD未触发'],['生产/现场','B_009、原数据、原源码、固定配置、主install保持；未连接164/发布运动','新初始化仍默认关闭，不建议现场启用']])
    htmltext+='<h2>8. 最少补测与后续可执行边界</h2><p>若要判断“缓坡是否被建大”，最低需要把三处可复找的实体地面点绑定到现场标记，独立测量两段高差：入口E（本图约[1.865,-6.369,-0.468]）、中段M（约[16.929,-175.132,-0.759]）、远端F（约[19.080,-315.315,-6.423]）。当前固定up估计M−E=+2.530502m、F−M=-3.597845m。三点高差用于验证物理坡度，不等于完成完整姿态或外参标定。要分离剩余角度/IMU模型问题，还需至少一个独立角度基准或能被多方向非重复结构可靠约束的运动片段；仅重放同一模糊参考不能认证它自身。</p><p>现有数据已能继续定位短区间配准，但不能把满秩、小形式σ或相邻对齐残差当可信运动真值。下一次算法修改应先解决上述明确的局部旋转增量不一致，保留未使用的验证数据；本轮不以缺高程真值为由跳过运算，也不因候选全失败而强开初始化。</p>'
    htmltext+='<h2>9. 保存位置与复现</h2><p>本轮未生成新候选地图。继续使用冻结B_009：</p><pre>'+html.escape(str(old/'maps/B_009/frontend_native_raw.pcd'))+'\n'+html.escape(str(old/'maps/B_009/map_bundle'))+'</pre>'
    htmltext+='<p>本轮源码：'+html.escape(str(source))+'；最终候选、逐段残余和失败证据：'+html.escape(str(out))+'。原始报告、旧候选、所有失败输出和生产文件均未覆盖。新代码尚未上传GitHub，本轮未请求发布。</p><pre>OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 '+html.escape(str(source/'reproduce.py'))+' --output /home/bh/A2W_FastLio2_controlled/captures/NEW_z_motion_reproduction</pre><p>要求新目录不存在，分析至少3GiB；依赖原只读预处理云/CSV和B_009。脚本独立构建本轮C++积分核，拒绝候选不会调用前端回放/建图；只有通过候选才允许独立构建、真实提交检查、全长重建与自身重复，仍需冻结几何验收。本轮已另建目录实际执行该入口，结果见REPRODUCTION_CHECK.json；通过分支因没有合法候选而NOT_RUN，不能声称已贯通真实联合建图。HTML图像已逐张检查，交互浏览器/Nav2/原速均未验收。</p>'
    htmltext+='<p>可检查证据：<a href="FINAL_RESULT.json">最终逐项状态</a>｜<a href="FINAL_VERIFICATION.json">构建/回归/Bundle校验</a>｜<a href="REPRODUCTION_CHECK.json">实际复现对照</a>｜<a href="SOURCE_PRESERVATION_CHECK.json">原始资料与生产边界</a>｜<a href="CODE_DIFF.patch">独立代码差异</a>。</p></body></html>'
    (out/'REPORT_ZH.html').write_text(htmltext)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();build(a.output)
