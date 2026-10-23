#!/usr/bin/env python3
import html
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
OUT=HERE.parents[1]/'captures/field_backend_repair_20261023_001'
def read(path):return json.loads((OUT/path).read_text())
def esc(s):return html.escape(str(s))
def table(head,rows):return '<table><thead><tr>'+''.join('<th>'+esc(x)+'</th>' for x in head)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+esc(x)+'</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table>'
def link(path,label):return '<a href="'+esc(path)+'">'+esc(label)+'</a>'
def section(title,content):return '<section><h2>'+esc(title)+'</h2>'+content+'</section>'
def metric(v):return v['floor']['absolute_return_height_difference_m']
def pair(v):
    s=metric(v);return f"{100*s['median']:.2f} / {100*s['p95']:.2f}"

def main():
    geom=read('geometry/final_bundle/report.json');masks=read('geometry/repeats123/report.json')
    rows=[]
    for g in geom['geometry']:
        c=g['comparisons']['final_bundle'];v=c['variants']
        rows.append([g['region'],c['common_floor_cell_count'],pair(v['raw']),pair(v['old_optimized']),pair(v['final_bundle'])])
    maskrows=[]
    for mask in range(8):
        label=f'run1_m{mask}';values=[]
        for g in masks['geometry'][1:5]:values.append(f"{100*metric(g['comparisons'][label]['variants'][label])['median']:.2f}")
        edges='+'.join(str(i) for i in range(3) if mask&(1<<i)) or '无回环控制'
        maskrows.append([mask,edges,*values,'FAIL（墙面保护）'])
    runs=[]
    for name in ['offline_smoke_004','offline_smoke_010','offline_smoke_011','offline_smoke_012','offline_smoke_013','offline_smoke_014']:
        path=OUT/'tests'/name/'result.json'
        if path.exists():
            d=json.loads(path.read_text());save=d.get('save',{});ap=path.parent/('ACCEPTANCE_v2.json' if (path.parent/'ACCEPTANCE_v2.json').exists() else 'ACCEPTANCE.json')
            status=json.loads(ap.read_text())['status'] if ap.exists() else '未执行完整正式门'
            if name.endswith('012'):status='负向测试通过：前端/ingress预期返回2'
            runs.append([name,d.get('requested_duration_s'),d.get('published_counts'),
                d.get('fixed_state_closed',{}).get('post'),save.get('response',{}).get('keyframe_count'),status])
    original=read('original_field_input_accounting.json')['callback_vs_recorded']
    b='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>828KF 后端保护、工程修复与验收</title><style>
    body{font:16px/1.65 system-ui,sans-serif;background:#eef2f6;color:#142435;max-width:1280px;margin:32px auto;padding:0 24px}h1{font-size:30px}h2{font-size:22px}section{background:white;padding:24px;margin:20px 0;border-radius:12px;overflow:auto}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:9px;border:1px solid #d7dee6;text-align:left}th{background:#eff5fb}a{color:#065aac}code,pre{font-family:ui-monospace,monospace}pre{white-space:pre-wrap;background:#f4f6f8;padding:15px}img{max-width:100%}.bad{color:#ac1b2e}.good{color:#146842}.note{background:#fff2d8;padding:15px;border-left:4px solid #e0a32a}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:15px}.card{padding:16px;background:#fff;border:1px solid #d0dbe5;border-radius:10px}</style><body>
    <h1>828KF 后端保护、工程修复与量化验收</h1>
    <p>FAST-LIO2 + 关键帧 + 保守回环 + iSAM2。仅本机离线研发；不连接164、不发运动命令、不覆盖原数据、主install或导航地图。</p>
    <div class="cards"><div class="card good"><b>工程缺陷：已修复并回归</b><br>必要端点身份、退出、协方差发布与可复现新构建。</div><div class="card"><b>归因：新因子交互已验证</b><br>历史精确因子缺失，无法认证旧图删边因果。</div><div class="card bad"><b>整体几何：FAIL</b><br>中段低面明显恢复，wall125 P95仍退步，候选默认关闭。</div><div class="card bad"><b>全长原速完整性：FAIL</b><br>发布源覆盖完整，但callback及新录Bag缺样；最终补丁短程通过。</div></div>'''
    b+=section('先看实际收益与退步', '<p>候选四个中段低面接缝中位数从10.97/12.71/14.09/10.38cm降至0.66/1.37/1.31/1.20cm；超过三次独立重算的ROI中位数重复包络（最大0.021cm）及预定工程门槛。它恢复的是相对几何代理，raw不是绝对真值。</p><p class="bad">墙面保护没有通过：Y≈−125m，同131个query点的P95从41.11cm升至55.75cm（+14.65cm，超过允许+5cm）；中位数虽由17.42cm降至5.58cm，不能掩盖尾部退步。邻域有81/131个身份变化，物理表面对应仍未知，保留FAIL。</p><img src="geometry/improvement_and_failure.png" alt="主低面收益与墙面失败"><img src="geometry/main_floor_comparison.png" alt="同点身份同尺度局部地面比较，纵轴放大">'+table(['区域','共同格数','raw 中位/P95 cm','旧优化 中位/P95 cm','新候选 中位/P95 cm'],rows)+'<p>Y≈−175/−315m墙面共同支撑不足，保持UNKNOWN。所有8区保留，无局部ICP、删困难区、Z裁剪或地面压平。斜面法向和高度自由；高度在同一固定参考位置沿同次INIT-up计算。候选返程邻域中位由旧图0.697升至1.591cm，仍在预定+1cm保护范围内，但这部分没有保留原图的全部收益。</p>')
    b+=section('冻结图八组合与历史复现边界','<p>bit0=31→0；bit1=113→82；bit2=788→90。固定828KF raw/body，不重跑前端改变直接对照。每组三条新测量完成全部8组合，三次独立配准；固定同一测量重放的8组CSV逐字节一致。</p>'+table(['mask','保留边bit','Y−75 cm','Y−125 cm','Y−175 cm','Y−225 cm','整体验收'],maskrows)+'<p class="note">历史审计1054文件未找到原始精确SE3因子。新all3与历史optimized的RMS约0.343m、最大约0.672m，历史精确复现失败。没有从最终位姿、修正模长或六位日志反推伪造因子。上表只证明新测量集合中的交互，不能据此认定原113或788是假回环。</p><p>新集合中113与788共同入图放大中段接缝；单条/其他组合没有同量级退步。三条Huber实际权重均为1：小白化残差和当前节点小修正无法保证远处几何安全。小角度沿长里程杆臂传播、再由晚期约束分配修正，能产生较大区域位移；单条物理测量是否正确仍缺证据。</p>'+link('frozen/historical_factor_availability.json','历史精确因子可用性审计')+' · '+link('frozen/run1/new_measurements.csv','17位精度新测量')+' · '+link('frozen/three_run_repeat_summary.json','独立配准与固定求解重复')+' · '+link('geometry/repeats123/report.json','全部24组几何结果'))
    b+=section('实际代码修复与候选机制','<ul><li>前端按必要、兼容DDS端点身份检查，允许合法额外调试订阅；RUNNING意外断连与STOPPING预期断连分开。信号handler只设置flag，当前回调结束后停止新入口、排空日志、关闭ROS；错误返回非零。后端invalid-keyframe后SIGINT的真实回归由exit0修为exit2，正式退出前检查在途任务并flush/close两类因子账本。</li><li>Odometry先组装同一时刻位姿与协方差再发布。修正[position,rotation]块顺序，IKFoM右体轴旋转切空间映射至ROS固定世界轴，交叉块同步变换。该接口缺陷不被宣称为地图接缝根因。</li><li>进程supervisor保存真实wait结果、绑定身份和日志closed/flush；快速退出与spawn失败分别记录，不伪造exec身份，超时只向创建后立即捕获的owned pidfd发送SIGTERM；显式恢复SIGCHLD默认，避免继承SIG_IGN把child7错记为0。stop.json不再用PID消失表示正常退出。</li><li>精确因子账本记录方向、SE3、旋转rad/平移m噪声、原始/白化/鲁棒残差、实际权重和提交序；配准支持扫描、初值、粗/精变换另存sidecar。</li><li>候选默认关闭：深拷贝iSAM2试优化，正式图无副作用；只用当时已有、非配准训练扫描的有限自由平面作保护。UNKNOWN/REJECT不入图，不按ID黑名单，不改Q、Huber尺度或统一降权。</li><li>v1因早期锚点占满预算而漏检，保留失败。v2只修正均匀时间/空间覆盖预算，3cm/5cm/1°保护阈值不变；三次均保留前两条并拒绝第三条。本地保护通过仍不能代替独立外部几何验收。</li></ul><p>保持用户指定false/T=[−.00862,.00914,.03916]/R=I/offset=0、Depth100 BEST_EFFORT、B握手和v4质量门。参数来源为用户指定配置，物理标定精度未认证；P/R/J、增强回环、新重力软约束未启用。</p>')
    b+=section('完整原速回放、保存与退出证据',table(['运行（均原速）','请求时长s；0=全长','实际发布输入','最终POST','保存KF','正式验收'],runs)+'<p>原始现场Bag总时长830.667s（166023 IMU、8188点云），828KF Bundle只是794.8s保存前缀；不要混成旧639s运动Bag。回放保留整数header和逐点时间，仅发布raw两话题，域87、TCP loopback。原生地图、状态、日志、录包均独立保存。</p><p class="note">live input clock/network gate与前台record_manager未在该离线入口执行，保存报告中相应capture_degraded仍保留；不能把离线通过写成现场全准入通过。最终状态与输入完整性见下面独立账本。</p>'+link('tests/offline_smoke_010/result.json','完整回放010结果')+' · '+link('RUN_RESULTS.json','统一运行与身份索引'))
    b+=section('原始现场Bag与callback的双向差异',f'<p>同一callback覆盖区间：callback {original["a_count"]}，Bag {original["b_count"]}，共同{original["matched_count"]}；callback-only <b>{original["a_only_count"]}</b>，Bag-only <b>{original["b_only_count"]}</b>。不能相减说成净少4条，更不能称为164驱动丢包率。</p><p>新回放另外对账：原Bag→发布→callback→再录Bag，并保留重复样本重数与每个整数时间戳；exit0或无cache-loss不是完整性证明。</p>'+link('original_field_input_accounting.json','原现场双向精确账本'))
    b+=section('地图、对照图与可复现交付','<p>'+link('frozen/bundles/guarded_v2_candidate_run1/global_map.pcd','候选原生PCD（整体FAIL，实验保留）')+' · '+link('frozen/bundles/guarded_v2_candidate_run1/metadata.yaml','完整Bundle目录元数据')+' · '+link('frozen/bundles/diagnostic_mask5_run1/global_map.pcd','mask5诊断PCD（不能用于ID剔边上线）')+'</p><p>候选828帧与6512118点；所有原始body云、raw index、描述子和配置保留，新的优化位姿统一作用于全局点云。836项manifest校验通过；全局地图抽样一致性通过，不能据此认证精度。</p><p>'+link('geometry/viewer.html','离线同点三维交互对照')+' · '+link('geometry/RESULTS_zh.txt','全部8区、墙面和重复性中文表')+' · '+link('geometry/final_bundle_consistency.json','Bundle与点身份校验')+' · '+link('../../experiments/field_backend_repair_20261023/RUNBOOK_ZH.txt','唯一新构建与现场操作命令')+'</p><p>viewer内嵌JS、无CDN，显示1万相同原始点身份；度量使用完整冻结评估点。静态HTML/JS检查通过；自动浏览器file://策略阻止了打开，未认证浏览器交互通过。</p>')
    b+=section('结果分类与下一次现场重点','<p><b>已确认：</b>工程缺陷修复、精确新因子账本与8组合、事务试算隔离、候选完整Bundle、四中段低面的重复性收益。<b>高概率机制：</b>早期角度约束与晚期约束交互，通过长里程杆臂分配修正。<b>未知：</b>历史原始因子、哪条物理误配、墙面真实对应、独立高程与姿态真值、真实网络/ARM64/运动验收。</p><p>推荐仅保留工程可靠性修复，几何guard继续关闭；原地图不覆盖、候选不切导航。下次先静止/手控短闭环，保存精确每条回环测量、支持扫描、提交前后快照和整数输入账本；重点复查中段往返低面、wall125与返程邻域，不按回环数量判断安全。命令以本次根目录RUNBOOK为准，嵌套历史build_003文档不是入口。</p>')
    b+=section('正式输入完整性、性能与失败证据','<p>013实际发布166023条IMU / 8188帧点云，已逐整数时间戳及重数确认覆盖原Bag全时长；前端adapted callback161271（缺4752），新录Bag IMU164740（缺1283）、点云8048（缺140）。callback-only895 / Bag-only4364；不能相减成净差，也不能解释成164驱动丢包率。录包累计告警最终1487条，重复打印值不能累加。</p><p>013保存844KF、8025条POST，received=processed=844、pending=0、active=none，五个真实退出码0；全POST到发布/ingress逐时间戳也通过。以上均不能替代输入完整性，正式门仍FAIL。</p><p class="note">013期间并行执行大Bag只读核算，产生额外I/O负载。这是本次测试安排的干扰，故不能用013认证孤立实时性能，也不能将010/013差异归因于发布QoS修正。010使用BE5发布；011/013/014恢复原Bag记录的IMU RELIABLE20、点云RELIABLE10发布QoS，前端订阅始终BE100；没有把前端改为RELIABLE。</p><p>013云发布返回→POST（scan_begin±1000ns唯一关联）中位23.06ms / P95 63.50ms / max1540.02ms，含等待和调度，非纯算法耗时。发布节拍偏离P95 14.35ms / max3104.53ms；前端CPU P95约66.85%单逻辑核、RSS max348.74MiB，后端RSS max654.86MiB。POST行group_last_imu_ns未填有效值，未伪造IMU触发耗时。纯服务时间/持续内部队列深度未完整观测。</p><p>最终014原速20秒：4000 IMU发布=callback=录包、189云发布=录包，176 POST / 9KF，实际5退出0；011原速120秒23964 IMU完整。最终C++错误退出和Python快速退出补丁仅在014及专项回归验证，不能追溯算成013验证。剩余磁盘约5.4GiB，低于另一轮完整录包所需空间；原始及失败数据未删除。</p>'+link('tests/offline_smoke_013/ACCEPTANCE_v2.json','013正式失败账本')+' · '+link('tests/offline_smoke_014/ACCEPTANCE_v2.json','014最终短程正式通过')+' · '+link('tests/offline_smoke_013/performance.json','013受干扰性能记录')+' · '+link('tests/backend_fault_green_001/result.json','后端故障退出回归'))
    b+=section('实际构建、测试与一键复现','<p>独立CMake/ament构建全部5包exit0；709个有界源文件身份含.cc/.msg/.srv及构建输入。部分构建必须覆盖变更依赖闭包，不能用frontend-only给旧backend重新背书。执行时保存实际二进制、运行库与配置SHA。原git diff、submodule及冻结输入哈希保持不变。</p><p>测试：mapping16个CTest套件、map4个套件；frontend21项、快速退出13项、输入/身份16项、几何6项，均通过。实际DDS owner移除负向测试前端和ingress均返回2；同前缀额外调试订阅不能顶替同会话UUID身份。旧launch测试未直接执行，使用显式会话的实际loopback集成与fault回归替代，不宣称全部仓库测试通过。</p><pre>cd /home/bh/A2W_FastLio2_controlled\npython3 experiments/field_backend_repair_20261023/reproduce.py my_fresh_repro_001</pre><p>该入口实际执行冻结8组合、guard8组合、完整Bundle/PCD与固定8区评估。已用 --reuse-build 完整运行 final_entry_001，几何FAIL按约定返回2并保留全部输出；不是脚本异常。默认会从本轮源码重新构建；--commands-only可打印命令。全长原速可另加 --full-replay-id 020，要求至少35GiB空闲；运行后独立正式门，禁止在回放时并行大Bag核算。</p><p>'+link('../../experiments/field_backend_repair_20261023/RUNBOOK_ZH.txt','按终端的现场操作命令')+' · '+link('build/build_manifest.json','最终编译身份')+' · '+link('CODE_CHANGES.json','实际代码变更清单')+' · '+link('CODE_CHANGES.patch','代码差异')+' · '+link('CODE_REPRODUCE.tar.gz','配套源码与复现工具包')+' · '+link('FINAL_PRESERVATION.json','原修改与冻结资产保留核验')+'</p>')
    b+=section('整图同点侧视与交付边界','<img src="geometry/whole_native_yz_xz.png" alt="相同一万点身份的原生YZ和XZ侧视"><p>每行各版本共用坐标范围，纵轴放大仅用于显示。局部完整点集侧视：'+link('geometry/guarded_v2_run1/raw_floor_sections.png','raw八区')+' · '+link('geometry/guarded_v2_run1/old_optimized_floor_sections.png','旧优化八区')+' · '+link('geometry/guarded_v2_run1/guarded_v2_run1_floor_sections.png','候选八区')+'。没有将全图Z跨度当作误差。</p><p>功能工程链及冻结图候选已交付；整体几何改善、全长原速输入完整性没有通过，不能结论为整体任务全部验收成功。候选、失败v1和两轮全长数据均保留，未切换生产或导航地图。本轮代码保存在独立目录，未将本轮修改自动推送GitHub。</p>')
    b+='</body></html>' 
    (OUT/'REPORT_ZH.html').write_text(b)
    print(OUT/'REPORT_ZH.html')

if __name__=='__main__':main()
