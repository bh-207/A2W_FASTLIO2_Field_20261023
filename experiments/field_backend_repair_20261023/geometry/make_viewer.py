#!/usr/bin/env python3
"""Offline 3D display of the same frozen original records in three saved poses."""
import os,sys,json,time
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
sys.dont_write_bytecode=True
import numpy as np
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs
from evaluate import WS,BUNDLE,ROOT,FROZEN,poses,read_csv,sha,write_json,io

OUT=ROOT
CANDIDATE=WS/'captures/field_backend_repair_20261023_001/frozen/bundles/guarded_v2_candidate_run1/optimized_poses.csv'

def main():
 rows=read_csv(BUNDLE/'keyframes/index.csv');selection=OUT/'viewer_point_selection.npz';manifest=OUT/'viewer_display_manifest.json'
 if selection.exists():
  frozen=np.load(selection);body=frozen['body'];ids=frozen['scan_id'];indices=frozen['point_index'];cloudhash=json.loads(manifest.read_text())['cloud_sha256']
  for rel,h in cloudhash.items():
   if sha(BUNDLE/rel)!=h:raise RuntimeError('display input cloud changed')
 else:
  body_parts=[];ids_parts=[];index_parts=[];cloudhash={};remaining=10000
  for i,row in enumerate(rows):
   _,cloud=io.read_pcd(BUNDLE/row['cloud']);xyz=io.xyz(cloud);n=min(len(xyz),remaining//(len(rows)-i));remaining-=n
   ix=np.unique(np.linspace(0,len(xyz)-1,n,dtype=np.int32));body_parts.append(xyz[ix]);ids_parts.append(np.full(len(ix),i,np.int32));index_parts.append(ix);cloudhash[row['cloud']]=sha(BUNDLE/row['cloud'])
  body=np.concatenate(body_parts);ids=np.concatenate(ids_parts);indices=np.concatenate(index_parts)
  np.savez_compressed(selection,body=body,scan_id=ids,point_index=indices)
 policy=json.loads((FROZEN/'selection_manifest.json').read_text());regions=policy['protocol']['regions']
 modes=[('raw','原始 raw',BUNDLE/'keyframes/index.csv','#3274d9'),('old','旧优化 old optimized',BUNDLE/'optimized_poses.csv','#d76a20'),('candidate','guarded v2 候选 · FAIL',CANDIDATE,'#19875a')]
 traces=[];allpositions=[];sourceposes={}
 identities=np.c_[ids,indices].tolist()
 for key,label,path,color in modes:
  pose=poses(read_csv(path));world=np.einsum('nij,nj->ni',pose[ids,:3,:3],body)+pose[ids,:3,3];allpositions.append(world);sourceposes[key]=dict(path=str(path),sha256=sha(path))
  traces.append(go.Scatter3d(x=np.round(world[:,0],4).tolist(),y=np.round(world[:,1],4).tolist(),z=np.round(world[:,2],4).tolist(),mode='markers',name=label,legendgroup=key,marker=dict(size=1.7,color=color,opacity=.65),customdata=identities,hovertemplate=label+'<br>keyframe %{customdata[0]} · PCD record %{customdata[1]}<br>X %{x:.3f} m · Y %{y:.3f} m · Z %{z:.3f} m<extra></extra>'))
  p=pose[:,:3,3];traces.append(go.Scatter3d(x=p[:,0].tolist(),y=p[:,1].tolist(),z=p[:,2].tolist(),mode='lines',name=label+' · 轨迹',legendgroup=key,line=dict(color=color,width=3),hoverinfo='skip'))
 centers=np.array([r['center'] for r in regions]);traces.append(go.Scatter3d(x=centers[:,0].tolist(),y=centers[:,1].tolist(),z=centers[:,2].tolist(),mode='markers+text',name='固定八 ROI 中心',marker=dict(size=4,color='#8855bb',symbol='diamond'),text=['Y '+str(y) for y in [-25,-75,-125,-175,-225,-275,-315]]+['回环区'],textposition='top center',hovertemplate='%{text}<br>旧协议固定位置<extra></extra>'))
 combined=np.concatenate(allpositions);lo=combined.min(0);hi=combined.max(0);pad=np.maximum((hi-lo)*.035,1.);ranges=np.c_[lo-pad,hi+pad].tolist()
 fig=go.Figure(traces);fig.update_layout(template='plotly_white',margin=dict(l=0,r=0,t=10,b=0),legend=dict(orientation='h',yanchor='bottom',y=1.02,x=0),scene=dict(xaxis=dict(title='map X (m)',range=ranges[0]),yaxis=dict(title='map Y (m)',range=ranges[1]),zaxis=dict(title='map Z (m)',range=ranges[2]),aspectmode='data',camera=dict(eye=dict(x=1.6,y=1.1,z=.8),projection=dict(type='orthographic'))),uirevision='fixed-global-camera',showlegend=True)
 figure=json.loads(fig.to_json());source=dict(created_unix_ns=time.time_ns(),display_point_count=len(ids),keyframe_count=len(rows),selection='Uniform original PCD record indices per frame, approximately equal per frame, total10000. Same keyframe/point indices in all3 modes. Display sampling only; metrics use separate full frozen8ROI selection.',point_selection_sha256=sha(selection),cloud_sha256=cloudhash,pose_sources=sourceposes,gauge='saved map XYZ; no alignment/ICP; native Z is not drift truth',same_axes_ranges_m=ranges,body_record_ids=[dict(keyframe_id=i,point_indices=indices[ids==i].tolist()) for i in range(len(rows))],geometry_acceptance='FAIL: wall125P95; wall175/315UNKNOWN',browser_validation='NOT_RUN; parent will inspect browser',rounding='Display world coordinates rounded to0.1mm only; sourcebody cache float64 and original point indices retained')
 # The viewer never fetches assets: Plotly runtime, data, controls and CSS are embedded.
 template='''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>固定输入三维地图对比</title><style>
body{margin:0;background:#f5f7f9;color:#192634;font:15px system-ui,sans-serif}header{padding:18px 24px 8px}h1{font-size:23px;margin:0 0 9px}.status{color:#a32626;font-weight:600}.note{margin:7px 0;color:#445463;line-height:1.55}.controls{display:flex;flex-wrap:wrap;gap:9px;align-items:center;padding:9px 24px}button,select{padding:8px 11px;border:1px solid #b8c3ce;border-radius:5px;background:white;color:#192634;font:inherit}button[aria-pressed=true]{background:#223f60;color:white}#plot{height:73vh;min-height:480px;margin:0 14px;background:white}footer{padding:12px 24px 24px;line-height:1.6;font-size:14px}a{color:#175bb2}#region-note{min-height:22px;padding:2px 24px;color:#3b4d5f}
</style></head><body><header><h1>固定点身份的三维地图对比</h1><div class="status">候选几何验收 FAIL · Y=-125 墙面 P95 保护失败；Y=-175 / -315 墙面未知</div><p class="note">三份地图使用相同的 10,000 个原始 PCD 记录身份与保存的 map gauge。仅作显示抽样，不能用于重算指标。坐标、相机和比例共用；没有 ICP、局部配准或地面压平。map Z 不是漂移真值。</p></header>
<div class="controls" aria-label="地图显示"><button id="all" aria-pressed="true">叠加三份</button><button id="raw" aria-pressed="false">只看原始</button><button id="old" aria-pressed="false">只看旧优化</button><button id="candidate" aria-pressed="false">只看候选</button><label>固定 ROI <select id="roi"><option value="all">全路线</option>__ROI_OPTIONS__</select></label><button id="reset">重置视角</button><button id="top">俯视</button><button id="side">侧视</button></div><div id="region-note">拖拽旋转，滚轮缩放；切换地图保留相机与同一比例。局部抽样较稀疏，请同时看完整冻结 ROI 剖面图。</div><div id="plot" role="img" aria-label="原始、旧优化和失败候选的同点身份三维地图"></div>
<footer><a href="guarded_v2_run1/guarded_v2_run1_floor_sections.png">八 ROI 同比例地面剖面</a> · <a href="guarded_v2_run1/guarded_v2_run1_sections.png">八 ROI 全景剖面</a> · <a href="viewer_display_manifest.json">显示身份与输入 SHA256 清单</a> · <a href="final_bundle/report.json">完整固定评估报告</a><br>定量验收使用全量冻结 ROI 点集，显示点总数与全量地图 6,512,118 点不同。真实物理表面身份和独立真值未知。</footer>
<script>__PLOTLY_JS__</script><script>
const figure=__FIGURE__,regions=__REGIONS__,globalRanges=__RANGES__;
const graph=document.getElementById('plot');let selected='all';
Plotly.newPlot(graph,figure.data,figure.layout,{responsive:true,displaylogo:false,scrollZoom:true});
function show(mode){selected=mode;const visible=figure.data.map((_,i)=>i===6||mode==='all'||Math.floor(i/2)==={raw:0,old:1,candidate:2}[mode]);Plotly.restyle(graph,{visible:visible});for(const name of ['all','raw','old','candidate'])document.getElementById(name).setAttribute('aria-pressed',name===mode?'true':'false');}
for(const name of ['all','raw','old','candidate'])document.getElementById(name).addEventListener('click',()=>show(name));
function focus(){const id=document.getElementById('roi').value;let ranges=globalRanges;if(id!=='all'){const p=regions[Number(id)].center;ranges=[[p[0]-7,p[0]+7],[p[1]-7,p[1]+7],[p[2]-2.5,p[2]+7.5]];}Plotly.relayout(graph,{'scene.xaxis.range':ranges[0],'scene.yaxis.range':ranges[1],'scene.zaxis.range':ranges[2],'scene.aspectmode':'data'});document.getElementById('region-note').textContent=id==='all'?'全路线：三份地图共用固定 XYZ 坐标范围。':regions[Number(id)].name+'：固定原协议中心，所有版本共用 ±7m 水平范围；显示点稀疏，精度结论见剖面与报告。';}
document.getElementById('roi').addEventListener('change',focus);
document.getElementById('reset').addEventListener('click',()=>{document.getElementById('roi').value='all';focus();Plotly.relayout(graph,{'scene.camera':figure.layout.scene.camera});});
document.getElementById('top').addEventListener('click',()=>Plotly.relayout(graph,{'scene.camera':{eye:{x:0,y:0,z:2.5},up:{x:0,y:1,z:0},projection:{type:'orthographic'}}}));
document.getElementById('side').addEventListener('click',()=>Plotly.relayout(graph,{'scene.camera':{eye:{x:2.5,y:0,z:0},up:{x:0,y:0,z:1},projection:{type:'orthographic'}}}));
</script></body></html>'''
 options=''.join('<option value="'+str(i)+'">'+r['name']+'</option>' for i,r in enumerate(regions))
 html=template.replace('__ROI_OPTIONS__',options).replace('__PLOTLY_JS__',get_plotlyjs()).replace('__FIGURE__',json.dumps(figure,separators=(',',':'))).replace('__REGIONS__',json.dumps(regions,separators=(',',':'))).replace('__RANGES__',json.dumps(ranges))
 (OUT/'viewer.html').write_text(html);source['html_sha256']=sha(OUT/'viewer.html');source['html_bytes']=(OUT/'viewer.html').stat().st_size;write_json(manifest,source)
 print('viewer',OUT/'viewer.html','display points',len(ids),'html bytes',source['html_bytes'])
if __name__=='__main__':main()
