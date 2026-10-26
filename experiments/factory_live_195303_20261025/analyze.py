#!/usr/bin/env python3
"""Descriptive same-identity map analysis; no correction, ICP or ground flattening."""
import csv,json,hashlib,math,sys
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
E=Path(__file__).resolve().parent;W=E.parents[1];O=W/'captures/factory_live_195303_20261025_001'
def rows(p):return list(csv.DictReader(p.open()))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def writecsv(p,x):
 if not x:return
 with p.open('w',newline='')as f:w=csv.DictWriter(f,x[0]);w.writeheader();w.writerows(x)
def pcd(p):
 h={}
 with p.open('rb')as f:
  while True:
   line=f.readline().decode().strip()
   if not line or line.startswith('#'):continue
   k,*v=line.split();h[k]=v
   if k=='DATA':offset=f.tell();break
 if h['DATA']!=['binary']:raise ValueError('binary XYZI PCD required: '+str(p))
 dtype=np.dtype([(n,'<'+{'F':'f','U':'u','I':'i'}[t]+s)for n,t,s in zip(h['FIELDS'],h['TYPE'],h['SIZE'])])
 n=int(h['POINTS'][0]);assert p.stat().st_size==offset+n*dtype.itemsize
 a=np.memmap(p,dtype=dtype,offset=offset,shape=(n,),mode='r')
 xyz=np.column_stack([a[k]for k in 'xyz']);return xyz,h
def tf(points,r):return points@Rotation.from_quat([float(r[k])for k in ['qx','qy','qz','qw']]).as_matrix().T+np.array([float(r[k])for k in ['tx','ty','tz']])
def xyzrows(rs,p='t'):return np.array([[float(r[p+k])for k in 'xyz']for r in rs])
def angle(a,b):return float(np.degrees(np.arccos(np.clip(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),-1,1))))
def stats(a):return dict(min=float(np.min(a)),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(np.max(a)))

def plane(a,q):
 # The normal is free within a broad candidate orientation range. This is a
 # geometric low-surface candidate, not an IMU, constant-height or flat prior.
 if len(a)<80:return None
 rng=np.random.default_rng(195303);sample=a[np.linspace(0,len(a)-1,min(1400,len(a)),dtype=int)]
 best=None
 for _ in range(100):
  t=sample[rng.choice(len(sample),3,replace=False)];n=np.cross(t[1]-t[0],t[2]-t[0]);norm=np.linalg.norm(n)
  if norm<1e-8:continue
  n=n/norm
  if n[2]<0:n=-n
  if n[2]<np.cos(np.deg2rad(45)):continue
  d=-n@t[0];mask=np.abs(sample@n+d)<.035
  if best is None or mask.sum()>best[0]:best=(int(mask.sum()),n,d)
 if best is None:return None
 mask=np.abs(a@best[1]+best[2])<.035
 for _ in range(2):
  b=a[mask]
  if len(b)<80:return None
  c=np.mean(b,axis=0);_,s,v=np.linalg.svd(b-c,full_matrices=False);n=v[-1]
  if n[2]<0:n=-n
  d=-n@c;mask=np.abs(a@n+d)<.035
 if n[2]<np.cos(np.deg2rad(45)) or s[1]/np.sqrt(len(b))<.25:return None
 # Bounded support, no arbitrary extrapolation to distant reference positions.
 if np.linalg.norm(q-b[:,:2].mean(axis=0))>4.5:return None
 return dict(normal=n.tolist(),d=float(d),height=float(-(n[:2]@q+d)/n[2]),count=int(mask.sum()),fraction=float(mask.mean()),rms=float(np.sqrt(np.mean((b@n+d)**2))),slope_deg=float(np.degrees(np.arccos(n[2]))))

def input_audit(run,out):
 source={'/lidar_imu':[],'/lidar_points':[]};record={k:[]for k in source}
 for r in csv.DictReader((run/'reconstruction/consumed_raw.csv').open()):
  source[r['topic']].append(int(r['header_ns']));record[r['topic']].append(int(r['record_ns']))
 mappings={s:'/lidar_imu'for s in ['imu_raw','imu_adapted','imu_logged','imu_buffered','imu_synced']}
 mappings.update({s:'/lidar_points'for s in ['cloud_raw','cloud_preprocessed','cloud_buffered','cloud_synced']})
 seen={s:bytearray(len(source[t])+1)for s,t in mappings.items()};counts=Counter();bad=[]
 for r in csv.DictReader((run/'session/frontend_input_stages.csv').open()):
  s=r['stage'];counts[s]+=1
  if s not in mappings:continue
  i=int(r['id']);t=int(r['stamp_ns']);expected=source[mappings[s]]
  if i<1 or i>len(expected) or t!=expected[i-1] or seen[s][i]:
   if len(bad)<100:bad.append(dict(stage=s,id=i,stamp_ns=t))
  else:seen[s][i]=1
 stage_audit={s:dict(expected=len(source[t]),observed=counts[s],unique_matching=sum(seen[s]),missing=len(source[t])-sum(seen[s]))for s,t in mappings.items()}
 info={};gaps=[]
 for topic,stamps in source.items():
  delta=np.diff(np.array(stamps,dtype=np.int64));threshold=20_000_000 if topic=='/lidar_imu'else 150_000_000
  inds=np.flatnonzero(delta>threshold)
  info[topic]=dict(count=len(stamps),first_ns=stamps[0],last_ns=stamps[-1],span_s=(stamps[-1]-stamps[0])*1e-9,median_interval_ms=float(np.median(delta))*1e-6,max_interval_ms=float(delta.max())*1e-6,nonmonotonic=int(np.sum(delta<0)),repeated=int(np.sum(delta==0)),gaps_over_threshold=len(inds))
  for i in inds:gaps.append(dict(topic=topic,before_id=int(i),after_id=int(i+1),before_ns=stamps[i],after_ns=stamps[i+1],gap_ms=float(delta[i])*1e-6))
 status=json.loads((run/'session/frontend_processing_status.json').read_text());log=json.loads((run/'session/runtime_state.csv.status.json').read_text())
 result=dict(source=info,stages=stage_audit,counts=dict(counts),identity_errors=bad,terminal_imu_ids=[i-1 for i in range(1,len(seen['imu_synced']))if not seen['imu_synced'][i]],processing_status=status,runtime_log_status=log)
 required=[s for s in mappings if s!='imu_synced']
 result['callback_buffer_identity_complete']=not bad and all(stage_audit[s]['missing']==0 for s in required)
 save(out/'input_audit.json',result);writecsv(out/'source_gaps.csv',gaps);return result

def main():
 run=O/'runs/B_003';maps=O/'maps/B_003';bundle=maps/'map_bundle';out=O/'analysis';out.mkdir(exist_ok=False)
 audit=input_audit(run,out)
 raw=rows(maps/'raw_keyframe_index.csv');opt=rows(bundle/'optimized_poses.csv');kf=rows(bundle/'keyframes/index.csv')
 assert len(raw)==len(opt)==len(kf) and all(int(r['keyframe_id'])==int(s['id'])for r,s in zip(raw,opt))
 rp,op=xyzrows(raw),xyzrows(opt);t=np.array([int(r['scan_end_ns'])for r in raw],dtype=np.int64);time_s=(t-t[0])*1e-9
 arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(rp,axis=0),axis=1))]
 state=rows(run/'session/runtime_state.csv');init=next(r for r in state if r['stage']=='INIT_COMMIT');post=[r for r in state if r['stage']=='POST_LIDAR_UPDATE']
 g0=np.array([float(init['g_O_'+k])for k in 'xyz']);up=-g0/np.linalg.norm(g0)
 cross=np.cross(up,[0,0,1]);c=float(up[2]);rot=Rotation.identity()if np.linalg.norm(cross)<1e-12 else Rotation.from_rotvec(cross/np.linalg.norm(cross)*np.arccos(c));A=rot.as_matrix()
 gp=xyzrows(post,'g_O_');ba=xyzrows(post,'ba_I_');bg=xyzrows(post,'bg_I_');p=xyzrows(post,'pos_O_I_');v=xyzrows(post,'vel_O_I_')
 ts=(np.array([int(r['state_stamp_ns'])for r in post],dtype=np.int64)-int(post[0]['state_stamp_ns']))*1e-9
 grav_change=np.array([angle(x,g0)for x in gp]);rpy=Rotation.from_quat(np.array([[float(r['q_O_I_'+k])for k in ['x','y','z','w']]for r in post])).as_euler('xyz',degrees=True)
 ext=xyzrows(state,'t_I_L_');extq=np.array([[float(r['q_I_L_'+k])for k in ['x','y','z','w']]for r in state])
 correction=np.linalg.norm(op-rp,axis=1)
 qr=Rotation.from_quat(np.array([[float(r[k])for k in ['qx','qy','qz','qw']]for r in raw]));qo=Rotation.from_quat(np.array([[float(r[k])for k in ['qx','qy','qz','qw']]for r in opt]));drot=np.degrees((qr.inv()*qo).magnitude())
 summary=dict(initial_g_camera_init=g0.tolist(),fixed_up=up.tolist(),initial_gravity_tilt_deg=angle(up,[0,0,1]),gravity_change_max_deg=float(grav_change.max()),gravity_change_final_deg=float(grav_change[-1]),ba_max_norm_m_s2=float(np.linalg.norm(ba,axis=1).max()),bg_max_norm_rad_s=float(np.linalg.norm(bg,axis=1).max()),fixed_translation_max_error_m=float(np.max(np.linalg.norm(ext-np.array([-.00862,.00914,.03916]),axis=1))),fixed_rotation_max_error_deg=float(np.max(np.degrees(Rotation.from_quat(extq).magnitude()))),path_length_m=float(arc[-1]),post_count=len(post),keyframes=len(raw),raw_endpoint_distance_m=float(np.linalg.norm(rp[-1]-rp[0])),optimized_endpoint_distance_m=float(np.linalg.norm(op[-1]-op[0])),trajectory_raw_native_z_span_m=float(np.ptp(p[:,2])),trajectory_raw_fixed_up_span_m=float(np.ptp(p@up)),trajectory_optimized_native_z_span_m=float(np.ptp(op[:,2])),trajectory_optimized_fixed_up_span_m=float(np.ptp(op@up)),backend_translation_correction_m=stats(correction),backend_rotation_correction_deg=stats(drot))
 mapstats={}
 for name,path in [('raw',maps/'frontend_native_raw.pcd'),('optimized',bundle/'global_map.pcd')]:
  cloud,header=pcd(path);finite=np.isfinite(cloud).all(axis=1)
  a=cloud[finite];mapstats[name]=dict(path=str(path),bytes=path.stat().st_size,points=len(cloud),finite_points=int(finite.sum()),bbox_min=a.min(axis=0).tolist(),bbox_max=a.max(axis=0).tolist(),extent=np.ptp(a,axis=0).tolist(),native_z_percentiles={str(q):float(np.percentile(a[:,2],q))for q in [.1,1,5,50,95,99,99.9]},coordinate_frame='camera_init'if name=='raw'else'map',meaning='all mapped structures; not ground elevation error')
  del cloud,a
 summary['map_pcd']=mapstats
 # Freeze sample identities using raw keyframe order, before optimized values.
 sample_ids=list(range(0,len(raw),max(1,len(raw)//130)));cloud_cache={}
 def body(i):
  if i not in cloud_cache:cloud_cache[i]=pcd(bundle/kf[i]['cloud'])[0]
  return cloud_cache[i]
 samples_raw=[];samples_opt=[];profiles=[]
 for i in sample_ids:
  b=body(i);ids=np.arange(0,len(b),max(1,len(b)//1100));points=b[ids]
  samples_raw.append(tf(points,raw[i]));samples_opt.append(tf(points,opt[i]))
  x=tf(b,raw[i])@A.T;origin=rp[i]@A.T;mask=(np.linalg.norm(x[:,:2]-origin[:2],axis=1)<5)&(x[:,2]<origin[2]-.15)&(x[:,2]>origin[2]-3)
  ids_floor=np.flatnonzero(mask);ids_floor=ids_floor[::max(1,len(ids_floor)//4000)]
  frozen=x[ids_floor];f=plane(frozen,origin[:2]);fo=plane(tf(b[ids_floor],opt[i])@A.T,origin[:2])if f else None
  row=dict(keyframe_id=i,source_scan_id=int(raw[i]['source_scan_id']),time_s=float(time_s[i]),path_m=float(arc[i]),reference_x=float(origin[0]),reference_y=float(origin[1]),raw_height=None,opt_height=None,raw_slope_deg=None,opt_slope_deg=None,status='UNKNOWN',support_points=0)
  if f and fo:
   row.update(raw_height=f['height'],opt_height=fo['height'],raw_slope_deg=f['slope_deg'],opt_slope_deg=fo['slope_deg'],status='LOW_PLANE_CANDIDATE_NOT_PHYSICAL_TRUTH',support_points=f['count'])
  profiles.append(row)
  if len(cloud_cache)>5:cloud_cache.clear()
 sraw=np.vstack(samples_raw);sopt=np.vstack(samples_opt)
 np.savez_compressed(out/'same_identity_visualization.npz',raw=sraw,optimized=sopt)
 save(out/'frozen_visualization_scan_ids.json',dict(keyframe_ids=sample_ids,point_selection='every floor(N/1100)-th original saved body point; same IDs and count both maps',no_alignment=True))
 writecsv(out/'local_low_plane_profile.csv',profiles)
 good=[r for r in profiles if r['raw_height']is not None]
 summary['local_low_plane_profile']=dict(valid=len(good),unknown=len(profiles)-len(good),raw_height_span_m=float(np.ptp([r['raw_height']for r in good]))if good else None,optimized_height_span_m=float(np.ptp([r['opt_height']for r in good]))if good else None,classification='automatic local low surfaces; physical ground/slope identity requires inspection; free normals, fixed initial-up direction, no correction')
 # Revisit identities use only raw XY/time/path. No local ICP or manual cropping.
 tree=cKDTree(rp[:,:2]);pairs=[];last_time=-1e9
 for i in range(len(raw)):
  if time_s[i]-last_time<25:continue
  candidates=[j for j in tree.query_ball_point(rp[i,:2],1.5)if j<i and time_s[i]-time_s[j]>90 and arc[i]-arc[j]>30]
  if candidates:
   j=min(candidates,key=lambda j:np.linalg.norm(rp[i,:2]-rp[j,:2]));pairs.append((i,j));last_time=time_s[i]
  if len(pairs)>=24:break
 save(out/'frozen_revisit_pairs.json',[dict(current=i,history=j,raw_xy_distance_m=float(np.linalg.norm(rp[i,:2]-rp[j,:2])),time_gap_s=float(time_s[i]-time_s[j]))for i,j in pairs])
 revisit=[]
 for i,j in pairs:
  b1,b2=body(i),body(j);a1,a2=tf(b1,raw[i]),tf(b2,raw[j]);q=(rp[i]+rp[j])/2
  ids1=np.flatnonzero(np.linalg.norm(a1[:,:2]-q[:2],axis=1)<4);ids2=np.flatnonzero(np.linalg.norm(a2[:,:2]-q[:2],axis=1)<4)
  ids1=ids1[::max(1,len(ids1)//6000)];ids2=ids2[::max(1,len(ids2)//6000)]
  for group in ['all_structure','low_surface_candidate']:
   id1,id2=ids1.copy(),ids2.copy()
   if group=='low_surface_candidate':
    id1=id1[((a1[id1]-rp[i])@up<-.15)&((a1[id1]-rp[i])@up>-3)]
    id2=id2[((a2[id2]-rp[j])@up<-.15)&((a2[id2]-rp[j])@up>-3)]
   row=dict(current=i,history=j,group=group,n_current=len(id1),n_history=len(id2),status='UNKNOWN',raw_nn_median_m=None,raw_nn_p95_m=None,opt_nn_median_m=None,opt_nn_p95_m=None,opt_fixed_raw_correspondence_p95_m=None,neighbor_identity_change_fraction=None,raw_plane_height_difference_m=None,opt_plane_height_difference_m=None,raw_normal_angle_deg=None,opt_normal_angle_deg=None)
   if min(len(id1),len(id2))>=80:
    x,y=a1[id1],a2[id2];xo,yo=tf(b1[id1],opt[i]),tf(b2[id2],opt[j]);d,k=cKDTree(y).query(x);do,ko=cKDTree(yo).query(xo);fixed=np.linalg.norm(xo-yo[k],axis=1)
    row.update(status='GEOMETRIC_CANDIDATE_PHYSICAL_ID_UNKNOWN',raw_nn_median_m=float(np.median(d)),raw_nn_p95_m=float(np.percentile(d,95)),opt_nn_median_m=float(np.median(do)),opt_nn_p95_m=float(np.percentile(do,95)),opt_fixed_raw_correspondence_p95_m=float(np.percentile(fixed,95)),neighbor_identity_change_fraction=float(np.mean(k!=ko)))
    if group=='low_surface_candidate':
     qxy=(q@A.T)[:2];f1,f2=plane(x@A.T,qxy),plane(y@A.T,qxy);o1,o2=plane(xo@A.T,qxy),plane(yo@A.T,qxy)
     if all(v is not None for v in [f1,f2,o1,o2]):row.update(raw_plane_height_difference_m=abs(f1['height']-f2['height']),opt_plane_height_difference_m=abs(o1['height']-o2['height']),raw_normal_angle_deg=angle(f1['normal'],f2['normal']),opt_normal_angle_deg=angle(o1['normal'],o2['normal']))
   revisit.append(row)
  cloud_cache.clear()
 writecsv(out/'revisit_comparison.csv',revisit)
 summary['revisit_candidates']=dict(pairs=len(pairs),valid_metrics=sum(r['status']!='UNKNOWN'for r in revisit),not_physical_loop_certification=True)
 save(out/'summary.json',summary)
 # Equal physical scale and identical visualization points in every view.
 lim=np.array([[min(sraw[:,k].min(),sopt[:,k].min()),max(sraw[:,k].max(),sopt[:,k].max())]for k in range(3)])
 fig,axs=plt.subplots(2,3,figsize=(18,11))
 for rr,(a,label)in enumerate([(sraw,'Frontend RAW'),(sopt,'Backend optimized')]):
  for cc,(ix,iy,title)in enumerate([(0,1,'XY'),(1,2,'YZ'),(0,2,'XZ')]):
   ax=axs[rr,cc];ax.scatter(a[:,ix],a[:,iy],s=.2,c=a[:,2],cmap='viridis',vmin=lim[2,0],vmax=lim[2,1]);ax.set(xlim=lim[ix],ylim=lim[iy],xlabel='XYZ'[ix]+' m',ylabel='XYZ'[iy]+' m',title=label+' '+title);ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.3)
 fig.tight_layout();fig.savefig(out/'map_same_points_views.png',dpi=165);plt.close(fig)
 fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True)
 axs[0].plot(time_s,rp[:,2],label='raw native Z');axs[0].plot(time_s,op[:,2],label='optimized native Z');axs[0].set_ylabel('native m')
 axs[1].plot(time_s,rp@up,label='raw fixed-up height');axs[1].plot(time_s,op@up,label='optimized fixed-up height');axs[1].set_ylabel('fixed-up m')
 axs[2].plot(time_s,correction,label='backend translation correction m');axs[2].plot(time_s,drot,label='backend rotation correction deg');axs[2].set_xlabel('source seconds after first keyframe')
 for ax in axs:ax.legend();ax.grid()
 fig.tight_layout();fig.savefig(out/'trajectory_height_backend.png',dpi=160);plt.close(fig)
 fig,axs=plt.subplots(4,1,figsize=(13,12),sharex=True)
 axs[0].plot(ts,grav_change,label='g relative to initialization deg');axs[0].plot(ts,rpy[:,0],label='roll deg');axs[0].plot(ts,rpy[:,1],label='pitch deg')
 for k in range(3):axs[1].plot(ts,ba[:,k],label='ba_'+'xyz'[k]);axs[2].plot(ts,bg[:,k],label='bg_'+'xyz'[k])
 axs[3].plot(ts,p[:,2],label='native Z');axs[3].plot(ts,p@up,label='fixed-up trajectory height')
 for ax in axs:ax.legend();ax.grid()
 axs[-1].set_xlabel('source seconds after first POST');fig.tight_layout();fig.savefig(out/'state_angles_bias.png',dpi=150);plt.close(fig)
 if good:
  fig,axs=plt.subplots(2,1,figsize=(13,8),sharex=True)
  for name,label in [('raw','raw'),('opt','optimized')]:axs[0].plot([r['path_m']for r in good],[r[name+'_height']for r in good],'.-',label=label+' low-plane height');axs[1].plot([r['path_m']for r in good],[r[name+'_slope_deg']for r in good],'.',label=label+' free local plane slope')
  for ax in axs:ax.grid();ax.legend()
  axs[0].set_ylabel('fixed initial-up height m');axs[1].set_ylabel('degrees');axs[1].set_xlabel('raw keyframe path length m');fig.tight_layout();fig.savefig(out/'local_low_plane_profile.png',dpi=150);plt.close(fig)
 fig=make_subplots(rows=1,cols=2,specs=[[{'type':'scene'}]*2],subplot_titles=['Frontend RAW','Backend optimized'])
 for c,a in enumerate([sraw,sopt],1):fig.add_trace(go.Scatter3d(x=a[:,0],y=a[:,1],z=a[:,2],mode='markers',marker=dict(size=1,color=a[:,2],colorscale='Viridis',cmin=lim[2,0],cmax=lim[2,1]),showlegend=False),row=1,col=c)
 scene=dict(aspectmode='data',xaxis=dict(range=lim[0].tolist(),title='X m'),yaxis=dict(range=lim[1].tolist(),title='Y m'),zaxis=dict(range=lim[2].tolist(),title='Z m'),camera=dict(eye=dict(x=1.3,y=.8,z=.6)))
 fig.update_layout(title='Same body point identities; no ICP, leveling or flattening',height=800,scene=scene,scene2=scene);fig.write_html(out/'map_3d.html',include_plotlyjs=True,full_html=True)
 print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
