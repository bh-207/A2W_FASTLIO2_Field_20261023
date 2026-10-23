#!/usr/bin/env python3
"""Fixed-input field geometry evaluator. Pure offline; never runs ROS or ICP.

All body point identities, floor labels, cell labels, INIT up and acceptance
thresholds are frozen before opening any candidate pose CSV. Legacy outputs
retain the original per-mode qualifying masks and per-mode centroid reference.
Common outputs use intersection masks and a fixed grid-center reference.
"""
import argparse,csv,hashlib,json,os,sys,time
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['MPLCONFIGDIR']='/tmp/a2w_field_backend_geometry_mpl'
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from geom_math import fit_plane,height_at,angle

WS=Path(__file__).resolve().parents[3]
SESSION=WS/'captures/field_fixed_sessions/fixed_short_20261023_001'
BUNDLE=SESSION/'bundles/map_bundle'
ORIGINAL=WS/'captures/field_map_review_20261023_001'
ROOT=WS/'captures/field_backend_repair_20261023_001/geometry'
FROZEN=ROOT/'frozen'
import pcd_io as io

ACCEPTANCE={
 'version':'field_geometry_v1_20261023',
 'primary_regions':['corridor_Y_'+str(y) for y in [-75,-125,-175,-225]],
 'minimum_primary_regions_improved':3,
 'primary_median_improvement_over_old_m':.05,
 'primary_median_max_above_raw_m':.03,
 'primary_p95_max_above_raw_m':.05,
 'return_region':'loop_788_90_outside_registration_neighborhood',
 'return_median_max_above_old_m':.01,'return_p95_max_above_old_m':.02,
 'wall_median_max_above_old_m':.02,'wall_p95_max_above_old_m':.05,
 'minimum_common_floor_cells':8,'minimum_common_wall_queries':25,
 'improvement_beyond_repeat_envelope_m':.01,
 'rationale':'Engineering geometric proxy targets frozen before candidate reading; 8 floor cells follows per-plane minimum support order, 25 wall queries follows original wall precondition. Unknown support never counts as pass.',
 'repeat_definition':'Independent complete optimization executions for identical declared input/config; within-pass heldout scatter is not run repeatability.',
 'physical_surface_identity':'unknown; geometric overlap only; no surveyed correspondences or physical ground truth',
 'acceptance_scope':'offline saved fixed-session geometry only; does not authorize deployment, live sensors, motion, or absolute accuracy claims'
}

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def write_json(path,value):
 Path(path).parent.mkdir(parents=True,exist_ok=True)
 Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist())+'\n')

def read_csv(path):
 with Path(path).open() as f:return list(csv.DictReader(f))

def poses(rows,expected=None):
 ids=[int(r['id']) for r in rows]
 if expected is None:expected=list(range(len(rows)))
 if sorted(ids)!=expected or len(set(ids))!=len(ids):raise ValueError('pose ids must match all 828 unique original ids')
 rows=sorted(rows,key=lambda r:int(r['id']));out=np.tile(np.eye(4),(len(rows),1,1))
 out[:,:3,3]=[[float(r['t'+a]) for a in 'xyz'] for r in rows]
 q=np.array([[float(r['q'+a]) for a in 'xyzw'] for r in rows])
 if not np.isfinite(out).all() or not np.isfinite(q).all() or np.any(np.linalg.norm(q,axis=1)<1e-10):raise ValueError('invalid nonfinite/zero-quaternion pose')
 out[:,:3,:3]=Rotation.from_quat(q).as_matrix();return out

def stats(values):
 x=np.asarray(values,float);x=x[np.isfinite(x)]
 if not len(x):return {'count':0}
 return dict(count=len(x),min=float(x.min()),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(x.max()),mean=float(x.mean()))

def freeze():
 FROZEN.mkdir(parents=True,exist_ok=True)
 inputs=[BUNDLE/'keyframes/index.csv',BUNDLE/'optimized_poses.csv',ORIGINAL/'geometry_protocol.json',SESSION/'runtime_state.csv']
 base={str(p):sha(p) for p in inputs}
 acceptance_file=FROZEN/'acceptance_thresholds.json'
 if acceptance_file.exists():
  if json.loads(acceptance_file.read_text())!=ACCEPTANCE:raise RuntimeError('frozen acceptance differs; do not tune after candidates')
 else:write_json(acceptance_file,ACCEPTANCE)
 meta_file=FROZEN/'selection_manifest.json'
 if meta_file.exists():
  m=json.loads(meta_file.read_text())
  if m['input_sha256']!=base:raise RuntimeError('frozen source metadata changed')
  for p,h in m['frozen_file_sha256'].items():
   if sha(FROZEN/p)!=h:raise RuntimeError('frozen point-selection cache changed: '+p)
  for p,h in m['cloud_sha256'].items():
   if sha(BUNDLE/p)!=h:raise RuntimeError('frozen source cloud changed: '+p)
  return m
 raw=read_csv(inputs[0]);tr=poses(raw);to=poses(read_csv(inputs[1]));protocol=json.loads(inputs[2].read_text())
 init=next(r for r in read_csv(inputs[3]) if r['stage']=='INIT_COMMIT')
 g=np.array([float(init['g_O_'+a]) for a in 'xyz']);up=(to[0]@np.linalg.inv(tr[0]))[:3,:3]@(-g/np.linalg.norm(g))
 ex=np.array([1.,0.,0.]);ex-=up*(ex@up);ex/=np.linalg.norm(ex);basis=np.column_stack([ex,np.cross(up,ex),up])
 cache={};cloudhash={};counts={};filehash={}
 for region in protocol['regions']:
  data={};counts[region['name']]={}
  for group in ['train','heldout','revisit']:
   parts={k:[] for k in ['body','scan_id','point_index','floor','cell']}
   for i in region[group]:
    if i not in cache:
     p=BUNDLE/raw[i]['cloud'];cache[i]=io.xyz(io.read_pcd(p)[1]);cloudhash[raw[i]['cloud']]=sha(p)
    body=cache[i];qo=body@to[i,:3,:3].T+to[i,:3,3]
    inside=(np.linalg.norm(body,axis=1)<=12)&(np.linalg.norm(((qo-np.asarray(region['center']))@basis)[:,:2],axis=1)<=6)
    ix=np.flatnonzero(inside);_,unique=np.unique(np.floor(qo[ix]/.12).astype(np.int64),axis=0,return_index=True);ix=ix[np.sort(unique)]
    h=(qo[ix]-to[i,:3,3])@up
    for k,v in dict(body=body[ix],scan_id=np.full(len(ix),i,dtype=np.int32),point_index=ix.astype(np.int32),floor=(h>=-2.5)&(h<=-.15),cell=np.floor((qo[ix]@basis)[:,:2]/.75).astype(np.int64)).items():parts[k].append(v)
   for k,v in parts.items():data[group+'_'+k]=np.concatenate(v)
   counts[region['name']][group]=len(data[group+'_body'])
  name=region['name']+'.npz';np.savez_compressed(FROZEN/name,**data);filehash[name]=sha(FROZEN/name)
 m=dict(frozen_unix_ns=time.time_ns(),input_sha256=base,cloud_sha256=cloudhash,frozen_file_sha256=filehash,point_counts=counts,
        protocol=protocol,up=up,basis=basis,
        common_reference='Fixed .75m grid cell center in INIT-up horizontal basis; X0=.75*(kx+.5)*ex+.75*(ky+.5)*ey. Same X0 for raw, old, all candidates.',
        point_identity='(original keyframe id, original binary/ASCII PCD record index); exact original optimized selection, downsample representative, floor label and cell tag retained',
        candidate_read_count_before_freeze=0)
 write_json(meta_file,m);return m

def transform(data,group,pose):
 body=data[group+'_body'];ids=data[group+'_scan_id']
 return np.einsum('nij,nj->ni',pose[ids,:3,:3],body)+pose[ids,:3,3]

def floor_eval(groups,data,up,basis):
 maps=[]
 for group,q in zip(['train','heldout','revisit'],groups):
  f=data[group+'_floor'];p=q[f];cells=data[group+'_cell'][f];maps.append({tuple(key):p[np.all(cells==key,axis=1)] for key in np.unique(cells,axis=0)})
 shared=set(maps[0])&set(maps[1])&set(maps[2]);records={};failed={}
 for key in sorted(shared):
  fits=[fit_plane(m[key],up) for m in maps]
  if any(f is None for f in fits):failed[str(key)]='not_planar_in_all_three_groups';continue
  normal=angle(fits[0]['normal'],fits[2]['normal']);hold=angle(fits[0]['normal'],fits[1]['normal'])
  ref=(fits[0]['center']+fits[2]['center'])/2
  fixed=basis[:,:2]@((np.array(key)+.5)*.75)
  h=[height_at(f,ref,up) for f in fits];hf=[height_at(f,fixed,up) for f in fits]
  records[key]=dict(cell=list(key),return_height_minus_train_m=h[2]-h[0],holdout_height_minus_train_m=h[1]-h[0],
      fixed_reference_return_height_minus_train_m=hf[2]-hf[0],fixed_reference_holdout_height_minus_train_m=hf[1]-hf[0],
      return_normal_angle_deg=normal,holdout_normal_angle_deg=hold,counts=[f['count'] for f in fits],normal_agreement_le_10_deg=normal<=10 and hold<=10)
 good={k:v for k,v in records.items() if v['normal_agreement_le_10_deg']}
 legacy=floor_summary(good.values(),False);legacy.update(shared_cells=len(shared),planar_cells=len(records),normal_consistent_cells=len(good),failed=dict(not_planar_in_all_three_groups=sum(v=='not_planar_in_all_three_groups' for v in failed.values())),all_planar_cells=list(records.values()),all_shared_cell_keys=[list(k) for k in sorted(shared)],failed_cell_reasons=failed)
 return dict(legacy=legacy,records=records,good=good,failed=failed,shared=shared)

def floor_summary(rows,fixed):
 rows=list(rows);prefix='fixed_reference_' if fixed else ''
 return dict(absolute_return_height_difference_m=stats([abs(r[prefix+'return_height_minus_train_m']) for r in rows]),
  signed_return_height_difference_m=stats([r[prefix+'return_height_minus_train_m'] for r in rows]),
  absolute_holdout_height_difference_m=stats([abs(r[prefix+'holdout_height_minus_train_m']) for r in rows]),
  return_normal_angle_deg=stats([r['return_normal_angle_deg'] for r in rows]),
  holdout_normal_angle_deg=stats([r['holdout_normal_angle_deg'] for r in rows]))

def wall_eval(train,query,up):
 if min(len(train),len(query))<25:return dict(status='unknown',support=np.zeros(len(query),bool),distance=np.full(len(query),np.nan),neighbors=np.empty((len(query),0),int),train_count=len(train),query_count=len(query))
 d,ix=cKDTree(train).query(query,k=12,workers=1);p=train[ix];c=p.mean(1);delta=p-c[:,None,:]
 val,vec=np.linalg.eigh(np.einsum('nki,nkj->nij',delta,delta)/12);n=vec[:,:,0]
 support=(d[:,-1]<=.8)&(val[:,1]>=.0005)&(val[:,0]/np.maximum(val[:,1],1e-12)<=.08)&(np.abs(n@up)<.5)
 distance=np.abs(np.sum((query-c)*n,axis=1))
 return dict(status='measured_local_wall_like_surfaces',support=support,distance=distance,neighbors=ix,
 train_count=len(train),query_count=len(query),nearest_neighbor_distance_all_m=stats(d[:,0]),planar_wall_supported_count=int(support.sum()),planar_wall_support_fraction=float(support.mean()),point_to_local_plane_abs_m=stats(distance[support]))

def public_wall(w):return {k:v for k,v in w.items() if k not in ['support','distance','neighbors']}

def common_views(metrics,labels):
 cells=set.intersection(*(set(metrics[l]['floor']['good']) for l in labels))
 out=dict(labels=labels,common_floor_cell_count=len(cells),common_floor_cells=[list(k) for k in sorted(cells)],variants={})
 for l in labels:
  f=metrics[l]['floor'];v=out['variants'][l]=dict(floor=floor_summary([f['good'][k] for k in sorted(cells)],True))
  v['floor'].update(individual_qualified_count=len(f['good']),qualified_not_common_count=len(set(f['good'])-cells))
 for name in ['wall_return','wall_holdout']:
  mask=np.logical_and.reduce([metrics[l][name]['support'] for l in labels]);old=metrics['old_optimized'][name]
  for l in labels:
   w=metrics[l][name];out['variants'][l][name]=dict(common_supported_query_count=int(mask.sum()),individual_supported_query_count=int(w['support'].sum()),
      point_to_local_plane_abs_m=stats(w['distance'][mask]),support_lost_vs_old=int(np.sum(old['support']&~w['support'])),support_gained_vs_old=int(np.sum(~old['support']&w['support'])),
      common_query_neighborhood_identity_changed_vs_old_count=int(np.sum(np.any(np.sort(w['neighbors'][mask],axis=1)!=np.sort(old['neighbors'][mask],axis=1),axis=1))) if w['neighbors'].shape[1] else 0,
      physical_surface_identity='unknown; same query point identities, dynamic nearest-12 train neighborhoods with identical support criteria')
 return out

def scalar(v,kind,stat):
 d=v['floor']['absolute_return_height_difference_m'] if kind=='floor' else v[kind]['point_to_local_plane_abs_m']
 return d.get(stat)

def acceptance(comparisons,labels,repeat_groups):
 result={}
 for label in labels:
  regions={r['region']:r['comparisons'][label] for r in comparisons};checks=[];improved=[];unknown=[]
  for name in ACCEPTANCE['primary_regions']:
   c=regions[name];v=c['variants'];p=scalar(v[label],'floor','median');r=scalar(v['raw'],'floor','median');o=scalar(v['old_optimized'],'floor','median');p95=scalar(v[label],'floor','p95');r95=scalar(v['raw'],'floor','p95')
   if c['common_floor_cell_count']<8 or None in [p,r,o,p95,r95]:unknown.append(name+':insufficient_common_floor_support');continue
   gain=o-p;improved.append(gain>=.05)
   checks.append(dict(region=name,median_improvement_m=gain,improves_5cm=gain>=.05,raw_median_protection=p<=r+.03,raw_p95_protection=p95<=r95+.05))
  passed=len(improved)==4 and sum(improved)>=3 and all(c['raw_median_protection'] and c['raw_p95_protection'] for c in checks)
  ret=regions[ACCEPTANCE['return_region']];v=ret['variants'];rm=scalar(v[label],'floor','median');om=scalar(v['old_optimized'],'floor','median');rp=scalar(v[label],'floor','p95');op=scalar(v['old_optimized'],'floor','p95')
  returncheck=None if ret['common_floor_cell_count']<8 or None in [rm,om,rp,op] else rm<=om+.01 and rp<=op+.02
  if returncheck is None:unknown.append('return_floor:insufficient_common_support')
  walls=[]
  for name,c in regions.items():
   v=c['variants'];wv=v[label]['wall_return'];wm=scalar(v[label],'wall_return','median');wo=scalar(v['old_optimized'],'wall_return','median');wp=scalar(v[label],'wall_return','p95');wop=scalar(v['old_optimized'],'wall_return','p95')
   ok=None if wv['common_supported_query_count']<25 or None in [wm,wo,wp,wop] else wm<=wo+.02 and wp<=wop+.05
   walls.append(dict(region=name,common_query_count=wv['common_supported_query_count'],passes=ok))
   if ok is None:unknown.append(name+':wall_insufficient_common_support')
  repeatcheck=[]
  families=[(family,members) for family,members in repeat_groups.items() if label in members]
  for family,members in families:
   for name in ACCEPTANCE['primary_regions']:
    rv=regions[name]['repeat_families'][family];vals=[scalar(rv['variants'][l],'floor','median') for l in members];old=scalar(rv['variants']['old_optimized'],'floor','median')
    if rv['common_floor_cell_count']<8 or None in vals or old is None:repeatcheck.append(dict(region=name,status='unknown'));continue
    envelope=max(vals)-min(vals);gain=old-scalar(rv['variants'][label],'floor','median')
    repeatcheck.append(dict(region=name,envelope_m=envelope,envelope_definition='max-minus-min ROI absolute-height median over independent complete optimizer runs on repeat-common qualified cells and fixed reference',improvement_m=gain,exceeds_envelope_plus_1cm=gain>envelope+.01))
  repeat_ok=len(repeatcheck)==4 and sum(r.get('exceeds_envelope_plus_1cm',False) for r in repeatcheck)>=3
  primary_fail=any(not c['raw_median_protection'] or not c['raw_p95_protection'] for c in checks) or (len(improved)==4 and sum(improved)<3)
  repeat_fail=bool(families) and any('envelope_m' in r for r in repeatcheck) and len([r for r in repeatcheck if 'envelope_m' in r])==4 and not repeat_ok
  if families and len([r for r in repeatcheck if 'envelope_m' in r])<4:unknown.append('repeat_floor:insufficient_common_support')
  hard_fail=primary_fail or returncheck is False or any(w['passes'] is False for w in walls) or repeat_fail
  if not families:unknown.append('no_independent_complete_optimization_repeat_group')
  status='FAIL' if hard_fail else ('UNKNOWN' if unknown else 'PASS')
  result[label]=dict(status=status,primary=checks,primary_5cm_improvement_count=sum(improved),return_floor_protection=returncheck,
      wall_protection=walls,repeat_evidence=repeatcheck,unknowns=unknown,physical_accuracy_certified=False)
 return result

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',action='append',default=[],metavar='LABEL=CSV');p.add_argument('--repeat',action='append',default=[],metavar='FAMILY=LABEL1,LABEL2');p.add_argument('--out',type=Path,default=ROOT/'baseline');p.add_argument('--no-plots',action='store_true');args=p.parse_args()
 manifest=freeze() # Must precede opening any new candidate.
 up=np.array(manifest['up']);basis=np.array(manifest['basis']);raw=poses(read_csv(BUNDLE/'keyframes/index.csv'));old=poses(read_csv(BUNDLE/'optimized_poses.csv'));variants={'raw':raw,'old_optimized':old};sources={}
 for item in args.candidate:
  label,path=item.split('=',1)
  if not label or label in variants or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in label):raise ValueError('unique safe candidate label required')
  candidate=poses(read_csv(path),list(range(len(raw))));position_error=np.linalg.norm(candidate[0,:3,3]-old[0,:3,3]);rotation_error=Rotation.from_matrix(candidate[0,:3,:3]@old[0,:3,:3].T).magnitude()
  if position_error>1e-4 or rotation_error>1e-5:raise ValueError('candidate differs from fixed first-pose map gauge: '+label)
  variants[label]=candidate;sources[label]=dict(path=str(Path(path).resolve()),sha256=sha(path),first_pose_position_error_m=position_error,first_pose_rotation_error_rad=rotation_error)
 repeat_groups={}
 for item in args.repeat:
  family,members=item.split('=',1);members=members.split(',')
  if len(members)<2 or len(set(members))!=len(members) or any(m not in sources for m in members):raise ValueError('repeat requires >=2 distinct candidate labels')
  repeat_groups[family]=members
 args.out.mkdir(parents=True,exist_ok=True);report=dict(manifest_sha256=sha(FROZEN/'selection_manifest.json'),acceptance_sha256=sha(FROZEN/'acceptance_thresholds.json'),candidate_sources=sources,repeat_groups=repeat_groups,geometry=[])
 report['repeat_trajectory_differences']={}
 for family,members in repeat_groups.items():
  pairwise=[]
  for a in range(len(members)):
   for b in range(a+1,len(members)):
    pa,pb=variants[members[a]],variants[members[b]]
    pairwise.append(dict(labels=[members[a],members[b]],position_difference_m=stats(np.linalg.norm(pa[:,:3,3]-pb[:,:3,3],axis=1)),rotation_difference_deg=stats(np.degrees(Rotation.from_matrix(pa[:,:3,:3]@pb[:,:3,:3].transpose(0,2,1)).magnitude()))))
  report['repeat_trajectory_differences'][family]=pairwise
 csvrows=[];plot_panels={l:[] for l in variants}
 for region in manifest['protocol']['regions']:
  name=region['name'];data=np.load(FROZEN/(name+'.npz'));metrics={};detail={}
  for label,pose in variants.items():
   groups=[transform(data,g,pose) for g in ['train','heldout','revisit']];floor=floor_eval(groups,data,up,basis)
   walls=[q[~data[g+'_floor']] for q,g in zip(groups,['train','heldout','revisit'])]
   metrics[label]=dict(floor=floor,wall_return=wall_eval(walls[0],walls[2],up),wall_holdout=wall_eval(walls[0],walls[1],up))
   qold=[transform(data,g,old) for g in ['train','heldout','revisit']]
   detail[label]=dict(floor=floor['legacy'],wall_return=public_wall(metrics[label]['wall_return']),wall_holdout=public_wall(metrics[label]['wall_holdout']),group_point_counts=[len(q) for q in groups],
      same_point_horizontal_movement_from_frozen_old_m=stats(np.concatenate([np.linalg.norm(((q-qo)@basis)[:,:2],axis=1) for q,qo in zip(groups,qold)])),
      frozen_cell_current_grid_tag_change_fraction=float(np.mean(np.concatenate([np.any(np.floor((q@basis)[:,:2]/.75).astype(np.int64)!=data[g+'_cell'],axis=1) for q,g in zip(groups,['train','heldout','revisit'])]))),
      frozen_cell_current_grid_displacement_m=stats(np.concatenate([np.linalg.norm((q@basis)[:,:2]-(data[g+'_cell']+.5)*.75,axis=1) for q,g in zip(groups,['train','heldout','revisit'])])))
   plot_panels[label].append((region,groups,[data[g+'_floor'] for g in ['train','heldout','revisit']]))
  labels=list(variants);common=common_views(metrics,labels);comparisons={}
  for label in sources:
   comparisons[label]=common_views(metrics,['raw','old_optimized',label]);comparisons[label]['repeat_families']={}
   for family,members in repeat_groups.items():
    if label in members:comparisons[label]['repeat_families'][family]=common_views(metrics,['raw','old_optimized']+members)
  record=dict(region=name,legacy=detail,common_all_variants=common,comparisons=comparisons)
  report['geometry'].append(record);write_json(args.out/(name+'.json'),record)
  # Exact support and neighbor masks are retained for audit, including unsupported identities.
  masks={}
  for label,m in metrics.items():
   for wall in ['wall_return','wall_holdout']:
    masks[label+'_'+wall+'_supported']=m[wall]['support'];masks[label+'_'+wall+'_neighbors']=m[wall]['neighbors'];masks[label+'_'+wall+'_distance']=m[wall]['distance']
  np.savez_compressed(args.out/(name+'_wall_support.npz'),**masks)
  for label in labels:
   for method,metric in [('legacy',detail[label]['floor']),('common_all',common['variants'][label]['floor'])]:
    s=metric['absolute_return_height_difference_m'];csvrows.append(dict(region=name,label=label,method=method,cells=s['count'],median_m=s.get('median'),p95_m=s.get('p95'),normal_median_deg=metric['return_normal_angle_deg'].get('median')))
  for candidate,comparison in comparisons.items():
   for label,metric in comparison['variants'].items():
    metric=metric['floor'];s=metric['absolute_return_height_difference_m'];csvrows.append(dict(region=name,label=label,method='common_triplet_'+candidate,cells=s['count'],median_m=s.get('median'),p95_m=s.get('p95'),normal_median_deg=metric['return_normal_angle_deg'].get('median')))
  print('evaluated',name,'labels',','.join(labels),flush=True)
 report['acceptance']=acceptance(report['geometry'],list(sources),repeat_groups)
 report['limitations']=['Physical same-surface identity unknown; geometric proxies do not certify true accuracy.','Frozen point and cell labels retain identities but candidates can move points relative to original cell footprint; displacement reported.','Dynamic wall neighborhoods can change train identities; common query identities and neighbor change counts reported.','Walls with <25 common supported queries remain unknown; floor fits retain all failed/unsupported cells.','No per-ROI ICP, flattening, absolute-Z penalty, or redefinition of INIT up. Genuine slope remains free.','Repeat envelope requires independent optimization executions; within-pass heldout scatter is separately reported.']
 write_json(args.out/'report.json',report)
 with (args.out/'summary.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(csvrows[0]));w.writeheader();w.writerows(csvrows)
 if not args.no_plots:plots(args.out,plot_panels,basis)
 print(json.dumps({label:dict(status=a['status'],primary_5cm_improvement_count=a['primary_5cm_improvement_count'],return_floor_protection=a['return_floor_protection'],wall_failures=[w['region'] for w in a['wall_protection'] if w['passes'] is False],unknowns=a['unknowns']) for label,a in report['acceptance'].items()},ensure_ascii=False,indent=2))

def plots(out,panels,basis):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 for label,regions in panels.items():
  fig,axes=plt.subplots(4,2,figsize=(15,14))
  for ax,(region,groups,floor_masks) in zip(axes.flat,regions):
   center=np.array(region['center'])
   for q,color,g in zip(groups,['tab:blue','tab:green','tab:orange'],['train','heldout','return']):
    step=max(1,len(q)//4500);local=(q[::step]-center)@basis;ax.scatter(local[:,1],local[:,2],s=.4,alpha=.45,color=color,label=g)
   ax.set(xlim=(-6,6),ylim=(-3,7),title=region['name'].replace('_outside_registration_neighborhood','\noutside registration neighborhood'),xlabel='Fixed horizontal axis 2 (m)',ylabel='Fixed INIT-up height (m)');ax.set_aspect('equal')
  axes[0,0].legend(markerscale=5,fontsize=8);fig.suptitle(label+'; same identities / same scale / no local alignment');fig.tight_layout();fig.savefig(out/(label+'_sections.png'),dpi=140);plt.close(fig)
  fig,axes=plt.subplots(4,2,figsize=(15,11))
  for ax,(region,groups,floor_masks) in zip(axes.flat,regions):
   center=np.array(region['center'])
   for q,f,color,g in zip(groups,floor_masks,['tab:blue','tab:green','tab:orange'],['train','heldout','return']):
    q=q[f];step=max(1,len(q)//4500);local=(q[::step]-center)@basis;ax.scatter(local[:,1],local[:,2],s=.4,alpha=.45,color=color,label=g)
   ax.set(xlim=(-6,6),ylim=(-1.2,.4),title=region['name'].replace('_outside_registration_neighborhood','\noutside registration neighborhood'),xlabel='Fixed horizontal axis 2 (m)',ylabel='Fixed INIT-up height (m)');ax.grid(alpha=.2)
  axes[0,0].legend(markerscale=5,fontsize=8);fig.suptitle(label+' floor zoom; same axes for every ROI and variant; vertical axis magnified');fig.tight_layout();fig.savefig(out/(label+'_floor_sections.png'),dpi=140);plt.close(fig)

if __name__=='__main__':main()
