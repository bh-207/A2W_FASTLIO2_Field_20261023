"""Post-selected examples for diagnosis, never an independent acceptance set."""
import importlib.util,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
E=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('analysis',E/'analyze.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
O=m.O;out=O/'analysis';base=O/'maps/B_003';bundle=base/'map_bundle'
raw=m.rows(base/'raw_keyframe_index.csv');opt=m.rows(bundle/'optimized_poses.csv');kfs=m.rows(bundle/'keyframes/index.csv')
s=json.loads((out/'summary.json').read_text());up=np.array(s['fixed_up']);axis=np.cross(up,[0,0,1]);A=Rotation.from_rotvec(axis/np.linalg.norm(axis)*np.arccos(up[2])).as_matrix()
pairs=[(978,52),(1635,1072),(1743,17)]
fig,axs=plt.subplots(2,3,figsize=(17,9));evidence=[]
for col,(i,j)in enumerate(pairs):
 rp=np.array([[float(raw[k][a])for a in ['tx','ty','tz']]for k in [i,j]])
 q=rp.mean(axis=0);qf=q@A.T;sets=[];allraw=[];identity=[]
 for k in [i,j]:
  b,_=m.pcd(bundle/kfs[k]['cloud']);r=m.tf(b,raw[k]);pos=np.array([float(raw[k][a])for a in ['tx','ty','tz']])
  ids=np.flatnonzero((np.linalg.norm(r[:,:2]-q[:2],axis=1)<4)&((r-pos)@up<-.15)&((r-pos)@up>-3))
  ids=ids[::max(1,len(ids)//6000)]
  sets.append((r[ids]@A.T,m.tf(b[ids],opt[k])@A.T));allraw.append(r[ids]@A.T);identity.append(dict(keyframe=k,body_point_indices=ids.tolist()))
 points=np.vstack(allraw);_,_,v=np.linalg.svd(points[:,:2]-points[:,:2].mean(axis=0),full_matrices=False);direction=v[0]
 bounds=[]
 for row,label in enumerate(['RAW','OPTIMIZED']):
  ax=axs[row,col]
  for (r,o),k,color in zip(sets,[i,j],['#1871b8','#e76e18']):
   x=r if row==0 else o;u=(x[:,:2]-qf[:2])@direction
   ax.scatter(u,x[:,2],s=1,color=color,alpha=.35,label='KF '+str(k))
   f=m.plane(x,qf[:2])
   if f:
    us=np.linspace(u.min(),u.max(),100);xy=qf[:2]+us[:,None]*direction;n=np.array(f['normal']);z=-(xy@n[:2]+f['d'])/n[2];ax.plot(us,z,color=color,lw=1.5)
   bounds.append([float(x[:,2].min()),float(x[:,2].max())])
  ax.set_title(label+f' KF {i} / {j}');ax.set_xlabel('local horizontal principal direction m');ax.set_ylabel('fixed initial-up height m');ax.legend();ax.grid(alpha=.3)
 lo=min(x[0]for x in bounds);hi=max(x[1]for x in bounds)
 for row in range(2):axs[row,col].set_ylim(lo,hi);axs[row,col].set_xlim(-5,5)
 evidence.append(dict(current=i,history=j,raw_reference_camera_init=q.tolist(),fixed_up_reference=qf.tolist(),horizontal_plot_axis=direction.tolist(),frozen_ids=identity,scope='illustration selected after metrics; physical surface identity not certified; no ICP or height detrending'))
fig.suptitle('Same frozen local body points, free planes; post-selected diagnostic examples. Z is enlarged for visibility.',fontsize=12)
fig.tight_layout();fig.savefig(out/'local_revisit_examples.png',dpi=165);plt.close(fig)
(out/'local_examples_identities.json').write_text(json.dumps(evidence,indent=2))
print('local examples saved')
