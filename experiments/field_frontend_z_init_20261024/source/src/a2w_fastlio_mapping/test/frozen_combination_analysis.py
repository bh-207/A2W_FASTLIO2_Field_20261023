#!/usr/bin/env python3
"""Component interaction of fixed measured factors; no geometry or truth claim."""
import csv,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
root=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
def read(p):
 r=list(csv.DictReader(open(p)));return np.array([[float(x[k]) for k in ['tx','ty','tz']] for x in r]),Rotation.from_quat([[float(x[k]) for k in ['qx','qy','qz','qw']] for x in r])
def stat(x):
 return {'median':np.median(x,axis=0).tolist(),'p95_abs':np.percentile(abs(x),95,axis=0).tolist(),'max_abs':np.max(abs(x),axis=0).tolist()}
allruns={}
for run in [1,2,3]:
 ps=[read(root/f'run{run}'/f'mask_{i}'/'optimized_poses.csv') for i in range(8)]
 t0,r0=ps[0];data={};columns=['id']
 for m,(t,r) in enumerate(ps):
  dt=t-t0;rv=(r0.inv()*r).as_rotvec()
  data[str(m)]={'world_translation_xyz_m':stat(dt),'right_local_rotation_xyz_rad':stat(rv),'id_max_displacement':int(np.argmax(np.linalg.norm(dt,axis=1))),'max_displacement_m':float(np.max(np.linalg.norm(dt,axis=1))),'max_rotation_rad':float(np.max(np.linalg.norm(rv,axis=1)))}
 interaction_t=ps[6][0]-ps[2][0]-ps[4][0]+t0
 rv=[(r0.inv()*x[1]).as_rotvec() for x in ps]
 interaction_r=rv[6]-rv[2]-rv[4]
 data['113_788_interaction']={'world_translation_xyz_m':stat(interaction_t),'right_local_rotation_xyz_rad':stat(interaction_r),'max_translation_norm_m':float(np.max(np.linalg.norm(interaction_t,axis=1))),'max_rotation_norm_rad':float(np.max(np.linalg.norm(interaction_r,axis=1)))}
 with open(out/f'run{run}_correction_components.csv','w') as f:
  w=csv.writer(f);w.writerow(['id','mask','dx_m','dy_m','dz_m','rx_rad','ry_rad','rz_rad'])
  for m,(t,r) in enumerate(ps):
   for i,(a,b) in enumerate(zip(t-t0,rv[m])):w.writerow([i,m,*a,*b])
 allruns[str(run)]=data
(out/'component_interaction.json').write_text(json.dumps({'scope':'CONDITIONAL_COMBINATION_NOT_PHYSICAL_FALSE_LOOP_IDENTIFICATION','rotation':'Log(R_raw.inverse*R_mask) right local','interaction':'mask6-mask2-mask4+mask0 tangent approximation for rotation','runs':allruns},indent=2))
