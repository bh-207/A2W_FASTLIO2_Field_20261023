#!/usr/bin/env python3
"""One rigid gauge placement, no leveling/ICP and no candidate-dependent fit."""
import argparse,csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def prepare(source,poses,baseline,timing,out):
    if out.exists():raise ValueError('fresh output required')
    n=json.loads(source.read_text());q=np.load(poses)
    if sha(poses)!=n['source_poses_sha256']:raise ValueError('independent pose evidence changed')
    if not n['accepted']:
        n['frontend_world_alignment']={'applied':False,'reason':'candidate rejected; original initialization fallback'}
        out.write_text(json.dumps(n,indent=2)+'\n');return n
    anchor=int(q['scan_ids'][0]);commit=int(n['commit_scan_id'])
    with baseline.open() as f:rows={int(r['source_scan_id']):r for r in csv.DictReader(f)}
    with timing.open() as f:h={int(r['raw_scan_id']):r for r in csv.DictReader(f)}
    r=rows[anchor];R=Rotation.from_quat([float(r[k]) for k in ('qx','qy','qz','qw')]).as_matrix();p=np.array([float(r[k]) for k in ('tx','ty','tz')]);lever=np.array(n['fixed_extrinsic_T']);Rext=np.array(n['fixed_extrinsic_R'])
    A=R@Rext;t=p+R@lever
    for key in ['g_W','v_WI_commit']:n[key]=(A@np.array(n[key])).tolist()
    n['p_WI_commit']=(A@np.array(n['p_WI_commit'])+t).tolist();n['R_WI_commit']=(A@np.array(n['R_WI_commit'])).tolist()
    J=np.eye(18)
    for k in [0,6,9]:J[k:k+3,k:k+3]=A
    C=J@np.array(n['full_covariance'])@J.T;n['full_covariance']=((C+C.T)/2).tolist()
    n['commit_header_stamp_ns']=int(h[commit]['header_ns'])
    n['frontend_world_alignment']={'applied':True,'kind':'single baseline anchor SE3 gauge placement only','anchor_scan_id':anchor,'R':A.tolist(),'t':t.tolist(),'baseline_index':str(baseline.resolve()),'baseline_index_sha256':sha(baseline),'not_gravity_or_motion_truth':True,'fixed_before_geometry_evaluation':True}
    n['input_binding']={'source_imu_sha256':n['source_imu_sha256'],'cloud_timing_sha256':sha(timing),'commit_header_stamp_ns':n['commit_header_stamp_ns']}
    out.write_text(json.dumps(n,indent=2,allow_nan=False)+'\n');return n
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('candidate',type=Path);p.add_argument('poses',type=Path);p.add_argument('baseline_index',type=Path);p.add_argument('timing',type=Path);p.add_argument('output',type=Path);a=p.parse_args();n=prepare(a.candidate,a.poses,a.baseline_index,a.timing,a.output);print(n['accepted'],n['reason'])
