"""One explicit gauge placement only. Rejected candidates never reach replay."""
import argparse,csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def prepare(candidate,baseline,timing,output):
    candidate,baseline,timing,output=map(Path,(candidate,baseline,timing,output))
    if output.exists():raise ValueError('fresh prepared output required')
    c=json.loads(candidate.read_text())
    if not c['accepted']:raise ValueError('rejected initialization: no fallback map will be created')
    if c['world_gauge']!='training_first_LiDAR':raise ValueError('explicit training coordinate gauge required')
    if c['extrinsic_est_en'] or c['time_offset']!=0 or not np.allclose(c['fixed_extrinsic_R'],np.eye(3),rtol=0,atol=1e-12) or not np.allclose(c['fixed_extrinsic_T'],[-.00862,.00914,.03916],rtol=0,atol=1e-12):
        raise ValueError('user fixed R/T/false/offset contract differs')
    for key in ['source_poses','source_imu']:
        if hashlib.sha256(Path(c[key]).read_bytes()).hexdigest()!=c[key+'_sha256']:raise ValueError('source evidence changed')
    if c['g_ba_constrained_rank']!=5 or not c['heldout_validation']['passed'] or c['heldout_validation']['used_in_fit']:raise ValueError('candidate validation incomplete')
    rows={int(r['source_scan_id']):r for r in csv.DictReader(baseline.open())};anchor=int(c['world_gauge_scan_id'])
    if anchor not in rows:raise ValueError('exact frozen B009 gauge scan required; no interpolation')
    b=rows[anchor];r=Rotation.from_quat([float(b[k])for k in ['qx','qy','qz','qw']]).as_matrix()
    t=np.array([float(b[k])for k in ['tx','ty','tz']])+r@np.array(c['fixed_extrinsic_T'])
    for key in ['g_W','v_WI_commit']:c[key]=(r@np.array(c[key])).tolist()
    c['p_WI_commit']=(r@np.array(c['p_WI_commit'])+t).tolist();c['R_WI_commit']=(r@np.array(c['R_WI_commit'])).tolist()
    J=np.eye(18)
    for k in [0,6,9]:J[k:k+3,k:k+3]=r
    C=J@np.array(c['full_covariance'])@J.T;c['full_covariance']=((C+C.T)*.5).tolist()
    stamps={int(a['raw_scan_id']):a for a in csv.DictReader(timing.open())};c['commit_header_stamp_ns']=int(stamps[c['commit_scan_id']]['header_ns'])
    c['frontend_world_alignment']=dict(applied=True,kind='single B009 gauge placement, not attitude truth',anchor_scan_id=anchor,R=r.tolist(),t=t.tolist(),baseline_index=str(baseline.resolve()),baseline_index_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest())
    c['input_binding']=dict(source_imu_sha256=c['source_imu_sha256'],cloud_timing_sha256=hashlib.sha256(timing.read_bytes()).hexdigest(),commit_header_stamp_ns=c['commit_header_stamp_ns'])
    output.write_text(json.dumps(c,indent=2,allow_nan=False));return c


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('candidate');p.add_argument('baseline');p.add_argument('timing');p.add_argument('output');a=p.parse_args();prepare(a.candidate,a.baseline,a.timing,a.output)
