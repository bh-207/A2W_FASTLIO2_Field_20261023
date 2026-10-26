"""Measured short-interval LiDAR/gyro disagreement, no estimated offset/unit fix."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,RotationSpline
from consistent_init import _integrate_all


def run(poses_path,imu_path,candidate_path,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    q=np.load(poses_path);c=json.loads(Path(candidate_path).read_text());origin=int(q['stamp_ns'][0]);t=(q['stamp_ns']-origin)*1e-9
    rows=list(csv.DictReader(open(imu_path)));it=(np.array([int(x['header_ns'])for x in rows],np.int64)-origin)*1e-9
    a=np.array([[float(x[k])for k in ['ax','ay','az']]for x in rows]);w=np.array([[float(x[k])for k in ['gx','gy','gz']]for x in rows])
    inside=(it>=t[0]-.01)&(it<=t[-1]+.01);it,a,w=it[inside],a[inside],w[inside]
    R=RotationSpline(t,Rotation.from_matrix(q['R']));records=[]
    for start in np.arange(0,t[-1]-1,1.):
        s=_integrate_all(it,a,w,[start,start+1],np.r_[np.zeros(6),c['bg_I'],np.zeros(3)],1.,np.zeros(3),.035,.002,False)[0]
        predicted=Rotation.from_quat(s[-1,3:7]);observed=R(start).inv()*R(start+1);error=(predicted.inv()*observed).as_rotvec()
        records.append(dict(start_s=float(start),rotation_error_deg=float(np.linalg.norm(error)*180/np.pi),
            error_rotvec_I_start_deg=(error*180/np.pi).tolist(),
            lidar_relative_rotvec_deg=(observed.as_rotvec()*180/np.pi).tolist(),gyro_relative_rotvec_deg=(predicted.as_rotvec()*180/np.pi).tolist()))
    records_sorted=sorted(records,key=lambda x:x['rotation_error_deg'],reverse=True)
    result=dict(frame='right rotation discrepancy at interval start, not Euler angles',
        source=str(Path(poses_path).resolve()),bg_I=c['bg_I'],interval_s=1.,windows=records,top_windows=records_sorted[:8],
        raw_IMU_maximum_gap_s=float(np.max(np.diff(it))),raw_IMU_gaps_above20ms=int(np.sum(np.diff(it)>.02)),
        physical_attribution='UNKNOWN: registered LiDAR rotation vs raw gyro plus fixed model; neither declared independent truth')
    (output/'angular_windows.json').write_text(json.dumps(result,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for j,(ax,name) in enumerate(zip(axes,['x','y','z'])):
        ax.plot([x['start_s']for x in records],[x['lidar_relative_rotvec_deg'][j]for x in records],label='LiDAR-only reference')
        ax.plot([x['start_s']for x in records],[x['gyro_relative_rotvec_deg'][j]for x in records],label='raw gyro minus fitted bg')
        ax.set_ylabel(name+' rotation vector deg / 1s');ax.grid(alpha=.3);ax.legend()
    axes[-1].set_xlabel('seconds after first window scan END');fig.tight_layout();fig.savefig(output/'angular_interval_comparison.png',dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('poses');p.add_argument('imu');p.add_argument('candidate');p.add_argument('output');a=p.parse_args();run(a.poses,a.imu,a.candidate,a.output)
