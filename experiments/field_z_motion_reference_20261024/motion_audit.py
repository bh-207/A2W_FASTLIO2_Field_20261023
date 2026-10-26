"""Short-horizon motion and full-XYZ shape evidence; never modifies maps."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,RotationSpline
from scipy.integrate import cumulative_trapezoid
from scipy.interpolate import CubicSpline


def chord_metrics(points):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or not np.all(np.isfinite(p)):
        raise ValueError('finite XYZ anchors required')
    chord=p[-1]-p[0];length=np.linalg.norm(chord)
    if length<=1e-9:raise ValueError('coincident endpoint anchors')
    direction=chord/length;delta=p-p[0];station=delta@direction
    perpendicular=delta-station[:,None]*direction
    v=np.diff(p,axis=0);norm=np.linalg.norm(v,axis=1)
    if np.any(norm<=1e-9):raise ValueError('duplicate consecutive anchors')
    turn=np.degrees(np.arccos(np.clip(np.sum(v[1:]*v[:-1],axis=1)/(norm[1:]*norm[:-1]),-1,1)))
    return dict(chord_length_m=float(length),chord_distance_m=np.linalg.norm(perpendicular,axis=1).tolist(),
        chord_station_m=station.tolist(),segment_length_m=norm.tolist(),
        segment_to_chord_deg=np.degrees(np.arccos(np.clip(v@direction/norm,-1,1))).tolist(),
        turn_angle_deg=turn.tolist())


def window_residual(t,acceleration,p0,v0,actual_end):
    t=np.asarray(t,float);a=np.asarray(acceleration,float)
    if len(t)<2 or np.any(np.diff(t)<=0) or a.shape!=(len(t),3):raise ValueError('invalid integration support')
    dv=cumulative_trapezoid(a,t,axis=0,initial=0.)
    dp=cumulative_trapezoid(dv,t,axis=0,initial=0.)
    elapsed=t[-1]-t[0]
    return dict(position_residual_m=(np.asarray(p0)+v0*elapsed+dp[-1]-actual_end).tolist(),
        velocity_residual_m_s=dv[-1].tolist(),force_double_integral_m=dp[-1].tolist())


def audit(poses_path,imu_path,candidate_path,output):
    out=Path(output);out.mkdir(parents=True,exist_ok=False)
    poses=np.load(poses_path);c=json.loads(Path(candidate_path).read_text());origin=int(poses['stamp_ns'][0])
    t=(poses['stamp_ns']-origin)*1e-9;r=poses['R'];p=poses['p']-np.einsum('nij,j->ni',r,np.array([-.00862,.00914,.03916]))
    rows=list(csv.DictReader(open(imu_path)));it=(np.array([int(x['header_ns'])for x in rows],np.int64)-origin)*1e-9
    a=np.array([[float(x[k])for k in ['ax','ay','az']]for x in rows]);w=np.array([[float(x[k])for k in ['gx','gy','gz']]for x in rows])
    keep=(it>=t[0])&(it<=t[-1]);it=it[keep];a=a[keep];w=w[keep]
    rotation=RotationSpline(t,Rotation.from_matrix(r));rt=rotation(it).as_matrix()
    raw_world=np.einsum('nij,nj->ni',rt,a*c['acc_scale'])
    bias_world=-np.einsum('nij,j->ni',rt,np.array(c['ba_I']));gravity=np.tile(c.get('diagnostic_original_frame_g_W',c['g_W']),(len(it),1))
    model=raw_world+bias_world+gravity
    # A differentiable position curve is a diagnostic only; no trajectory or map
    # is replaced by it. Report finite integrals in addition to derivatives.
    spline=CubicSpline(t,p);lidar_a=spline(it,2)
    gyro_ref=rotation(it,1);gyro_residual=w-np.array(c['bg_I'])-gyro_ref
    train_end=(int(c['training_last_stamp_ns'])-origin)*1e-9
    train_start=(int(c['training_first_stamp_ns'])-origin)*1e-9
    records=[]
    for start in np.arange(0,t[-1]-.5,1.):
        m=(it>=start)&(it<min(start+1,t[-1]));tr=it[m]
        if len(tr)<2:continue
        avg=lambda x:np.trapz(x[m],tr,axis=0)/(tr[-1]-tr[0])
        records.append(dict(start_s=float(start),end_s=float(min(start+1,t[-1])),
            phase='prefix' if start<train_start else 'train' if start<train_end else 'holdout',
            model_acc_W=avg(model).tolist(),raw_specific_force_W=avg(raw_world).tolist(),
            bias_W=avg(bias_world).tolist(),g_W=c.get('diagnostic_original_frame_g_W',c['g_W']),lidar_acc_W=avg(lidar_a).tolist(),
            accel_residual_W=avg(model-lidar_a).tolist(),gyro_residual_I=avg(gyro_residual).tolist()))
    horizons=[]
    for horizon in [.5,1.,2.,5.]:
        for start in np.arange(np.ceil(train_start),t[-1]-horizon,1.):
            end=start+horizon;grid=np.r_[start,it[(it>start)&(it<end)],end]
            force=np.column_stack([np.interp(grid,it,model[:,j])for j in range(3)])
            q=window_residual(grid,force,spline(start),spline(start,1),spline(end))
            dr=Rotation.identity()
            for l,rr in zip(grid[:-1],grid[1:]):
                mid=.5*(l+rr);omega=np.array([np.interp(mid,it,w[:,j])for j in range(3)])-c['bg_I']
                dr=dr*Rotation.from_rotvec(omega*(rr-l))
            observed=rotation(start).inv()*rotation(end)
            q.update(start_s=float(start),end_s=float(end),horizon_s=horizon,
                position_residual_norm_m=float(np.linalg.norm(q['position_residual_m'])),
                rotation_error_deg=float((dr.inv()*observed).magnitude()*180/np.pi))
            horizons.append(q)
    summary=dict(source=str(Path(poses_path).resolve()),candidate=str(Path(candidate_path).resolve()),
        frozen_parameters=True,frame='independent first LiDAR scan',start_training_s=train_start,end_training_s=train_end,
        one_second_windows=records,reset_horizons=horizons,
        interpretation='LiDAR attitude held fixed for force residual; each finite horizon resets to LiDAR position/velocity, not an EKF update. Spline derivatives are descriptive and are not independent truth.')
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    np.savez_compressed(out/'decomposition.npz',t=it,raw_world=raw_world,bias_world=bias_world,
        model_acc=model,lidar_acc=lidar_a,gyro_residual=gyro_residual,train=[train_start,train_end])
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(13,13),sharex=True)
    for j,color in enumerate(['tab:red','tab:green','tab:blue']):
        axes[0].plot([q['start_s']for q in records],[q['accel_residual_W'][j]for q in records],color=color,label='XYZ'[j])
        axes[1].plot([q['start_s']for q in records],[q['gyro_residual_I'][j]for q in records],color=color,label='XYZ'[j])
    for h in [.5,1.,2.,5.]:
        z=[q for q in horizons if q['horizon_s']==h]
        axes[2].plot([q['start_s']for q in z],[q['position_residual_norm_m']for q in z],label=str(h)+' s')
        axes[3].plot([q['start_s']for q in z],[q['rotation_error_deg']for q in z],label=str(h)+' s')
    for ax,label in zip(axes,['1s force residual (m/s2)','1s gyro residual (rad/s)','reset position residual (m)','reset gyro/LiDAR angle (deg)']):
        ax.set_ylabel(label);ax.grid(alpha=.3);ax.axvline(train_end,color='k',linestyle='--');ax.legend()
    axes[-1].set_xlabel('seconds from independent window first scan END')
    fig.suptitle('Frozen rejected parameters: reference motion vs raw IMU; no threshold changed')
    fig.tight_layout();fig.savefig(out/'short_horizon_residuals.png',dpi=145);plt.close(fig)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('poses');p.add_argument('imu');p.add_argument('candidate');p.add_argument('output')
    a=p.parse_args();audit(a.poses,a.imu,a.candidate,a.output)
