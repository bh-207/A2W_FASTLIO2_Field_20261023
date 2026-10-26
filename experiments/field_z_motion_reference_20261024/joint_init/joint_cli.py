"""Offline batch candidate: fit, genuinely held-out prediction, commit at end.

The future validation samples are available only at final commit. No candidate
may be injected at the earlier training endpoint and called causally validated.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from joint_math import JointInput,solve_joint
from lidar_registration import skew


def propagate_state(state,imu_t,acc,gyro,end_times,acc_scale,accel_sigma,gyro_sigma):
    state={k:(v.copy() if isinstance(v,np.ndarray) else v) for k,v in state.items()}
    result=[]
    for stop in end_times:
        if stop<state["t"] or stop>imu_t[-1] or state["t"]<imu_t[0]:raise ValueError("propagation timestamps not bracketed")
        nodes=np.r_[state["t"],imu_t[(imu_t>state["t"])&(imu_t<stop)],stop]
        for left,right in zip(nodes[:-1],nodes[1:]):
            dt=right-left
            if dt<=0:continue
            if dt>.05:raise ValueError("raw IMU propagation gap")
            mid=.5*(left+right)
            a=np.array([np.interp(mid,imu_t,acc[:,j]) for j in range(3)])*acc_scale-state["ba"]
            w=np.array([np.interp(mid,imu_t,gyro[:,j]) for j in range(3)])-state["bg"]
            rotation_mid=state["R"]@Rotation.from_rotvec(.5*w*dt).as_matrix()
            acceleration=rotation_mid@a+state["g"]
            state["p"]+=state["v"]*dt+.5*acceleration*dt**2;state["v"]+=acceleration*dt
            state["R"]=state["R"]@Rotation.from_rotvec(w*dt).as_matrix()
            f=np.zeros((18,18));f[:3,6:9]=np.eye(3);f[3:6,3:6]=-skew(w);f[3:6,15:18]=-np.eye(3)
            f[6:9,3:6]=-rotation_mid@skew(a);f[6:9,9:12]=np.eye(3);f[6:9,12:15]=-rotation_mid
            phi=np.eye(18)+f*dt+.5*(f@f)*dt**2
            ga=np.zeros((18,3));ga[:3]=.5*rotation_mid*dt**2;ga[6:9]=rotation_mid*dt
            gg=np.zeros((18,3));gg[3:6]=np.eye(3)*dt
            state["cov"]=phi@state["cov"]@phi.T+ga@ga.T*(acc_scale*accel_sigma)**2+gg@gg.T*gyro_sigma**2
            state["cov"]=(state["cov"]+state["cov"].T)*.5
        state["t"]=float(stop)
        result.append({k:(v.copy() if isinstance(v,np.ndarray) else v) for k,v in state.items()})
    return result


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(poses_path,imu_csv,output,acc_scale,train_fraction=.75,accel_sigma=.035,gyro_sigma=.002,
        maximum_holdout_position=.5,maximum_holdout_rotation=.05,maximum_fit_seconds=30.):
    if not np.isfinite(train_fraction) or not 0.<train_fraction<1.:
        raise ValueError("train_fraction must leave a nonempty independent holdout")
    for value in (acc_scale,accel_sigma,gyro_sigma,maximum_holdout_position,maximum_holdout_rotation,maximum_fit_seconds):
        if not np.isfinite(value) or value<=0:raise ValueError("positive finite fitting/holdout configuration required")
    poses=np.load(poses_path)
    stamp_ns=poses["stamp_ns"].astype(np.int64);origin=int(stamp_ns[0]);t=(stamp_ns-origin)*1e-9
    if str(poses["provenance"])!="raw_scan_registration":raise ValueError("independent LiDAR provenance required")
    with open(imu_csv) as stream:rows=list(csv.DictReader(stream))
    stamp_key="header_ns" if "header_ns" in rows[0] else "stamp_ns"
    imu_ns=np.array([int(row[stamp_key]) for row in rows],np.int64)
    it=(imu_ns-origin)*1e-9
    acc=np.array([[float(row[k]) for k in ("ax","ay","az")] for row in rows])
    gyro=np.array([[float(row[k]) for k in ("gx","gy","gz")] for row in rows])
    training=np.flatnonzero(t<=t[-1]*train_fraction)
    if len(training)<6:raise ValueError("insufficient training scans")
    if len(t)-int(training[-1])-1<3 or t[-1]-t[training[-1]]<.5:
        raise ValueError("independent holdout requires >=3 scans spanning >=0.5 seconds after training")
    # Keep a bounded mathematical solve while allowing LiDAR registration to
    # cover the complete acquisition window. Select the latest bounded training
    # segment; all unused training-prefix scans remain provenance-only.
    if t[training[-1]]-t[training[0]]>maximum_fit_seconds:
        training=training[t[training]>=t[training[-1]]-maximum_fit_seconds]
    if len(training)<6:raise ValueError("insufficient training scans after maximum_fit_seconds")
    first,last=int(training[0]),int(training[-1]);selection=np.arange(first,last+1)
    covariance=poses["pose_cov"]
    if covariance.ndim==3:covariance=covariance[selection]
    else:
        ids=np.concatenate([np.arange(6*i,6*(i+1)) for i in selection]);covariance=covariance[np.ix_(ids,ids)]
    start=max(0,np.searchsorted(it,t[first])-1);stop=min(len(it),np.searchsorted(it,t[last])+2)
    data=JointInput(t[selection],poses["p"][selection],poses["R"][selection],it[start:stop],acc[start:stop],gyro[start:stop],
                    covariance,"raw_scan_registration",accel_sigma,gyro_sigma,np.array([-.00862,.00914,.03916]),acc_scale)
    result=solve_joint(data)
    result.update(schema="independent_lidar_imu_joint_candidate_v1",commit_stamp_ns=int(stamp_ns[-1]),
         commit_scan_id=int(poses["scan_ids"][-1]),training_scan_ids=poses["scan_ids"][selection].tolist(),
         heldout_scan_ids=poses["scan_ids"][last+1:].tolist(),
         training_first_stamp_ns=int(stamp_ns[first]),training_last_stamp_ns=int(stamp_ns[last]),
         source_poses=str(Path(poses_path).resolve()),source_poses_sha256=_sha(poses_path),
         source_imu=str(Path(imu_csv).resolve()),source_imu_sha256=_sha(imu_csv),
         fixed_extrinsic_R=np.eye(3).tolist(),fixed_extrinsic_T=[-.00862,.00914,.03916],extrinsic_est_en=False,time_offset=0.,
         batch_lookahead=True,commit_policy="No EKF initialization until all training and held-out validation scans through commit_stamp_ns are available; earlier scans are init_skip",
         raw_accel_sigma=accel_sigma,raw_gyro_sigma=gyro_sigma,
         enabled_by_default=False,fallback="retain existing gated initialization; no synthesized bias/state on rejection")
    if result["accepted"]:
        state=dict(t=float(t[last]),p=np.array(result["p_WI_commit"]),R=np.array(result["R_WI_commit"]),v=np.array(result["v_WI_commit"]),
                    g=np.array(result["g_W"]),ba=np.array(result["ba_I"]),bg=np.array(result["bg_I"]),cov=np.array(result["full_covariance"]))
        heldout=np.arange(last+1,len(t));predictions=propagate_state(state,it,acc,gyro,t[heldout],acc_scale,accel_sigma,gyro_sigma)
        checks=[]
        for idx,predicted in zip(heldout,predictions):
            p_lidar=poses["p"][idx]-poses["R"][idx]@data.t_il
            dp=predicted["p"]-p_lidar;dr=Rotation.from_matrix(poses["R"][idx].T@predicted["R"]).as_rotvec()
            checks.append(dict(stamp_ns=int(stamp_ns[idx]),position_error_m=float(np.linalg.norm(dp)),rotation_error_rad=float(np.linalg.norm(dr)),
                               position_error_W=dp.tolist(),rotation_error_I=dr.tolist()))
        maxp=max(check["position_error_m"] for check in checks);maxr=max(check["rotation_error_rad"] for check in checks)
        passed=maxp<=maximum_holdout_position and maxr<=maximum_holdout_rotation
        result["heldout_validation"]=dict(passed=passed,used_in_fit=False,samples=len(checks),
             max_position_error_m=maxp,max_rotation_error_rad=maxr,maximum_position_limit_m=maximum_holdout_position,
             maximum_rotation_limit_rad=maximum_holdout_rotation,checks=checks,
             minimum_holdout_samples=3,minimum_holdout_span_seconds=.5,
             limitations="same independently registered LiDAR sequence; shares scan-registration drift but never uses FAST-LIO state; no external ground truth")
        final=predictions[-1]
        result.update(p_WI_commit=final["p"].tolist(),R_WI_commit=final["R"].tolist(),v_WI_commit=final["v"].tolist(),
                       full_covariance=final["cov"].tolist(),commit_t=float(t[-1]))
        if not passed:result.update(accepted=False,reason="independent_lidar_holdout_prediction_failed")
    else:result["heldout_validation"]=dict(passed=False,not_run_reason=result["reason"])
    result["candidate_holdout"]=result["heldout_validation"]
    result["joint_protocol_version"]="S2_5d_observability_known_norm_9.809_v2"
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:result[k] for k in ("accepted","reason","g_ba_rank","g_ba_constrained_rank","commit_stamp_ns")}),flush=True)
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("poses");parser.add_argument("imu_csv");parser.add_argument("output")
    parser.add_argument("--acc-scale",type=float,required=True);parser.add_argument("--train-fraction",type=float,default=.75)
    parser.add_argument("--maximum-fit-seconds",type=float,default=30.)
    parser.add_argument("--accel-sigma",type=float,default=.035);parser.add_argument("--gyro-sigma",type=float,default=.002)
    parser.add_argument("--maximum-holdout-position",type=float,default=.5);parser.add_argument("--maximum-holdout-rotation",type=float,default=.05)
    args=parser.parse_args();run(args.poses,args.imu_csv,args.output,args.acc_scale,args.train_fraction,args.accel_sigma,args.gyro_sigma,
                                args.maximum_holdout_position,args.maximum_holdout_rotation,args.maximum_fit_seconds)
