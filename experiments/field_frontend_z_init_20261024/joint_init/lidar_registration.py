"""LiDAR-only point-to-plane registration and previous-LiDAR-motion deskew.

No IMU, FAST-LIO pose, map, body cloud, orientation message or ROS state is read.
The covariance is a linearized local ICP covariance plus an explicit per-scan
systematic floor, not a calibration/ground-truth uncertainty certificate.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def window_rows(rows,start_seconds,duration_seconds,stride):
    if start_seconds<0 or duration_seconds<=0 or stride<1:raise ValueError("invalid scan window")
    first=int(rows[0]["stamp_ns"])
    selected=[row for row in rows if start_seconds<=(int(row["stamp_ns"])-first)*1e-9<=start_seconds+duration_seconds][::stride]
    if len(selected)<2:raise ValueError("scan window has fewer than two scans")
    return selected


def voxel(points,size=.35):
    points=np.asarray(points,float)
    valid=np.all(np.isfinite(points),axis=1)&(np.linalg.norm(points,axis=1)>.7)&(np.linalg.norm(points,axis=1)<45.)
    points=points[valid]
    if not len(points):return points
    _,indices=np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
    return points[np.sort(indices)]


def normals(points,neighbors=12):
    tree=cKDTree(points);_,index=tree.query(points,k=min(neighbors,len(points)),workers=1)
    neighbor=points[index];centered=neighbor-neighbor.mean(axis=1,keepdims=True)
    _,vectors=np.linalg.eigh(np.einsum("nki,nkj->nij",centered,centered))
    return vectors[:,:,0]


def register_point_to_plane(source,target,initial_R=None,initial_t=None,maximum_distance=.8,
                            maximum_iterations=30):
    source=np.asarray(source,float);target=np.asarray(target,float)
    if len(source)<100 or len(target)<100:return dict(accepted=False,reason="too_few_points")
    target_normals=normals(target);tree=cKDTree(target)
    rotation=np.eye(3) if initial_R is None else np.array(initial_R,float)
    translation=np.zeros(3) if initial_t is None else np.array(initial_t,float)
    converged=False;history=[]
    def robust_objective(candidate_r,candidate_t):
        transformed=source@candidate_r.T+candidate_t
        distance,index=tree.query(transformed,workers=1)
        residual=np.sum(target_normals[index]*(transformed-target[index]),axis=1)
        absolute=np.abs(residual)
        rho=np.where(absolute<=.10,.5*residual**2,.10*(absolute-.05))
        # Keep the same source count so losing correspondences cannot improve
        # the optimization objective merely by dropping difficult points.
        rho[distance>=maximum_distance]=.10*(maximum_distance-.05)
        return float(np.mean(rho))
    for iteration in range(maximum_iterations):
        transformed=source@rotation.T+translation
        distance,index=tree.query(transformed,workers=1)
        keep=distance<maximum_distance
        if np.sum(keep)<100:return dict(accepted=False,reason="too_few_correspondences")
        transformed=transformed[keep];match=target[index[keep]];normal=target_normals[index[keep]]
        residual=np.sum(normal*(transformed-match),axis=1)
        # Huber merely bounds unrelated point correspondences in this external
        # ICP. Its 0.10m width is logged; it does not alter any front-end Q.
        weight=np.minimum(1.,.10/np.maximum(abs(residual),1e-12))
        jacobian=np.c_[normal,np.cross(transformed,normal)]
        info=jacobian.T@(weight[:,None]*jacobian)
        values=np.linalg.eigvalsh(info)
        if values[0]<values[-1]*1e-7:return dict(accepted=False,reason="geometry_degenerate",eigenvalues=values.tolist())
        delta=np.linalg.solve(info,-jacobian.T@(weight*residual))
        objective=robust_objective(rotation,translation)
        step=1.
        for backtrack in range(16):
            update=Rotation.from_rotvec(delta[3:]*step).as_matrix()
            trial_t=update@translation+delta[:3]*step;trial_r=update@rotation
            trial_objective=robust_objective(trial_r,trial_t)
            if trial_objective<=objective+1e-14:break
            step*=.5
        else:return dict(accepted=False,reason="line_search_no_descent",history=history)
        delta*=step
        history.append(dict(iteration=iteration+1,delta_translation_m=float(np.linalg.norm(delta[:3])),
                            delta_rotation_rad=float(np.linalg.norm(delta[3:])),weighted_rms=float(np.sqrt(np.average(residual**2,weights=weight))),
                            robust_objective=objective,step=step))
        translation=trial_t;rotation=trial_r
        if np.linalg.norm(delta[:3])<1e-5 and np.linalg.norm(delta[3:])<1e-5:
            converged=True;break
    # Fresh correspondences certify the returned transform rather than previous
    # iteration's correspondence set.
    transformed=source@rotation.T+translation;dist,index=tree.query(transformed,workers=1)
    keep=dist<maximum_distance;normal=target_normals[index[keep]]
    residual=np.sum(normal*(transformed[keep]-target[index[keep]]),axis=1)
    jacobian=np.c_[normal,np.cross(transformed[keep],normal)]
    info=jacobian.T@jacobian;values=np.linalg.eigvalsh(info)
    rms=float(np.sqrt(np.mean(residual**2)));overlap=float(np.mean(keep))
    reason="accepted"
    if values[0]<values[-1]*1e-7:reason="geometry_degenerate"
    elif not converged:reason="icp_not_converged"
    elif overlap<.55:reason="low_overlap"
    elif rms>.15:reason="point_plane_residual_above_limit"
    if reason!="accepted":return dict(accepted=False,reason=reason,rms=rms,overlap=overlap,history=history)
    covariance=np.linalg.inv(info)*max(.01**2,rms**2)
    covariance+=np.diag([.005**2]*3+[.002**2]*3)
    return dict(accepted=True,reason=reason,R=rotation,t=translation,covariance=covariance,
                rms=rms,overlap=overlap,iterations=iteration+1,
                hessian_eigenvalues=values.tolist(),correspondences=int(np.sum(keep)),
                objective_history=[entry["robust_objective"] for entry in history]+[robust_objective(rotation,translation)],
                line_search_steps=[entry["step"] for entry in history])

def deskew_constant_velocity(points,times,velocity_body,omega_body):
    points=np.asarray(points,float);times=np.asarray(times,float)
    dt=times-np.max(times)
    rotation=Rotation.from_rotvec(dt[:,None]*np.asarray(omega_body,float)).as_matrix()
    return np.einsum("nij,nj->ni",rotation,points)+dt[:,None]*np.asarray(velocity_body,float)


def read_scan(path):
    path=Path(path)
    if path.suffix==".npz":
        data=np.load(path);return data["xyz"],data["offset_s"]
    if path.suffix==".npy":
        data=np.load(path);return data[:,:3],data[:,3]/1000.
    with path.open("rb") as stream:
        header={}
        while True:
            line=stream.readline().decode("ascii").strip()
            if not line:continue
            if line.startswith("#"):continue
            key,*values=line.split();header[key]=values
            if key=="DATA":break
        names=header["FIELDS"];sizes=list(map(int,header["SIZE"]));counts=list(map(int,header.get("COUNT",["1"]*len(names))))
        dtype=np.dtype([(name,{"F":"f","U":"u","I":"i"}[kind]+str(size),(count,)) if count>1 else (name,{"F":"f","U":"u","I":"i"}[kind]+str(size)) for name,size,kind,count in zip(names,sizes,header["TYPE"],counts)])
        if header["DATA"]==["binary"]:data=np.frombuffer(stream.read(),dtype=dtype,count=int(header["POINTS"][0]))
        elif header["DATA"]==["ascii"]:data=np.loadtxt(stream,dtype=dtype)
        else:raise ValueError("PCD must be DATA binary/ascii, not compressed")
        xyz=np.c_[data["x"],data["y"],data["z"]].astype(float)
        times=np.asarray(data["curvature"],float)/1000.
        return xyz,times


def register_sequence(manifest_path,output_path,stride=3,maximum_seconds=60.,start_seconds=0.):
    manifest_path=Path(manifest_path);output_path=Path(output_path);output_path.mkdir(parents=True,exist_ok=True)
    with manifest_path.open() as stream:rows=list(csv.DictReader(stream))
    rows=window_rows(rows,start_seconds,maximum_seconds,stride)
    rotations=[];positions=[];stamps=[];scan_ids=[];records=[]
    pose_cov=np.zeros((6*len(rows),6*len(rows)));previous=None
    velocity=np.zeros(3);omega=np.zeros(3);r_world=np.eye(3);p_world=np.zeros(3)
    for i,row in enumerate(rows):
        path=Path(row["path"])
        if not path.is_absolute():path=manifest_path.parent/path
        xyz,offset=read_scan(path)
        if np.any(~np.isfinite(offset)) or np.max(offset)-np.min(offset)>.3:raise ValueError("invalid scan timing")
        stamp=int(row["end_stamp_ns"]) if row.get("end_stamp_ns") else int(row["stamp_ns"])+int(round(float(np.max(offset))*1e9))
        cloud=voxel(deskew_constant_velocity(xyz,offset,velocity,omega))
        if previous is not None:
            dt=(stamp-stamps[-1])*1e-9
            initial_r=Rotation.from_rotvec(omega*dt).as_matrix()
            initial_t=velocity*dt
            reg=register_point_to_plane(cloud,previous,initial_r,initial_t)
            public={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in reg.items()}
            public.update(scan_id=row.get("scan_id",str(i)),stamp_ns=stamp);records.append(public)
            if not reg["accepted"]:
                (output_path/"registration_failure.json").write_text(json.dumps(public,indent=2))
                raise RuntimeError("independent LiDAR registration rejected "+str(public))
            r_inc=reg["R"];t_inc=reg["t"]
            f=np.zeros((6,6));f[:3,:3]=np.eye(3);f[:3,3:]=-r_world@skew(t_inc);f[3:,3:]=r_inc.T
            gain=np.zeros((6,6));gain[:3,:3]=r_world;gain[:3,3:]=-r_world@skew(t_inc);gain[3:,3:]=r_inc.T
            old=slice(6*(i-1),6*i);new=slice(6*i,6*(i+1))
            pose_cov[new,:6*i]=f@pose_cov[old,:6*i];pose_cov[:6*i,new]=pose_cov[new,:6*i].T
            pose_cov[new,new]=f@pose_cov[old,old]@f.T+gain@reg["covariance"]@gain.T
            p_world=p_world+r_world@t_inc;r_world=r_world@r_inc
            velocity=r_inc.T@t_inc/dt;omega=Rotation.from_matrix(r_inc).as_rotvec()/dt
        else:
            pose_cov[:6,:6]=np.eye(6)*1e-12
        rotations.append(r_world.copy());positions.append(p_world.copy());stamps.append(stamp);scan_ids.append(row.get("scan_id",str(i)))
        previous=cloud
        if i%10==0:print(json.dumps({"stage":"independent_lidar_icp","done":i+1,"total":len(rows),"stamp_ns":stamp}),flush=True)
    manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    np.savez(output_path/"lidar_poses.npz",stamp_ns=np.array(stamps,np.int64),p=np.array(positions),R=np.array(rotations),pose_cov=pose_cov,
             scan_ids=np.array(scan_ids),provenance="raw_scan_registration",source_manifest=str(manifest_path.resolve()),manifest_sha256=manifest_sha256)
    (output_path/"registration.json").write_text(json.dumps(dict(source="original_preprocessed_scans_not_fastlio_body_or_map",stride=stride,start_seconds=start_seconds,duration_seconds=maximum_seconds,
        deskew="previous LiDAR-only constant body angular rate and local velocity; scan-end reference; first scan uncorrected",voxel_m=.35,
        max_pair_distance_m=.8,huber_width_m=.10,systematic_covariance_floor_translation_m=.005,systematic_covariance_floor_rotation_rad=.002,
        covariance="correlated relative-pose composition; ignores repeated geometric scene systematic correlation",source_manifest=str(manifest_path.resolve()),manifest_sha256=manifest_sha256,registrations=records),indent=2))
    return output_path/"lidar_poses.npz"


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("manifest");parser.add_argument("output")
    parser.add_argument("--stride",type=int,default=3);parser.add_argument("--seconds",type=float,default=60.)
    parser.add_argument("--start-seconds",type=float,default=0.)
    args=parser.parse_args();print(register_sequence(args.manifest,args.output,args.stride,args.seconds,args.start_seconds))
