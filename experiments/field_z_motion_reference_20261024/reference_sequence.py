"""Causal bounded LiDAR-only reference. No EKF/IMU pose is read or used.

Local-map covariance is a conservative sequential composition diagnostic, not
an exact shared-submap posterior; it cannot alone certify initialization.
"""
import argparse,csv,hashlib,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from joint_init.lidar_registration import (register_point_to_plane,voxel,
    read_scan,window_rows,deskew_constant_velocity,skew,normals)


def register_gicp(source,target,initial_R=None,initial_t=None):
    import open3d as o3d
    from scipy.spatial import cKDTree
    if len(source)<100 or len(target)<100:return dict(accepted=False,reason='too_few_points')
    src=o3d.geometry.PointCloud();src.points=o3d.utility.Vector3dVector(source)
    tgt=o3d.geometry.PointCloud();tgt.points=o3d.utility.Vector3dVector(target)
    ns=normals(source);nt=normals(target)
    src.normals=o3d.utility.Vector3dVector(ns);tgt.normals=o3d.utility.Vector3dVector(nt)
    transform=np.eye(4)
    if initial_R is not None:transform[:3,:3]=initial_R
    if initial_t is not None:transform[:3,3]=initial_t
    api=o3d.pipelines.registration
    estimation=api.TransformationEstimationForGeneralizedICP(.001,api.HuberLoss(.1))
    # Same method/settings for every frequency and submap ablation. This is a
    # different registration objective, explicitly not the rejected original ICP.
    result=api.registration_generalized_icp(src,tgt,.8,transform,estimation,
                     api.ICPConvergenceCriteria(relative_fitness=1e-8,relative_rmse=1e-8,max_iteration=60))
    trans=np.asarray(result.transformation);r,t=trans[:3,:3],trans[:3,3]
    # Verify a further optimization reaches a stable transform, not just a small
    # point-plane RMS or a maximum-iteration exit.
    check=api.registration_generalized_icp(src,tgt,.8,trans,estimation,
                     api.ICPConvergenceCriteria(relative_fitness=1e-8,relative_rmse=1e-8,max_iteration=10))
    delta=np.linalg.inv(trans)@np.asarray(check.transformation)
    stable=np.linalg.norm(delta[:3,3])<1e-5 and Rotation.from_matrix(delta[:3,:3]).magnitude()<1e-5
    x=source@r.T+t;dist,index=cKDTree(target).query(x,workers=1);keep=dist<.8
    normal=nt[index[keep]];residual=np.sum(normal*(x[keep]-target[index[keep]]),axis=1)
    j=np.c_[normal,np.cross(x[keep],normal)];info=j.T@j;eig=np.linalg.eigvalsh(info)
    rms=float(np.sqrt(np.mean(residual**2)));overlap=float(np.mean(keep))
    reason='accepted'
    if eig[0]<eig[-1]*1e-7:reason='geometry_degenerate'
    elif not stable:reason='gicp_not_converged'
    elif overlap<.55:reason='low_overlap'
    elif rms>.15:reason='point_plane_residual_above_limit'
    if reason!='accepted':return dict(accepted=False,reason=reason,rms=rms,overlap=overlap)
    covariance=np.linalg.inv(info)*max(.01**2,rms**2)+np.diag([.005**2]*3+[.002**2]*3)
    return dict(accepted=True,reason=reason,R=r,t=t,covariance=covariance,rms=rms,overlap=overlap,
                hessian_eigenvalues=eig.tolist(),correspondences=int(np.sum(keep)),
                engine='Open3D '+o3d.__version__+' GICP',numerical_stability_extra10=True)


def build_submap(history,reference_R,reference_p,window):
    if not isinstance(window,int) or not 1<=window<=30 or not history:
        raise ValueError('nonempty history and finite window 1..30 required')
    kept=history[-window:]
    xyz=[(h['cloud']@h['R'].T+h['p']-reference_p)@reference_R for h in kept]
    return np.vstack(xyz),[h['scan_id'] for h in kept]


def deskew_trajectory(xyz,offset,header_ns,end_ns,reference):
    from scipy.spatial.transform import Slerp
    ns=np.asarray(reference['stamp_ns'],np.int64)
    indices=np.flatnonzero(ns<=end_ns)
    if len(indices)<2:raise ValueError('causal reference must bracket scan acquisition')
    ns=ns[indices];origin=int(ns[0]);t=(ns-origin)*1e-9
    sample=(int(header_ns)-origin)*1e-9+offset;end=(int(end_ns)-origin)*1e-9
    if min(sample)<t[0]-1e-9 or max(sample)>t[-1]+1e-9 or end>t[-1]+1e-9:raise ValueError('unbracketed scan')
    # Slerp/linear position use ONLY bracketing endpoints, not a global spline
    # whose derivatives would introduce future-scan support.
    interp=Slerp(t,Rotation.from_matrix(reference['R'][indices]));rr=interp(np.clip(sample,t[0],t[-1])).as_matrix();re=interp(end).as_matrix()
    p=np.column_stack([np.interp(sample,t,reference['p'][indices,j])for j in range(3)])
    pe=np.array([np.interp(end,t,reference['p'][indices,j])for j in range(3)])
    return (np.einsum('nij,nj->ni',rr,xyz)+p-pe)@re


def run(manifest,output,stride=1,mode='pair',window=10,polish=False,
        start=150.,seconds=65.,deskew='previous',match=None,engine='original',deskew_reference=None):
    manifest,output=Path(manifest).resolve(),Path(output).resolve()
    if output.exists():raise ValueError('new output directory required')
    output.mkdir(parents=True);beg=time.monotonic()
    rows=window_rows(list(csv.DictReader(manifest.open())),start,seconds,stride)
    ref=np.load(deskew_reference) if deskew_reference else None
    if deskew=='trajectory' and ref is None:raise ValueError('trajectory compensation requires explicit reference')
    poses=[];stamps=[];ids=[];records=[];history=[];r=np.eye(3);p=np.zeros(3)
    velocity=np.zeros(3);omega=np.zeros(3);pc=np.zeros((6*len(rows),6*len(rows)))
    def save(failed=None):
        n=len(poses)
        if n:
            np.savez(output/'lidar_poses.npz',stamp_ns=np.array(stamps,np.int64),
                p=np.array([q[1] for q in poses]),R=np.array([q[0] for q in poses]),
                pose_cov=pc[:6*n,:6*n],scan_ids=np.array(ids),provenance='raw_scan_registration',
                source_manifest=str(manifest),manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
        summary=dict(source='original_preprocessed_scans_not_fastlio',mode=mode,stride=stride,engine=engine,
             window=window,polish_on_stall=polish,deskew=deskew,start_seconds=start,duration_seconds=seconds,
             completed_scans=n,expected_scans=len(rows),complete=failed is None and n==len(rows),
             elapsed_seconds=time.monotonic()-beg,failure=failed,registrations=records,
             covariance_scope='sequential relative composition; shared submap geometry correlations not certified',
             voxel_m=.35,max_pair_distance_m=.8,huber_width_m=.10,
             source_manifest=str(manifest),manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
        (output/'registration.json').write_text(json.dumps(summary,indent=2))
    for i,row in enumerate(rows):
        path=Path(row['path']);path=path if path.is_absolute() else manifest.parent/path
        xyz,offset=read_scan(path)
        if not np.all(np.isfinite(offset)) or np.ptp(offset)>.3:raise ValueError('invalid scan timing')
        stamp=int(row['stamp_ns'])+int(round(float(np.max(offset))*1e9))
        if deskew=='trajectory' and i:
            cloud=voxel(deskew_trajectory(xyz,offset,int(row['stamp_ns']),stamp,ref))
        else:cloud=voxel(deskew_constant_velocity(xyz,offset,velocity,omega) if deskew in ['previous','current_iterated'] else xyz)
        if i:
            dt=(stamp-stamps[-1])*1e-9
            target,sources=build_submap(history,r,p,1 if mode=='pair' else window)
            if mode=='submap':target=voxel(target)
            if engine=='gicp':reg=register_gicp(cloud,target,Rotation.from_rotvec(omega*dt).as_matrix(),velocity*dt)
            else:reg=register_point_to_plane(cloud,target,Rotation.from_rotvec(omega*dt).as_matrix(),velocity*dt,
                                           polish_on_stall=polish)
            refinements=[]
            if deskew=='current_iterated' and reg['accepted']:
                # Causal predictor/corrector: same frozen historical target,
                # re-deskew ORIGINAL acquisition points using the current pair
                # motion rather than the previous pair's stale average rate.
                for refinement in range(2):
                    dr0,dp0=reg['R'],reg['t']
                    cloud=voxel(deskew_constant_velocity(xyz,offset,dr0.T@dp0/dt,Rotation.from_matrix(dr0).as_rotvec()/dt))
                    reg=register_point_to_plane(cloud,target,dr0,dp0,polish_on_stall=polish)
                    refinements.append(dict(iteration=refinement+1,accepted=reg['accepted'],reason=reg['reason'],
                        previous_rate_rad_s=(Rotation.from_matrix(dr0).as_rotvec()/dt).tolist()))
                    if not reg['accepted']:break
            public={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in reg.items()}
            public.update(scan_id=int(row['scan_id']),stamp_ns=stamp,target_scan_ids=sources,
                          source_points=len(cloud),target_points=len(target),current_motion_refinements=refinements)
            records.append(public)
            if not reg['accepted']:save(public);print('REJECT',output,public['scan_id'],reg['reason'],flush=True);return False
            dr,dp=reg['R'],reg['t'];f=np.zeros((6,6));f[:3,:3]=np.eye(3);f[:3,3:]=-r@skew(dp);f[3:,3:]=dr.T
            gain=np.zeros((6,6));gain[:3,:3]=r;gain[:3,3:]=-r@skew(dp);gain[3:,3:]=dr.T
            old=slice(6*(i-1),6*i);new=slice(6*i,6*(i+1))
            pc[new,:6*i]=f@pc[old,:6*i];pc[:6*i,new]=pc[new,:6*i].T
            pc[new,new]=f@pc[old,old]@f.T+gain@reg['covariance']@gain.T
            p=p+r@dp;r=r@dr;velocity=dr.T@dp/dt;omega=Rotation.from_matrix(dr).as_rotvec()/dt
        else:pc[:6,:6]=np.eye(6)*1e-12
        poses.append((r.copy(),p.copy()));stamps.append(stamp);ids.append(int(row['scan_id']))
        history.append(dict(scan_id=int(row['scan_id']),cloud=cloud,R=r.copy(),p=p.copy()))
        history=history[-window:]
        if i%50==0:print(json.dumps(dict(done=i+1,total=len(rows),elapsed_s=round(time.monotonic()-beg,2))),flush=True)
    save()
    if match:
        original=np.load(match);lookup={n:i for i,n in enumerate(ids)}
        selection=np.array([lookup[int(n)] for n in original['scan_ids']],int)
        a=np.load(output/'lidar_poses.npz');ix=np.concatenate([np.arange(6*k,6*(k+1)) for k in selection])
        np.savez(output/'matched_original213.npz',**{k:(a[k][np.ix_(ix,ix)] if k=='pose_cov' else a[k][selection] if k in ['stamp_ns','p','R','scan_ids'] else a[k]) for k in a.files})
    print('DONE',output,flush=True);return True


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('manifest');a.add_argument('output')
    a.add_argument('--stride',type=int,default=1);a.add_argument('--mode',choices=['pair','submap'],default='pair')
    a.add_argument('--window',type=int,default=10);a.add_argument('--polish',action='store_true')
    a.add_argument('--start',type=float,default=150.);a.add_argument('--seconds',type=float,default=65.)
    a.add_argument('--deskew',choices=['previous','none','trajectory','current_iterated'],default='previous');a.add_argument('--match')
    a.add_argument('--deskew-reference')
    a.add_argument('--engine',choices=['original','gicp'],default='original')
    q=a.parse_args();ok=run(q.manifest,q.output,q.stride,q.mode,q.window,q.polish,q.start,q.seconds,q.deskew,q.match,q.engine,q.deskew_reference)
    raise SystemExit(0 if ok else 2)
