"""Joint motion initialization with a single consistent inertial trajectory."""
import ctypes
import argparse,csv,hashlib,json
from pathlib import Path
import numpy as np


def _integrate_all(it,acc,gyro,times,params,alpha,first,sa,sw,covariance):
    arrays=[np.ascontiguousarray(x,dtype=np.float64)for x in [it,acc,gyro,times,params,first]]
    it,acc,gyro,times,params,first=arrays
    if len(it)<2 or len(times)<2 or len(params)!=12 or len(first)!=3 or acc.shape!=(len(it),3) or gyro.shape!=acc.shape:
        raise ValueError('invalid inertial integration shapes')
    if not all(np.all(np.isfinite(x))for x in arrays) or np.any(np.diff(it)<=0) or np.any(np.diff(times)<=0):raise ValueError('invalid finite monotonic input')
    library=ctypes.CDLL(str(Path(__file__).resolve().parent/'build/libinertial_reference.so'))
    fn=library.integrate;ptr=ctypes.POINTER(ctypes.c_double)
    fn.argtypes=[ctypes.c_int,ptr,ptr,ptr,ctypes.c_int,ptr,ptr,ctypes.c_double,ptr,ctypes.c_double,ctypes.c_double,ctypes.c_int,ptr,ptr,ptr,ptr]
    fn.restype=ctypes.c_int;n=len(times)
    state=np.zeros((n,10));c=np.zeros((6*n,6*n) if covariance else (1,1));x=np.zeros((9,6*n) if covariance else (1,1));last=np.zeros((9,9))
    def p(a):return a.ctypes.data_as(ptr)
    code=fn(len(it),p(it),p(acc),p(gyro),n,p(times),p(params),alpha,p(first),sa,sw,int(covariance),p(state),p(c),p(x),p(last))
    if code:raise ValueError('inertial integration rejected support: '+str(code))
    return state,c,x,last


def integrate(*args):return _integrate_all(*args)[:3]


def relative_imu_pose_covariance(p,r,pc,lever):
    from joint_init.lidar_registration import skew
    n=len(p);r0=r[0];relative_r=np.einsum('ij,njk->nik',r0.T,r)
    relative_p=(p-p[0])@r0-np.einsum('nij,j->ni',relative_r,lever)
    T=np.zeros((6*n,6*n))
    for i in range(n):
        T[6*i:6*i+3,6*i:6*i+3]+=r0.T
        T[6*i:6*i+3,6*i+3:6*i+6]+=relative_r[i]@skew(lever)
        T[6*i+3:6*i+6,6*i+3:6*i+6]+=np.eye(3)
        T[6*i:6*i+3,:3]-=r0.T
        T[6*i:6*i+3,3:6]+=skew(relative_p[i])
        T[6*i+3:6*i+6,3:6]-=relative_r[i].T
    C=T@pc@T.T
    return (C+C.T)*.5,T


def validate_poses(q):
    if str(q['provenance'])!='raw_scan_registration':raise ValueError('independent raw scan registration required')
    stamps=np.asarray(q['stamp_ns']);n=len(stamps);p,r,C=q['p'],q['R'],q['pose_cov']
    if n<6 or stamps.dtype.kind not in 'iu' or np.any(np.diff(stamps)<=0):raise ValueError('integer monotonic LiDAR timestamps required')
    if p.shape!=(n,3) or r.shape!=(n,3,3) or C.shape!=(6*n,6*n):raise ValueError('invalid independent pose shape')
    if not all(np.all(np.isfinite(x))for x in [p,r,C]):raise ValueError('nonfinite reference')
    if np.max(np.abs(np.linalg.det(r)-1))>1e-6 or np.max(np.abs(np.einsum('nji,njk->nik',r,r)-np.eye(3)))>1e-6:raise ValueError('invalid reference SO3')
    if np.max(np.abs(C-C.T))>1e-8 or np.linalg.eigvalsh(C).min()<-1e-12:raise ValueError('invalid reference covariance')


def run(poses_path,imu_path,output,alpha=1.0136393576435327,training_end_stamp_ns=None):
    """Direct joint fit: a single gyro-propagated p/R/v is fitted AND committed.

    Fixed first-pose coordinates are gauge choices, not attitude priors. Shared
    raw IMU interpolation noise is carried by the C++ augmented covariance;
    estimator/direct-propagation cross covariance prevents double counting.
    """
    from scipy.linalg import cholesky,solve_triangular
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    from joint_init.joint_math import _g_basis
    poses_path,imu_path,output=map(Path,(poses_path,imu_path,output))
    if output.exists():raise ValueError('new candidate required')
    q=np.load(poses_path);validate_poses(q);ns=q['stamp_ns'].astype(np.int64);relative=(ns-ns[0])*1e-9
    cutoff=.75*relative[-1] if training_end_stamp_ns is None else (int(training_end_stamp_ns)-int(ns[0]))*1e-9
    indices=np.flatnonzero(relative<=cutoff)
    if len(indices)<6:raise ValueError('frozen training selection insufficient')
    indices=indices[relative[indices]>=relative[indices[-1]]-30.]
    first,last=int(indices[0]),int(indices[-1]);fullsel=np.arange(first,len(ns));n=len(indices)
    if n<6 or len(ns)-last-1<3 or (int(ns[-1])-int(ns[last]))*1e-9<.5:raise ValueError('training/heldout support insufficient: >=3 scans and >=0.5s required')
    origin=int(ns[first]);t=(ns-origin)*1e-9;times=t[indices]
    gauge_r=q['R'][first];gauge_p=q['p'][first];lever=np.array([-.00862,.00914,.03916])
    r=np.einsum('ij,njk->nik',gauge_r.T,q['R'][indices])
    pl=(q['p'][indices]-gauge_p)@gauge_r;pi=pl-np.einsum('nij,j->ni',r,lever)
    all_r=np.einsum('ij,njk->nik',gauge_r.T,q['R'][fullsel])
    all_pi=(q['p'][fullsel]-gauge_p)@gauge_r-np.einsum('nij,j->ni',all_r,lever)
    rows=list(csv.DictReader(imu_path.open()));it=(np.array([int(z['header_ns'])for z in rows],np.int64)-origin)*1e-9
    acc=np.array([[float(z[k])for k in ['ax','ay','az']]for z in rows]);gyro=np.array([[float(z[k])for k in ['gx','gy','gz']]for z in rows])
    mask=(it>=-.02)&(it<=t[-1]+.02);it,acc,gyro=it[mask],acc[mask],gyro[mask]
    cov=q['pose_cov'];ix=np.concatenate([np.arange(6*i,6*(i+1))for i in indices]);pc=cov[np.ix_(ix,ix)]
    pc,_=relative_imu_pose_covariance(q['p'][indices],q['R'][indices],pc,lever)
    seed=-alpha*np.mean(acc[(it>=0)&(it<1.)],axis=0);seed=seed/np.linalg.norm(seed)*9.809
    velocity=(pi[min(3,n-1)]-pi[0])/times[min(3,n-1)]
    starts=[(seed,np.r_[np.zeros(8),velocity]),(-seed,np.r_[np.zeros(8),velocity])]
    fits=[]
    for branch,(ref,z0) in enumerate(starts):
        basis=_g_basis(ref)
        def unpack(z):
            g=ref+basis@z[:2];g=g/np.linalg.norm(g)*9.809
            return np.r_[g,z[2:]]
        def states(z,c=False,all_times=None):
            return _integrate_all(it,acc,gyro,times if all_times is None else all_times,
                                  unpack(z),alpha,-lever,.035,.002,c)
        def residual(z):
            s=states(z)[0];pred=Rotation.from_quat(s[:,3:7]).as_matrix()
            angle=Rotation.from_matrix(np.einsum('nji,njk->nik',r,pred)).as_rotvec()
            return np.c_[s[:,:3]-pi,angle].ravel()[6:]
        z=z0
        # Re-linearize noise after each fit; no frozen latent LiDAR mean is
        # silently mixed with parameters estimated in a different pose mean.
        for outer in range(3):
            _,noise,_,_=states(z,True);C=pc[6:,6:]+noise[6:,6:]+np.eye(6*n-6)*1e-12
            lower=cholesky(C,lower=True)
            fit=least_squares(lambda x:solve_triangular(lower,residual(x),lower=True),z,
                              xtol=1e-10,ftol=1e-10,gtol=1e-9,max_nfev=100)
            change=np.linalg.norm(fit.x-z);z=fit.x
            if change<1e-7:break
        par=unpack(z);cost=float(fit.fun@fit.fun)
        print(json.dumps(dict(branch=branch,fit_success=bool(fit.success),chi2=cost/max(1,len(fit.fun)-11),
                              g=par[:3].tolist(),ba=par[3:6].tolist(),bg=par[6:9].tolist())),flush=True)
        fits.append(dict(ref=ref,basis=basis,z=z,fit=fit,par=par,cost=cost,lower=lower))
        # Closures above capture loop variables: all subsequent processing below
        # uses numerical saved arrays, not retained callbacks.
        fits[-1]['train_states']=states(z,True)
        fits[-1]['full_states']=states(z,True,t[fullsel])
        # Prediction-state Jacobian in p, right theta, v, g, ba, bg order.
        base=fits[-1]['full_states'][0][-1];base_R=Rotation.from_quat(base[3:7]).as_matrix()
        S=np.empty((18,11));eps=1e-6
        for k in range(11):
            dz=np.zeros(11);dz[k]=eps;ss=states(z+dz,False,t[fullsel])[0][-1];pp=unpack(z+dz)
            dd=np.r_[ss[:3]-base[:3],Rotation.from_matrix(base_R.T@Rotation.from_quat(ss[3:7]).as_matrix()).as_rotvec(),
                     ss[7:]-base[7:],pp[:9]-par[:9]]
            S[:,k]=dd/eps
        fits[-1]['S']=S
    plausible=[f for f in fits if f['fit'].success and np.linalg.norm(f['par'][3:6])<=1.5 and np.linalg.norm(f['par'][6:9])<=.15]
    chosen=min(plausible or fits,key=lambda f:f['cost']);fit=chosen['fit'];par=chosen['par'];z=chosen['z']
    equivalent=[f for f in plausible if f['cost']<=chosen['cost']+3.84]
    information=fit.jac.T@fit.jac
    def reject_unobservable(reason,rank_value=None):
        rejected=dict(schema='independent_lidar_imu_joint_candidate_v1',method='direct_inertial_pose_likelihood_consistent_mean_v2',
            accepted=False,reason=reason,covariance_valid=False,g_ba_constrained_rank=rank_value,
            unresolved_direction_variance='unbounded; no pseudoinverse used to synthesize a candidate',
            heldout_validation=dict(passed=False,not_run_reason=reason),
            enabled_by_default=False,real_frontend_commit=False,source_poses=str(poses_path.resolve()),source_imu=str(imu_path.resolve()),
            source_poses_sha256=hashlib.sha256(poses_path.read_bytes()).hexdigest(),source_imu_sha256=hashlib.sha256(imu_path.read_bytes()).hexdigest(),
            fixed_extrinsic_R=np.eye(3).tolist(),fixed_extrinsic_T=lever.tolist(),extrinsic_est_en=False,time_offset=0.)
        output.write_text(json.dumps(rejected,indent=2));return rejected
    try:
        np.linalg.cholesky(information[5:,5:])
        H=information[:5,:5]-information[:5,5:]@np.linalg.solve(information[5:,5:],information[5:,:5])
    except np.linalg.LinAlgError:return reject_unobservable('nuisance_state_unobservable')
    sv=np.sqrt(np.maximum(0,np.linalg.eigvalsh(H)))[::-1];rank=int(np.sum(sv>sv[0]*.005))
    if rank<5:return reject_unobservable('insufficient_conditioned_g_ba_excitation',rank)
    try:
        np.linalg.cholesky(information);parameter_cov=np.linalg.inv(information)
    except np.linalg.LinAlgError:return reject_unobservable('numerical_information_not_invertible',rank)
    S=chosen['S'];full_states,noise,cross,lastp=chosen['full_states']
    # Preserve the free-gravity sanity check in THIS direct inertial model.
    # A non-identifiable free radial direction must not masquerade as a measured
    # gravity norm, and a clearly inconsistent identifiable norm must reject.
    def free_residual(x):
        s=_integrate_all(it,acc,gyro,times,x,alpha,-lever,.035,.002,False)[0]
        angle=Rotation.from_matrix(np.einsum('nji,njk->nik',r,Rotation.from_quat(s[:,3:7]).as_matrix())).as_rotvec()
        return solve_triangular(chosen['lower'],np.c_[s[:,:3]-pi,angle].ravel()[6:],lower=True)
    free=least_squares(free_residual,par,xtol=1e-10,ftol=1e-10,gtol=1e-9,max_nfev=100)
    free_info=free.jac.T@free.jac
    free_cond=free_info[:6,:6]-free_info[:6,6:]@np.linalg.solve(free_info[6:,6:],free_info[6:,:6])
    free_sv=np.sqrt(np.maximum(0,np.linalg.eigvalsh(free_cond)))[::-1]
    free_rank=int(np.sum(free_sv>free_sv[0]*.005));free_norm=float(np.linalg.norm(free.x[:3]))
    # Prediction residual e = J dz + IMU noise - LiDAR noise; dz=-K e.
    # State contains the same direct IMU noise. Retain BOTH cross terms.
    lower=chosen['lower'];K=parameter_cov@fit.jac.T@solve_triangular(lower,np.eye(6*n-6),lower=True)
    X=np.zeros((6*n-6,18));X[:,:9]=cross[:,:6*n][:,6:].T
    direct=np.zeros((18,18));direct[:9,:9]=lastp
    Cstate=S@parameter_cov@S.T+direct-S@K@X-X.T@K.T@S.T
    chi2=chosen['cost']/max(1,len(fit.fun)-11);Cstate=(Cstate+Cstate.T)*.5*max(1.,chi2)
    sigma_deg=float(np.sqrt(max(0,np.linalg.eigvalsh(Cstate[9:12,9:12]).max()))/9.809*180/np.pi)
    heldout=[]
    pred_r=Rotation.from_quat(full_states[:,3:7]).as_matrix()
    for i,k in enumerate(fullsel):
        if k<=last:continue
        dp=full_states[i,:3]-all_pi[i];dr=Rotation.from_matrix(all_r[i].T@pred_r[i]).as_rotvec()
        heldout.append(dict(stamp_ns=int(ns[k]),position_error_m=float(np.linalg.norm(dp)),rotation_error_rad=float(np.linalg.norm(dr)),position_error_W=(gauge_r@dp).tolist()))
    maxp=max(q['position_error_m']for q in heldout);maxr=max(q['rotation_error_rad']for q in heldout)
    reason='accepted'
    if not plausible:reason='no_physically_plausible_solution'
    elif len(equivalent)>1:reason='ambiguous_equivalent_solutions'
    elif rank<5:reason='insufficient_conditioned_g_ba_excitation'
    elif not fit.success:reason='joint_optimization_failed'
    elif not free.success:reason='free_gravity_sanity_optimization_failed'
    elif free_rank==6 and abs(free_norm-9.809)>.5:reason='gravity_norm_inconsistent'
    elif chi2>3.:reason='training_inconsistency'
    elif sigma_deg>1.:reason='gravity_uncertainty_above_limit'
    elif maxp>.5 or maxr>.05:reason='independent_lidar_holdout_prediction_failed'
    elif np.linalg.eigvalsh(Cstate).min()<-1e-9:reason='covariance_not_psd'
    final=full_states[-1]
    result=dict(schema='independent_lidar_imu_joint_candidate_v1',accepted=reason=='accepted',reason=reason,
        method='direct_inertial_pose_likelihood_consistent_mean_v2',g_ba_rank=free_rank,g_ba_constrained_rank=rank,
        free_gravity_fit_success=bool(free.success),unconstrained_gravity_norm=free_norm,unconstrained_gravity_norm_identifiable=free_rank==6,
        unconstrained_g_ba_singular_values=free_sv.tolist(),maximum_gravity_norm_error=.5,
        conditioned_g_ba_singular_values=sv.tolist(),g_W=par[:3].tolist(),ba_I=par[3:6].tolist(),bg_I=par[6:9].tolist(),
        p_WI_commit=final[:3].tolist(),R_WI_commit=pred_r[-1].tolist(),v_WI_commit=final[7:].tolist(),
        covariance_order=['p_WI','theta_I_right','v_WI','g_W','ba_I','bg_I'],
        full_covariance=Cstate.tolist(),full_covariance_order=['p_WI','theta_I_right','v_WI','g_W','ba_int_I','bg_I'],
        world_gauge='training_first_LiDAR',world_gauge_scan_id=int(q['scan_ids'][first]),
        covariance_gauge='conditional on chosen training-first coordinate gauge; common-prefix uncertainty cancels via full relative-pose Jacobian',
        diagnostic_original_frame_g_W=(gauge_r@par[:3]).tolist(),
        covariance_valid=bool(np.linalg.eigvalsh(Cstate).min()>=-1e-9),gravity_direction_sigma_degrees=sigma_deg,
        covariance_assumptions='first-order fixed-R/T/alpha raw-IMU iid sample noise plus supplied correlated LiDAR covariance; exact reuse of interpolated endpoint noise via active-noise augmentation; no unknown ICP systematic certification',
        estimator_direct_noise_cross_terms=True,acc_scale=alpha,raw_accel_sigma=.035,raw_gyro_sigma=.002,
        residual_reduced_chi2=chi2,maximum_training_reduced_chi2=3.,branches=[dict(cost=f['cost'],g_local=f['par'][:3].tolist(),ba=f['par'][3:6].tolist(),bg=f['par'][6:9].tolist())for f in fits],
        source_poses=str(poses_path.resolve()),source_imu=str(imu_path.resolve()),source_poses_sha256=hashlib.sha256(poses_path.read_bytes()).hexdigest(),source_imu_sha256=hashlib.sha256(imu_path.read_bytes()).hexdigest(),
        training_first_stamp_ns=int(ns[first]),training_last_stamp_ns=int(ns[last]),commit_stamp_ns=int(ns[-1]),commit_scan_id=int(q['scan_ids'][-1]),
        training_scan_ids=q['scan_ids'][indices].tolist(),heldout_scan_ids=q['scan_ids'][last+1:].tolist(),
        heldout_validation=dict(passed=maxp<=.5 and maxr<=.05,used_in_fit=False,samples=len(heldout),checks=heldout,max_position_error_m=maxp,max_rotation_error_rad=maxr,maximum_position_limit_m=.5,maximum_rotation_limit_rad=.05),
        fixed_extrinsic_R=np.eye(3).tolist(),fixed_extrinsic_T=lever.tolist(),extrinsic_est_en=False,time_offset=0.,enabled_by_default=False,
        commit_policy='all validation must be available before committing final propagated mean/covariance; no raw-LiDAR attitude substituted for inertial fitted mean',real_frontend_commit=False,
        training_selection='original .75/30s' if training_end_stamp_ns is None else 'explicit training-only frozen end timestamp')
    output.write_text(json.dumps(result,indent=2,allow_nan=False))
    np.savez_compressed(output.with_suffix('.npz'),t=t[fullsel],states=full_states,observed_p=all_pi,observed_R=all_r,train_end_index=last-first,params=par,gauge_R=gauge_r,gauge_p=gauge_p)
    print(json.dumps({k:result[k]for k in ['accepted','reason','residual_reduced_chi2','gravity_direction_sigma_degrees']},indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('poses');p.add_argument('imu');p.add_argument('output');p.add_argument('--training-end-stamp-ns',type=int)
    a=p.parse_args();run(a.poses,a.imu,a.output,training_end_stamp_ns=a.training_end_stamp_ns)
