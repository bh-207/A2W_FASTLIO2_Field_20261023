"""Independent LiDAR / scaled raw IMU initialization, with explicit degeneracy.

All rotations map IMU to the independent first-scan world. T_IL maps LiDAR
to IMU, fixed rotation I. No FAST-LIO state is an input. Pose covariance uses
[world translation, right/body rotation] tangents and may include correlations.
"""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import block_diag, cholesky, solve_triangular
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, RotationSpline

@dataclass
class JointInput:
    lidar_t: object
    lidar_p: object
    lidar_R: object
    imu_t: object
    acc: object
    gyro: object
    pose_cov: object
    provenance: str
    accel_sigma: float
    gyro_sigma: float
    t_il: object
    acc_scale: float = 1.
    rank_ratio: float = .005
    maximum_imu_gap: float = .05
    maximum_gravity_norm_error: float = .5
    maximum_bias_accel: float = 1.5
    maximum_bias_gyro: float = .15


def _check(data):
    if data.provenance!="raw_scan_registration":
        raise ValueError("independent raw scan registration provenance required")
    for name in ("lidar_t","lidar_p","lidar_R","imu_t","acc","gyro","pose_cov","t_il"):
        setattr(data,name,np.asarray(getattr(data,name),dtype=float))
        if not np.all(np.isfinite(getattr(data,name))):raise ValueError("nonfinite "+name)
    n=len(data.lidar_t); m=len(data.imu_t)
    if n<6 or m<10:raise ValueError("insufficient samples")
    if data.lidar_p.shape!=(n,3) or data.lidar_R.shape!=(n,3,3):raise ValueError("invalid LiDAR pose shape")
    if data.acc.shape!=(m,3) or data.gyro.shape!=(m,3):raise ValueError("invalid IMU shape")
    if data.t_il.shape!=(3,):raise ValueError("invalid fixed T_IL")
    for stamps in (data.lidar_t,data.imu_t):
        if np.any(np.diff(stamps)<=0):raise ValueError("nonmonotonic timestamps")
    if np.max(np.diff(data.imu_t))>data.maximum_imu_gap:raise ValueError("IMU timestamp gap")
    if data.imu_t[0]>data.lidar_t[0] or data.imu_t[-1]<data.lidar_t[-1]:raise ValueError("IMU does not bracket LiDAR")
    for value in (data.acc_scale,data.accel_sigma,data.gyro_sigma,data.rank_ratio):
        if not np.isfinite(value) or value<=0:raise ValueError("invalid positive config")
    if np.max(np.abs(np.linalg.det(data.lidar_R)-1))>1e-6 or np.max(np.abs(np.einsum("nji,njk->nik",data.lidar_R,data.lidar_R)-np.eye(3)))>1e-6:
        raise ValueError("invalid LiDAR rotation")
    pc=data.pose_cov
    if pc.shape==(n,6,6):pc=block_diag(*pc)
    if pc.shape!=(6*n,6*n) or np.max(np.abs(pc-pc.T))>1e-8:raise ValueError("invalid pose covariance")
    if np.linalg.eigvalsh(pc).min()<-1e-12:raise ValueError("pose covariance not positive semidefinite")
    return pc


def _intervals(data,rotations):
    """Midpoint integrations, with measured IMU support and LiDAR-only SO3 spline."""
    intervals=[]
    slerp=RotationSpline(data.lidar_t,Rotation.from_matrix(rotations))
    for left,right in zip(data.lidar_t[:-1],data.lidar_t[1:]):
        inside=data.imu_t[(data.imu_t>left)&(data.imu_t<right)]
        nodes=np.r_[left,inside,right]; dt=np.diff(nodes); mid=.5*(nodes[1:]+nodes[:-1])
        r=slerp(mid).as_matrix()
        a=np.column_stack([np.interp(mid,data.imu_t,data.acc[:,j]) for j in range(3)])*data.acc_scale
        w=np.column_stack([np.interp(mid,data.imu_t,data.gyro[:,j]) for j in range(3)])
        weight=(right-mid)*dt
        af=np.einsum("nij,nj->ni",r,a)
        intervals.append(dict(dt=right-left,subdt=dt,mid=mid,r=r,w=w,
             vf=np.einsum("n,ni->i",dt,af),pf=np.einsum("n,ni->i",weight,af),
             vr=np.einsum("n,nij->ij",dt,r),pr=np.einsum("n,nij->ij",weight,r)))
    return intervals


def _linear(data,p,r,intervals):
    # p_WI = p_WL - R_WI t_IL; this lever arm matters while rotating.
    pi=p-np.einsum("nij,j->ni",r,data.t_il)
    matrices=[]; values=[]
    for i,(left,right) in enumerate(zip(intervals[:-1],intervals[1:])):
        dl=left["dt"];dr=right["dt"]
        force=left["vf"]-left["pf"]/dl+right["pf"]/dr
        bias=left["vr"]-left["pr"]/dl+right["pr"]/dr
        matrices.append(np.c_[.5*(dl+dr)*np.eye(3),-bias])
        values.append((pi[i+2]-pi[i+1])/dr-(pi[i+1]-pi[i])/dl-force)
    return np.vstack(matrices),np.concatenate(values),pi


def _preintegrate(intervals,bg):
    result=[]
    for interval in intervals:
        # SO(3) preintegration uses raw gyro, independent of acceleration/gravity.
        r=np.eye(3)
        for w,dt in zip(interval["w"],interval["subdt"]):
            r=r@Rotation.from_rotvec((w-bg)*dt).as_matrix()
        result.append(r)
    return np.array(result)


def _gyro_residual(r,predicted):
    observed=np.einsum("nji,njk->nik",r[:-1],r[1:])
    return Rotation.from_matrix(np.einsum("nji,njk->nik",predicted,observed)).as_rotvec().ravel()


def _g_basis(g):
    unit=g/np.linalg.norm(g);axis=np.eye(3)[np.argmin(abs(unit))]
    first=np.cross(unit,axis);first/=np.linalg.norm(first)
    return np.c_[first,np.cross(unit,first)]


def _initial_s2_solution(a,y,intervals,maximum_bias):
    # A rough force direction is ONLY an optimizer seed, never a residual/prior.
    # Both hemispheres are optimized; the known force equation and explicit bias
    # bound distinguish the usual antipodal gravity / ~2g bias branch.
    seed=-sum((interval["vf"] for interval in intervals),np.zeros(3))
    if np.linalg.norm(seed)<1e-8:seed=np.linalg.lstsq(a,y,rcond=None)[0][:3]
    if np.linalg.norm(seed)<1e-8:seed=np.array([0.,0.,-1.])
    seed=seed/np.linalg.norm(seed)*9.809
    candidates=[]
    for reference in (seed,-seed):
        tangent=_g_basis(reference)
        def unpack(z):
            g=reference+tangent@z[:2]
            return np.r_[g/np.linalg.norm(g)*9.809,z[2:]]
        ba=np.linalg.lstsq(a[:,3:],y-a[:,:3]@reference,rcond=None)[0]
        fit=least_squares(lambda z:a@unpack(z)-y,np.r_[np.zeros(2),ba],
                          xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=150)
        value=unpack(fit.x);cost=float(np.linalg.norm(a@value-y))
        candidates.append((value,cost,fit.success))
    plausible=[c for c in candidates if np.linalg.norm(c[0][3:])<=maximum_bias]
    selected=min(plausible or candidates,key=lambda item:item[1])
    return selected[0],selected[2]


def _s2_diagnostic(a,g,ratio):
    # Mx has units m/s² per gravity-direction radian, matching IKFoM S2_Mx(0)
    # up to a harmless orthogonal change of its two tangent coordinates.
    mx=9.809*_g_basis(g)
    constrained=np.c_[a[:,:3]@mx,a[:,3:]]
    _,singular,vt=np.linalg.svd(constrained,full_matrices=False)
    rank=int(np.sum(singular>singular[0]*ratio))
    return dict(g_ba_constrained_rank=rank,s2_g_ba_rank=rank,s2_g_ba_singular_values=singular.tolist(),
                s2_g_ba_nullspace=vt[rank:].tolist(),s2_g_ba_tangent_order=["g_direction_rad_2","ba_int_I_3"],
                s2_g_Mx_world=mx.tolist(),s2_excitation_relative_threshold=ratio)


def solve_joint(data):
    pc=_check(data)
    intervals=_intervals(data,data.lidar_R)
    a,y,pi=_linear(data,data.lidar_p,data.lidar_R,intervals)
    _,singular,vt=np.linalg.svd(a,full_matrices=False)
    rank=int(np.sum(singular>singular[0]*data.rank_ratio))
    base=dict(accepted=False,reason="insufficient_g_ba_excitation",g_ba_rank=rank,
              g_ba_singular_values=singular.tolist(),g_ba_nullspace=vt[rank:].tolist(),
              acc_scale=data.acc_scale,source=data.provenance,
              branch_check_scope="enumerates detected free-rank5 one-dimensional sphere intersections; does not prove global uniqueness for arbitrary noisy full-rank motion",
              window=[float(data.lidar_t[0]),float(data.lidar_t[-1])],
              covariance_assumptions="linearized LiDAR pose covariance incl supplied correlations; raw IMU iid per-sample noise; fixed extrinsic/scale; excludes unmodeled scan registration systematic error")
    x_acc,initial_fit_success=_initial_s2_solution(a,y,intervals,data.maximum_bias_accel)
    base.update(_s2_diagnostic(a,x_acc[:3],data.rank_ratio))
    # The real model has 2 gravity-direction + 3 bias DOF. The free six-column
    # rank is diagnostic only: known |g| can remove its radial null direction.
    if base["s2_g_ba_rank"]<5:
        base.update(reason="insufficient_s2_g_ba_excitation",covariance_valid=False,
                    unresolved_direction_variance="unbounded_without_additional_independent_motion")
        return base
    bg_fit=least_squares(lambda b:_gyro_residual(data.lidar_R,_preintegrate(intervals,b)),np.zeros(3),
                         xtol=1e-9,ftol=1e-9,gtol=1e-9,max_nfev=30)
    x=np.r_[x_acc,bg_fit.x]
    predicted=_preintegrate(intervals,x[6:])
    gyro0=_gyro_residual(data.lidar_R,predicted)
    ng=len(gyro0);na=len(y);nres=na+ng;npose=len(data.lidar_t)
    # Derivative of SO(3) residual wrt gyro bias from actual raw preintegration.
    jbg=np.empty((ng,3)); eps=1e-6
    for k in range(3):
        delta=np.zeros(3);delta[k]=eps
        jbg[:,k]=(_gyro_residual(data.lidar_R,_preintegrate(intervals,x[6:]+delta))-gyro0)/eps
    # Linear measurement form e ~= d - design * x, including bg linearization.
    design=block_diag(a,-jbg)
    target=np.r_[y,gyro0-jbg@x[6:]]

    def direct(p,r,ints,x_value):
        p_imu=p-np.einsum("nij,j->ni",r,data.t_il)
        last=ints[-1];dt=last["dt"]
        # End velocity from last LiDAR displacement and IMU acceleration integral.
        vel=(p_imu[-1]-p_imu[-2])/dt+.5*x_value[:3]*dt+last["vf"]-last["pf"]/dt-(last["vr"]-last["pr"]/dt)@x_value[3:6]
        return np.r_[p_imu[-1],np.zeros(3),vel]

    # Shared LiDAR endpoints induce correlated triple-position and rotation
    # residuals. Propagate them rather than claiming each factor independent.
    jp=np.empty((nres,6*npose)); jd=np.empty((9,6*npose)); d0=direct(data.lidar_p,data.lidar_R,intervals,x)
    e0=np.r_[y-a@x[:6],gyro0]
    for i in range(npose):
        for k in range(6):
            p=data.lidar_p.copy();r=data.lidar_R.copy()
            if k<3:p[i,k]+=eps;ints=intervals
            else:
                v=np.zeros(3);v[k-3]=eps;r[i]=r[i]@Rotation.from_rotvec(v).as_matrix()
                ints=_intervals(data,r)
            ap,yp,_=_linear(data,p,r,ints)
            ep=np.r_[yp-ap@x[:6],_gyro_residual(r,predicted)]
            jp[:,6*i+k]=(ep-e0)/eps
            dp=direct(p,r,ints,x)
            if i==npose-1 and k>=3:dp[3+k-3]=eps
            jd[:,6*i+k]=(dp-d0)/eps

    # Raw sample interpolation derivatives for acceleration, preserving overlap
    # of neighboring intervals/triples. ba remains in acc_scale * raw SI domain.
    nimu=len(data.imu_t);jacc=np.zeros((na,3*nimu));jvelacc=np.zeros((3,3*nimu))
    for i,interval in enumerate(intervals):
        dt=interval["dt"]
        for tm,h,r in zip(interval["mid"],interval["subdt"],interval["r"]):
            idx=min(max(np.searchsorted(data.imu_t,tm)-1,0),nimu-2)
            frac=(tm-data.imu_t[idx])/(data.imu_t[idx+1]-data.imu_t[idx])
            # In triple i this interval is left; in triple i-1 it is right.
            wp=(data.lidar_t[i+1]-tm)*h
            for sample,weight in ((idx,1-frac),(idx+1,frac)):
                b=data.acc_scale*weight*r
                if i<len(intervals)-1:jacc[3*i:3*i+3,3*sample:3*sample+3]-=(h-wp/dt)*b
                if i>0:jacc[3*(i-1):3*i,3*sample:3*sample+3]-=wp/dt*b
                if i==len(intervals)-1:jvelacc[:,3*sample:3*sample+3]+=(h-wp/dt)*b
    # Integrating white per-sample gyro noise: rotations are correlated through
    # LiDAR covariance above. Cross-interval gyro interpolation noise is small;
    # conservative factor 2 bounds endpoint sharing for nonoverlapping scans.
    cg=np.zeros((ng,ng))
    for i,interval in enumerate(intervals):
        variance=2*data.gyro_sigma**2*np.sum(interval["subdt"]**2)
        cg[3*i:3*i+3,3*i:3*i+3]=variance*np.eye(3)
    noise=block_diag(jacc@jacc.T*data.accel_sigma**2,cg)
    cov=jp@pc@jp.T+noise+np.eye(nres)*1e-12
    lower=cholesky(cov,lower=True)
    aw=solve_triangular(lower,design,lower=True);yw=solve_triangular(lower,target,lower=True)
    free_gls=np.linalg.lstsq(aw,yw,rcond=None)[0]
    unconstrained_gravity_norm=float(np.linalg.norm(free_gls[:3]))
    # The existing front-end state has an S2 gravity of magnitude 9.809. Enforce
    # only this known magnitude while jointly reoptimizing its TWO free direction
    # coordinates and biases; simply normalizing g after fitting would change the
    # acceleration model without refitting ba. No roll/pitch prior is introduced.
    # Keep the physically screened S2 solution as a seed. A rank-deficient
    # free GLS minimum-norm solution is not a gravity direction estimate.
    seeds=[np.r_[x_acc,bg_fit.x]]
    if rank==5:
        # The one free null direction intersects the known-radius sphere in
        # up to two physically distinct states. Test both; a local tangent
        # full rank does not prove global branch uniqueness.
        _,_,weighted_vt=np.linalg.svd(aw,full_matrices=False)
        null=weighted_vt[-1];ng=null[:3];g0=free_gls[:3]
        coefficients=[float(ng@ng),float(2*g0@ng),float(g0@g0-9.809**2)]
        if coefficients[0]>1e-12:
            for root in np.roots(coefficients):
                if abs(root.imag)<1e-7:seeds.append(free_gls+float(root.real)*null)
    fits=[]
    for seed in seeds:
        reference=seed[:3]/np.linalg.norm(seed[:3])*9.809;basis=_g_basis(reference)
        def unpack(z):
            g=reference+basis@z[:2]
            return np.r_[g/np.linalg.norm(g)*9.809,z[2:]]
        fit=least_squares(lambda z:aw@unpack(z)-yw,np.r_[np.zeros(2),seed[3:]],
                          xtol=1e-11,ftol=1e-11,gtol=1e-10,max_nfev=100)
        value=unpack(fit.x);cost=float(np.linalg.norm(aw@value-yw)**2)
        if not any(np.linalg.norm(value-other[0])<1e-3 for other in fits):
            fits.append((value,cost,fit,reference,basis))
    plausible=[fit for fit in fits if fit[2].success and np.linalg.norm(fit[0][3:6])<=data.maximum_bias_accel and np.linalg.norm(fit[0][6:])<=data.maximum_bias_gyro]
    chosen=min(plausible or fits,key=lambda fit:fit[1])
    # Predeclared 1-DOF 95% chi-square separation, fixed before real motion data.
    equivalent=[fit for fit in plausible if fit[1]<=chosen[1]+3.84]
    if len(equivalent)>1:
        base.update(reason="ambiguous_s2_branch",covariance_valid=False,
                    s2_branch_delta_chi2_limit=3.84,
                    s2_branch_candidates=[dict(g_W=fit[0][:3].tolist(),ba_I=fit[0][3:6].tolist(),bg_I=fit[0][6:].tolist(),weighted_squared_residual=fit[1]) for fit in equivalent])
        return base
    x,_,constrained,g_reference,tangent=chosen
    base.update(_s2_diagnostic(a,x[:3],data.rank_ratio))
    if base["s2_g_ba_rank"]<5:
        base.update(reason="insufficient_s2_g_ba_excitation",covariance_valid=False,
                    unresolved_direction_variance="unbounded_without_additional_independent_motion")
        return base
    differential=np.zeros((9,8));gtemp=g_reference+tangent@constrained.x[:2]
    differential[:3,:2]=(np.eye(3)-np.outer(x[:3],x[:3])/9.809**2)@tangent*9.809/np.linalg.norm(gtemp)
    differential[3:,2:]=np.eye(6)
    information=aw.T@aw
    reduced_information=differential.T@information@differential
    covx=differential@np.linalg.inv(reduced_information)@differential.T
    gain=differential@np.linalg.solve(reduced_information,differential.T@aw.T)@solve_triangular(lower,np.eye(nres),lower=True)
    residual=target-design@x
    whitened=solve_triangular(lower,residual,lower=True)
    residual_dof=max(1,nres-8)  # g direction2 + ba3 + bg3.
    reduced_chi2=float(whitened@whitened/residual_dof)
    # Inflate all propagated uncertainty on model inconsistency; never shrink it
    # below the uncertainty supplied by independent scan registration/raw IMU.
    inflation=max(1.,reduced_chi2)
    d=direct(data.lidar_p,data.lidar_R,intervals,x)
    hx=np.zeros((18,9));hx[6:9,:3]=.5*intervals[-1]["dt"]*np.eye(3)
    hx[6:9,3:6]=-(intervals[-1]["vr"]-intervals[-1]["pr"]/intervals[-1]["dt"])
    hx[9:18]=np.eye(9)
    hpose=hx@gain@jp;hpose[:9]+=jd
    hacc=hx@gain[:,:na]@jacc;hacc[6:9]+=jvelacc
    hgyro=hx@gain[:,na:]
    fullcov=hpose@pc@hpose.T+hacc@hacc.T*data.accel_sigma**2+hgyro@cg@hgyro.T
    # e covariance floor represents finite modeling precision too.
    fullcov+=hx@gain@(np.eye(nres)*1e-12)@gain.T@hx.T
    fullcov=(fullcov+fullcov.T)*.5*inflation
    reason="accepted"
    if rank==6 and abs(unconstrained_gravity_norm-9.809)>data.maximum_gravity_norm_error:reason="gravity_norm_inconsistent"
    elif np.linalg.norm(x[3:6])>data.maximum_bias_accel:reason="accel_bias_above_limit"
    elif np.linalg.norm(x[6:])>data.maximum_bias_gyro:reason="gyro_bias_above_limit"
    elif not bg_fit.success:reason="gyro_optimization_failed"
    elif not initial_fit_success:reason="initial_s2_joint_optimization_failed"
    elif not constrained.success:reason="s2_joint_optimization_failed"
    elif np.linalg.eigvalsh(fullcov).min()<-1e-9:reason="covariance_not_psd"
    # Diagnostic-only EIV posterior: the GLS marginalization permits latent
    # LiDAR pose corrections. The old direct commit keeps raw LiDAR p/R,
    # so expose this mismatch before considering any algorithm change.
    latent=-pc@jp.T@np.linalg.solve(cov,residual)
    latent=latent.reshape(npose,6)
    base["eiv_pose_diagnostic"]=dict(
        order="world translation, right/body rotation",mean_is_not_applied=True,
        correction=latent.tolist(),
        max_translation_correction_m=float(np.max(np.linalg.norm(latent[:,:3],axis=1))),
        max_rotation_correction_deg=float(np.max(np.linalg.norm(latent[:,3:],axis=1))*180/np.pi),
        end_rotation_correction_deg=float(np.linalg.norm(latent[-1,3:])*180/np.pi),
        raw_commit_rotation_retained=True,
        unweighted_s2_g_ba=x_acc.tolist(),
        original_gls_g_ba=x[:6].tolist())
    base.update(accepted=reason=="accepted",reason=reason,covariance_valid=True,g_W=x[:3].tolist(),
                gravity_norm=float(np.linalg.norm(x[:3])),unconstrained_gravity_norm=unconstrained_gravity_norm,ba_I=x[3:6].tolist(),
                unconstrained_gravity_norm_identifiable=rank==6,
                ba_raw_equivalent_report_only=(x[3:6]/data.acc_scale).tolist(),bg_I=x[6:].tolist(),
                covariance_g_ba_bg=(covx*inflation).tolist(),
                full_covariance=fullcov.tolist(),full_covariance_order=["p_WI","theta_I_right","v_WI","g_W","ba_int_I","bg_I"],
                covariance_order=["p_WI","theta_I_right","v_WI","g_W","ba_I","bg_I"],
                p_WI_commit=d[:3].tolist(),R_WI_commit=data.lidar_R[-1].tolist(),v_WI_commit=d[6:9].tolist(),
                commit_t=float(data.lidar_t[-1]),residual_reduced_chi2=reduced_chi2,
                residual_degrees_of_freedom=residual_dof,residual_dimension=nres,
                residual_position_velocity_rms=float(np.sqrt(np.mean(residual[:na]**2))),
                residual_rotation_rms=float(np.sqrt(np.mean(residual[na:]**2))))
    return base
