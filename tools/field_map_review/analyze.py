"""Read-only analysis of the fixed-extrinsic 828KF field map; no registration/ICP."""
import csv,json,re,os,sys,hashlib
from collections import Counter
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['MPLCONFIGDIR']='/tmp/a2w_field_review_mpl'
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
WS=Path('/home/bh/A2W_FastLio2_controlled');OUT=Path(__file__).resolve().parent
S=WS/'captures/field_fixed_sessions/fixed_short_20261023_001';B=S/'bundles/map_bundle'
sys.path.insert(0,str(WS/'experiments/offline_v4/gravity'))
import gravity_geometry as io


def read(p):
    with p.open() as f:return list(csv.DictReader(f))


def T(row):
    m=np.eye(4);m[:3,3]=[float(row['t'+a]) for a in 'xyz'];m[:3,:3]=Rotation.from_quat([float(row['q'+a]) for a in 'xyzw']).as_matrix();return m


def stats(x):
    x=np.asarray(x,float);x=x[np.isfinite(x)]
    if not len(x):return dict(count=0)
    return dict(count=len(x),min=float(x.min()),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(x.max()),mean=float(x.mean()))


def angle(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    return np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b),axis=-1),np.sum(a*b,axis=-1)))


def plane(p):
    if len(p)<8:return None
    c=p.mean(0);d=p-c;v,V=np.linalg.eigh(d.T@d/len(p));n=V[:,0]
    if n@UP<0:n=-n
    residual=np.percentile(abs(d@n),95)
    if v[1]<.0025 or v[0]/max(v[1],1e-12)>.08 or residual>.05 or n@UP<np.cos(np.deg2rad(35)):return None
    return c,n,float(residual)


def cell_floor(groups,cells):
    result=[];failed=Counter()
    maps=[]
    for q,k in zip(groups,cells):
        m={}
        for key in np.unique(k,axis=0):
            idx=np.all(k==key,axis=1);m[tuple(key)]=q[idx]
        maps.append(m)
    shared=set(maps[0])&set(maps[1])&set(maps[2])
    for key in sorted(shared):
        fits=[plane(m[key]) for m in maps]
        if any(f is None for f in fits):failed['not_planar_in_all_three_groups']+=1;continue
        c,n,_=fits[0];target_c=fits[2][0];reference=(c+target_c)/2
        # Evaluate each free plane at the same horizontal location: n·(X-c)=0.
        # X=reference+lambda*UP, so height=reference·UP - n·(reference-c)/(n·UP).
        heights=[float(reference@UP-f[1]@(reference-f[0])/(f[1]@UP)) for f in fits]
        normal=float(angle(fits[0][1],fits[2][1]));hold=float(angle(fits[0][1],fits[1][1]))
        result.append(dict(cell=list(key),return_height_minus_train_m=heights[2]-heights[0],
                           holdout_height_minus_train_m=heights[1]-heights[0],return_normal_angle_deg=normal,
                           holdout_normal_angle_deg=hold,counts=[len(m[key]) for m in maps],
                           normal_agreement_le_10_deg=normal<=10 and hold<=10))
    good=[r for r in result if r['normal_agreement_le_10_deg']]
    return dict(shared_cells=len(shared),planar_cells=len(result),normal_consistent_cells=len(good),failed=dict(failed),
                absolute_return_height_difference_m=stats([abs(r['return_height_minus_train_m']) for r in good]),
                signed_return_height_difference_m=stats([r['return_height_minus_train_m'] for r in good]),
                absolute_holdout_height_difference_m=stats([abs(r['holdout_height_minus_train_m']) for r in good]),
                return_normal_angle_deg=stats([r['return_normal_angle_deg'] for r in good]),all_planar_cells=result,
                physical_surface_identity='unknown; same fixed XY cell and independent scan groups only')


def wall_metrics(train,query):
    if min(len(train),len(query))<25:return dict(status='unknown',train_count=len(train),query_count=len(query))
    d,ix=cKDTree(train).query(query,k=12,workers=1)
    neighborhoods=train[ix];center=neighborhoods.mean(1);delta=neighborhoods-center[:,None,:]
    val,vec=np.linalg.eigh(np.einsum('nki,nkj->nij',delta,delta)/12);n=vec[:,:,0]
    selected=(d[:,-1]<=.8)&(val[:,1]>=.0005)&(val[:,0]/np.maximum(val[:,1],1e-12)<=.08)&(np.abs(n@UP)<.5)
    distance=np.abs(np.sum((query-center)*n,axis=1))
    return dict(status='measured_local_wall_like_surfaces',train_count=len(train),query_count=len(query),
                nearest_neighbor_distance_all_m=stats(d[:,0]),planar_wall_supported_count=int(selected.sum()),
                planar_wall_support_fraction=float(selected.mean()),point_to_local_plane_abs_m=stats(distance[selected]),
                note='No ICP or local pose adjustment. Point-to-plane support is descriptive, not independently identified objects.')


def main():
    global UP
    raw=read(B/'keyframes/index.csv');opt=read(B/'optimized_poses.csv')
    TR=np.array([T(r) for r in raw]);TO=np.array([T(r) for r in opt]);pos=TO[:,:3,3];rawpos=TR[:,:3,3]
    stamps=np.array([int(r['stamp_ns']) for r in raw],np.int64);time=(stamps-stamps[0])/1e9
    runtime=read(S/'runtime_state.csv');init=next(r for r in runtime if r['stage']=='INIT_COMMIT')
    gv=np.array([float(init['g_O_'+a]) for a in 'xyz']);C0=TO[0]@np.linalg.inv(TR[0]);UP=C0[:3,:3]@(-gv/np.linalg.norm(gv))
    ex=np.array([1.,0.,0.]);ex-=UP*(ex@UP);ex/=np.linalg.norm(ex);basis=np.column_stack([ex,np.cross(UP,ex),UP])
    logs=(S/'logs/backend.log').read_text().splitlines();committed=[];profiles=[]
    for l in logs:
        if '[LoopCommitted]' in l:
            d=dict(re.findall(r'(\w+)=([^\s]+)',l));i=int(d['source_id']);j=int(d['target_id'])
            for k in ['source_id','target_id','sensor_stamp_ns','commit_wall_ns','correspondences','factors']:d[k]=int(d[k])
            for k in ['fitness','overlap','optimized_translation_m','optimized_rotation_rad','gtsam_ms','process_to_commit_ms']:d[k]=float(d[k])
            d.update(source_relative_s=float(time[i]),target_relative_s=float(time[j]),time_separation_s=float(time[i]-time[j]),
                     raw_pose_separation_m=float(np.linalg.norm(rawpos[i]-rawpos[j])),optimized_pose_separation_m=float(np.linalg.norm(pos[i]-pos[j])),
                     optimized_pose_z_difference_m=float(pos[i,2]-pos[j,2]),optimized_pose_up_difference_m=float((pos[i]-pos[j])@UP),
                     optimized_rotation_correction_deg=float(np.rad2deg(d['optimized_rotation_rad'])))
            committed.append(d)
        if '[LoopProfile]' in l:profiles.append(l)
    reasons=Counter(re.search(r' reason=(.*?) sc_distance=',l).group(1) for l in profiles)
    saved_post=[r for r in runtime if r['stage']=='POST_LIDAR_UPDATE' and int(r['state_stamp_ns'])<=int(stamps[-1])+1000]
    ptime=(np.array([int(r['state_stamp_ns']) for r in saved_post],np.int64)-stamps[0])/1e9
    def xyz(prefix, axes='xyz'):return np.array([[float(r[prefix+a]) for a in axes] for r in saved_post])
    state_g=xyz('g_O_');ba=xyz('ba_I_');bg=xyz('bg_I_');vel=xyz('vel_O_I_');state_p=xyz('pos_O_I_')
    q=xyz('q_O_I_','xyzw');rpy=Rotation.from_quat(q).as_euler('xyz',degrees=True)
    gangle=angle(state_g/np.linalg.norm(state_g,axis=1)[:,None],gv/np.linalg.norm(gv))
    b0=np.array([float(init['ba_I_'+a]) for a in 'xyz']);w0=np.array([float(init['bg_I_'+a]) for a in 'xyz'])
    state=dict(saved_POST=len(saved_post),all_POST=len(runtime)-1,init_gravity_camera_init=gv.tolist(),init_up_map=UP.tolist(),
               initial_anti_g_tilt_deg=float(np.degrees(np.arccos(UP[2]))),g_direction_change_deg=stats(gangle),last_g_direction_change_deg=float(gangle[-1]),
               ba_change_from_INIT_m_s2=stats(np.linalg.norm(ba-b0,axis=1)),bg_change_from_INIT_rad_s=stats(np.linalg.norm(bg-w0,axis=1)),
               roll_deg=stats(rpy[:,0]),pitch_deg=stats(rpy[:,1]),speed_m_s=stats(np.linalg.norm(vel,axis=1)),
               fixed_extrinsic_max_T_error_m=float(max(abs(float(r['t_I_L_'+a])-v) for r in runtime for a,v in zip('xyz',[-.00862,.00914,.03916]))),
               fixed_extrinsic_max_R_quaternion_error=float(max(abs(float(r['q_I_L_'+a])-v) for r in runtime for a,v in zip('xyzw',[0,0,0,1]))))
    correction_p=pos-rawpos;correction_a=np.degrees(Rotation.from_matrix(TO[:,:3,:3]@TR[:,:3,:3].transpose(0,2,1)).magnitude())
    trajectory=dict(saved_duration_s=float(time[-1]),raw_XY_length_m=float(np.linalg.norm(np.diff(rawpos[:,:2],axis=0),axis=1).sum()),
                    optimized_XY_length_m=float(np.linalg.norm(np.diff(pos[:,:2],axis=0),axis=1).sum()),
                    optimized_endpoint_displacement_xyz_m=(pos[-1]-pos[0]).tolist(),optimized_endpoint_displacement_m=float(np.linalg.norm(pos[-1]-pos[0])),
                    endpoint_is_same_physical_place_verified=False,
                    optimized_position_correction_m=stats(np.linalg.norm(correction_p,axis=1)),optimized_Z_correction_m=stats(correction_p[:,2]),
                    max_absolute_optimized_Z_correction_m=float(np.abs(correction_p[:,2]).max()),optimized_angle_correction_deg=stats(correction_a))
    far=int(np.argmin(pos[:,1]));regions=[]
    for y in [-25.,-75.,-125.,-175.,-225.,-275.,-315.]:
        a=int(np.argmin(abs(pos[:far+1,1]-y)));j=far+1+int(np.argmin(abs(pos[far+1:,1]-y)))
        ids_out=np.arange(max(0,a-4),min(far+1,a+5));ids_back=np.arange(max(far+1,j-4),min(len(raw),j+5))
        regions.append(dict(name='corridor_Y_'+str(int(y)),a=a,b=j,train=ids_out[ids_out%2==0].tolist(),heldout=ids_out[ids_out%2==1].tolist(),revisit=ids_back.tolist(),
                            center=((pos[a]+pos[j])/2).tolist(),pose_separation_m=float(np.linalg.norm(pos[a]-pos[j])),time_separation_s=float(time[j]-time[a])))
    regions.append(dict(name='loop_788_90_outside_registration_neighborhood',a=90,b=788,train=[96,98,100,102,104],heldout=[97,99,101,103,105],revisit=list(range(794,803)),
                        center=((pos[90]+pos[788])/2).tolist(),pose_separation_m=float(np.linalg.norm(pos[90]-pos[788])),time_separation_s=float(time[788]-time[90]),
                        note='Excludes target85..95/current783..793 used by configured +/-5 keyframe local maps; view overlap still not surveyed identity.'))
    policy=dict(regions=regions,reference='one saved map gauge; no per-region ICP',ROI_horizontal_radius_m=6.,source_range_m=12.,voxel_m=.12,
                floor_relative_up_to_observer_m=[-2.5,-.15],floor_cells_m=.75,plane_abs_residual_p95_limit_m=.05,plane_tilt_limit_deg=35,
                evaluation='same selected scan point identities for raw/optimized, heldout first-pass vs return, selection frozen using existing optimized map before metric calculation',
                ground_truth='unknown; geometric proxies only',all_ROIs_retained=True)
    (OUT/'geometry_protocol.json').write_text(json.dumps(policy,indent=2)+'\n')
    cache={};panels={};scores=[]
    def cloud(i):
        if i not in cache:
            cache[i]=io.xyz(io.read_pcd(B/raw[i]['cloud'])[1])
        return cache[i]
    for region in regions:
        center=np.asarray(region['center']);data={k:[] for k in ['raw','optimized','floor','cell']}
        for group in ['train','heldout','revisit']:
            qr=[];qo=[];flo=[];cell=[]
            for i in region[group]:
                body=cloud(i);qopt=body@TO[i,:3,:3].T+pos[i];qraw=body@TR[i,:3,:3].T+rawpos[i]
                inside=(np.linalg.norm(body,axis=1)<=12)&(np.linalg.norm(((qopt-center)@basis)[:,:2],axis=1)<=6)
                ix=np.flatnonzero(inside)
                _,unique=np.unique(np.floor(qopt[ix]/.12).astype(np.int64),axis=0,return_index=True);ix=ix[np.sort(unique)]
                h=(qopt[ix]-pos[i])@UP;qr.append(qraw[ix]);qo.append(qopt[ix]);flo.append((h>=-2.5)&(h<=-.15))
                cell.append(np.floor((qopt[ix]@basis)[:,:2]/.75).astype(np.int64))
            for key,value in [('raw',qr),('optimized',qo),('floor',flo),('cell',cell)]:data[key].append(np.concatenate(value))
        panels[region['name']]=data
        for mode in ['raw','optimized']:
            low=[q[f] for q,f in zip(data[mode],data['floor'])];cells=[c[f] for c,f in zip(data['cell'],data['floor'])]
            walls=[q[~f] for q,f in zip(data[mode],data['floor'])]
            score=dict(region=region['name'],mode=mode,pose_pair_distance_m=region['pose_separation_m'],
                       floor=cell_floor(low,cells),wall_return=wall_metrics(walls[0],walls[2]),wall_holdout=wall_metrics(walls[0],walls[1]),
                       group_point_counts=[len(q) for q in data[mode]])
            scores.append(score)
        print('evaluated',region['name'],flush=True)
    report=dict(source_session=str(S),loops_committed=committed,loop_profile_count=len(profiles),loop_profile_reason_counts=dict(reasons),
                trajectory=trajectory,state=state,geometry=scores,physical_accuracy_certified=False,source_Bundle_verified=None)
    (OUT/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False,
        default=lambda value: value.item() if isinstance(value,np.generic) else value.tolist())+'\n')
    np.savez_compressed(OUT/'state_and_trajectory.npz',saved_time_s=time,raw_positions=rawpos,optimized_positions=pos,post_time_s=ptime,g=state_g,gangle_deg=gangle,ba=ba,bg=bg,rpy_deg=rpy,post_position=state_p)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(14,9))
    axes[0,0].plot(rawpos[:,0],rawpos[:,1],label='raw',lw=1);axes[0,0].plot(pos[:,0],pos[:,1],label='optimized',lw=1);axes[0,0].set_aspect('equal');axes[0,0].set_title('Saved route, about 701m XY');axes[0,0].legend()
    for e in committed:
        i=e['source_id'];j=e['target_id'];axes[0,0].plot(pos[[i,j],0],pos[[i,j],1],'r-');axes[0,0].annotate(f'{i}->{j}',pos[i,:2])
    axes[0,1].plot(time,rawpos[:,2],label='raw');axes[0,1].plot(time,pos[:,2],label='optimized');axes[0,1].set_title('Native Z, not drift truth');axes[0,1].legend()
    axes[1,0].plot(time,pos@UP,label='fixed INIT-up height');axes[1,0].set_title('Same route, initial-gravity projection');axes[1,0].legend()
    axes[1,1].plot(time,np.linalg.norm(correction_p,axis=1),label='position correction (m)');axes[1,1].plot(time,correction_p[:,2],label='signed Z correction (m)');axes[1,1].legend();axes[1,1].set_title('Backend correction, same global gauge')
    for ax in axes.flat:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(OUT/'trajectory_and_backend.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(14,11))
    axes[0,0].plot(ptime,gangle);axes[0,0].set_title('World g direction change from INIT (deg)')
    axes[0,1].plot(ptime,rpy[:,:2]);axes[0,1].legend(['roll','pitch']);axes[0,1].set_title('Body attitude (deg): includes normal robot motion')
    axes[1,0].plot(ptime,ba);axes[1,0].legend(['ba x','ba y','ba z']);axes[1,0].set_title('Estimated accelerometer bias (m/s^2)')
    axes[1,1].plot(ptime,bg);axes[1,1].legend(['bg x','bg y','bg z']);axes[1,1].set_title('Estimated gyro bias (rad/s)')
    axes[2,0].plot(ptime,state_p[:,2]);axes[2,0].set_title('Frontend native Z (m)')
    axes[2,1].plot(time,correction_a);axes[2,1].set_title('Saved backend angular correction (deg)')
    for ax in axes.flat:ax.grid(alpha=.3);ax.set_xlabel('Sensor time from first saved keyframe (s)')
    fig.tight_layout();fig.savefig(OUT/'state_diagnostics.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(4,2,figsize=(15,14))
    for ax,region in zip(axes.flat,regions):
        data=panels[region['name']];center=np.asarray(region['center']);colors=['tab:blue','tab:green','tab:orange']
        for q,color,label in zip(data['optimized'],colors,['outbound train','outbound heldout','return']):
            step=max(1,len(q)//4500);local=(q[::step]-center)@basis
            ax.scatter(local[:,1],local[:,2],s=.4,alpha=.45,color=color,label=label)
        ax.set_xlim(-6,6);ax.set_ylim(-3,7);ax.set_aspect('equal');ax.set_title(region['name']);ax.set_xlabel('Local fixed-axis along-corridor (m)');ax.set_ylabel('Fixed INIT-up height (m)')
    axes[0,0].legend(markerscale=5,fontsize=8);fig.suptitle('Same saved gauge, no local ICP; orange return vs first pass. Equal scales.')
    fig.tight_layout();fig.savefig(OUT/'local_return_sections.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(loops=len(committed),trajectory=trajectory,state=state),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
