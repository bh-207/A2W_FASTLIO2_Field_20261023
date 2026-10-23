"""Read-only floor-like local patch height survey. No SLAM update or map flattening."""
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
from collections import Counter

os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['MPLCONFIGDIR']='/tmp/a2w_floor_height_mpl'
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation

WS=Path('/home/bh/A2W_FastLio2_controlled')
OUT=Path(__file__).resolve().parent
SESSION=WS/'captures/field_fixed_sessions/fixed_short_20261023_001'
BUNDLE=SESSION/'bundles/map_bundle'
sys.path.insert(0,str(WS/'experiments/offline_v4/gravity'))
import gravity_geometry as io


def rows(path):
    with path.open() as f:return list(csv.DictReader(f))


def pose(row):
    t=np.eye(4);t[:3,3]=[float(row['t'+a]) for a in 'xyz']
    t[:3,:3]=Rotation.from_quat([float(row['q'+a]) for a in 'xyzw']).as_matrix()
    return t


def span(x):
    v=np.asarray(x,float)
    return dict(count=len(v),min_m=float(v.min()),max_m=float(v.max()),max_minus_min_m=float(np.ptp(v)),
                p01_m=float(np.percentile(v,1)),p99_m=float(np.percentile(v,99)),
                p99_minus_p01_m=float(np.percentile(v,99)-np.percentile(v,1)))


def main():
    raw=rows(BUNDLE/'keyframes/index.csv'); optimized=rows(BUNDLE/'optimized_poses.csv')
    if [r['id'] for r in raw]!=[r['id'] for r in optimized]:raise ValueError('pose IDs differ')
    init=next(r for r in rows(SESSION/'runtime_state.csv') if r['stage']=='INIT_COMMIT')
    g=np.array([float(init['g_O_'+a]) for a in 'xyz']);upO=-g/np.linalg.norm(g)
    C0=pose(optimized[0])@np.linalg.inv(pose(raw[0]));up=C0[:3,:3]@upO
    # One fixed map direction only; neither poses nor points are changed.
    ex=np.array([1.,0.,0.]);ex-=up*(ex@up);ex/=np.linalg.norm(ex)
    ey=np.cross(up,ex);basis=np.column_stack([ex,ey,up])
    protocol=dict(source_bundle=str(BUNDLE),map_frame='map',height_reference='same-run INIT up mapped by first saved optimized/raw SE3',
                  up_map=up.tolist(),initial_g_camera_init=g.tolist(),map_from_camera_init_at_first_saved_keyframe=C0.tolist(),
                  point_selection='source keyframe body points, within 3m horizontally and 0.15..2.5m below observer along fixed initial up',
                  local_cells_m=.6,minimum_points=8,minimum_second_eigenvalue_m2=.0036,
                  maximum_smallest_to_second_eigenvalue_ratio=.08,maximum_plane_abs_residual_p95_m=.04,
                  maximum_normal_to_initial_up_deg=35.,slope_free=True,
                  semantic_identity='floor-like planar candidates, not manually labelled or surveyed ground',
                  excluded_surfaces='walls/ceilings/out-of-nearby-support or nonplanar cells explicitly rejected; unobserved ground unknown',
                  transformations_applied_to_map=False,absolute_height_accuracy_certified=False)
    (OUT/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2)+'\n')
    centers=[];ids=[];counts=[];normals=[];radii=[];rejections=Counter();source_count=0
    nearpoints=[]
    for i,(r,o) in enumerate(zip(raw,optimized)):
        _,arr=io.read_pcd(BUNDLE/r['cloud']);body=io.xyz(arr);T=pose(o)
        relative=body@T[:3,:3].T;local=relative@basis
        mask=np.isfinite(local).all(1)&(np.linalg.norm(local[:,:2],axis=1)<=3)&(local[:,2]>=-2.5)&(local[:,2]<=-.15)
        local=local[mask];source_count+=len(local)
        if len(local)<8:rejections['scan_insufficient_near_low_points']+=1;continue
        keys=np.floor(local[:,:2]/.6).astype(np.int64)
        order=np.lexsort((keys[:,1],keys[:,0]));cuts=np.r_[0,1+np.flatnonzero(np.any(np.diff(keys[order],axis=0),axis=1)),len(order)]
        for lo,hi in zip(cuts[:-1],cuts[1:]):
            p=local[order[lo:hi]]
            if len(p)<8:rejections['cell_fewer_than_8_points']+=1;continue
            center=p.mean(0);delta=p-center
            v,V=np.linalg.eigh(delta.T@delta/len(p));n=V[:,0]
            if n[2]<0:n=-n
            if v[1]<.0036:rejections['cell_insufficient_2d_support']+=1;continue
            if v[0]/v[1]>.08:rejections['cell_nonplanar_or_mixed_surface']+=1;continue
            if n[2]<np.cos(np.deg2rad(35)):rejections['cell_wall_or_excessive_tilt']+=1;continue
            if np.percentile(np.abs(delta@n),95)>.04:rejections['cell_plane_residual_too_large']+=1;continue
            centerM=center@basis.T+T[:3,3]
            centers.append(centerM);ids.append(i);counts.append(len(p));normals.append(basis@n);radii.append(np.linalg.norm(center[:2]))
            nearpoints.append(p@basis.T+T[:3,3])
        if i%150==0:print('processed',i,'of',len(raw),flush=True)
    centers=np.asarray(centers);ids=np.asarray(ids);radii=np.asarray(radii);nearpoints=np.concatenate(nearpoints)
    if not len(centers):raise ValueError('no qualified floor-like patches')
    trajectory=np.asarray([pose(r)[:3,3] for r in optimized])
    _,allarr=io.read_pcd(BUNDLE/'global_map.pcd');allxyz=io.xyz(allarr)
    result=dict(ground_identity_certified=False,reference_up_map=up.tolist(),
                init_anti_gravity_to_native_z_deg=float(np.rad2deg(np.arccos(np.clip(up[2],-1,1)))),
                keyframe_count=len(raw),qualified_floor_like_patch_count=len(centers),qualified_floor_like_point_count=len(nearpoints),
                near_low_source_points=source_count,rejected= dict(rejections),
                floor_like_patch_centers_native_Z=span(centers[:,2]),
                floor_like_patch_centers_initial_gravity_height=span(centers@up),
                floor_like_qualified_points_native_Z=span(nearpoints[:,2]),
                floor_like_qualified_points_initial_gravity_height=span(nearpoints@up),
                trajectory_native_Z=span(trajectory[:,2]),trajectory_initial_gravity_height=span(trajectory@up),
                whole_map_native_Z=span(allxyz[:,2]),
                caveat='Estimated map floor-like heights include real slopes/steps, coordinate tilt and mapping error. No surveyed elevation truth; the span is not drift error.')
    # Near footprint subset is separately shown, never replaces the full patch result.
    footprint=radii<=1.5
    result['near_footprint_patches_radius_le_1_5m']=dict(count=int(footprint.sum()),native_Z=span(centers[footprint,2]),initial_gravity_height=span(centers[footprint]@up))
    per_frame=[]
    for i,T in enumerate(trajectory):
        chosen=centers[ids==i]
        per_frame.append(dict(keyframe=i,stamp_ns=int(raw[i]['stamp_ns']),patch_count=len(chosen),
                              map_x=T[0],map_y=T[1],body_z=T[2],
                              floor_patch_median_z=float(np.median(chosen[:,2])) if len(chosen) else None,
                              floor_patch_median_initial_gravity_height=float(np.median(chosen@up)) if len(chosen) else None))
    good=[r for r in per_frame if r['patch_count']>=3]
    result['per_keyframe_median_floor_height']=dict(qualified_frames=len(good),
                native_Z=span([r['floor_patch_median_z'] for r in good]),
                initial_gravity_height=span([r['floor_patch_median_initial_gravity_height'] for r in good]))
    for coord,label in [(centers[:,2],'native_Z'),(centers@up,'initial_gravity_height')]:
        low=int(np.argmin(coord));high=int(np.argmax(coord))
        result['extrema_'+label]={'low':dict(keyframe=int(ids[low]),center_map=centers[low].tolist(),height_m=float(coord[low])),
                                  'high':dict(keyframe=int(ids[high]),center_map=centers[high].tolist(),height_m=float(coord[high]))}
    with (OUT/'floor_profile.csv').open('x') as stream:
        w=csv.DictWriter(stream,fieldnames=list(per_frame[0]));w.writeheader();w.writerows(per_frame)
    np.savez_compressed(OUT/'patches.npz',centers_map=centers,frame_ids=ids,counts=np.asarray(counts),normals_map=np.asarray(normals),up_map=up)
    (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(14,9))
    for ax,values,title in [(axes[0,0],centers[:,2],'Floor-like patches: native map Z (m)'),(axes[0,1],centers@up,'Same patches: initial-gravity projection (m)')]:
        im=ax.scatter(centers[:,0],centers[:,1],c=values,s=3,cmap='viridis');fig.colorbar(im,ax=ax);ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)')
    t=np.array([(r['stamp_ns']-int(raw[0]['stamp_ns']))/1e9 for r in good])
    axes[1,0].plot(t,[r['floor_patch_median_z'] for r in good]);axes[1,0].set_title('Per-keyframe median floor-like patch Z');axes[1,0].set_ylabel('native Z (m)')
    axes[1,1].plot(t,[r['floor_patch_median_initial_gravity_height'] for r in good]);axes[1,1].set_title('Same patches: fixed initial-gravity height');axes[1,1].set_ylabel('projected height (m)')
    for ax in axes[1]:ax.set_xlabel('Sensor time from first saved keyframe (s)');ax.grid(alpha=.25)
    fig.suptitle('New 828KF map: local planar low-surface diagnostic; no surveyed ground truth')
    fig.tight_layout();fig.savefig(OUT/'ground_height.png',dpi=150);plt.close(fig)
    sources=[BUNDLE/'global_map.pcd',BUNDLE/'keyframes/index.csv',BUNDLE/'optimized_poses.csv',SESSION/'runtime_state.csv',Path(__file__)]
    manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (OUT/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
