"""Full XYZ diagnostic of frozen ground anchors; never used to flatten maps."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from motion_audit import chord_metrics


def run(source,output):
    source=Path(source);output=Path(output)
    if output.exists():raise ValueError('new shape output required')
    output.mkdir(parents=True);g=json.loads(source.read_text());up=np.array(g['fixed_up']);result={}
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(14,11));a=fig.add_subplot(221,projection='3d');b=fig.add_subplot(222);c=fig.add_subplot(223);d=fig.add_subplot(224)
    for group,style in [('train','-'),('heldout','--'),('revisit',':')]:
        rows=g['runs']['B009']['profiles'][group]['anchors']
        if any(x['status']!='SUPPORTED' for x in rows):
            result[group]=dict(status='UNKNOWN',reason='frozen anchor unsupported');continue
        p=np.array([x['same_fixed_up_reference_ground_xyz']for x in rows]);q=chord_metrics(p)
        direction=(p[-1]-p[0])/q['chord_length_m'];vertical=up-direction*(direction@up);vertical/=np.linalg.norm(vertical)
        lateral=np.cross(direction,vertical)
        displacement=p-p[0];v=displacement@vertical;lat=displacement@lateral
        station=np.array([x['station_m']for x in rows]);h=p@up
        q.update(status='DIAGNOSTIC_NOT_TRUTH',anchors=rows,up=up.tolist(),
                 in_chord_up_plane_deviation_m=v.tolist(),lateral_deviation_m=lat.tolist(),
                 entry_to_peak_m=float(np.max(h)-h[0]),peak_to_end_m=float(h[-1]-np.max(h)),
                 endpoint_native_Z_difference_m=float(p[-1,2]-p[0,2]),endpoint_up_difference_m=float(h[-1]-h[0]),
                 independent_survey='UNKNOWN',map_modified=False)
        result[group]=q
        a.plot(p[:,0],p[:,1],p[:,2],style,label=group);b.plot(station,h-h[0],style,label=group)
        c.plot(station,v,style,label=group+' in chord/up plane');c.plot(station,lat,style,label=group+' lateral',alpha=.5)
        d.plot(.5*(station[1:]+station[:-1]),q['segment_to_chord_deg'],style,label=group)
    a.set_xlabel('native X m');a.set_ylabel('native Y m');a.set_zlabel('native Z m')
    p=np.array([x['same_fixed_up_reference_ground_xyz']for x in result['train']['anchors']]);a.plot(p[[0,-1],0],p[[0,-1],1],p[[0,-1],2],'k--',label='endpoint chord')
    a.set_box_aspect([1,1,.65]);a.view_init(elev=20,azim=-65)
    a.set_xticks([float(p[:,0].min()),float(p[:,0].max())]);a.set_yticks([-315,-150,0]);a.set_zticks([-6,-3,0])
    a.set_title('3D display axes independently scaled; metrics use original XYZ',fontsize=8)
    for k,label in [(0,'E'),(4,'M'),(7,'F')]:a.text(*p[k],label,fontsize=10)
    for ax,y in [(b,'height relative to entry (m)'),(c,'chord-perpendicular component (m)'),(d,'3D segment vs chord (deg)')]:
        ax.set_xlabel('frozen route station (m)');ax.set_ylabel(y);ax.grid(alpha=.3);ax.legend(fontsize=7)
    a.legend();fig.suptitle('B009 fixed physical-query anchors: coordinate reference vs actual XYZ polyline shape\nNo local ICP, trend subtraction, or per-candidate up selection')
    fig.tight_layout();fig.savefig(output/'xyz_chord_and_height.png',dpi=160);plt.close(fig)
    record=dict(schema='frozen_anchor_xyz_chord_shape_v1',source=str(source.resolve()),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        groups=result,interpretation='Chord distance and 3D angles are invariant to a single SE3. They include actual route curvature and terrain; they are diagnostics, not a flattening objective or drift ground truth.')
    (output/'shape.json').write_text(json.dumps(record,indent=2))
    print(json.dumps({k:{x:v[x]for x in ['entry_to_peak_m','peak_to_end_m','chord_distance_m','in_chord_up_plane_deviation_m','turn_angle_deg']}for k,v in result.items()},indent=2))
    return record


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('output');a=p.parse_args();run(a.source,a.output)
