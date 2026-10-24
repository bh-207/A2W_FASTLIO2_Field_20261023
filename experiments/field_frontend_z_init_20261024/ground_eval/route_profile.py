#!/usr/bin/env python3
"""All-eight fixed-location ground profile, frozen before candidate inspection."""
import argparse
import json
from pathlib import Path
import os
import numpy as np
import ground_eval as ge


def freeze(frozen,destination):
    frozen,destination=Path(frozen),Path(destination)
    m=ge.verify_frozen(frozen)
    if destination.exists():
        old=json.loads(destination.read_text())
        if old['ground_manifest_sha256']!=ge.sha(frozen/'manifest.json'):
            raise ValueError('existing route freeze belongs to another ground freeze')
        return old
    up=np.array(m['up'])
    regions=[r for r in m['regions'] if not r['region'].startswith('corridor_')]+[r for r in m['regions'] if r['region'].startswith('corridor_')]
    anchors=[];segments=[];station=0.
    for i,r in enumerate(regions):
        if i:
            delta=np.array(r['anchor_reference'])-regions[i-1]['anchor_reference']
            delta-=up*(delta@up)
            length=float(np.linalg.norm(delta))
            if length<1e-6:
                raise ValueError('route contains coincident fixed anchors')
            segments.append(dict(from_region=regions[i-1]['region'],to_region=r['region'],
                                 fixed_horizontal_distance_m=length,fixed_horizontal_direction=(delta/length).tolist()))
            station+=length
        anchors.append(dict(region=r['region'],route_station_m=station,
                            fixed_reference=r['anchor_reference'],fixed_cell=r['anchor_cell']))
    result=dict(schema='field_all_eight_fixed_route_v1',ground_manifest_path=str((frozen/'manifest.json').resolve()),
                ground_manifest_sha256=ge.sha(frozen/'manifest.json'),candidate_read_count_before_freeze=0,
                fixed_up=up.tolist(),anchors=anchors,segments=segments,
                selection='Same eight ground cells and exact point identities from original baseline; no new selection.',
                position_truth='Physical surveyed position and elevation UNKNOWN; fixed baseline geometric footprint proxy.',
                interpretation='Polyline segment distances/directions are frozen; off-corridor approach is not treated as a collinear corridor station.')
    ge.write_json(destination,result)
    return result


def measured(route,report,baseline=False):
    lookup={r['region']:r for r in report['regions']}
    result={}
    up=np.array(route['fixed_up'])
    for group in ge.GROUPS:
        anchors=[]
        for a in route['anchors']:
            fit=lookup[a['region']]['groups'][group]['baseline_anchor' if baseline else 'anchor']
            anchors.append(dict(**a,fit=fit))
        segments=[]
        for i,s in enumerate(route['segments']):
            a,b=anchors[i]['fit'],anchors[i+1]['fit']
            item=dict(s)
            if a['status']!='SUPPORTED' or b['status']!='SUPPORTED':
                item.update(status='UNKNOWN',reason='one_or_both_frozen_anchor_supports_missing')
            else:
                dh=b['height_m']-a['height_m'];direction=np.array(s['fixed_horizontal_direction'])
                item.update(status='SUPPORTED',height_difference_m=dh,
                            signed_segment_grade_pct=100*dh/s['fixed_horizontal_distance_m'],
                            from_local_plane_grade_pct=-100*np.dot(a['normal'],direction)/np.dot(a['normal'],up),
                            to_local_plane_grade_pct=-100*np.dot(b['normal'],direction)/np.dot(b['normal'],up),
                            inter_anchor_normal_angle_deg=ge.angle(np.array(a['normal']),np.array(b['normal'])))
            segments.append(item)
        result[group]=dict(anchors=anchors,segments=segments)
    return result


def evaluate(route_path,report_path,destination):
    route_path,report_path,destination=map(Path,(route_path,report_path,destination))
    route=json.loads(route_path.read_text());report=json.loads(report_path.read_text())
    if report['manifest_sha256']!=route['ground_manifest_sha256']:
        raise ValueError('report does not use the pre-candidate frozen ground manifest')
    if ge.sha(Path(route['ground_manifest_path']))!=route['ground_manifest_sha256']:
        raise ValueError('ground freeze changed')
    result=dict(schema='field_all_eight_route_profile_report_v1',route_freeze_sha256=ge.sha(route_path),
                source_geometry_report_sha256=ge.sha(report_path),absolute_height_and_grade_truth='UNKNOWN_NO_INDEPENDENT_SURVEY',
                evaluated=measured(route,report),baseline=measured(route,report,True))
    destination.mkdir(parents=True,exist_ok=True)
    ge.write_json(destination/'all8_route_profile.json',result)
    os.environ.setdefault('MPLCONFIGDIR','/tmp/a2w_fixed_ground_mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
    for g,color in zip(ge.GROUPS,('tab:blue','tab:green','tab:orange')):
        for mode,style in [('baseline','--'),('evaluated','-')]:
            profile=result[mode][g]
            x=[a['route_station_m'] for a in profile['anchors']]
            axes[0].plot(x,[a['fit'].get('height_m',np.nan) for a in profile['anchors']],style,marker='o',markersize=3,color=color,label=g+' '+mode)
            axes[1].plot([(x[i]+x[i+1])/2 for i in range(len(x)-1)],
                         [s.get('signed_segment_grade_pct',np.nan) for s in profile['segments']],style,marker='o',markersize=3,color=color)
    for ax in axes:ax.grid(alpha=.25)
    axes[0].legend(ncol=3,fontsize=8)
    axes[0].set_ylabel('Fixed-location ground height along original INIT up (m)')
    axes[1].set_ylabel('Signed fixed-polyline segment grade (%)')
    axes[1].set_xlabel('Original-baseline fixed horizontal polyline station (m)')
    fig.suptitle('Entry / near / middle / far ground locations; exact frozen scan-point identities\nHeight and distance scales differ; no flattening or independent absolute truth')
    fig.tight_layout();fig.savefig(destination/'all8_route_profile.png',dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    f=sub.add_parser('freeze');f.add_argument('--frozen',type=Path,required=True);f.add_argument('--out',type=Path,required=True)
    e=sub.add_parser('evaluate');e.add_argument('--route-frozen',type=Path,required=True);e.add_argument('--report',type=Path,required=True);e.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='freeze':
        r=freeze(args.frozen,args.out);print('FROZEN',len(r['anchors']),'fixed anchors; candidate reads',r['candidate_read_count_before_freeze'])
    else:
        r=evaluate(args.route_frozen,args.report,args.out);print('EVALUATED all8 fixed route profile')
