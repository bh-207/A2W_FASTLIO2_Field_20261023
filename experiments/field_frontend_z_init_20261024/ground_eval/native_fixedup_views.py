#!/usr/bin/env python3
"""Coordinate views of the existing frozen ground anchors; no new fit or selection."""
import argparse
import csv
import json
import os
from pathlib import Path
import numpy as np
import ground_eval as ge


def coordinate_profile(route,report,locations):
    up=np.array(route['fixed_up'])
    lookup={r['region']:r for r in report['regions']}
    original={(r['region'],r['group']):np.array(r['baseline_ground_xyz']) for r in locations['locations']}
    result={}
    for g in ge.GROUPS:
        anchors=[]
        for a in route['anchors']:
            fit=lookup[a['region']]['groups'][g]['anchor']
            row=dict(region=a['region'],station_m=a['route_station_m'],status=fit['status'])
            if fit['status']=='SUPPORTED':
                ref=np.array(a['fixed_reference']);h=fit['height_m']
                q=ref+up*(h-ref@up)
                row.update(fixed_up_height_m=h,native_z_m=float(q[2]),same_fixed_up_reference_ground_xyz=q.tolist(),normal=fit['normal'])
            else:
                row['reason']=fit.get('reason','frozen_anchor_unsupported')
            anchors.append(row)
        segments=[]
        for i,s in enumerate(route['segments']):
            a,b=anchors[i:i+2]
            native_delta=original[(b['region'],g)]-original[(a['region'],g)]
            native_distance=float(np.linalg.norm(native_delta[:2]))
            row=dict(from_region=a['region'],to_region=b['region'],
                     fixed_up_horizontal_distance_m=s['fixed_horizontal_distance_m'],
                     B004_fixed_native_xy_distance_m=native_distance)
            if a['status']=='SUPPORTED' and b['status']=='SUPPORTED':
                dh=b['fixed_up_height_m']-a['fixed_up_height_m'];dz=b['native_z_m']-a['native_z_m']
                row.update(status='SUPPORTED',fixed_up_height_difference_m=dh,native_z_height_difference_m=dz,
                           fixed_up_signed_grade_pct=100*dh/s['fixed_horizontal_distance_m'],
                           native_z_change_per_frozen_native_xy_distance_pct=100*dz/native_distance,
                           native_z_reference_projection_term_m=dz-up[2]*dh)
            else:
                row.update(status='UNKNOWN',reason='missing_frozen_anchor_support')
            segments.append(row)
        result[g]=dict(anchors=anchors,segments=segments)
    return result


def make(route_path,locations_path,runs,destination):
    route_path,locations_path,destination=map(Path,(route_path,locations_path,destination))
    route=json.loads(route_path.read_text());locations=json.loads(locations_path.read_text())
    if route['ground_manifest_sha256']!=locations['ground_manifest_sha256']:
        raise ValueError('coordinate reference locations do not use the identical frozen ground selection')
    result=dict(schema='field_frozen_native_z_and_fixed_up_views_v1',route_freeze_sha256=ge.sha(route_path),
                fixed_locations_sha256=ge.sha(locations_path),ground_manifest_sha256=route['ground_manifest_sha256'],
                fixed_up=route['fixed_up'],runs={},
                native_z_definition='Native map Z of the SAME free-plane intersection on the original fixed-up line through each frozen horizontal reference. No separate candidate ray, rotation or point selection.',
                native_grade_definition='Native-Z change divided by native-XY distance of the B004 ground-anchor pair frozen before candidate reading. Fixed-up grade uses the original frozen up-horizontal distance.',
                projection_identity='delta native Z = up_z * delta fixed-up height + delta frozen-reference native Z',
                absolute_height_grade_accuracy='UNKNOWN_WITHOUT_INDEPENDENT_SURVEY',
                inference='C runs that requested a rejected initializer and fell back are original-algorithm executions, not joint-enabled accuracy evidence.')
    anchor_rows=[];segment_rows=[]
    for label,path in runs.items():
        path=Path(path);report=json.loads(path.read_text())
        if report['manifest_sha256']!=route['ground_manifest_sha256']:
            raise ValueError('geometry report changed frozen ground set')
        run_dir=Path(report['candidate_index_path']).parent.parent
        run_result=json.loads((run_dir/'result.json').read_text())
        if run_result.get('exit_code')!=0 or not run_result.get('full_original_counts'):
            raise ValueError('incomplete run is not valid geometry comparison evidence')
        receipt_path=run_dir/'reconstruction/joint_initialization_receipt.json'
        receipt=json.loads(receipt_path.read_text()) if receipt_path.exists() else dict(requested=False,committed=False,fallback_to_original=False)
        effective='joint_committed' if receipt.get('committed') else 'original'
        profile=coordinate_profile(route,report,locations)
        result['runs'][label]=dict(source_report_path=str(path.resolve()),source_report_sha256=ge.sha(path),
            source_result_sha256=ge.sha(run_dir/'result.json'),initialization_receipt=receipt,
            initialization_receipt_sha256=ge.sha(receipt_path) if receipt_path.exists() else None,
            effective_initialization=effective,structure_protection=report['structure_protection_status'],
            seam_protection=report['seam_protection_status'],profiles=profile)
        for group,p in profile.items():
            for a in p['anchors']:
                anchor_rows.append(dict(run=label,group=group,region=a['region'],station_m=a['station_m'],status=a['status'],
                    fixed_up_height_m=a.get('fixed_up_height_m'),native_z_m=a.get('native_z_m')))
            for s in p['segments']:
                segment_rows.append(dict(run=label,group=group,**s))
    destination.mkdir(parents=True,exist_ok=True)
    ge.write_json(destination/'native_z_fixed_up_profiles.json',result)
    for name,records in [('fixed_anchor_heights.csv',anchor_rows),('fixed_segment_grades.csv',segment_rows)]:
        keys=list(dict.fromkeys(k for row in records for k in row))
        with (destination/name).open('w') as f:
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(records)
    os.environ.setdefault('MPLCONFIGDIR','/tmp/a2w_fixed_ground_mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,figsize=(13,13),sharex=True)
    for (label,r),color in zip(result['runs'].items(),('tab:blue','tab:orange','tab:green','tab:red')):
        state=' rejected -> original' if r['initialization_receipt'].get('fallback_to_original') else ' original'
        for group,style in zip(ge.GROUPS,('-','--',':')):
            p=r['profiles'][group];x=[a['station_m'] for a in p['anchors']]
            axes[0].plot(x,[a.get('native_z_m',np.nan) for a in p['anchors']],style,color=color,alpha=.8,label=label+state+' '+group)
            axes[1].plot(x,[a.get('fixed_up_height_m',np.nan) for a in p['anchors']],style,color=color,alpha=.8)
            mid=[(x[i]+x[i+1])/2 for i in range(len(x)-1)]
            axes[2].plot(mid,[s.get('native_z_change_per_frozen_native_xy_distance_pct',np.nan) for s in p['segments']],style,color=color,alpha=.8)
            axes[3].plot(mid,[s.get('fixed_up_signed_grade_pct',np.nan) for s in p['segments']],style,color=color,alpha=.8)
    for ax in axes:ax.grid(alpha=.25)
    axes[0].legend(ncol=3,fontsize=7)
    for ax,title in zip(axes,['Native map Z (m)','Fixed-up height (m)','Native-Z segment grade (%)','Fixed-up segment grade (%)']):ax.set_ylabel(title)
    axes[-1].set_xlabel('B004 fixed ground-polyline station (m)')
    fig.suptitle('Unchanged scan/point/anchor set and original INIT direction\nC requested initializer rejected: effective original fallback; absolute physical accuracy UNKNOWN')
    fig.text(.5,.005,'Distances fixed to B004: native XY for native-Z grade; original up-horizontal for fixed-up grade.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.02,1,1));fig.savefig(destination/'native_z_fixed_up_comparison.png',dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route-frozen',type=Path,required=True);parser.add_argument('--locations',type=Path,required=True)
    parser.add_argument('--run',action='append',required=True,metavar='LABEL=REPORT_JSON');parser.add_argument('--out',type=Path,required=True)
    a=parser.parse_args();make(a.route_frozen,a.locations,dict(s.split('=',1) for s in a.run),a.out)
