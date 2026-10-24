#!/usr/bin/env python3
"""Observed two-original-run geometry envelope in the identical frozen gauge."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import ground_eval as ge


def compare(first_path,second_path,destination,output_name='B004_B008_observed_envelope'):
    first_path,second_path,destination=map(Path,(first_path,second_path,destination))
    a,b=[json.loads(p.read_text()) for p in (first_path,second_path)]
    if a['manifest_sha256']!=b['manifest_sha256']:
        raise ValueError('repeat reports do not use the identical frozen point/position/up manifest')
    if not np.array_equal(a['fixed_up'],b['fixed_up']) or not np.array_equal(a['fixed_longitudinal_axis'],b['fixed_longitudinal_axis']):
        raise ValueError('repeat gauge changed')
    pre=[ge.load_preprocess(r['candidate_preprocess_identity_path']) for r in (a,b)]
    if pre[0]!=pre[1]:
        raise ValueError('repeat original source/preprocess identity differs')
    ai,bi=[ge.load_index(r['candidate_index_path']) for r in (a,b)]
    if set(ai)!=set(bi):
        raise ValueError('repeat POST source IDs differ')
    dp=[];dh=[];dr=[]
    for sid in sorted(ai):
        if ai[sid]['source_header_stamp_ns']!=bi[sid]['source_header_stamp_ns']:
            raise ValueError('repeat POST exact header identity differs')
        ra,ta=ge.pose(ai[sid]);rb,tb=ge.pose(bi[sid])
        dp.append(np.linalg.norm(tb-ta));dh.append((tb-ta)@a['fixed_up'])
        dr.append(np.degrees(Rotation.from_matrix(rb@ra.T).magnitude()))
    result=dict(schema='field_two_original_run_observed_envelope_v1',reports=[str(first_path.resolve()),str(second_path.resolve())],
        report_sha256=[ge.sha(first_path),ge.sha(second_path)],ground_manifest_sha256=a['manifest_sha256'],
        fixed_up=a['fixed_up'],fixed_axis=a['fixed_longitudinal_axis'],
        preprocessing_source_count=len(pre[0]),all_preprocessing_identities_equal=True,post_count=len(ai),
        observed_pose_translation_difference_m=ge.stats(dp),observed_pose_up_height_difference_m=ge.stats(dh),
        observed_pose_absolute_up_height_difference_m=ge.stats(abs(np.array(dh))),observed_pose_rotation_difference_deg=ge.stats(dr),
        regions=[],absolute_physical_accuracy='UNKNOWN_WITHOUT_INDEPENDENT_MEASUREMENT',
        repeat_interpretation='Observed max-minus-min envelope of two complete original-algorithm executions. Not a confidence interval or proof of a worst-case bound.',
        attribution='Observed same-input differences only; receipts and build manifests determine effective initialization/source identity. No physical accuracy improvement or purely stochastic origin is inferred.')
    anchors=[];all_height=[];all_normal=[];all_grade=[]
    for ar,br in zip(a['regions'],b['regions']):
        if ar['region']!=br['region']:
            raise ValueError('repeat region identity/order changed')
        rec=dict(region=ar['region'],groups={})
        for group in ge.GROUPS:
            ga,gb=ar['groups'][group],br['groups'][group]
            ca={tuple(c['cell']):c for c in ga['cells'] if c['baseline_supported']}
            cb={tuple(c['cell']):c for c in gb['cells'] if c['baseline_supported']}
            if set(ca)!=set(cb):
                raise ValueError('repeat frozen cell denominators changed')
            height=[];normal=[];grade=[];unknown=[]
            for key in sorted(ca):
                if ca[key]['status']!='SUPPORTED' or cb[key]['status']!='SUPPORTED':
                    unknown.append(list(key));continue
                height.append(cb[key]['height_m']-ca[key]['height_m'])
                normal.append(ge.angle(np.array(ca[key]['normal']),np.array(cb[key]['normal'])))
                grade.append(cb[key]['longitudinal_grade_pct']-ca[key]['longitudinal_grade_pct'])
            aa,ab=ga['anchor'],gb['anchor']
            anchor=dict(status='UNKNOWN')
            if aa['status']=='SUPPORTED' and ab['status']=='SUPPORTED':
                anchor=dict(status='SUPPORTED',signed_height_difference_m=ab['height_m']-aa['height_m'],
                            normal_difference_deg=ge.angle(np.array(aa['normal']),np.array(ab['normal'])),
                            local_grade_difference_percentage_points=ab['longitudinal_grade_pct']-aa['longitudinal_grade_pct'])
                anchors.append(abs(anchor['signed_height_difference_m']))
            rec['groups'][group]=dict(fixed_baseline_cell_count=len(ca),repeat_common_supported_count=len(height),unknown_cells=unknown,
                signed_height_difference_m=ge.stats(height),absolute_height_difference_m=ge.stats(abs(np.array(height))),
                normal_difference_deg=ge.stats(normal),local_grade_difference_percentage_points=ge.stats(grade),anchor=anchor)
            all_height.extend(height);all_normal.extend(normal);all_grade.extend(grade)
        result['regions'].append(rec)
    result.update(all_fixed_supported_cell_absolute_height_difference_m=ge.stats(abs(np.array(all_height))),
                  all_fixed_supported_cell_normal_difference_deg=ge.stats(all_normal),
                  all_fixed_supported_cell_absolute_local_grade_difference_percentage_points=ge.stats(abs(np.array(all_grade))),
                  fixed_anchor_absolute_height_difference_m=ge.stats(anchors),
                  fixed_baseline_cells_missing_in_either_run=sum(len(v['unknown_cells']) for r in result['regions'] for v in r['groups'].values()))
    destination.mkdir(parents=True,exist_ok=True)
    if not output_name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in output_name):
        raise ValueError('safe output name required')
    ge.write_json(destination/(output_name+'.json'),result)
    print(json.dumps({k:result[k] for k in ['preprocessing_source_count','post_count','all_fixed_supported_cell_absolute_height_difference_m','fixed_anchor_absolute_height_difference_m','fixed_baseline_cells_missing_in_either_run']}))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first',type=Path,required=True);parser.add_argument('--second',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--name',default='B004_B008_observed_envelope')
    args=parser.parse_args();compare(args.first,args.second,args.out,args.name)
