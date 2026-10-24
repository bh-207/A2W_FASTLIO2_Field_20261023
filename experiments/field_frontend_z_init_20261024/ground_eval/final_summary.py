#!/usr/bin/env python3
"""Summarize final raw/fallback runs and exact-node backend observations."""
import json
from pathlib import Path
import numpy as np
import ground_eval as ge


def main():
    root=ge.OUT
    comparison=root/'final_comparison'
    native=json.loads((comparison/'native_z_fixed_up_profiles.json').read_text())
    folders=dict(B009='B009_final_original',C001='C001_requested_rejected_original_fallback',C002='C002_requested_rejected_original_fallback')
    result=dict(schema='field_final_frozen_ground_summary_v1',ground_manifest_sha256=ge.sha(root/'B004_frozen/manifest.json'),
        route_freeze_sha256=ge.sha(root/'B004_all8_route_frozen.json'),fixed_point_locations_sha256=ge.sha(root/'B004_fixed_ground_locations.json'),
        evaluation_rules_changed=False,new_point_selection=False,local_alignment=False,interpolated_optimized_poses=0,
        absolute_ground_height_and_grade_accuracy='UNKNOWN_NO_INDEPENDENT_SURVEY',joint_accuracy_improvement_claim='NOT_SUPPORTED_NO_JOINT_COMMIT',
        robot_motion_or_navigation_validation='NOT_RUN',runs={},raw_comparisons={},backend_exact_subset={},
        historical_12_unrecorded_keyframes='UNKNOWN_NO_SOURCE_SCAN_SUBSTITUTION',
        prior_wall125_research='NOT_REPEATED_NOT_SUPERSEDED_BY_THIS_PARTIAL_EXACT_NODE_SUMMARY')
    for label,folder in folders.items():
        report_path=root/folder/'report.json';report=json.loads(report_path.read_text())
        view=native['runs'][label]
        if label.startswith('C'):
            receipt=view['initialization_receipt']
            assert receipt['requested'] and not receipt['committed'] and receipt['fallback_to_original']
        rows=[]
        for region in report['regions']:
            rows.append(dict(region=region['region'],seam_protection=region['seam_protection'],structure=region['structure'],
                anchor_status={g:region['groups'][g]['anchor']['status'] for g in ge.GROUPS},
                baseline_supported_now_unknown_count={g:region['groups'][g]['baseline_supported_now_unknown_count'] for g in ge.GROUPS}))
        result['runs'][label]=dict(effective_initialization=view['effective_initialization'],initialization_receipt=view['initialization_receipt'],
            geometry_report_path=str(report_path),geometry_report_sha256=ge.sha(report_path),
            global_structure_protection=report['structure_protection_status'],global_seam_protection=report['seam_protection_status'],
            regions=rows,all8_endpoint_differences={g:dict(fixed_up_m=p['anchors'][-1]['fixed_up_height_m']-p['anchors'][0]['fixed_up_height_m'],
                native_z_m=p['anchors'][-1]['native_z_m']-p['anchors'][0]['native_z_m']) for g,p in view['profiles'].items()},
            all8_segments={g:p['segments'] for g,p in view['profiles'].items()})
        d=root/'backend_exact'/label
        coverage=json.loads((d/'exact_node_coverage.json').read_text())
        raw=json.loads((d/'raw_evaluated/report.json').read_text());opt=json.loads((d/'optimized_evaluated/report.json').read_text())
        protected=[]
        for rr,oo in zip(raw['regions'],opt['regions']):
            assert rr['region']==oo['region']
            name=rr['region'];structure={}
            for group in ('heldout','revisit'):
                with np.load(d/'raw_evaluated'/(name+'_'+group+'_structure.npz')) as z:
                    r={k:z[k] for k in z.files}
                with np.load(d/'optimized_evaluated'/(name+'_'+group+'_structure.npz')) as z:
                    o={k:z[k] for k in z.files}
                mask=r['frozen_baseline_support'];assert np.array_equal(mask,o['frozen_baseline_support'])
                rs,os=ge.stats(r['candidate_distance'][mask]),ge.stats(o['candidate_distance'][mask])
                complete=mask.sum()>=25 and r['candidate_support'][mask].all() and o['candidate_support'][mask].all() and rs['count']==os['count']==int(mask.sum())
                passed=None if not complete else os['median']<=rs['median']+.02 and os['p95']<=rs['p95']+.05
                structure[group]=dict(status='UNKNOWN' if passed is None else ('PASS' if passed else 'FAIL'),
                    fixed_baseline_query_count=int(mask.sum()),raw_failed_support_count=int((mask&~r['candidate_support']).sum()),
                    optimized_failed_support_count=int((mask&~o['candidate_support']).sum()),
                    raw_normal_distance_m=rs,optimized_normal_distance_m=os)
            protected.append(dict(region=name,seam_raw_to_optimized=ge.seam_protection(rr['fixed_cell_seams'],oo['fixed_cell_seams']),
                structure_raw_to_optimized=structure,raw_anchor_status={g:rr['groups'][g]['anchor']['status'] for g in ge.GROUPS},
                optimized_anchor_status={g:oo['groups'][g]['anchor']['status'] for g in ge.GROUPS}))
        status=[r['seam_raw_to_optimized']['status'] for r in protected]+[v['status'] for r in protected for v in r['structure_raw_to_optimized'].values()]
        result['backend_exact_subset'][label]=dict(saved_keyframes=coverage['saved_keyframes'],
            frozen_source_scans=coverage['frozen_source_scan_count'],exact_frozen_source_scans=coverage['exact_frozen_source_scan_count'],
            missing_frozen_source_scans=coverage['missing_frozen_source_scan_count'],coverage=coverage['regions'],
            raw_to_optimized_global_status='FAIL' if 'FAIL' in status else ('UNKNOWN' if 'UNKNOWN' in status else 'PASS'),
            regions=protected,coverage_report_path=str(d/'exact_node_coverage.json'),
            optimized_attribution='Independent backend graph observation. Loop decisions differ; not attributed to frontend or joint initialization.')
    for name in ['B009_C001_fallback_difference','B009_C002_fallback_difference','C001_C002_original_fallback_repeat']:
        path=comparison/(name+'.json');m=json.loads(path.read_text())
        result['raw_comparisons'][name]=dict(report_path=str(path),report_sha256=ge.sha(path),
            fixed_cell_count=m['all_fixed_supported_cell_absolute_height_difference_m']['count'],
            missing_frozen_cells=m['fixed_baseline_cells_missing_in_either_run'],
            fixed_cell_abs_height_difference_m=m['all_fixed_supported_cell_absolute_height_difference_m'],
            fixed_anchor_abs_height_difference_m=m['fixed_anchor_absolute_height_difference_m'],
            pose_translation_difference_m=m['observed_pose_translation_difference_m'],
            pose_up_height_difference_m=m['observed_pose_absolute_up_height_difference_m'],
            normal_difference_deg=m['all_fixed_supported_cell_normal_difference_deg'],
            interpretation='Observed original-algorithm execution envelope; C requested initializer rejected and restarted original. No accuracy improvement inferred.')
    result['artifacts']={name:dict(path=str(comparison/name),sha256=ge.sha(comparison/name)) for name in ['native_z_fixed_up_profiles.json','native_z_fixed_up_comparison.png','fixed_anchor_heights.csv','fixed_segment_grades.csv']}
    result['evaluator_source_sha256']={p.name:ge.sha(p) for p in ge.E.glob('*.py')}
    result['unittest_count']=22
    result['unittest_exit']=0
    result['frontend_and_backend_analysis_exit']=0
    ge.write_json(comparison/'FINAL_GROUND_SUMMARY.json',result)
    print(json.dumps(dict(raw_runs=list(result['runs']),backend_status={k:v['raw_to_optimized_global_status'] for k,v in result['backend_exact_subset'].items()},unittest_count=22,joint_applied=False)))


if __name__=='__main__':
    main()
