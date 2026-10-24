#!/usr/bin/env python3
"""Apply only exactly corresponding saved backend node poses to frozen POST points."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import ground_eval as ge


def build(run,bundle,frozen,out):
    run,bundle,frozen,out=map(Path,(run,bundle,frozen,out))
    m=ge.verify_frozen(frozen)
    post_path=run/'reconstruction/post_index.csv'
    post=ge.load_index(post_path);sids=sorted(post)
    end=np.array([int(post[s]['scan_end_ns']) for s in sids],np.int64)
    raw=ge.rows(bundle/'keyframes/index.csv');opt=ge.rows(bundle/'optimized_poses.csv')
    optimized={int(r['id']):r for r in opt}
    if len(optimized)!=len(opt) or set(optimized)!={int(r['id']) for r in raw}:
        raise ValueError('optimized/raw keyframe node IDs differ')
    if not raw:
        raise ValueError('empty backend bundle')
    r0,t0=ge.pose(raw[0]);ro,to=ge.pose(optimized[int(raw[0]['id'])])
    if np.linalg.norm(to-t0)>1e-4 or Rotation.from_matrix(ro@r0.T).magnitude()>1e-5:
        raise ValueError('optimized first node differs from fixed raw map gauge; no alignment permitted')
    mapping={};unmatched=[];raw_rows=[];opt_rows=[];errors=[]
    for k in raw:
        try:
            ix,delta=ge.unique_quantized_match(int(k['stamp_ns']),end)
        except ValueError:
            unmatched.append(dict(keyframe_id=int(k['id']),stamp_ns=int(k['stamp_ns']),status='UNKNOWN_NO_EXACT_SOURCE_SCAN'))
            continue
        sid=sids[ix]
        if sid in mapping:
            raise ValueError('two saved backend nodes map to one POST source scan')
        rk,tk=ge.pose(k);rp,tp=ge.pose(post[sid])
        dt=float(np.linalg.norm(tk-tp));dr=float(Rotation.from_matrix(rk@rp.T).magnitude())
        if dt>1e-5 or dr>1e-5:
            raise ValueError('raw backend node pose does not match same POST source gauge')
        errors.append(dict(source_scan_id=sid,keyframe_id=int(k['id']),translation_m=dt,rotation_rad=dr))
        mapping[sid]=dict(keyframe_id=int(k['id']),source_header_stamp_ns=int(post[sid]['source_header_stamp_ns']),
                          keyframe_stamp_ns=int(k['stamp_ns']),post_end_ns=int(end[ix]),quantization_difference_ns=delta)
        for source,destination in [(k,raw_rows),(optimized[int(k['id'])],opt_rows)]:
            row={key:value for key,value in post[sid].items() if not key.startswith('_')}
            row.update(cloud=post[sid]['_cloud'],point_ids=post[sid]['_point_ids'],backend_keyframe_id=int(k['id']))
            row.update({key:source[key] for key in ('tx','ty','tz','qx','qy','qz','qw')})
            destination.append(row)
    out.mkdir(parents=True,exist_ok=True)
    for name,records in [('raw_exact_nodes.csv',raw_rows),('optimized_exact_nodes.csv',opt_rows)]:
        with (out/name).open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    selected={v['source_scan_id'] for v in m['selected_clouds'].values()}
    coverage=[]
    for region in m['regions']:
        coverage.append(dict(region=region['region'],groups={g:dict(
            frozen_source_scan_count=len(region['groups'][g]['scan_ids']),
            exact_node_source_scan_ids=sorted(set(region['groups'][g]['scan_ids'])&set(mapping)),
            missing_node_source_scan_ids=sorted(set(region['groups'][g]['scan_ids'])-set(mapping)),
            status='EXACT_NODES_AVAILABLE' if set(region['groups'][g]['scan_ids'])<=set(mapping) else 'UNKNOWN_INCOMPLETE_FROZEN_NODE_SUPPORT') for g in ge.GROUPS}))
    result=dict(schema='field_backend_exact_frozen_subset_v1',run=str(run.resolve()),bundle=str(bundle.resolve()),
        ground_manifest_sha256=ge.sha(frozen/'manifest.json'),saved_keyframes=len(raw),exact_POST_node_count=len(mapping),
        frozen_source_scan_count=len(selected),exact_frozen_source_scan_count=len(selected&set(mapping)),
        missing_frozen_source_scan_count=len(selected-set(mapping)),missing_frozen_source_scan_ids=sorted(selected-set(mapping)),
        source_scan_to_keyframe=mapping,unmatched_saved_nodes=unmatched,raw_node_pose_checks=errors,
        regions=coverage,interpolated_poses=0,between_node_correction_propagation=0,
        interpretation='Only exact source POST scans with saved optimized node poses are evaluated. Missing nodes remain NaN/UNKNOWN in unchanged frozen evaluator; no interpolation, scan substitution or per-region registration.',
        absolute_accuracy='UNKNOWN_WITHOUT_INDEPENDENT_MEASUREMENT',
        cross_run_optimized_attribution='Loop decisions/graphs can differ; optimized differences are independent backend observations, not an effect attributed to frontend initialization.',
        input_sha256={str(p.resolve()):ge.sha(p) for p in [post_path,bundle/'keyframes/index.csv',bundle/'optimized_poses.csv',bundle/'metadata.yaml']})
    ge.write_json(out/'exact_node_coverage.json',result)
    print(json.dumps({k:result[k] for k in ['saved_keyframes','exact_POST_node_count','frozen_source_scan_count','exact_frozen_source_scan_count','missing_frozen_source_scan_count']}))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--frozen',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.run,a.bundle,a.frozen,a.out)
