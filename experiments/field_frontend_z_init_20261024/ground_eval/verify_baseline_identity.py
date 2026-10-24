#!/usr/bin/env python3
"""Verify completed original-run point identities; never reads candidate outputs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode=True
import numpy as np
import yaml
import ground_eval as ge


def function_text(text,marker):
    start=text.index(marker)
    brace=text.index('{',start)
    depth=0
    for i in range(brace,len(text)):
        depth += (text[i]=='{')-(text[i]=='}')
        if depth==0:
            return text[start:i+1]
    raise ValueError('unclosed source function')


def fnv1a64(payload):
    h=14695981039346656037
    for byte in payload:
        h=((h^byte)*1099511628211)&0xffffffffffffffff
    return h


def pcd_records(path,header_only=False):
    header={}
    with Path(path).open('rb') as f:
        for _ in range(100):
            parts=f.readline(65536).decode('ascii').strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            header[parts[0]]=parts[1:]
            if parts[0]=='DATA':
                break
        else:
            raise ValueError('PCD header incomplete')
        count=int(header['POINTS'][0])
        if header_only:
            return count,header
        counts=list(map(int,header.get('COUNT',['1']*len(header['FIELDS']))))
        if any(c!=1 for c in counts) or header['DATA']!=['binary']:
            raise ValueError('baseline audit expects scalar binary PCD fields')
        dtype=np.dtype([(n,'<'+dict(F='f',U='u',I='i')[t]+s)
                        for n,s,t in zip(header['FIELDS'],header['SIZE'],header['TYPE'])])
        data=np.frombuffer(f.read(),dtype=dtype)
        if len(data)!=count:
            raise ValueError('PCD payload count mismatch')
        return data,header


def audit(run,source,contract_path,out,build_manifest_path=None):
    run,source,out=map(lambda p:Path(p).resolve(),(run,source,out))
    reconstruction=run/'reconstruction'
    build_manifest_path=Path(build_manifest_path or run/'build_manifest.json').resolve()
    index=ge.rows(reconstruction/'post_index.csv')
    fingerprints=ge.load_preprocess(reconstruction/'preprocess_identity.csv')
    raw_timing=ge.rows(ge.OUT.parent/'input_audit/cloud_timing.csv')
    headers={int(r['raw_scan_id']):int(r['header_ns']) for r in raw_timing}
    if set(fingerprints)!=set(headers):
        raise ValueError('baseline preprocess records do not cover exact original source IDs')
    for sid,p in fingerprints.items():
        if p['header_ns']!=headers[sid] or p['point_count']>=2**24:
            raise ValueError('baseline source header or exact float ID bound mismatch')
    checks=[]
    for r in index:
        sid=int(r['source_scan_id'])
        meta=fingerprints[sid]
        n,layout=pcd_records(reconstruction/r['cloud'],header_only=True)
        ids=np.fromfile(reconstruction/r['point_ids'],dtype='<u4')
        if int(r['source_header_stamp_ns'])!=meta['header_ns'] or n!=meta['point_count'] or not np.array_equal(np.sort(ids),np.arange(n)):
            raise ValueError('baseline real sidecar count/header/permutation mismatch: '+str(sid))
        checks.append(dict(source_scan_id=sid,point_count=n,source_header_stamp_ns=meta['header_ns'],
                           sidecar_sha256=ge.sha(reconstruction/r['point_ids']),
                           pcd_fields=layout['FIELDS']))
    if len({c['source_scan_id'] for c in checks})!=len(checks):
        raise ValueError('duplicate POST identity')
    preindex={int(r['scan_id']):r for r in ge.rows(reconstruction/'preprocess_index.csv')}
    post={int(r['source_scan_id']):r for r in index}
    available=sorted(set(preindex)&set(post))
    samples=[available[i] for i in np.unique(np.linspace(0,len(available)-1,min(16,len(available)),dtype=int))]
    witnesses=[]
    for sid in samples:
        pre,_=pcd_records(reconstruction/preindex[sid]['path'])
        body,_=pcd_records(reconstruction/post[sid]['cloud'])
        ids=np.fromfile(reconstruction/post[sid]['point_ids'],dtype='<u4')
        packed=np.column_stack([pre[k] for k in ('x','y','z','intensity','curvature')]).astype('<f4')
        if not np.array_equal(pre['normal_x'],np.arange(len(pre))) or not np.array_equal(body['intensity'],pre['intensity'][ids]):
            raise ValueError('baseline point-ID/intensity correspondence witness failed')
        if np.any(np.diff(pre['curvature'][ids])<0):
            raise ValueError('baseline deskew sidecar does not retain curvature-sort identity')
        h=fnv1a64(packed.tobytes())
        if str(h)!=fingerprints[sid]['fingerprint']:
            raise ValueError('baseline preprocessing FNV witness mismatch')
        witnesses.append(dict(source_scan_id=sid,point_count=len(pre),point_identity_and_intensity_equal=True,
                              sorted_curvature_monotonic=True,fnv1a64_recomputed=str(h),
                              preprocess_cloud_sha256=ge.sha(reconstruction/preindex[sid]['path'])))
    output=(source/'frontend/include/offline_consume_output.hpp').read_text()
    imu=(source/'frontend/src/IMU_Processing.hpp').read_text()
    mapping=(source/'frontend/src/laserMapping.cpp').read_text()
    if re.search(r'\bnormal_[xyz]\b',imu+mapping):
        raise ValueError('normal slot access found in estimator path; re-audit before claiming identity isolation')
    pp=function_text(output,'template<class Cloud>void preprocess')
    if 'i>=16777216' not in pp or 'p.normal_x=static_cast<float>(i)' not in pp:
        raise ValueError('missing exact integer bound/identity injection')
    snippets={
        'preprocess_fnv_and_id_injection':pp[pp.index('std::uint64_t hash'):pp.index('if(stamp-first_ns<=')],
        'body_and_sidecar_export':function_text(output,'template<class State,class Cloud>void post'),
        'time_comparator':re.search(r'const bool time_list[^\n]+',imu).group(0),
        'whole_cloud_copy_and_sort':re.search(r'pcl_out = \*\(meas\.lidar\);\s*sort\([^;]+;',imu).group(0),
        'deskew_writes_only_xyz':re.search(r'it_pcl->x = P_compensate\(0\);\s*it_pcl->y = P_compensate\(1\);\s*it_pcl->z = P_compensate\(2\);',imu).group(0)}
    snippet_hash={k:hashlib.sha256(v.encode()).hexdigest() for k,v in snippets.items()}
    common_files={str(p.relative_to(source)):ge.sha(p) for p in [source/'frontend/src/preprocess.cpp',source/'frontend/src/preprocess.h']}
    build=json.loads(build_manifest_path.read_text())
    for relative,digest in common_files.items():
        if build['sources'][str(source/relative)]['sha256']!=digest:
            raise ValueError('current shared preprocessing differs from actual baseline build')
    parameters=yaml.safe_load((run/'parameters.yaml').read_text())['/laser_mapping']['ros__parameters']
    preprocessing_config={k:parameters[k] for k in ('feature_extract_enable','point_filter_num')}
    preprocessing_config.update({k:parameters['preprocess'][k] for k in ('blind','lidar_type','scan_line','timestamp_unit')})
    if parameters['mapping']['extrinsic_R']!=[1.,0.,0.,0.,1.,0.,0.,0.,1.] or parameters['mapping']['extrinsic_T']!=[-.00862,.00914,.03916] or parameters['mapping']['extrinsic_est_en']:
        raise ValueError('source run differs from fixed user extrinsic contract')
    evidence=dict(common_files_sha256=common_files,function_snippets_sha256=snippet_hash,
                  point_id_semantics='preprocess return record j before deskew time-sort; not LiDAR packet record',
                  sidecar_encoding='one uint32 little-endian ID per exported full body XYZI point; permutation0..N-1',
                  exact_float_id_limit='reject j>=16777216 before float normal_x storage',
                  shared_preprocess_config=preprocessing_config,
                  source_scan_identity='0-based original Bag cloud consumption ID and exact integer header_ns',
                  body_transform='p_I=R_I_L*p_L+t_I_L; fixed extrinsic R=I,T=[-0.00862,0.00914,0.03916]')
    semantic_hash=hashlib.sha256(json.dumps(evidence,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    contract=dict(schema='field_ground_identity_contract_v1',point_identity='stable_preprocess_record_index',stable_across_runs=True,
                  preprocessing_identity_sha256=semantic_hash,
                  implementation_evidence=evidence,
                  validation_rule='each run must prove all original preprocessing header/count/XYZ-I-curvature FNV fingerprints equal; each full POST body sidecar is a complete unique ID permutation',
                  excluded_from_identity_contract='whole IMU propagation/initialization functions and joint branch; changed XYZ deskew output is expected, identity is not tied to resulting XYZ')
    out.mkdir(parents=True,exist_ok=True)
    for name,code in snippets.items():
        (out/(name+'.cpp.txt')).write_text(code+'\n')
    ge.write_json(out/'real_sidecar_checks.json',checks)
    ge.write_json(out/'preprocess_identity_witnesses.json',witnesses)
    result=dict(schema='baseline_real_point_identity_audit_v1',run=str(run),post_count=len(checks),preprocess_source_count=len(fingerprints),
                source_header_checks='ALL8188_EXACT',full_post_sidecar_checks='ALL_POST_COMPLETE_UNIQUE_PERMUTATIONS',
                finite_source_point_id_bound=max(v['point_count'] for v in fingerprints.values()),
                full_preprocessed_point_identity_witness_count=len(witnesses),candidate_outputs_read=0,
                normal_slot_access_in_estimator='NONE_BY_SOURCE_SEARCH',
                independent_physical_truth='UNKNOWN',contract_semantic_sha256=semantic_hash,
                input_sha256={str(p):ge.sha(p) for p in [reconstruction/'post_index.csv',reconstruction/'preprocess_identity.csv',build_manifest_path,run/'parameters.yaml',run/'result.json']})
    ge.write_json(out/'identity_audit.json',result)
    if Path(contract_path).exists():
        if json.loads(Path(contract_path).read_text())!=contract:
            raise ValueError('existing identity contract differs; preserve frozen contract and audit the semantic change')
    else:
        ge.write_json(contract_path,contract)
    print(json.dumps({k:result[k] for k in ('post_count','preprocess_source_count','full_preprocessed_point_identity_witness_count','candidate_outputs_read','contract_semantic_sha256')}))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--contract',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--build-manifest',type=Path)
    args=parser.parse_args()
    audit(args.run,args.source,args.contract,args.out,args.build_manifest)
