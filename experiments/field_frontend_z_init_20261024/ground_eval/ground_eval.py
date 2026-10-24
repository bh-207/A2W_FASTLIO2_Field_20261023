#!/usr/bin/env python3
"""Offline fixed-identity ground profiles. No ROS, ICP, flattening or Z clipping."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree
from scipy.spatial.transform import Rotation
from pcd_io import read_cloud

WS = Path(__file__).resolve().parents[3]
E = Path(__file__).resolve().parent
OUT = WS/'captures/field_frontend_z_init_20261024_001/ground_eval'
SESSION = WS/'captures/field_fixed_sessions/fixed_short_20261023_001'
BUNDLE = SESSION/'bundles/map_bundle'
OLD = WS/'captures/field_backend_repair_20261023_001/geometry/frozen'
GROUPS = ('train', 'heldout', 'revisit')
PROTOCOL = dict(version='fixed_ground_profile_v1_20261024', range_m=12., roi_radius_m=6.,
                representative_voxel_m=.12, cell_m=.75, minimum_points=8,
                second_eigenvalue_min_m2=.0025, eigenvalue_ratio_max=.08,
                plane_residual_p95_max_m=.05, normal_up_angle_max_deg=35.,
                reference_hull_margin_m=.10, floor_selection_body_height_m=[-2.5,-.15],
                structure_k=12, structure_neighbor_max_m=.8,
                structure_second_eigenvalue_min_m2=.0005,
                structure_median_degradation_max_m=.02, structure_p95_degradation_max_m=.05,
                seam_median_degradation_max_m=.02, seam_p95_degradation_max_m=.05,
                minimum_structure_support=25,
                absolute_accuracy='unknown_without_independent_survey',
                grade_acceptance='unknown_without_independent_survey_no_flatter_is_better_rule')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def write_json(path, value):
    def default(v):
        return v.tolist() if isinstance(v, np.ndarray) else v.item()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False, default=default)+'\n')


def rows(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))


def pose(row):
    t = np.array([float(row['t'+a]) for a in 'xyz'])
    q = np.array([float(row['q'+a]) for a in 'xyzw'])
    if not np.isfinite(t).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError('nonfinite pose or nonunit quaternion')
    return Rotation.from_quat(q).as_matrix(), t


def transform(body, row):
    r, t = pose(row)
    return np.asarray(body) @ r.T + t


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    if not len(a):
        return dict(count=0)
    return dict(count=len(a), min=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a,95)), max=float(a.max()))


def angle(a, b):
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b)), np.dot(a,b))))


def make_basis(up):
    up = np.asarray(up,float)
    up = up/np.linalg.norm(up)
    ex = np.array([1.,0.,0.])
    ex -= up*(ex@up)
    if np.linalg.norm(ex) < 1e-6:
        raise ValueError('INIT up parallel to fixed X; explicit alternative required')
    ex /= np.linalg.norm(ex)
    return np.column_stack([ex,np.cross(up,ex),up])


def measure_patch(points, reference, up, basis, axis):
    p = np.asarray(points,float)
    reference, up, axis = map(lambda a:np.asarray(a,float), (reference,up,axis))
    count = len(p)
    bad = dict(status='UNKNOWN', point_count=count)
    if count < PROTOCOL['minimum_points'] or not np.isfinite(p).all():
        return dict(bad, reason='missing_nonfinite_or_fewer_than_8_frozen_points')
    center = p.mean(0)
    d = p-center
    val, vec = np.linalg.eigh(d.T@d/count)
    n = vec[:,0]
    if n@up < 0:
        n = -n
    residual = float(np.percentile(abs(d@n),95))
    detail = dict(point_count=count, residual_p95_m=residual, eigenvalues_m2=val.tolist(),
                  normal=n.tolist(), normal_angle_to_up_deg=angle(n,up))
    if val[1] < .0025 or val[0]/max(val[1],1e-12) > .08 or residual > .05 or n@up < np.cos(np.deg2rad(35)):
        return dict(detail, status='UNKNOWN', reason='frozen_points_no_longer_supported_ground_plane')
    xy = p@basis[:,:2]
    try:
        hull = ConvexHull(xy)
    except QhullError:
        return dict(detail, status='UNKNOWN', reason='degenerate_horizontal_footprint')
    violation = float(np.max(hull.equations[:,:2]@(reference@basis[:,:2])+hull.equations[:,2]))
    detail['reference_hull_halfplane_violation_m'] = violation
    if violation > PROTOCOL['reference_hull_margin_m']:
        return dict(detail, status='UNKNOWN', reason='fixed_reference_outside_frozen_point_support')
    h = float(reference@up-n@(reference-center)/(n@up))
    return dict(detail, status='SUPPORTED', height_m=h,
                longitudinal_grade_pct=float(-100*(n@axis)/(n@up)),
                transverse_grade_pct=float(-100*(n@np.cross(up,axis))/(n@up)))


def profile_segments(regions, patches, axis):
    result = []
    for a,b in zip(regions,regions[1:]):
        ds = float((np.array(b['anchor_reference'])-a['anchor_reference'])@axis)
        item = dict(from_region=a['region'], to_region=b['region'], fixed_station_difference_m=ds)
        pa,pb = patches.get(a['region'],{}),patches.get(b['region'],{})
        if ds <= 0 or pa.get('status') != 'SUPPORTED' or pb.get('status') != 'SUPPORTED':
            result.append(dict(item, status='UNKNOWN', reason='missing_fixed_anchor_support_or_station'))
        else:
            dh = pb['height_m']-pa['height_m']
            result.append(dict(item,status='SUPPORTED',height_difference_m=dh,
                               longitudinal_grade_pct=100*dh/ds,
                               grade_angle_deg=float(np.degrees(np.arctan2(dh,ds)))))
    return result


def unique_quantized_match(stamp, keys):
    keys = np.asarray(keys,np.int64)
    delta = keys-int(stamp)
    candidates = np.flatnonzero(abs(delta) <= 256)
    if len(candidates) != 1:
        raise ValueError('timestamp requires unique match within 256ns double ULP')
    ix = int(candidates[0])
    return ix,int(delta[ix])


def match_point_ids(requested, available, body):
    requested, available = np.asarray(requested,np.int64),np.asarray(available,np.int64)
    if len(np.unique(available)) != len(available):
        raise ValueError('duplicate point identities')
    order = np.argsort(available)
    sorted_ids = available[order]
    ix = np.searchsorted(sorted_ids,requested)
    valid = ix < len(order)
    if len(order):
        valid &= sorted_ids[np.minimum(ix,len(order)-1)] == requested
    out = np.full((len(requested),3),np.nan)
    out[valid] = np.asarray(body)[order[ix[valid]]]
    return out,valid


def load_index(path):
    path = Path(path).resolve()
    out = {}
    for r in rows(path):
        key = int(r.get('source_scan_id',r.get('id','-1')))
        if key < 0 or key in out:
            raise ValueError('nonnegative unique source scan identities required')
        pose(r)
        r = dict(r)
        if r.get('cloud'):
            r['_cloud'] = str((path.parent/r['cloud']).resolve())
        if r.get('point_ids'):
            r['_point_ids'] = str((path.parent/r['point_ids']).resolve())
        out[key] = r
    return out


def load_contract(path):
    c = json.loads(Path(path).read_text())
    if c.get('schema') != 'field_ground_identity_contract_v1' or c.get('point_identity') != 'stable_preprocess_record_index' or not c.get('stable_across_runs') or not c.get('implementation_evidence') or not c.get('preprocessing_identity_sha256'):
        raise ValueError('explicit proved preprocessing identity contract required')
    return c


def load_preprocess(path):
    result = {}
    for r in rows(path):
        sid = int(r['source_scan_id'])
        if sid in result:
            raise ValueError('duplicate preprocessing source scan identity')
        result[sid] = dict(header_ns=int(r['header_ns']),point_count=int(r['point_count']),
                           fingerprint=r['xyz_intensity_curvature_fnv1a64'])
    return result


def verify_preprocess_identity(sid,baseline,current):
    if sid not in baseline or sid not in current or baseline[sid] != current[sid]:
        raise ValueError('candidate preprocessing identity/fingerprint mismatch for source scan '+str(sid))


def verify_frozen(directory):
    directory = Path(directory)
    m = json.loads((directory/'manifest.json').read_text())
    if m['protocol'] != PROTOCOL:
        raise ValueError('frozen protocol differs; never tune after candidate reading')
    for path,h in m['input_sha256'].items():
        if sha(path) != h:
            raise ValueError('frozen source changed: '+path)
    for path,h in m['selection_sha256'].items():
        if sha(directory/path) != h:
            raise ValueError('frozen selection changed: '+path)
    return m


def cell_measurements(data, world, up, basis, axis):
    result = {}
    for g in GROUPS:
        p = world[g]
        mask = data[g+'_floor']
        cells = data[g+'_cell']
        result[g] = {}
        for key in sorted(set(map(tuple,cells[mask]))):
            select = mask & np.all(cells==key,axis=1)
            ref = basis[:,:2]@((np.asarray(key)+.5)*.75)
            result[g][key] = measure_patch(p[select],ref,up,basis,axis)
    return result


def freeze_baseline_cell_tags(data,world,basis):
    data = dict(data)
    for g in GROUPS:
        data[g+'_legacy_cell'] = data[g+'_cell'].copy()
        data[g+'_cell'] = np.floor((world[g]@basis)[:,:2]/.75).astype(np.int64)
    return data


def freeze_regions(destination, mode, source_regions, data_and_world, basis, inputs, extra):
    destination = Path(destination)
    if (destination/'manifest.json').exists():
        m = verify_frozen(destination)
        if m['mode']!=mode or m.get('index_path')!=extra.get('index_path') or any(str(Path(p).resolve()) not in m['input_sha256'] for p in inputs):
            raise ValueError('existing freeze belongs to different input request; preserve it and use a new directory')
        return m
    destination.mkdir(parents=True,exist_ok=True)
    up = basis[:,2]
    # Provisional axis only affects reported grades, never ground qualification.
    provisional = basis[:,1]
    regions = []
    selection = {}
    for region in source_regions:
        data, world = data_and_world(region)
        measured = cell_measurements(data,world,up,basis,provisional)
        keys = sorted(set.intersection(*(set(measured[g]) for g in GROUPS)))
        good = [k for k in keys if all(measured[g][k]['status']=='SUPPORTED' for g in GROUPS)]
        center = np.array(region['center'])@basis[:,:2]
        good.sort(key=lambda k:(np.linalg.norm((np.array(k)+.5)*.75-center),k))
        if not good:
            raise ValueError('no baseline-supported fixed anchor: '+region['name'])
        anchor = good[0]
        ref = basis[:,:2]@((np.array(anchor)+.5)*.75)
        name = region['name']+'.npz'
        np.savez_compressed(destination/name,**data)
        selection[name] = sha(destination/name)
        regions.append(dict(region=region['name'], center=region['center'],
                            anchor_cell=anchor,anchor_reference=ref,
                            baseline_joint_supported_cells=[list(k) for k in sorted(good)],
                            frozen_joint_cell_count=len(keys),
                            groups={g:dict(point_count=len(data[g+'_body']),
                                          scan_ids=sorted(set(map(int,data[g+'_scan_id'])))) for g in GROUPS}))
    corridor = [r for r in regions if r['region'].startswith('corridor_')]
    axis = np.array(corridor[-1]['anchor_reference'])-corridor[0]['anchor_reference']
    axis -= up*(axis@up)
    axis /= np.linalg.norm(axis)
    for r in regions:
        r['station_m'] = float((r['anchor_reference']-corridor[0]['anchor_reference'])@axis)
    m = dict(schema='field_fixed_ground_manifest_v1',mode=mode,
             created_unix_ns=time.time_ns(),candidate_read_count_before_freeze=0,
             protocol=PROTOCOL,up=up,basis=basis,longitudinal_axis=axis,regions=regions,
             input_sha256={str(Path(p).resolve()):sha(p) for p in inputs},
             selection_sha256=selection,**extra)
    write_json(destination/'manifest.json',m)
    return m


def freeze_historical(destination):
    old = json.loads((OLD/'selection_manifest.json').read_text())
    index = load_index(BUNDLE/'keyframes/index.csv')
    inputs = [OLD/'selection_manifest.json',BUNDLE/'keyframes/index.csv',SESSION/'runtime_state.csv']
    def source(region):
        path = OLD/(region['name']+'.npz')
        inputs.append(path)
        if sha(path) != old['frozen_file_sha256'][path.name]:
            raise ValueError('old frozen selection hash mismatch')
        with np.load(path) as z:
            data = {k:z[k] for k in z.files}
        world = {}
        for g in GROUPS:
            p = np.empty_like(data[g+'_body'])
            for scan in np.unique(data[g+'_scan_id']):
                ix = data[g+'_scan_id']==scan
                p[ix] = transform(data[g+'_body'][ix],index[int(scan)])
            world[g] = p
        return freeze_baseline_cell_tags(data,world,np.array(old['basis'])),world
    return freeze_regions(destination,'historical_828_reference_only',old['protocol']['regions'],
                          source,np.array(old['basis']),inputs,
                          dict(index_path=str(BUNDLE/'keyframes/index.csv'),
                               point_identity='old_keyframe_id_and_original_pcd_record_index',
                               comparison_scope='historical_only_not_valid_frontend_same_point_comparison',
                               up_source='old INIT_COMMIT transformed by saved initial map gauge'))


def scan_mapping(timing_path,original_index_path=None,original_trace_path=None):
    timing = rows(timing_path)
    header = np.array([int(r.get('header_ns',r.get('source_header_stamp_ns'))) for r in timing],np.int64)
    runtime = [r for r in rows(original_trace_path or SESSION/'runtime_state.csv') if r['stage']=='POST_LIDAR_UPDATE']
    ends = np.array([int(r['scan_end_ns']) for r in runtime],np.int64)
    out = {}
    for r in rows(original_index_path or BUNDLE/'keyframes/index.csv'):
        ix,de = unique_quantized_match(int(r['stamp_ns']),ends)
        begin = int(runtime[ix]['scan_begin_ns'])
        try:
            j,db = unique_quantized_match(begin,header)
        except ValueError:
            j = int(np.argmin(abs(header-begin)))
            out[int(r['id'])] = dict(status='UNMATCHED_SOURCE_NOT_RECORDED',source_scan_id=None,
                                    runtime_scan_begin_ns=begin,end_quantization_error_ns=de,
                                    nearest_recorded_header_difference_ns=int(header[j]-begin))
        else:
            out[int(r['id'])] = dict(status='MATCHED',source_scan_id=int(timing[j].get('raw_scan_id',timing[j].get('source_scan_id'))),
                                    source_header_stamp_ns=int(header[j]),end_quantization_error_ns=de,
                                    begin_quantization_error_ns=db)
    return out


def freeze_baseline(destination,index_path,trace_path,timing_path,contract_path,preprocess_path):
    old = json.loads((OLD/'selection_manifest.json').read_text())
    index = load_index(index_path)
    contract = load_contract(contract_path)
    preprocess = load_preprocess(preprocess_path)
    init = next(r for r in rows(trace_path) if r['stage']=='INIT_COMMIT')
    up = -np.array([float(init['g_O_'+a]) for a in 'xyz'])
    basis = make_basis(up)
    mapping = scan_mapping(timing_path)
    inputs = [OLD/'selection_manifest.json',index_path,trace_path,timing_path,contract_path,preprocess_path,
              BUNDLE/'keyframes/index.csv',SESSION/'runtime_state.csv']
    selected_clouds = {}
    region_defs = []
    for r in old['protocol']['regions']:
        r = dict(r)
        for k in ('a','b'):
            if mapping[r[k]]['source_scan_id'] not in index:
                raise ValueError('baseline missing required original source scan')
        r['center'] = np.mean([pose(index[mapping[r[k]]['source_scan_id']])[1] for k in ('a','b')],axis=0).tolist()
        region_defs.append(r)
    def source(region):
        data,world = {},{}
        for group in GROUPS:
            parts = {k:[] for k in ('body','scan_id','point_index','floor','cell')}
            allworld = []
            for kf in region[group]:
                ident = mapping[kf]
                sid = ident['source_scan_id']
                if sid is None:
                    continue  # Explicitly accounted in historical_scan_mapping; never use a nearby scan.
                if sid not in index or int(index[sid]['source_header_stamp_ns']) != ident['source_header_stamp_ns']:
                    raise ValueError('baseline scan header identity mismatch')
                row = index[sid]
                if Path(row['_cloud']).suffix!='.npz' and not row.get('_point_ids'):
                    raise ValueError('modern PCD requires explicit preprocess point-ID sidecar')
                body,pids = read_cloud(row['_cloud'],row.get('_point_ids'))
                if sid not in preprocess or preprocess[sid]['point_count']!=len(body) or preprocess[sid]['header_ns']!=ident['source_header_stamp_ns']:
                    raise ValueError('baseline preprocessing fingerprint count/header mismatch')
                q = transform(body,row)
                inside = (np.linalg.norm(body,axis=1)<=12)&(np.linalg.norm((q-np.array(region['center']))@basis[:,:2],axis=1)<=6)
                ix = np.flatnonzero(inside)
                _,unique = np.unique(np.floor(q[ix]/.12).astype(np.int64),axis=0,return_index=True)
                ix = ix[np.sort(unique)]
                h = (q[ix]-pose(row)[1])@basis[:,2]
                vals = dict(body=body[ix],scan_id=np.full(len(ix),sid,np.int64),
                            point_index=pids[ix],floor=(h>=-2.5)&(h<=-.15),
                            cell=np.floor((q[ix]@basis)[:,:2]/.75).astype(np.int64))
                for k,v in vals.items():
                    parts[k].append(v)
                allworld.append(q[ix])
                path = row['_cloud']
                if path not in selected_clouds:
                    inputs.append(path)
                    if row.get('_point_ids'):
                        inputs.append(row['_point_ids'])
                    selected_clouds[path] = dict(source_scan_id=sid,source_header_stamp_ns=ident['source_header_stamp_ns'],point_count=len(body),sha256=sha(path),
                                                 point_identity_sha256=row.get('point_identity_sha256'))
            for k,v in parts.items():
                data[group+'_'+k] = np.concatenate(v)
            world[group] = np.concatenate(allworld)
        return data,world
    return freeze_regions(destination,'complete_original_baseline',region_defs,source,basis,inputs,
                          dict(index_path=str(Path(index_path).resolve()),point_identity='source_scan_id_and_stable_preprocess_record_index',
                               comparison_scope='complete_original_vs_candidate_same_scan_points',
                               up_source=str(Path(trace_path).resolve())+' INIT_COMMIT',
                               identity_contract=contract,selected_clouds=selected_clouds,
                               preprocess_identity=preprocess,
                               historical_scan_mapping=mapping,
                               input_completeness='NOT_VERIFIED_BY_GEOMETRY_TOOL_SEE_FULL_INPUT_LEDGER'))


def frozen_world(data,index,historical,cloud_meta,contract):
    world,identity = {},{}
    cache = {}
    for g in GROUPS:
        p = np.full_like(data[g+'_body'],np.nan)
        for scan in np.unique(data[g+'_scan_id']):
            sid = int(scan)
            ix = data[g+'_scan_id']==sid
            if sid not in index:
                identity[sid] = dict(status='UNKNOWN',reason='missing_source_scan')
                continue
            r = index[sid]
            if historical:
                body = data[g+'_body'][ix]
                valid = np.ones(len(body),bool)
                identity[sid] = dict(status='historical_rigid_retransform_only')
            else:
                original = cloud_meta[sid]
                if int(r['source_header_stamp_ns']) != original['source_header_stamp_ns']:
                    raise ValueError('candidate source header mismatch')
                if sid not in cache:
                    if Path(r['_cloud']).suffix!='.npz' and not r.get('_point_ids'):
                        raise ValueError('candidate PCD lacks explicit preprocess point-ID sidecar')
                    cache[sid] = read_cloud(r['_cloud'],r.get('_point_ids'))
                raw,ids = cache[sid]
                if len(raw) != original['point_count']:
                    identity[sid] = dict(status='UNKNOWN',reason='preprocess_point_count_changed')
                    continue
                if original.get('point_identity_sha256') and r.get('point_identity_sha256') != original['point_identity_sha256']:
                    raise ValueError('candidate preprocess point identity hash changed')
                body,valid = match_point_ids(data[g+'_point_index'][ix],ids,raw)
                identity[sid] = dict(status='SUPPORTED' if valid.all() else 'UNKNOWN',
                                     missing_selected_points=int((~valid).sum()),
                                     identity_evidence=contract['implementation_evidence'])
            p[ix] = transform(body,r)
        world[g] = p
    return world,identity


def fixed_structure(train,query,up,neighbors=None):
    if len(train)<12 or len(query)==0:
        return dict(support=np.zeros(len(query),bool),distance=np.full(len(query),np.nan),neighbors=np.empty((len(query),0),int),normal=np.full((len(query),3),np.nan))
    if neighbors is None:
        _,neighbors = cKDTree(train).query(query,k=12,workers=1)
    p = train[neighbors]
    valid = np.isfinite(p).all((1,2)) & np.isfinite(query).all(1)
    normal = np.full((len(query),3),np.nan)
    distance = np.full(len(query),np.nan)
    support = np.zeros(len(query),bool)
    if valid.any():
        pv = p[valid]
        c = pv.mean(1)
        d = pv-c[:,None,:]
        val,vec = np.linalg.eigh(np.einsum('nki,nkj->nij',d,d)/12)
        n = vec[:,:,0]
        support[valid] = (np.linalg.norm(pv-query[valid,None,:],axis=2).max(1)<=.8)&(val[:,1]>=.0005)&(val[:,0]/np.maximum(val[:,1],1e-12)<=.08)&(abs(n@up)<.5)
        normal[valid] = n
        distance[valid] = abs(np.sum((query[valid]-c)*n,axis=1))
    return dict(support=support,distance=distance,neighbors=neighbors,normal=normal)


def seam_protection(baseline,candidate):
    bs = stats([abs(r['return_minus_train_m']) for r in baseline if 'return_minus_train_m' in r])
    cs = stats([abs(r['return_minus_train_m']) for r in candidate if 'return_minus_train_m' in r])
    complete = bs['count']==cs['count']==len(baseline)==len(candidate)
    passed = None if not complete or not len(baseline) else cs['median']<=bs['median']+.02 and cs['p95']<=bs['p95']+.05
    return dict(status='UNKNOWN' if passed is None else ('PASS' if passed else 'FAIL'),
                fixed_baseline_count=len(baseline),candidate_supported_count=cs['count'],
                baseline_absolute_return_minus_train_m=bs,candidate_absolute_return_minus_train_m=cs)


def evaluate(directory,index_path,destination,contract_path=None,preprocess_path=None):
    directory,destination = Path(directory),Path(destination)
    m = verify_frozen(directory)  # Always precedes candidate reads.
    historical = m['mode']=='historical_828_reference_only'
    contract = None
    if not historical:
        if not contract_path:
            raise ValueError('candidate identity contract required')
        contract = load_contract(contract_path)
        if contract != m['identity_contract']:
            raise ValueError('candidate preprocessing identity contract differs from baseline')
        if not preprocess_path:
            raise ValueError('candidate preprocessing fingerprint CSV required')
        current_preprocess = load_preprocess(preprocess_path)
        baseline_preprocess = {int(k):v for k,v in m['preprocess_identity'].items()}
        for sid in baseline_preprocess:
            verify_preprocess_identity(sid,baseline_preprocess,current_preprocess)
    index = load_index(index_path)
    baseline_index = load_index(m['index_path'])
    cloud_meta = {v['source_scan_id']:v for v in m.get('selected_clouds',{}).values()}
    up,basis,axis = map(np.array,(m['up'],m['basis'],m['longitudinal_axis']))
    report = dict(schema='field_fixed_ground_report_v1',manifest_sha256=sha(directory/'manifest.json'),
                  evaluator_sha256=sha(Path(__file__)),pcd_reader_sha256=sha(E/'pcd_io.py'),
                  candidate_index_path=str(Path(index_path).resolve()),candidate_index_sha256=sha(index_path),
                  mode=m['mode'],absolute_accuracy='UNKNOWN_NO_INDEPENDENT_MEASUREMENT',
                  grade_accuracy='UNKNOWN_NO_FLATTER_IS_BETTER_ACCEPTANCE',
                  fixed_up=up,fixed_longitudinal_axis=axis,regions=[])
    if preprocess_path:
        report['candidate_preprocess_identity_path'] = str(Path(preprocess_path).resolve())
        report['candidate_preprocess_identity_sha256'] = sha(preprocess_path)
    if not historical:
        report['candidate_selected_cloud_sources'] = {}
        for sid in cloud_meta:
            if sid not in index:
                continue
            row = index[sid]
            source = dict(cloud_path=row['_cloud'],cloud_sha256=sha(row['_cloud']))
            if row.get('_point_ids'):
                source.update(point_ids_path=row['_point_ids'],point_ids_sha256=sha(row['_point_ids']))
            report['candidate_selected_cloud_sources'][sid] = source
    destination.mkdir(parents=True,exist_ok=True)
    anchors = {g:{} for g in GROUPS}
    baseline_anchors = {g:{} for g in GROUPS}
    table = []
    for region in m['regions']:
        with np.load(directory/(region['region']+'.npz')) as z:
            data = {k:z[k] for k in z.files}
        world,identities = frozen_world(data,index,historical,cloud_meta,contract)
        baseline,_ = frozen_world(data,baseline_index,historical,cloud_meta,contract)
        cells = cell_measurements(data,world,up,basis,axis)
        basecells = cell_measurements(data,baseline,up,basis,axis)
        details = dict(region=region['region'],station_m=region['station_m'],
                       anchor_cell=region['anchor_cell'],anchor_reference=region['anchor_reference'],
                       identity_status=identities,groups={},structure={})
        for g in GROUPS:
            key = tuple(region['anchor_cell'])
            anchor = cells[g].get(key,dict(status='UNKNOWN',reason='missing_frozen_anchor'))
            anchors[g][region['region']] = anchor
            baseline_anchors[g][region['region']] = basecells[g][key]
            recs = []
            for key in sorted(set(map(tuple,data[g+'_cell'][data[g+'_floor']]))):
                rec = dict(cell=key,baseline_supported=basecells[g][key]['status']=='SUPPORTED',**cells[g][key])
                if rec['baseline_supported'] and rec['status']=='SUPPORTED':
                    rec['height_change_from_baseline_m'] = rec['height_m']-basecells[g][key]['height_m']
                    rec['normal_change_from_baseline_deg'] = angle(np.array(rec['normal']),np.array(basecells[g][key]['normal']))
                recs.append(rec)
                if rec['baseline_supported']:
                    table.append(dict(region=region['region'],group=g,cell_x=key[0],cell_y=key[1],
                                      status=rec['status'],height_m=rec.get('height_m'),
                                      height_change_from_baseline_m=rec.get('height_change_from_baseline_m'),
                                      longitudinal_grade_pct=rec.get('longitudinal_grade_pct'),
                                      normal_angle_to_up_deg=rec.get('normal_angle_to_up_deg'),
                                      normal_change_from_baseline_deg=rec.get('normal_change_from_baseline_deg'),reason=rec.get('reason','')))
            details['groups'][g] = dict(anchor=anchor,baseline_anchor=basecells[g][tuple(region['anchor_cell'])],cells=recs,
                frozen_baseline_supported_count=sum(r['baseline_supported'] for r in recs),
                baseline_supported_now_unknown_count=sum(r['baseline_supported'] and r['status']!='SUPPORTED' for r in recs))
        for g in ('heldout','revisit'):
            # Exact baseline nearest-12 identities. Candidate cannot change neighbors.
            tf,qf = ~data['train_floor'],~data[g+'_floor']
            old = fixed_structure(baseline['train'][tf],baseline[g][qf],up)
            current = fixed_structure(world['train'][tf],world[g][qf],up,old['neighbors'])
            mask = old['support']
            lost = mask & ~current['support']
            supported = mask & current['support']
            bs,cs = stats(old['distance'][mask]),stats(current['distance'][mask])
            enough = mask.sum()>=25
            passed = None
            if enough and not lost.any() and cs.get('count')==int(mask.sum()):
                passed = cs['median']<=bs['median']+.02 and cs['p95']<=bs['p95']+.05
            details['structure'][g] = dict(status='UNKNOWN' if passed is None else ('PASS' if passed else 'FAIL'),
                fixed_baseline_supported_query_count=int(mask.sum()),support_lost_count=int(lost.sum()),
                baseline_abs_normal_distance_m=bs,candidate_abs_normal_distance_m=cs,
                fixed_support_normal_change_deg=stats(np.degrees(np.arccos(np.clip(abs(np.sum(old['normal'][supported]*current['normal'][supported],axis=1)),-1,1)))),
                physical_wall_device_identity='unknown_geometric_proxy_only')
            np.savez_compressed(destination/(region['region']+'_'+g+'_structure.npz'),
                                frozen_baseline_support=mask,candidate_support=current['support'],
                                fixed_train_neighbors=old['neighbors'],
                                baseline_distance=old['distance'],candidate_distance=current['distance'])
        seam,base_seam = [],[]
        for key in map(tuple,region['baseline_joint_supported_cells']):
            rr = [cells[g].get(key,{}) for g in GROUPS]
            br = [basecells[g][key] for g in GROUPS]
            base_seam.append(dict(cell=key,return_minus_train_m=br[2]['height_m']-br[0]['height_m']))
            if all(r.get('status')=='SUPPORTED' for r in rr):
                seam.append(dict(cell=key,return_minus_train_m=rr[2]['height_m']-rr[0]['height_m'],
                                 heldout_minus_train_m=rr[1]['height_m']-rr[0]['height_m'],
                                 return_normal_difference_deg=angle(np.array(rr[2]['normal']),np.array(rr[0]['normal']))))
            else:
                seam.append(dict(cell=key,status='UNKNOWN',reason='frozen_seam_support_missing'))
        details['fixed_cell_seams'] = seam
        details['seam_protection'] = seam_protection(base_seam,seam)
        details['seam_summary'] = dict(frozen_cell_count=len(seam),unknown_count=sum(r.get('status')=='UNKNOWN' for r in seam),
            signed_return_minus_train_m=stats([r['return_minus_train_m'] for r in seam if 'return_minus_train_m' in r]),
            absolute_return_minus_train_m=stats([abs(r['return_minus_train_m']) for r in seam if 'return_minus_train_m' in r]))
        report['regions'].append(details)
        write_json(destination/(region['region']+'.json'),details)
        print('evaluated',region['region'],flush=True)
    corridor = [r for r in m['regions'] if r['region'].startswith('corridor_')]
    report['longitudinal_profiles'] = {g:dict(anchors=[dict(region=r['region'],station_m=r['station_m'],**anchors[g][r['region']]) for r in corridor],segments=profile_segments(corridor,anchors[g],axis)) for g in GROUPS}
    report['baseline_longitudinal_profiles'] = {g:dict(anchors=[dict(region=r['region'],station_m=r['station_m'],**baseline_anchors[g][r['region']]) for r in corridor],segments=profile_segments(corridor,baseline_anchors[g],axis)) for g in GROUPS}
    report['structure_protection_status'] = 'FAIL' if any(v['status']=='FAIL' for r in report['regions'] for v in r['structure'].values()) else ('UNKNOWN' if any(v['status']=='UNKNOWN' for r in report['regions'] for v in r['structure'].values()) else 'PASS')
    report['seam_protection_status'] = 'FAIL' if any(r['seam_protection']['status']=='FAIL' for r in report['regions']) else ('UNKNOWN' if any(r['seam_protection']['status']=='UNKNOWN' for r in report['regions']) else 'PASS')
    report['limitations'] = ['No independent physical height/grade truth; lower elevation or flatter slope is not a pass.',
        'Ground, wall and equipment labels are frozen geometric proxies, not manually surveyed physical identities.',
        'Baseline-supported cells remain in denominators when candidate support is missing.',
        'Fixed INIT up, points, cell reference, axis and station distances; no candidate alignment, clipping or detrending.',
        'Historical 828 uses saved body points and is not a same-input-point frontend comparison.' if historical else 'Point identity relies on checked source scans and supplied preprocessing implementation contract.']
    write_json(destination/'report.json',report)
    plot_profiles(report,destination/'ground_profile.png')
    if table:
        with (destination/'cells.csv').open('w') as f:
            w = csv.DictWriter(f,fieldnames=list(table[0]))
            w.writeheader()
            w.writerows(table)
    return report


def plot_profiles(report,path):
    os.environ.setdefault('MPLCONFIGDIR','/tmp/a2w_fixed_ground_mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,1,figsize=(12,8),sharex=True)
    for g,color in zip(GROUPS,('tab:blue','tab:green','tab:orange')):
        for key,style,label in (('baseline_longitudinal_profiles','--','baseline'),('longitudinal_profiles','-','evaluated')):
            p = report[key][g]
            a = p['anchors']
            x = [r['station_m'] for r in a]
            y = [r.get('height_m',np.nan) if r['status']=='SUPPORTED' else np.nan for r in a]
            axes[0].plot(x,y,style,marker='o',markersize=3,color=color,label=g+' '+label)
            s = p['segments']
            axes[1].plot([(x[i]+x[i+1])/2 for i in range(len(s))],
                [r.get('longitudinal_grade_pct',np.nan) for r in s],style,marker='o',markersize=3,color=color)
    axes[0].set_ylabel('Ground height along fixed INIT up (m)')
    axes[1].set_ylabel('Signed segment longitudinal grade (%)')
    axes[1].set_xlabel('Fixed baseline corridor station (m)')
    for ax in axes:
        ax.grid(alpha=.25)
    axes[0].legend(ncol=3,fontsize=8)
    fig.suptitle('Same frozen ground anchors / fixed up / no alignment or detrending\nHeight and station scales differ; physical absolute accuracy UNKNOWN')
    fig.tight_layout()
    fig.savefig(path,dpi=150)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action',required=True)
    h = sub.add_parser('freeze-historical')
    h.add_argument('--out',type=Path,default=OUT/'historical_frozen')
    b = sub.add_parser('freeze-baseline')
    b.add_argument('--index',type=Path,required=True)
    b.add_argument('--trace',type=Path,required=True)
    b.add_argument('--timing',type=Path,required=True)
    b.add_argument('--contract',type=Path,required=True)
    b.add_argument('--preprocess',type=Path,required=True)
    b.add_argument('--out',type=Path,default=OUT/'baseline_frozen')
    e = sub.add_parser('evaluate')
    e.add_argument('--frozen',type=Path,required=True)
    e.add_argument('--index',type=Path,required=True)
    e.add_argument('--contract',type=Path)
    e.add_argument('--preprocess',type=Path)
    e.add_argument('--out',type=Path,required=True)
    a = p.parse_args()
    if a.action=='freeze-historical':
        m = freeze_historical(a.out)
        print('FROZEN',m['mode'],len(m['regions']))
    elif a.action=='freeze-baseline':
        m = freeze_baseline(a.out,a.index,a.trace,a.timing,a.contract,a.preprocess)
        print('FROZEN',m['mode'],len(m['regions']))
    else:
        r = evaluate(a.frozen,a.index,a.out,a.contract,a.preprocess)
        print(json.dumps(dict(absolute_accuracy=r['absolute_accuracy'],structure=r['structure_protection_status'])))


if __name__=='__main__':
    main()
