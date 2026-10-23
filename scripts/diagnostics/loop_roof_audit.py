#!/usr/bin/env python3
"""Offline retrieval-only height/attitude experiment, no gravity truth inferred.

All rules are fixed before evaluating ranks. Original full clouds remain the
evaluation set; retrieval subsets never replace registration/evaluation clouds.
No optimized pose is used to create a descriptor or a local-ground reference.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

VERSION = "roof_retrieval_proxy_v3_fixed_rules_20261020"
VARIANTS = ("full_v3", "body_crop_proxy_v3", "ground_crop_proxy_v3", "balanced_proxy_v3")
RULES = dict(body_high_threshold_m=2., local_ground_high_threshold_m=3.,
             ground_fit_radius_m=12., ground_xy_cell_m=1., ground_cell_quantile=.1,
             ground_cell_quantile_method="lower",
             ground_fit_lower_quantile_max=.55, ground_fit_trials=128,
             ground_fit_inlier_m=.15, ground_max_body_slope_deg=35.,
             ground_min_bins=12, ground_min_xy_span_m=3.,
             balanced_xy_cell_m=1., balanced_height_cell_m=1., balanced_max_per_cell=8,
             negative_odom_distance_proxy_m=30., nearby_odom_distance_proxy_m=5.)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def csv_read(path):
    with Path(path).open() as stream:
        return list(csv.DictReader(stream))


def csv_write(path, rows):
    if not rows:
        return
    with Path(path).open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_cloud(path):
    with Path(path).open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD DATA header missing")
            key, *values = line.decode("ascii").split()
            header[key] = values
            if key == "DATA":
                break
        if header["DATA"] != ["binary"] or header["FIELDS"] != ["x", "y", "z", "intensity"]:
            raise ValueError("requires original PointXYZI binary PCD")
        cloud = np.frombuffer(stream.read(), dtype="<f4").reshape(-1, 4).copy()
        if not np.isfinite(cloud[:, :3]).all():
            raise ValueError("nonfinite cloud")
        return cloud


def bins_and_values(cloud):
    xyz = np.asarray(cloud[:, :3], dtype=np.float64)
    radius = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = radius <= 80.
    azimuth = np.arctan2(xyz[valid, 1], xyz[valid, 0])
    azimuth[azimuth < 0] += 2*np.pi
    rings = np.clip(np.ceil(radius[valid]/4).astype(int), 1, 20)-1
    sectors = np.clip(np.ceil(azimuth/(2*np.pi/60)).astype(int), 1, 60)-1
    return rings*60+sectors, (xyz[valid, 2]+2).astype(np.float32), valid


def describe(cloud):
    if not len(cloud) or not np.isfinite(cloud[:, :3]).all():
        raise ValueError("Scan Context requires a nonempty finite cloud")
    indices, values, _ = bins_and_values(cloud)
    result = np.zeros(1200, dtype=np.float32)
    np.maximum.at(result, indices, values)
    return result.reshape(20, 60)


def fit_local_ground(cloud):
    """Lower local surface proxy in body coordinates; not a gravity estimate.

    Returns z=a*x+b*y+c and fit diagnostics. The original XYZ is never rotated
    or flattened. A true slope remains in XYZ and in the fitted a,b values.
    """
    xyz = np.asarray(cloud[:, :3], dtype=np.float64)
    xyz = xyz[np.hypot(xyz[:, 0], xyz[:, 1]) <= RULES["ground_fit_radius_m"]]
    if len(xyz) < 30:
        return None
    cells = np.floor(xyz[:, :2]/RULES["ground_xy_cell_m"]).astype(int)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    ordered = np.argsort(inverse, kind="stable")
    pieces = np.split(ordered, np.flatnonzero(np.diff(inverse[ordered]))+1)
    representatives = np.array([
        [np.median(xyz[p, 0]), np.median(xyz[p, 1]),
         np.quantile(xyz[p, 2], RULES["ground_cell_quantile"], method="lower")] for p in pieces])
    representatives = representatives[representatives[:, 2] <=
        np.quantile(representatives[:, 2], RULES["ground_fit_lower_quantile_max"])]
    if len(representatives) < RULES["ground_min_bins"]:
        return None
    design = np.column_stack([representatives[:, :2], np.ones(len(representatives))])
    values = representatives[:, 2]
    rng = np.random.default_rng(1701)
    best = None
    for _ in range(RULES["ground_fit_trials"]):
        sample = rng.choice(len(design), 3, replace=False)
        try:
            plane = np.linalg.solve(design[sample], values[sample])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(plane[:2]) > np.tan(np.deg2rad(RULES["ground_max_body_slope_deg"])):
            continue
        residual = np.abs(design@plane-values)/np.sqrt(1+np.sum(plane[:2]**2))
        mask = residual <= RULES["ground_fit_inlier_m"]
        score = (int(np.sum(mask)), -float(plane[2]))
        if best is None or score > best[0]:
            best = (score, plane, mask)
    if best is None or best[0][0] < RULES["ground_min_bins"]:
        return None
    plane, mask = best[1:]
    if np.min(np.ptp(representatives[mask, :2], axis=0)) < RULES["ground_min_xy_span_m"]:
        return None
    for _ in range(3):
        plane = np.linalg.lstsq(design[mask], values[mask], rcond=None)[0]
        residual = np.abs(design@plane-values)/np.sqrt(1+np.sum(plane[:2]**2))
        new_mask = residual <= RULES["ground_fit_inlier_m"]
        if np.sum(new_mask) < RULES["ground_min_bins"]:
            return None
        mask = new_mask
    if np.linalg.norm(plane[:2]) > np.tan(np.deg2rad(RULES["ground_max_body_slope_deg"])):
        return None
    return np.r_[plane, np.sum(mask), np.sqrt(np.mean(residual[mask]**2))]


def ground_heights(cloud, plane):
    if plane is None:
        return None
    a, b, c = plane[:3]
    return (cloud[:, 2]-a*cloud[:, 0]-b*cloud[:, 1]-c)/np.sqrt(1+a*a+b*b)


def retrieval_subsets(cloud, plane):
    body_mask = cloud[:, 2] <= RULES["body_high_threshold_m"]
    height = ground_heights(cloud, plane)
    ground_mask = np.ones(len(cloud), dtype=bool) if height is None else height <= RULES["local_ground_high_threshold_m"]
    balanced_height = cloud[:, 2] if height is None else height
    xy = np.floor(cloud[:, :2]/RULES["balanced_xy_cell_m"]).astype(np.int64)
    strata = np.floor(balanced_height/RULES["balanced_height_cell_m"]).astype(np.int64)
    # Fixed multiplicative point-index hash spreads retained acquisition samples.
    point_hash = np.arange(len(cloud), dtype=np.uint64)*np.uint64(11400714819323198485)
    order = np.lexsort((point_hash, strata, xy[:, 1], xy[:, 0]))
    keys = np.column_stack([xy[order], strata[order]])
    starts = np.r_[True, np.any(np.diff(keys, axis=0), axis=1)]
    offsets = np.arange(len(cloud))-np.maximum.accumulate(np.where(starts, np.arange(len(cloud)), 0))
    kept = np.sort(order[offsets < RULES["balanced_max_per_cell"]])
    return dict(full_v3=cloud, body_crop_proxy_v3=cloud[body_mask] if np.any(body_mask) else cloud,
                ground_crop_proxy_v3=cloud[ground_mask] if np.any(ground_mask) else cloud,
                balanced_proxy_v3=cloud[kept])


def height_stats(cloud, plane):
    full = describe(cloud).ravel()
    indices, values, valid = bins_and_values(cloud)
    occupied = full > 0
    result = dict(points=len(cloud), descriptor_radius_points=int(np.sum(valid)),
                  nonzero_sc_cells=int(np.sum(occupied)), ground_reference_available=plane is not None)
    for prefix, heights, threshold in (
        ("body_proxy", cloud[:, 2], RULES["body_high_threshold_m"]),
        ("local_ground", ground_heights(cloud, plane), RULES["local_ground_high_threshold_m"])):
        if heights is None:
            result[prefix+"_high_point_fraction"] = None
            result[prefix+"_high_max_cell_fraction"] = None
            continue
        high = heights[valid] > threshold
        maximum = np.zeros(1200, dtype=np.float32)
        np.maximum.at(maximum, indices[high], values[high])
        result[prefix+"_high_point_fraction"] = float(np.mean(high))
        result[prefix+"_high_max_cell_fraction"] = float(np.mean(maximum[occupied] == full[occupied])) if np.any(occupied) else None
    for key, value in zip(("ground_a", "ground_b", "ground_c", "ground_support_bins", "ground_fit_rms_m"), plane if plane is not None else [None]*5):
        result[key] = float(value) if value is not None else None
    return result


def direct_compare(query, candidate):
    best, best_shift = float("inf"), 0
    for shift in range(60):
        total, effective = 0., 0
        for sector in range(60):
            dot = nq = nc = 0.
            for ring in range(20):
                q, c = float(query[ring, sector]), float(candidate[ring, (sector+shift)%60])
                dot += q*c
                nq += q*q
                nc += c*c
            if nq > 0 and nc > 0:
                total += dot/math.sqrt(nq*nc)
                effective += 1
        value = 1-total/effective if effective else 1.
        if value < best:
            best, best_shift = value, shift
    yaw = best_shift*2*np.pi/60
    return max(0., best), yaw-2*np.pi if yaw > np.pi else yaw


def compare_all(query, candidates):
    query, candidates = query.astype(np.float64), candidates.astype(np.float64)
    nq, nc = np.linalg.norm(query, axis=0), np.linalg.norm(candidates, axis=1)
    q = np.divide(query, nq[None, :], out=np.zeros_like(query), where=nq[None, :] > 0)
    c = np.divide(candidates, nc[:, None, :], out=np.zeros_like(candidates), where=nc[:, None, :] > 0)
    corr = np.fft.irfft(np.sum(np.fft.rfft(c, axis=2)*np.conj(np.fft.rfft(q, axis=1)), axis=1), n=60, axis=1)
    effective = np.rint(np.fft.irfft(np.fft.rfft((nc > 0).astype(float), axis=1)*
        np.conj(np.fft.rfft((nq > 0).astype(float))), n=60, axis=1))
    values = 1-np.divide(corr, effective, out=np.zeros_like(corr), where=effective > 0)
    shifts = np.argmin(values, axis=1)
    distances = np.maximum(0., values[np.arange(len(c)), shifts])
    yaw = shifts*2*np.pi/60
    yaw[yaw > np.pi] -= 2*np.pi
    # Preserve source earliest-shift tie semantics for sparse/cropped variants.
    near_ties = np.sum(values <= values[np.arange(len(c)), shifts, None]+1e-12, axis=1) > 1
    for index in np.flatnonzero(near_ties):
        distances[index], yaw[index] = direct_compare(query, candidates[index])
    return distances, yaw


def full_cloud_support(bundle, loops, planes):
    """Single-frame full-point support using fixed v2 measured loop transforms.

    This is not the original +/-5 localmap correspondence set, nor truth.
    All original source points remain in the NN distance denominator.
    """
    result = []
    for row in loops:
        current, candidate = int(row["current_id"]), int(row["candidate_id"])
        source = read_cloud(bundle/f"keyframes/clouds/{current:06d}.pcd")
        target = read_cloud(bundle/f"keyframes/clouds/{candidate:06d}.pcd")
        rotation = Rotation.from_quat([float(row[f"relative_q{k}"]) for k in "xyzw"])
        translation = np.array([float(row[f"relative_t{k}"]) for k in "xyz"])
        transformed = rotation.inv().apply(source[:, :3]-translation)
        distances = cKDTree(target[:, :3]).query(transformed, workers=1)[0]
        height = ground_heights(source, planes[current])
        layers = dict(all=np.ones(len(source), dtype=bool), body_proxy_high=source[:, 2] > 2)
        if height is not None:
            layers.update(local_ground_low=height <= 1, local_ground_mid=(height > 1)&(height <= 3), local_ground_high=height > 3)
        for layer, mask in layers.items():
            if not np.any(mask):
                continue
            selected = distances[mask]
            matches = selected <= .5
            result.append(dict(current=current, candidate=candidate, layer=layer,
                full_evaluation_points=int(np.sum(mask)), matched_under_0_5m=int(np.sum(matches)),
                full_layer_coverage=float(np.mean(matches)), all_distance_p50_m=float(np.median(selected)),
                all_distance_p95_m=float(np.quantile(selected,.95)),
                matched_rms_m=float(np.sqrt(np.mean(selected[matches]**2))) if np.any(matches) else None))
    return result


def confirmation_observations(events, raw_rows, descriptors):
    """Audit compatible previous pending observations, not their independence.

    Uses logged v2 states only as offline measurement evidence. These states
    never enter descriptor construction, height screening or ground fitting.
    """
    pending=[e for e in events if e["reason"]=="loop_confirmation_pending"]
    result=[]
    def pose(event,prefix):
        rotation=Rotation.from_quat([float(event[prefix+"_q"+k]) for k in "xyzw"])
        translation=np.array([float(event[prefix+"_t"+k]) for k in "xyz"])
        return rotation,translation
    for committed in (e for e in events if e["graph_committed"]=="1"):
        current=int(committed["current"])
        predicted_r,predicted_t=pose(committed,"predicted")
        corrected_r,corrected_t=pose(committed,"corrected")
        previous=None
        for event in pending:
            prior=int(event["current"])
            if not 0<current-prior<=10:
                continue
            pr,pt=pose(event,"predicted")
            cr,ct=pose(event,"corrected")
            dr=cr*pr.inv()
            dt=ct-dr.apply(pt)
            if np.linalg.norm(dr.apply(predicted_t)+dt-corrected_t)>.5 or (
                corrected_r.inv()*dr*predicted_r).magnitude()>.1:
                continue
            if previous is None or prior>int(previous["current"]):
                previous=event
        if previous is None:
            result.append(dict(current=current,previous=None,frame_gap=None,time_gap_s=None,
                raw_body_origin_motion_proxy_m=None,full_sc_observation_distance=None,
                body_high_max_cells_aligned_jaccard=None,
                interpretation="no compatible previous pending event reconstructed"))
            continue
        prior=int(previous["current"])
        distance,yaw=direct_compare(descriptors[current],descriptors[prior])
        shift=int(round((yaw%(2*np.pi))*60/(2*np.pi)))%60
        high_current=descriptors[current]>4
        high_previous=np.roll(descriptors[prior]>4,-shift,axis=1)
        union=np.sum(high_current|high_previous)
        now=np.array([float(raw_rows[current][k]) for k in ("tx","ty","tz")])
        before=np.array([float(raw_rows[prior][k]) for k in ("tx","ty","tz")])
        result.append(dict(current=current,previous=prior,frame_gap=current-prior,
            time_gap_s=(int(raw_rows[current]["stamp_ns"])-int(raw_rows[prior]["stamp_ns"]))/1e9,
            raw_body_origin_motion_proxy_m=float(np.linalg.norm(now-before)),
            full_sc_observation_distance=distance,
            body_high_max_cells_aligned_jaccard=float(np.sum(high_current&high_previous)/union) if union else None,
            interpretation="compatible logged pending predecessor; descriptor similarity does not prove independent geometric evidence"))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[2]
    parser.add_argument("--bundle", type=Path, default=root/"maps/factory_map_1016")
    parser.add_argument("--v2", type=Path, default=root/"artifacts/loop_recall_20261020/enhanced_v2_full")
    parser.add_argument("--out", type=Path, default=root/"artifacts/loop_recall_v3_20261020/roof")
    args = parser.parse_args()
    started = time.perf_counter()
    args.out.mkdir(parents=True, exist_ok=True)
    raw_rows = csv_read(args.bundle/"keyframes/index.csv")
    loops = csv_read(args.v2/"loop_constraints.csv")
    events = csv_read(args.v2/"events.csv")
    failed = [e for e in events if e["reason"] == "translation_jump_above_limit" and e["has_pose_check"] == "1"]
    failed.sort(key=lambda e: (-float(e["overlap"]), float(e["fitness"])))
    # Fixed top-ten high-overlap failures, plus the specifically requested pairs.
    failure_pairs = list(dict.fromkeys((int(e["current"]),int(e["candidate"])) for e in failed))[:10]
    focus_pairs = [(630,21),(635,18),(640,14),(630,490),(635,490),(640,515),
                   (1322,582),(1298,70),(1214,1170)] + [
                       (int(e["current_id"]),int(e["candidate_id"])) for e in loops]+failure_pairs
    query_ids = sorted(set(range(617,647)) | {q for q,_ in focus_pairs})
    descriptors = {v:[] for v in VARIANTS}
    stats, planes = [], []
    for index, row in enumerate(raw_rows):
        cloud = read_cloud(args.bundle/row["cloud"])
        plane = fit_local_ground(cloud)
        planes.append(plane)
        subsets = retrieval_subsets(cloud, plane)
        record = dict(id=index, **height_stats(cloud, plane))
        for variant, subset in subsets.items():
            descriptors[variant].append(describe(subset))
            record[variant+"_retrieval_points"] = len(subset)
        stats.append(record)
        if index%250 == 0:
            print(f"rebuilt {index+1}/{len(raw_rows)} body clouds; {time.perf_counter()-started:.1f}s",flush=True)
    descriptors = {key:np.asarray(value,dtype=np.float32) for key,value in descriptors.items()}
    blob = (args.bundle/"descriptors/scan_context.bin").read_bytes()
    magic,count,rings,sectors = struct.unpack_from("<8sQQQ",blob)
    assert magic == b"A2WSCV1\0" and (count,rings,sectors)==(len(raw_rows),20,60)
    original = np.frombuffer(blob,dtype=np.dtype([("id","<u8"),("values","<f4",(20,60))]),offset=32)["values"]
    assert np.array_equal(original,descriptors["full_v3"]), "regenerated SC differs from production saved descriptors"
    csv_write(args.out/"cloud_height_proxies.csv", stats)
    cache_path = args.out/"descriptors_roof_proxy_v3.npz"
    np.savez(cache_path, version=VERSION, **descriptors)
    xyz = np.array([[float(r[k]) for k in ("tx","ty","tz")] for r in raw_rows])
    focused, retrieval, direct_checks, caches = [], [], [], {}
    for variant, bank in descriptors.items():
        distance_rows, yaw_rows = [], []
        for query in query_ids:
            distances, yaws = compare_all(bank[query],bank)
            distance_rows.append(distances)
            yaw_rows.append(yaws)
            eligible = np.arange(max(0,query-30))
            order = eligible[np.lexsort((eligible,distances[eligible]))]
            passed = order[distances[order] <= .2]
            spatial = np.linalg.norm(xyz-xyz[query],axis=1)
            top5 = passed[:5]
            nearby = passed[spatial[passed] <= 5]
            retrieval.append(dict(variant=variant,query=query,eligible_count=len(order),
                top5_ids=" ".join(map(str,top5)),top5_scores=" ".join(f"{distances[x]:.9f}" for x in top5),
                nearby_odom_proxy_under5m_top5=int(np.sum(spatial[top5] <= 5)),
                far_odom_proxy_over30m_top5=int(np.sum(spatial[top5] > 30)),
                first_nearby_rank_1based=int(np.flatnonzero(order==nearby[0])[0])+1 if len(nearby) else None,
                best_far_proxy_score=float(np.min(distances[eligible[spatial[eligible]>30]])) if np.any(spatial[eligible]>30) else None))
            for q,candidate in focus_pairs:
                if q != query or candidate > query-31:
                    continue
                rank=int(np.flatnonzero(order==candidate)[0])+1
                focused.append(dict(variant=variant,query=query,candidate=candidate,rank_1based=rank,
                    score=float(distances[candidate]),yaw_rad=float(yaws[candidate]),
                    odom_distance_proxy_m=float(spatial[candidate]),within_0_2=bool(distances[candidate]<=.2)))
                if (q,candidate) in [(630,21),(635,18),(640,14)] or len(direct_checks)<20:
                    direct=direct_compare(bank[query],bank[candidate])
                    direct_checks.append(dict(variant=variant,query=query,candidate=candidate,
                        distance_error=abs(direct[0]-distances[candidate]),yaw_error=abs(direct[1]-yaws[candidate])))
        caches[variant+"_distances"] = np.asarray(distance_rows)
        caches[variant+"_yaw_rad"] = np.asarray(yaw_rows)
    ranking_path = args.out/"selected_query_sc_cache_roof_v3.npz"
    np.savez(ranking_path,version=VERSION,query_ids=query_ids,**caches)
    csv_write(args.out/"focused_pair_rankings.csv",focused)
    csv_write(args.out/"retrieval_and_far_proxies.csv",retrieval)
    csv_write(args.out/"fixed_v2_commit_full_cloud_support.csv",full_cloud_support(args.bundle,loops,planes))
    confirmation_rows=confirmation_observations(events,raw_rows,descriptors["full_v3"])
    csv_write(args.out/"confirmation_observation_proxies.csv",confirmation_rows)
    query_index={query:index for index,query in enumerate(query_ids)}
    # Freeze far-proxy pairs selected by FULL retrieval, then compare the very
    # same candidates in all variants. These are not ground-truth negatives.
    fixed_far=[]
    for query in range(617,647):
        eligible=np.arange(query-30)
        full_dist=caches["full_v3_distances"][query_index[query]]
        full_order=eligible[np.lexsort((eligible,full_dist[eligible]))]
        for candidate in full_order[:5]:
            distance=np.linalg.norm(xyz[query]-xyz[candidate])
            if distance<=RULES["negative_odom_distance_proxy_m"]:
                continue
            for variant in VARIANTS:
                values=caches[variant+"_distances"][query_index[query]]
                order=eligible[np.lexsort((eligible,values[eligible]))]
                fixed_far.append(dict(variant=variant,query=query,candidate=int(candidate),
                    full_selected_far_odom_proxy_m=float(distance),score=float(values[candidate]),
                    rank_1based=int(np.flatnonzero(order==candidate)[0])+1,
                    within_0_2=bool(values[candidate]<=.2)))
    csv_write(args.out/"fixed_full_selected_far_proxy_pairs.csv",fixed_far)
    attitude=[]
    for query,candidate in dict.fromkeys(focus_pairs):
        q=Rotation.from_quat([float(raw_rows[query]["q"+axis]) for axis in "xyzw"])
        c=Rotation.from_quat([float(raw_rows[candidate]["q"+axis]) for axis in "xyzw"])
        angles=(c.inv()*q).as_euler("xyz",degrees=True)
        attitude.append(dict(query=query,candidate=candidate,relative_roll_deg=float(angles[0]),
            relative_pitch_deg=float(angles[1]),relative_yaw_deg=float(angles[2]),
            reference="relative body attitude from raw odometry; not absolute gravity"))
    csv_write(args.out/"relative_body_attitude_proxies.csv",attitude)
    mechanism=[]
    for frame in [0,21,630,635]:
        cloud=read_cloud(args.bundle/raw_rows[frame]["cloud"])
        base=describe(cloud)
        for axis,degree in [("x",2),("x",-2),("y",2),("y",-2),("y",5),("z",6),("z",180)]:
            rotated=cloud.copy()
            rotated[:,:3]=Rotation.from_euler(axis,degree,degrees=True).apply(cloud[:,:3]).astype(np.float32)
            distance,yaw=direct_compare(base,describe(rotated))
            mechanism.append(dict(frame=frame,artificial_axis=axis,artificial_degrees=degree,
                sc_distance=distance,sc_yaw_hint_rad=yaw,
                interpretation="synthetic operator sensitivity only; not real revisit/gravity evidence"))
    csv_write(args.out/"artificial_rotation_mechanism.csv",mechanism)
    assert max(r["distance_error"] for r in direct_checks)<1e-12
    assert max(r["yaw_error"] for r in direct_checks)<1e-12
    metadata=dict(version=VERSION,rules=RULES,descriptor_path=str(cache_path),descriptor_sha256=sha(cache_path),
        ranking_cache_sha256=sha(ranking_path),original_descriptor_sha256=hashlib.sha256(blob).hexdigest(),
        bundle_manifest_sha256=sha(args.bundle/"manifest.sha256"),source_script_sha256=sha(__file__),
        regenerated_full_matches_original_bitwise=True,cloud_count=len(raw_rows),query_ids=query_ids,
        committed_constraint_count=len(loops),failure_pairs=failure_pairs,direct_checks=direct_checks,
        local_ground_fit_available=sum(p is not None for p in planes),
        elapsed_s=time.perf_counter()-started,threads={k:os.environ.get(k) for k in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS")},
        coordinate_reference="body-relative and fitted local-ground proxies only; no physical gravity truth or future poses used",
        evaluation="all original source and target points retained; fixed v2 single-frame measured-transform NN check, not original localmap registration evidence",
        cache_boundary="fresh descriptors and selected-query distances/yaw; no v2 cache reused; offline timings not online throughput",
        command=[sys.executable,*sys.argv],python=sys.version,numpy=np.__version__,
        descriptor_source_path="third_party/scancontext_tro/src/scan_context.cpp",
        descriptor_source_sha256=sha(root/"third_party/scancontext_tro/src/scan_context.cpp"),
        distance_source_path="src/a2w_fastlio_common/src/scan_context_index.cpp",
        distance_source_sha256=sha(root/"src/a2w_fastlio_common/src/scan_context_index.cpp"),
        proxy_limit="High points are not semantically labelled roof. A fitted lower surface can be equipment or an unobserved floor; no historical gravity is reconstructed.")
    (args.out/"metadata.json").write_text(json.dumps(metadata,indent=2)+"\n")
    summary={"version":VERSION,"scope":"selected-query causal pilot, not full online replay",
             "variants":{},"height_statistics":{}}
    for variant in VARIANTS:
        records=[r for r in retrieval if r["variant"]==variant and 617<=r["query"]<=646]
        commits=[next(r for r in focused if r["variant"]==variant and r["query"]==int(loop["current_id"]) and r["candidate"]==int(loop["candidate_id"])) for loop in loops]
        fixed=[r for r in fixed_far if r["variant"]==variant]
        summary["variants"][variant]=dict(
            first_pass_median_nonzero_cells=float(np.median(np.count_nonzero(descriptors[variant][617:647],axis=(1,2)))),
            first_pass_median_body_high_max_cell_fraction=float(np.median([
                np.mean(descriptor[descriptor>0]>4) for descriptor in descriptors[variant][617:647]])),
            first_pass_nearby_proxy_queries_top5=sum(r["nearby_odom_proxy_under5m_top5"]>0 for r in records),
            first_pass_nearby_proxy_slots_top5=sum(r["nearby_odom_proxy_under5m_top5"] for r in records),
            first_pass_far_proxy_slots_top5=sum(r["far_odom_proxy_over30m_top5"] for r in records),
            committed_pair_top5_count=sum(r["rank_1based"]<=5 and r["score"]<=.2 for r in commits),
            committed_pair_within_score_count=sum(r["score"]<=.2 for r in commits),
            fixed_far_proxy_pair_count=len(fixed),fixed_far_proxy_within_score_count=sum(r["within_0_2"] for r in fixed),
            fixed_far_proxy_median_score=float(np.median([r["score"] for r in fixed])),
            key_pair_results=[r for r in focused if r["variant"]==variant and (r["query"],r["candidate"]) in [(630,21),(635,18),(640,14)]])
    for name,ids in dict(first_pass=set(range(617,647)),committed_endpoints={int(r[k]) for r in loops for k in ("current_id","candidate_id")},
                         major_failure_endpoints={i for pair in failure_pairs for i in pair}).items():
        selected=[stats[i] for i in sorted(ids)]
        summary["height_statistics"][name]={"frames":len(ids)}
        for field in ("body_proxy_high_point_fraction","body_proxy_high_max_cell_fraction","local_ground_high_point_fraction","local_ground_high_max_cell_fraction"):
            values=[r[field] for r in selected if r[field] is not None]
            summary["height_statistics"][name][field+"_median"]=float(np.median(values)) if values else None
    (args.out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({k:metadata[k] for k in ("cloud_count","committed_constraint_count","local_ground_fit_available","elapsed_s")},indent=2))


if __name__ == "__main__":
    main()
