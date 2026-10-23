#!/usr/bin/python3
"""Freeze original 59 surface pairs/full clouds, then evaluate named trajectories.

No physical gravity or semantic roof label is inferred. Height strata use the
original reliable fitted local-ground proxy, preserving original XYZ/slopes.
The 1026 original ground pairs also receive rotation-invariant fixed-anchor
and point-to-plane checks. All selections, points, normals and correspondences
are frozen once from the original Bundle and reused for every variant.
"""
import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

VERSION="fixed_layers_local_ground_proxy_v3_20261020"
LAYERS=("all","local_ground_proxy_low_le_0_25m","local_ground_proxy_mid_0_25_to_3m","local_ground_proxy_high_gt_3m")


def load_module(name, filename):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(filename))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


geometry=load_module("_fixed_geometry","loop_recall_geometry.py")
roof=load_module("_roof_reader","loop_roof_audit.py")


def local_ground_heights(points, observation):
    if not observation.get("reliable"):
        raise ValueError("local-ground proxy is unreliable; no gravity fallback allowed")
    normal=np.asarray(observation["normal"],dtype=float)
    length=np.linalg.norm(normal)
    if not np.isfinite(normal).all() or not np.isfinite(observation["d"]) or length<1e-12:
        raise ValueError("invalid local-ground proxy")
    return (points@normal+observation["d"])/length


def normal_estimates(points):
    distances,indices=cKDTree(points).query(points,k=min(12,len(points)),workers=1)
    local=points[indices]
    centered=local-local.mean(axis=1)[:,None,:]
    values,vectors=np.linalg.eigh(np.einsum("nki,nkj->nij",centered,centered))
    valid=(distances[:,-1]<=.6)&(values[:,1]>1e-5)&(values[:,0]/np.maximum(values[:,1],1e-20)<.3)
    return vectors[:,:,0],valid


def stats(values):
    values=np.asarray(values,dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("nonfinite metric; no silent exclusion")
    if not len(values):
        return dict(p50=None,p95=None,max=None,rms=None)
    return dict(p50=float(np.median(values)),p95=float(np.quantile(values,.95)),
                max=float(np.max(values)),rms=float(np.sqrt(np.mean(values**2))))


def layer_metrics(a,b,na,nb,valid_a,valid_b,ha,hb,ab,ba):
    """All point-distance denominators retained; normal confidence fixed before variants."""
    delta_a=a-b[ab]
    delta_b=b-a[ba]
    fixed=np.r_[np.linalg.norm(delta_a,axis=1),np.linalg.norm(delta_b,axis=1)]
    nearest=np.r_[cKDTree(b).query(a,workers=1)[0],cKDTree(a).query(b,workers=1)[0]]
    point_plane=np.r_[np.abs(np.einsum("ij,ij->i",delta_a,nb[ab])),
                       np.abs(np.einsum("ij,ij->i",delta_b,na[ba]))]
    plane_valid=np.r_[valid_b[ab],valid_a[ba]]
    normal_valid=np.r_[valid_a&valid_b[ab],valid_b&valid_a[ba]]
    source_normals=np.r_[na,nb]
    target_normals=np.r_[nb[ab],na[ba]]
    angle=np.degrees(np.arctan2(np.linalg.norm(np.cross(source_normals,target_normals),axis=1),
                                np.abs(np.einsum("ij,ij->i",source_normals,target_normals))))
    heights=np.r_[ha,hb]
    masks=(np.ones(len(heights),dtype=bool),heights<=.25,(heights>.25)&(heights<=3),heights>3)
    result=[]
    for name,mask in zip(LAYERS,masks):
        fixed_stats=stats(fixed[mask]);nn_stats=stats(nearest[mask])
        p2p=stats(point_plane[mask&plane_valid]);norm=stats(angle[mask&normal_valid])
        count=int(np.sum(mask))
        result.append(dict(layer=name,directed_points=count,fixed_correspondences=count,
            fixed_match_p50_m=fixed_stats["p50"],fixed_match_p95_m=fixed_stats["p95"],
            fixed_match_max_m=fixed_stats["max"],fixed_match_rms_m=fixed_stats["rms"],
            fixed_coverage_under_0_5m=float(np.mean(fixed[mask]<=.5)) if count else None,
            nn_p50_m=nn_stats["p50"],nn_p95_m=nn_stats["p95"],nn_max_m=nn_stats["max"],
            nn_coverage_under_0_5m=float(np.mean(nearest[mask]<=.5)) if count else None,
            fixed_target_normal_points=int(np.sum(mask&plane_valid)),point_to_plane_p95_m=p2p["p95"],
            point_to_plane_rms_m=p2p["rms"],fixed_two_normal_points=int(np.sum(mask&normal_valid)),
            normal_angle_p95_deg=norm["p95"],normal_angle_p50_deg=norm["p50"]))
    return result


def ground_invariant_rows(poses,pairs,anchors):
    rotations=Rotation.from_quat(poses[:,3:]).as_matrix()
    result=[]
    for pair,anchor in zip(pairs,anchors):
        i,j=pair["i"],pair["j"]
        a=rotations[i]@anchor[0]+poses[i,:3]
        b=rotations[j]@anchor[1]+poses[j,:3]
        ni=np.asarray(pair["na"],dtype=float);nj=np.asarray(pair["nb"],dtype=float)
        di,dj=pair["da"]/np.linalg.norm(ni),pair["db"]/np.linalg.norm(nj)
        ni=rotations[i]@(ni/np.linalg.norm(ni));nj=rotations[j]@(nj/np.linalg.norm(nj))
        di-=ni@poses[i,:3];dj-=nj@poses[j,:3]
        p2p=np.array([abs(nj@a+dj),abs(ni@b+di)])
        angle=np.degrees(np.arctan2(np.linalg.norm(np.cross(ni,nj)),abs(ni@nj)))
        result.append([i,j,float(np.linalg.norm(a-b)),float(np.sqrt(np.mean(p2p**2))),
                       float(np.max(p2p)),float(angle)])
    return result


def freeze(bundle,evidence,cache,legacy_report):
    started=time.perf_counter()
    manifest=geometry.validate_manifest(bundle,evidence)
    index=roof.csv_read(bundle/"keyframes/index.csv")
    saved=geometry.read_pose_csv(bundle/"optimized_poses.csv",len(index))
    observations=json.loads((evidence/"ground_observations.json").read_text())
    ground_pairs=json.loads((evidence/"ground_pairs.json").read_text())
    regions=json.loads((evidence/"regions.json").read_text())
    memberships={name:set(ids) for name,ids in regions["keyframes"].items()}
    memberships["far"]=set().union(*(ids for name,ids in memberships.items() if name.startswith("H_")))
    original,audit=geometry.freeze_surfaces(bundle,index,observations,ground_pairs,saved,memberships)
    if len(original)!=59 or len(ground_pairs)!=1026:
        raise ValueError("original 59 non-ground/1026 ground-pair contract changed")
    if legacy_report.exists():
        legacy=json.loads(legacy_report.read_text())["fixed_evidence"]["non_ground"]
        if legacy["fixed_points_and_matches_sha256"]!=audit["fixed_points_and_matches_sha256"]:
            raise ValueError("original v2 fixed non-ground correspondences changed")
    arrays={"saved_poses":saved}
    frames=sorted({int(pair[k]) for pair in original for k in ("i","j")})
    trees={}
    rotations=Rotation.from_quat(saved[:,3:]).as_matrix()
    for frame in frames:
        points=roof.read_cloud(bundle/index[frame]["cloud"])[:,:3].astype(float)
        normals,valid=normal_estimates(points)
        arrays[f"frame_{frame}_points"]=points
        arrays[f"frame_{frame}_normals"]=normals
        arrays[f"frame_{frame}_normal_valid"]=valid
        arrays[f"frame_{frame}_heights"]=local_ground_heights(points,observations[frame])
        trees[frame]=cKDTree(points)
    pair_meta=[]
    for position,pair in enumerate(original):
        i,j=pair["i"],pair["j"]
        da,ia=trees[i].query(pair["a"],workers=1);db,ib=trees[j].query(pair["b"],workers=1)
        if max(np.max(da),np.max(db))>1e-10:
            raise ValueError("legacy wall points are not exact original full-cloud points")
        arrays[f"pair_{position}_wall_a"]=ia
        arrays[f"pair_{position}_wall_b"]=ib
        arrays[f"pair_{position}_wall_ab"]=pair["ab"]
        arrays[f"pair_{position}_wall_ba"]=pair["ba"]
        aw=arrays[f"frame_{i}_points"]@rotations[i].T+saved[i,:3]
        bw=arrays[f"frame_{j}_points"]@rotations[j].T+saved[j,:3]
        arrays[f"pair_{position}_full_ab"]=cKDTree(bw).query(aw,workers=1)[1]
        arrays[f"pair_{position}_full_ba"]=cKDTree(aw).query(bw,workers=1)[1]
        pair_meta.append({key:pair[key] for key in ("i","j","scope","region")})
    anchors=[]
    for pair in ground_pairs:
        body=[]
        for endpoint,normal_key,d_key in (("i","na","da"),("j","nb","db")):
            frame=pair[endpoint];normal=rotations[frame]@np.asarray(pair[normal_key])
            offset=pair[d_key]-normal@saved[frame,:3]
            xy=np.asarray(pair["xy"])
            if abs(normal[2])<1e-8:
                raise ValueError("original ground proxy vertical; cannot freeze original reference anchor")
            world=np.r_[xy,-(offset+normal[:2]@xy)/normal[2]]
            body.append(rotations[frame].T@(world-saved[frame,:3]))
        anchors.append(body)
    arrays["ground_anchors"]=np.array(anchors)
    cache.parent.mkdir(parents=True,exist_ok=True)
    np.savez(cache,**arrays)
    metadata=dict(version=VERSION,frame_count=len(index),frames=frames,pairs=pair_meta,
        ground_pairs=ground_pairs,memberships={name:sorted(ids) for name,ids in memberships.items()},
        legacy_non_ground_audit=audit,bundle_manifest=manifest,cache_sha256=geometry.sha256(cache),
        input_sha256={str(path):geometry.sha256(path) for path in (
            evidence/"ground_observations.json",evidence/"ground_pairs.json",evidence/"regions.json",
            bundle/"optimized_poses.csv")},elapsed_s=time.perf_counter()-started,
        normal_confidence="fixed original body neighbours k12, radius<=.6m, lambda1>1e-5 and lambda0/lambda1<.3; point-distance denominators never filtered",
        reference="original fitted local-ground height proxy only; not physical gravity or semantic roof truth",
        frozen_selection_bias="pairs, overlap regions, correspondences and anchors use original saved map; reused identically across all variants")
    cache.with_suffix(".json").write_text(json.dumps(metadata,indent=2)+"\n")
    print(f"froze {len(original)} legacy/full pairs, {len(frames)} complete clouds, {len(ground_pairs)} ground pairs",flush=True)
    return metadata,arrays


def evaluate_variant(poses,metadata,arrays):
    rotations=Rotation.from_quat(poses[:,3:]).as_matrix()
    worlds={}
    for frame in metadata["frames"]:
        worlds[frame]=(arrays[f"frame_{frame}_points"]@rotations[frame].T+poses[frame,:3],
                       arrays[f"frame_{frame}_normals"]@rotations[frame].T)
    rows=[]
    for position,pair in enumerate(metadata["pairs"]):
        i,j=pair["i"],pair["j"]
        for pointset in ("legacy_non_ground","full_original_clouds"):
            wall=pointset=="legacy_non_ground"
            a=arrays[f"pair_{position}_wall_a"] if wall else np.arange(len(worlds[i][0]))
            b=arrays[f"pair_{position}_wall_b"] if wall else np.arange(len(worlds[j][0]))
            prefix=f"pair_{position}_{'wall' if wall else 'full'}"
            values=layer_metrics(a=worlds[i][0][a],b=worlds[j][0][b],na=worlds[i][1][a],nb=worlds[j][1][b],
                valid_a=arrays[f"frame_{i}_normal_valid"][a],valid_b=arrays[f"frame_{j}_normal_valid"][b],
                ha=arrays[f"frame_{i}_heights"][a],hb=arrays[f"frame_{j}_heights"][b],
                ab=arrays[prefix+"_ab"],ba=arrays[prefix+"_ba"])
            rows.extend([dict(i=i,j=j,pointset=pointset,scope=pair["scope"],region=pair["region"],**value) for value in values])
    ground=ground_invariant_rows(poses,metadata["ground_pairs"],arrays["ground_anchors"])
    return rows,ground


def aggregate(rows):
    result={}
    for pointset in ("legacy_non_ground","full_original_clouds"):
        for layer in LAYERS:
            chosen=[r for r in rows if r["pointset"]==pointset and r["layer"]==layer]
            key=pointset+":"+layer
            result[key]=dict(pair_count=len(chosen),directed_points=sum(r["directed_points"] for r in chosen))
            for metric in ("fixed_match_p95_m","fixed_match_max_m","fixed_coverage_under_0_5m",
                           "nn_p95_m","point_to_plane_p95_m","normal_angle_p95_deg"):
                values=[r[metric] for r in chosen if r[metric] is not None]
                result[key][metric]=stats(values)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle",type=Path)
    parser.add_argument("--evidence",type=Path,default=Path("artifacts/map_repair_1016"))
    parser.add_argument("--freeze-cache",type=Path,required=True)
    parser.add_argument("--variant",action="append",default=[],help="NAME=complete_pose_csv (repeatable)")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--legacy-report",type=Path,default=Path("artifacts/loop_recall_20261020/geometry_v2_final/geometry_report.json"))
    args=parser.parse_args()
    bundle,evidence=args.bundle.resolve(),args.evidence.resolve()
    cache,output=args.freeze_cache.resolve(),args.output.resolve()
    geometry.check_destination(cache,bundle,evidence)
    geometry.check_destination(output,bundle,evidence)
    if output.exists() and any(output.iterdir()):
        raise ValueError("output must be a fresh directory; frozen cache is separate and reusable")
    output.mkdir(parents=True,exist_ok=True)
    if cache.exists():
        metadata=json.loads(cache.with_suffix(".json").read_text())
        if metadata["version"]!=VERSION or geometry.sha256(cache)!=metadata["cache_sha256"]:
            raise ValueError("frozen cache version/hash mismatch")
        if geometry.sha256(bundle/"manifest.sha256")!=metadata["bundle_manifest"]["manifest_sha256"]:
            raise ValueError("frozen cache belongs to a different Bundle")
        with np.load(cache,allow_pickle=False) as saved:
            arrays={key:saved[key].copy() for key in saved.files}
    else:
        metadata,arrays=freeze(bundle,evidence,cache,args.legacy_report)
    all_metrics={};all_rows={};pose_hashes={}
    for variant in args.variant:
        name,path=variant.split("=",1)
        if not name or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for character in name) or name in all_metrics:
            raise ValueError("variant names must be unique safe filename components")
        path=Path(path).resolve()
        poses=geometry.read_pose_csv(path,metadata["frame_count"])
        rows,ground=evaluate_variant(poses,metadata,arrays)
        roof.csv_write(output/f"{name}_layer_pairs.csv",rows)
        geometry.write_csv(output/f"{name}_ground_invariant.csv",
            ["i","j","frozen_anchor_distance_m","symmetric_point_to_plane_rms_m","symmetric_point_to_plane_max_m","plane_normal_angle_deg"],ground)
        all_metrics[name]=aggregate(rows)
        all_metrics[name]["ground_invariant"]={key:stats([row[column] for row in ground]) for key,column in (
            ("frozen_anchor_distance_m",2),("point_to_plane_rms_m",3),("plane_normal_angle_deg",5))}
        all_rows[name]=rows;pose_hashes[name]=dict(path=str(path),sha256=geometry.sha256(path))
        print(f"evaluated {name}: {len(rows)} layer rows, {len(ground)} fixed ground rows",flush=True)
    comparisons=[]
    if all_rows:
        reference_name=next(iter(all_rows))
        reference=all_rows[reference_name]
        for name,rows in all_rows.items():
            if name==reference_name:continue
            for before,after in zip(reference,rows):
                if tuple(before[k] for k in ("i","j","pointset","layer","directed_points"))!=tuple(after[k] for k in ("i","j","pointset","layer","directed_points")):
                    raise ValueError("variant-dependent evaluation selection detected")
                for metric in ("fixed_match_p95_m","fixed_match_max_m","point_to_plane_p95_m","normal_angle_p95_deg"):
                    if before[metric] is None or after[metric] is None:continue
                    comparisons.append(dict(reference=reference_name,variant=name,i=before["i"],j=before["j"],
                        pointset=before["pointset"],layer=before["layer"],metric=metric,
                        before=before[metric],after=after[metric],delta=after[metric]-before[metric]))
    roof.csv_write(output/"paired_layer_changes.csv",comparisons)
    report=dict(version=VERSION,frozen_cache=str(cache),frozen_cache_sha256=metadata["cache_sha256"],
        legacy_non_ground=metadata["legacy_non_ground_audit"],ground_pairs=len(metadata["ground_pairs"]),
        trajectories=pose_hashes,metrics=all_metrics,reference=metadata["reference"],
        selection_bias=metadata["frozen_selection_bias"],normal_confidence=metadata["normal_confidence"],
        command=[sys.executable,*sys.argv],source_sha256=geometry.sha256(__file__),
        threads={name:os.environ.get(name) for name in ("OMP_NUM_THREADS","OPENBLAS_NUM_THREADS")},
        limitation="all-original-point internal consistency, no physical gravity/roof labels or independent ground truth; correspondence selection and evaluation pairs have been used in prior tuning")
    (output/"summary.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")


if __name__=="__main__":
    main()
