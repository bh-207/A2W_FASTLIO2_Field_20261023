#!/usr/bin/env python3
"""Read-only final Bundle/geometry-input consistency audit; writes evidence only."""
import argparse,os,sys
from pathlib import Path
os.environ['OPENBLAS_NUM_THREADS']='1'
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from evaluate import BUNDLE,ROOT,sha,poses,read_csv,write_json,io

def main():
 p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--candidate-csv',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
 limits=__import__('json').loads((ROOT/'final_bundle_verify_limits.json').read_text());b=a.bundle.resolve()
 # Every Bundle manifest entry is verified independently of its metadata status.
 checked=[]
 for line in (b/'manifest.sha256').read_text().splitlines():
  expected,rel=line.split(maxsplit=1);rel=rel.lstrip('*');path=(b/rel).resolve()
  if not path.is_relative_to(b):raise ValueError('unsafe Bundle manifest path')
  actual=sha(path)
  if actual!=expected:raise ValueError('Bundle manifest mismatch: '+rel)
  checked.append(rel)
 raw=read_csv(BUNDLE/'keyframes/index.csv');target_raw=read_csv(b/'keyframes/index.csv')
 if raw!=target_raw:raise ValueError('raw keyframe index changed')
 cp=poses(read_csv(a.candidate_csv));bp=poses(read_csv(b/'optimized_poses.csv'))
 translation=np.linalg.norm(cp[:,:3,3]-bp[:,:3,3],axis=1);rotation=Rotation.from_matrix(cp[:,:3,:3]@bp[:,:3,:3].transpose(0,2,1)).magnitude()
 if translation.max()>1e-10 or rotation.max()>1e-10:raise ValueError('Bundle candidate poses changed')
 _,global_data=io.read_pcd(b/'global_map.pcd');global_xyz=io.xyz(global_data)
 if not np.isfinite(global_xyz).all():raise ValueError('nonfinite global map')
 tree=cKDTree(global_xyz);bodyhash=[];max_distance=0.;sample_count=0;total=0
 for r in raw:
  i=int(r['id']);rel=r['cloud'];source_hash=sha(BUNDLE/rel);target_hash=sha(b/rel)
  if source_hash!=target_hash:raise ValueError('body cloud identity changed: '+rel)
  _,body=io.read_pcd(b/rel);xyz=io.xyz(body);total+=len(xyz)
  ix=np.unique(np.linspace(0,len(xyz)-1,min(len(xyz),limits['selected_points_per_keyframe']),dtype=int))
  expected=xyz[ix]@bp[i,:3,:3].T+bp[i,:3,3];distance,_=tree.query(expected,k=1,workers=1)
  max_distance=max(max_distance,float(distance.max()));sample_count+=len(ix);bodyhash.append(dict(id=i,sha256=source_hash,point_count=len(xyz)))
 if total!=len(global_xyz):raise ValueError('leaf0 global point count differs from body sum')
 if max_distance>limits['global_map_selected_identity_rounding_limit_m']:raise ValueError('selected identity absent from rebuilt global map')
 result=dict(status='VERIFIED_EXPORT_CONSISTENCY_ONLY',bundle=str(b),candidate_csv=str(a.candidate_csv.resolve()),candidate_csv_sha256=sha(a.candidate_csv),bundle_pose_csv_sha256=sha(b/'optimized_poses.csv'),manifest_entries_checked=len(checked),all_828_body_cloud_hashes_equal=True,raw_index_equal=True,pose_max_translation_difference_m=float(translation.max()),pose_max_rotation_difference_rad=float(rotation.max()),global_map_points=len(global_xyz),body_point_sum=total,selected_original_identity_global_point_checks=sample_count,selected_identity_max_nearest_distance_m=max_distance,limits=limits,body_clouds=bodyhash,limitations=['Global map check uses16original records per frame; no complete6.5million-point bijection or surveyed truth claimed.','Export consistency is independent of geometry acceptance; candidate still requires fixed eight ROI report.'],geometry_acceptance_status='SEE_FIXED_EVALUATOR_REPORT')
 write_json(a.out,result);print({k:v for k,v in result.items() if k not in ['body_clouds','limits','limitations']})
if __name__=='__main__':main()
