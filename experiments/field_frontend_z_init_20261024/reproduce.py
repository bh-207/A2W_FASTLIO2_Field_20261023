#!/usr/bin/env python3
"""Controlled, isolated, complete-input frontend experiment. Never live/production."""
import argparse,csv,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
E=Path(__file__).resolve().parent;W=E.parents[1];AUD=W/'captures/field_frontend_z_init_20261024_001/input_audit';REF=W/'captures/field_fixed_sessions/fixed_short_20261023_001/bundles/map_bundle'
def call(args,env):print('RUN',*[str(x)for x in args],flush=True);subprocess.run([str(x)for x in args],env=env,check=True,cwd=W)
def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root.resolve()
 if root.exists():raise ValueError('new experiment root required; nothing overwritten')
 if shutil.disk_usage(root.parent).free<12*1024**3:raise ValueError('paced mode needs at least12GiB free; no extra raw Bag is recorded')
 if not(AUD/'imu_raw.csv').is_file():raise ValueError('first run input_audit/extract_input_audit.py using original field Bag; see README_ZH.txt')
 root.mkdir();env=dict(os.environ,OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',A2W_REPAIR_OUTPUT=str(root))
 call([sys.executable,E/'build.py','--build',root/'build'],env)
 call([sys.executable,E/'run_consumption.py','B_009','--root',root],env)
 b=root/'runs/B_009';frozen=root/'geometry_frozen'
 call([sys.executable,E/'ground_eval/ground_eval.py','freeze-baseline','--index',b/'reconstruction/post_index.csv','--trace',b/'session/runtime_state.csv','--timing',AUD/'cloud_timing.csv','--contract',E/'config/identity_contract.json','--preprocess',b/'reconstruction/preprocess_identity.csv','--out',frozen],env)
 call([sys.executable,E/'joint_init/lidar_registration.py',b/'reconstruction/preprocess_index.csv',root/'joint_motion','--start-seconds','150','--seconds','65','--stride','3'],env)
 rows=list(csv.DictReader((b/'session/init_quality.csv').open()));row=next(r for r in rows if r['stage']=='initialized');scale=9.81/np.linalg.norm([float(row['mean_'+k])for k in ['ax','ay','az']])
 call([sys.executable,E/'joint_init/joint_cli.py',root/'joint_motion/lidar_poses.npz',AUD/'imu_raw.csv',root/'candidate.json','--acc-scale',repr(float(scale))],env)
 call([sys.executable,E/'prepare_joint_candidate.py',root/'candidate.json',root/'joint_motion/lidar_poses.npz',b/'reconstruction/post_index.csv',AUD/'cloud_timing.csv',root/'candidate_frontend.json'],env)
 for name in ['C_001','C_002']:call([sys.executable,E/'run_consumption.py',name,'--root',root,'--candidate',root/'candidate_frontend.json','--export-seconds','0'],env)
 binary=root/'build/backend_install/a2w_fastlio_mapping/lib/a2w_fastlio_mapping/field_consumption_backend'
 for name in ['B_009','C_001','C_002']:
  run=root/'runs'/name
  call([sys.executable,E/'input_audit/audit_consumption_stages.py','--run',run,'--original',AUD,'--output',root/'audit'/name],env)
  call(['bash',E/'run.sh',binary,run/'reconstruction/post_index.csv',REF,root/'maps'/name,'--source-manifest',run/'build_manifest.json'],env)
  call([sys.executable,E/'export_raw_map.py',root/'maps'/name],env)
  call([sys.executable,E/'ground_eval/ground_eval.py','evaluate','--frozen',frozen,'--index',run/'reconstruction/post_index.csv','--contract',E/'config/identity_contract.json','--preprocess',run/'reconstruction/preprocess_identity.csv','--out',root/'geometry'/name],env)
 call([sys.executable,E/'make_map_views.py',root,root/'map_views'],env)
 states={name:json.loads((root/'runs'/name/'result.json').read_text())for name in ['B_009','C_001','C_002']}
 (root/'reproduction_summary.json').write_text(json.dumps(dict(runs=states,realtime_acceptance='NOT_RUN',physical_calibration='NOT_CERTIFIED',geometry_improvement_proven=False,warning='This runs experiments; exit0 proves execution, never geometric acceptance. Explicitly inspect joint receipts, holdout and frozen geometry.'),indent=2)+'\n')
 print('DONE',root/'reproduction_summary.json')
if __name__=='__main__':main()
