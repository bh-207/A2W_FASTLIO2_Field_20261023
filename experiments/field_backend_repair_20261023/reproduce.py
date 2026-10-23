#!/usr/bin/env python3
"""Fresh frozen eight-mask attribution, protected Bundle and fixed geometry.

Only saved local data is read. A failed geometric gate returns2 and keeps output.
The optional full original-rate replay separately requires >=35GiB free space.
"""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import os
import build

HERE=Path(__file__).resolve().parent
WORKSPACE=HERE.parents[1]
OUT=WORKSPACE/'captures/field_backend_repair_20261023_001'
BUNDLE=WORKSPACE/'captures/field_fixed_sessions/fixed_short_20261023_001/bundles/map_bundle'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name');parser.add_argument('--reuse-build',action='store_true')
    parser.add_argument('--commands-only',action='store_true')
    parser.add_argument('--full-replay-id',type=int)
    args=parser.parse_args()
    if not args.name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.name):
        raise ValueError('safe fresh experiment name required')
    root=OUT/'reproductions'/args.name
    if root.exists():raise ValueError('new output name required: '+str(root))
    runner=['bash',str(HERE/'run.sh')]
    binary=OUT/'build/backend_install/a2w_fastlio_mapping/lib/a2w_fastlio_mapping'
    measurements=OUT/'frozen/run1/new_measurements.csv'
    commands=[]
    if not args.reuse_build:commands.append([sys.executable,str(HERE/'build.py')])
    commands.extend([
        runner+[str(binary/'field_frozen_graph'),str(BUNDLE),str(root/'eight_masks'),'--measurements',str(measurements)],
        runner+[str(binary/'field_frozen_graph'),str(BUNDLE),str(root/'guarded_masks'),'--measurements',str(measurements),'--guard'],
        runner+[str(binary/'field_bundle_from_graph'),str(BUNDLE),str(root/'guarded_masks/mask_7/optimized_poses.csv'),str(root/'candidate_bundle')],
        [sys.executable,str(HERE/'geometry/evaluate.py'),'--candidate','candidate='+str(root/'candidate_bundle/optimized_poses.csv'),'--out',str(root/'geometry')]])
    if args.commands_only:
        for command in commands:print(shlex.join(command))
        return 0
    required=(35 if args.full_replay_id is not None else 2)*2**30
    if shutil.disk_usage(OUT).free<required:raise ValueError('insufficient independent output space; original/failure data will not be deleted')
    if args.reuse_build:
        manifest=json.loads((OUT/'build/build_manifest.json').read_text())
        if manifest['sources']!=build.source_hashes():raise ValueError('source identity changed; rebuild all packages')
        for value in manifest['binaries'].values():
            if build.digest(value['path'])!=value['sha256']:raise ValueError('binary identity mismatch')
    root.mkdir(parents=True)
    with (root/'commands.json').open('x') as stream:json.dump(commands,stream,indent=2)
    for index,command in enumerate(commands):
        with (root/('step_'+str(index)+'.log')).open('x') as stream:
            subprocess.run(command,cwd=WORKSPACE,stdout=stream,stderr=subprocess.STDOUT,check=True)
    geometry=json.loads((root/'geometry/report.json').read_text())['acceptance']['candidate']
    result=dict(frozen_candidate_geometry=geometry['status'],historical_exact_factors_recovered=False,
                input_measurements=str(measurements),candidate_bundle=str(root/'candidate_bundle'),recommended=False)
    if args.full_replay_id is not None:
        if args.full_replay_id<20:raise ValueError('use a fresh offline replay id>=20')
        name='offline_smoke_'+format(args.full_replay_id,'03d')
        env=dict(os.environ,ROS_DOMAIN_ID='87',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                 A2W_REPLAY_SECONDS='0',A2W_REPLAY_RATE='1',A2W_OFFLINE_GEOMETRY_GUARD='1')
        command=runner+[sys.executable,str(HERE/'source/experiments/field_fixed_20261023/offline_smoke.py'),name]
        subprocess.run(command,cwd=WORKSPACE,env=env,check=False)
        run=OUT/'tests'/name
        if (run/'result.json').is_file():
            subprocess.run([sys.executable,str(HERE/'accept_run.py'),str(run)],check=False)
        result['full_replay_acceptance']=json.loads((run/'ACCEPTANCE.json').read_text())['status'] if (run/'ACCEPTANCE.json').is_file() else 'FAIL'
    (root/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if geometry['status']=='PASS' and result.get('full_replay_acceptance','PASS_SCOPED_OFFLINE')=='PASS_SCOPED_OFFLINE' else 2

if __name__=='__main__':raise SystemExit(main())
