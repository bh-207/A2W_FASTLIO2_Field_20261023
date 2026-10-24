#!/usr/bin/env python3
"""Fresh sequential reconstruction; no source publisher, recorder or robot."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import uuid
import time
import shutil
import yaml

E=Path(__file__).resolve().parent
W=E.parents[1]
BAG=W/'captures/field_fixed_sessions/fixed_short_20261023_001/bag'

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('run_id');p.add_argument('--root',type=Path,default=Path('/tmp/a2w_frontend_z_init_20261024_001'));p.add_argument('--seconds',type=float,default=0.0);p.add_argument('--candidate',type=Path);p.add_argument('--export-seconds',type=float,default=230.);p.add_argument('--debug',action='store_true')
    a=p.parse_args()
    if not a.run_id.replace('_','').isalnum() or a.seconds<0:raise ValueError('safe fresh run ID and nonnegative scope required')
    root=a.root.resolve();manifest=root/'build/build_manifest.json';identity=json.loads(manifest.read_text());binary=Path(identity['binaries']['frontend']['path'])
    if Path(identity['source_root']).resolve()!=E/'source' or digest(binary)!=identity['binaries']['frontend']['sha256']:raise ValueError('current independent frontend build required')
    run=root/'runs'/a.run_id;run.mkdir(parents=True,exist_ok=False);session=run/'session';session.mkdir()
    params=yaml.safe_load((E/'config/baseline.yaml').read_text().replace('@SESSION@',str(session)));r=next(iter(params.values()))['ros__parameters']
    r['offline']={'consume_bag':str(BAG),'output':str(run/'reconstruction'),'maximum_input_seconds':a.seconds,'preprocess_export_seconds':a.export_seconds}
    if a.candidate:
        candidate=json.loads(a.candidate.read_text())
        for key in ['source_imu','source_poses']:
            if digest(candidate[key])!=candidate[key+'_sha256']:raise ValueError('joint evidence input hash differs: '+key)
        if Path(candidate['source_imu']).resolve()!=(W/'captures/field_frontend_z_init_20261024_001/input_audit/imu_raw.csv').resolve():raise ValueError('candidate does not use this original Bag IMU audit')
        r['frontend_init_v4'].update(joint_initialization_enabled=True,joint_candidate_path=str(a.candidate.resolve()))
    paramfile=run/'parameters.yaml';paramfile.write_text(yaml.safe_dump(params,sort_keys=False))
    dds=run/'localhost.xml';dds.write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="any"><General><AllowMulticast>false</AllowMulticast><Transport>tcp</Transport></General><TCP><Port>39801</Port><NoDelay>true</NoDelay></TCP><Discovery><Peers><Peer Address="127.0.0.1:39801"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>\n')
    env=dict(os.environ,A2W_REPAIR_OUTPUT=str(root),A2W_FIELD_MODE='offline-test',A2W_CAPTURE_ROOT=str(root),A2W_HANDOFF_DIRECTORY=str(session),A2W_REQUIRED_OBSERVER_NAME='/field_capture_observer_'+uuid.uuid4().hex,ROS_DOMAIN_ID='87',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file://'+str(dds),OPENBLAS_NUM_THREADS='1')
    command=['bash',str(E/'run.sh'),str(binary),'--ros-args','--params-file',str(paramfile)]
    if a.debug:command=command[:2]+['gdb','--batch','-ex','run','-ex','thread apply all bt','--args']+command[2:]
    record={'mode':'sequential_consumption_not_realtime','source':str(BAG),'seconds':a.seconds,'command':command,'parameters_sha256':digest(paramfile),'frontend_sha256':digest(binary),'build_manifest_sha256':digest(manifest),'candidate_sha256':digest(a.candidate) if a.candidate else None,'production_changed':False,'robot_connected':False}
    shutil.copyfile(manifest,run/'build_manifest.json')
    (run/'identity.json').write_text(json.dumps(record,indent=2)+'\n')
    began=time.monotonic()
    with (run/'frontend.log').open('x') as out:code=subprocess.run(command,cwd=W,env=env,stdout=out,stderr=subprocess.STDOUT).returncode
    record['exit_code']=code;record['elapsed_wall_seconds']=time.monotonic()-began
    if (run/'reconstruction/joint_initialization_receipt.json').exists():record['joint_initialization']=json.loads((run/'reconstruction/joint_initialization_receipt.json').read_text())
    if (run/'reconstruction/consumption_summary.json').exists():record['consumption']=json.loads((run/'reconstruction/consumption_summary.json').read_text())
    # Exact integer headers AND multiplicities; never cancel opposite gaps.
    expected={};observed={}
    if (run/'reconstruction/consumed_raw.csv').exists():
        for row in csv.DictReader((run/'reconstruction/consumed_raw.csv').open()):expected.setdefault(row['topic'],Counter())[int(row['header_ns'])]+=1
    if (session/'frontend_input_stages.csv').exists():
        for row in csv.DictReader((session/'frontend_input_stages.csv').open()):
            if row['stage'] in ['imu_raw','cloud_raw']:observed.setdefault('/lidar_imu' if row['stage']=='imu_raw' else '/lidar_points',Counter())[int(row['stamp_ns'])]+=1
    record['delivery']={k:dict(source=sum(v.values()),callback=sum(observed.get(k,{}).values()),missing=sum((v-observed.get(k,Counter())).values()),unexpected=sum((observed.get(k,Counter())-v).values())) for k,v in expected.items()}
    record['callback_complete']=bool(expected) and all(v['missing']==v['unexpected']==0 for v in record['delivery'].values())
    record['full_original_counts']=sum(expected.get('/lidar_imu',{}).values())==166023 and sum(expected.get('/lidar_points',{}).values())==8188
    (run/'result.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))
    return 0 if code==0 and record['callback_complete'] and (a.seconds>0 or record['full_original_counts']) else 2

if __name__=='__main__':raise SystemExit(main())
