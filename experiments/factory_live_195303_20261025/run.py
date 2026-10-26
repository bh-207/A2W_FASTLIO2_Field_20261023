#!/usr/bin/env python3
"""Fresh full-input run of the frozen frontend and actual conservative backend."""
import argparse
from collections import Counter,defaultdict
import csv,hashlib,json,os,shutil,subprocess,time,uuid,re
from pathlib import Path
import yaml

E=Path(__file__).resolve().parent;W=E.parents[1]
BAG=Path('/home/bh/A2W_FastLio2/bags/factory_live_20261024_195303_YOI7fO')
O=W/'captures/factory_live_195303_20261025_001'
REF=W/'captures/field_fixed_sessions/fixed_short_20261023_001/bundles/map_bundle'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def libraries(binary,path,env):
 text=subprocess.check_output(['bash',str(E/'run.sh'),'ldd',str(binary)],env=env,text=True)
 if 'not found' in text:raise ValueError('unresolved runtime library: '+text)
 libs={}
 for name in re.findall(r'=> (/\S+)',text):
  p=Path(name).resolve()
  if p.is_relative_to(O/'build')or p.is_relative_to(W/'.deps'):libs[str(p)]=sha(p)
 dump(path,dict(binary=str(binary),sha256=sha(binary),resolved_experiment_dependency_sha256=libs,ldd=text))
def main():
 p=argparse.ArgumentParser();p.add_argument('--id',default='B_003');p.add_argument('--backend-only',action='store_true');p.add_argument('--debug',action='store_true');a=p.parse_args()
 if not a.id.replace('_','').isalnum():raise ValueError('safe run ID required')
 manifest=O/'build/build_manifest.json';build=json.loads(manifest.read_text())
 binary=Path(build['binaries']['frontend']['path'])
 if Path(build['source_root'])!=E/'source' or sha(binary)!=build['binaries']['frontend']['sha256']:raise ValueError('build identity mismatch')
 meta=yaml.safe_load((BAG/'metadata.yaml').read_text())['rosbag2_bagfile_information']
 expected={r['topic_metadata']['name']:int(r['message_count']) for r in meta['topics_with_message_count']}
 run=O/'runs'/a.id
 if not a.backend_only:
  if shutil.disk_usage(O).free<25*2**30:raise ValueError('at least25GiB free for fresh full reconstruction')
  run.mkdir(parents=True,exist_ok=False);session=run/'session';session.mkdir()
  original={str(BAG/n):dict(size=(BAG/n).stat().st_size,mtime_ns=(BAG/n).stat().st_mtime_ns,ino=(BAG/n).stat().st_ino) for n in meta['relative_file_paths']+['metadata.yaml']}
  dump(run/'original_files_before.json',original)
  params=yaml.safe_load((E/'config/baseline.yaml').read_text().replace('@SESSION@',str(session)))
  values=params['/laser_mapping']['ros__parameters']
  values['offline']=dict(consume_bag=str(BAG),output=str(run/'reconstruction'),maximum_input_seconds=0.0,preprocess_export_seconds=0.0,expected_imu_count=expected['/lidar_imu'],expected_cloud_count=expected['/lidar_points'])
  values['frontend_init_v4']['joint_initialization_enabled']=False
  assert values['mapping']['extrinsic_est_en'] is False and values['mapping']['extrinsic_T']==[-.00862,.00914,.03916]
  parameter=run/'parameters.yaml';parameter.write_text(yaml.safe_dump(params,sort_keys=False))
  dds=run/'localhost.xml';dds.write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="any"><General><AllowMulticast>false</AllowMulticast><Transport>tcp</Transport></General><TCP><Port>39821</Port><NoDelay>true</NoDelay></TCP><Discovery><Peers><Peer Address="127.0.0.1:39821"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
  env=dict(os.environ,A2W_REPAIR_OUTPUT=str(O),A2W_FIELD_MODE='offline-test',A2W_CAPTURE_ROOT=str(O),A2W_HANDOFF_DIRECTORY=str(session),A2W_REQUIRED_OBSERVER_NAME='/field_capture_observer_'+uuid.uuid4().hex,ROS_DOMAIN_ID='87',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file://'+str(dds),OPENBLAS_NUM_THREADS='1')
  env.update(ROS_LOG_DIR=str(run/'ros_logs'),ROS_HOME=str(run/'ros_home'))
  command=['/usr/bin/time','-v','-o',str(run/'frontend_resources.txt'),'bash',str(E/'run.sh'),str(binary),'--ros-args','--params-file',str(parameter)]
  if a.debug:command=command[:6]+['gdb','--batch','-ex','run','-ex','thread apply all bt','--args']+command[6:]
  libraries(binary,run/'frontend_library_identity.json',env)
  identity=dict(source=str(BAG),source_metadata_sha256=sha(BAG/'metadata.yaml'),mode='full_sequential_consumption_not_realtime',expected=expected,command=command,frontend_sha256=sha(binary),joint_initialization_enabled=False,physical_calibration_verified=False)
  shutil.copyfile(manifest,run/'build_manifest.json');dump(run/'identity.json',identity)
  began=time.monotonic()
  with (run/'frontend.log').open('x') as f:code=subprocess.run(command,cwd=W,env=env,stdout=f,stderr=subprocess.STDOUT).returncode
  identity.update(exit_code=code,elapsed_wall_seconds=time.monotonic()-began)
  consumed=defaultdict(Counter);observed=defaultdict(Counter)
  if (run/'reconstruction/consumed_raw.csv').exists():
   for r in csv.DictReader((run/'reconstruction/consumed_raw.csv').open()):consumed[r['topic']][int(r['header_ns'])]+=1
  if (session/'frontend_input_stages.csv').exists():
   for r in csv.DictReader((session/'frontend_input_stages.csv').open()):
    if r['stage'] in ['imu_raw','cloud_raw']:observed['/lidar_imu' if r['stage']=='imu_raw' else '/lidar_points'][int(r['stamp_ns'])]+=1
  delivery={k:dict(expected=expected[k],read=sum(consumed[k].values()),callback=sum(observed[k].values()),missing=sum((consumed[k]-observed[k]).values()),unexpected=sum((observed[k]-consumed[k]).values())) for k in ['/lidar_imu','/lidar_points']}
  identity['delivery']=delivery
  identity['full_input_complete']=all(x['read']==x['callback']==x['expected'] and x['missing']==x['unexpected']==0 for x in delivery.values())
  identity['source_unchanged']=all(Path(n).stat().st_size==s['size'] and Path(n).stat().st_mtime_ns==s['mtime_ns'] and Path(n).stat().st_ino==s['ino'] for n,s in original.items())
  dump(run/'result.json',identity);print(json.dumps(identity,indent=2),flush=True)
  if code!=0 or not identity['full_input_complete'] or not identity['source_unchanged']:raise RuntimeError('frontend or complete-input gate failed; evidence preserved')
 else:
  result=json.loads((run/'result.json').read_text())
  if result['exit_code']!=0 or not result['full_input_complete']:raise ValueError('backend requires successful complete frontend')
 env=dict(os.environ,A2W_REPAIR_OUTPUT=str(O),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='3')
 backend=O/'build/backend_install/a2w_fastlio_mapping/lib/a2w_fastlio_mapping/field_consumption_backend'
 libraries(backend,run/'backend_library_identity.json',env)
 dest=O/'maps'/a.id;dest.parent.mkdir(exist_ok=True)
 with (run/'backend.log').open('x') as f:
  subprocess.run(['/usr/bin/time','-v','-o',str(run/'backend_resources.txt'),'bash',str(E/'run.sh'),str(backend),str(run/'reconstruction/post_index.csv'),str(REF),str(dest),'--source-manifest',str(run/'build_manifest.json')],env=env,cwd=W,stdout=f,stderr=subprocess.STDOUT,check=True)
 subprocess.run(['python3',str(E/'export_raw_map.py'),str(dest)],env=env,cwd=W,check=True)
 print('COMPLETE',dest,flush=True)
if __name__=='__main__':main()
