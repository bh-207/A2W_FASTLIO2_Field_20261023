#!/usr/bin/env python3
"""Plan by default; root-coordinated synthetic raw->bag->FAST-LIO->Bundle smoke.

No ROS import/init/process unless --run --allow-localhost-ros and an explicit
matching localhost-only nonzero domain. Never connects a driver/robot or motion.
"""
from __future__ import annotations
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from frontend_init_v4_runtime_smoke import load_fixture
from mapping_capture_lib import (read_json,write_json,sha256,parameters,process_identity,
    process_matches,committed_raw_evidence,runtime_shutdown,verify_bundle,TOPICS)

EXTERNAL_DDS=['ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','CYCLONEDDS_URI',
    'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML']

def validate_environment(env,domain,authorized):
    if not authorized or not 1<=domain<=232:
        raise ValueError('explicit --run --allow-localhost-ros and domain1..232 required')
    if env.get('ROS_LOCALHOST_ONLY')!='1' or env.get('ROS_DOMAIN_ID')!=str(domain):
        raise ValueError('caller must explicitly set ROS_LOCALHOST_ONLY=1 and matching ROS_DOMAIN_ID')
    for name in EXTERNAL_DDS:
        if env.get(name):raise ValueError('external DDS/discovery profile forbidden: '+name)

def make_plan(repo,output,build,fixture,domain):
    repo,output,build,fixture=map(Path,[repo,output,build,fixture])
    return dict(schema='controlled_capture_smoke_plan_v1',starts_ros=False,source_provenance='synthetic',
        robot_connected=False,motion_commands=False,gravity_alignment=False,domain=domain,
        fixture=str(fixture),build_root=str(build),output=str(output),raw_record_cache_mib=1,
        prerequisites=['Root completes controlled backend build and sources backend_install/setup.bash',
            'Explicit localhost-only matching nonzero domain and no external DDS profiles',
            'Frozen synthetic30scan fixture, NumPy for float64 point-time shifts'],
        init_command=['/usr/bin/python3',str(repo/'scripts/mapping_capture.py'),'init','synthetic_smoke',
            '--sessions-root',str(output/'sessions'),'--build-root',str(build),'--synthetic-test',
            '--base-config',str(output/'setup/synthetic_jt128.yaml'),'--interface',str(output/'setup/interface.json')],
        phases=['init synthetic-only profile with fixture extrinsic/evidence','publish one transient-local synthetic tf_static before bag',
            'record exact6 topics and observe actual committed SQLite raw rows','independent static prelude>=2s then start3 actual nodes',
            'post-start static>=1.2s and exact actual runtime INIT ready','only now choose one frozen delta_ns and stream unchanged-value30scan/IMU',
            'save nonempty MapBundle with manifest and backend processed>=1','normal stop nodes/recorder, publisher control-file normal shutdown','verify closed bag6topics/runtime/Bundle; retain negatives'])

def shift_imu(values,delta_ns):
    return dict(stamp_ns=round(values[0]*1e9)+delta_ns,accel=list(values[1:4]),gyro=list(values[4:7]))

def shift_scan(scan,delta_ns):
    import numpy as np
    begin,end,packed=scan
    if len(packed)%32:raise ValueError('frozen PointCloud2 must be32byte points')
    shifted=bytearray(packed)
    times=np.ndarray((len(shifted)//32,),dtype='<f8',buffer=shifted,offset=24,strides=(32,))
    times += delta_ns/1e9
    return begin+delta_ns,end+delta_ns,bytes(shifted)

def choose_frozen_anchor(prelude_begin_ns,now_ns):
    period=100_000_000
    return prelude_begin_ns+((now_ns+300_000_000-prelude_begin_ns+period-1)//period)*period

def matrix_quaternion(values):
    # Fixture configuration only, no fit/projection/world alignment.
    r=[values[i:i+3] for i in [0,3,6]];trace=sum(r[i][i] for i in range(3))
    if trace>0:
        s=2*math.sqrt(trace+1);q=[(r[2][1]-r[1][2])/s,(r[0][2]-r[2][0])/s,(r[1][0]-r[0][1])/s,s/4]
    else:
        i=max(range(3),key=lambda k:r[k][k]);j,k=(i+1)%3,(i+2)%3
        s=2*math.sqrt(1+r[i][i]-r[j][j]-r[k][k]);q=[0.,0.,0.,0.]
        q[i]=s/4;q[j]=(r[j][i]+r[i][j])/s;q[k]=(r[k][i]+r[i][k])/s;q[3]=(r[k][j]-r[j][k])/s
    if not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1)>1e-6:raise ValueError('fixture rotation is not a proper unit quaternion')
    return q

def parse_cli_json(text):
    result=None;decoder=json.JSONDecoder()
    for match in re.finditer(r'(?m)^\{',text):
        try:value,_=decoder.raw_decode(text[match.start():])
        except ValueError:continue
        if isinstance(value,dict):result=value
    if result is None:raise ValueError('CLI produced no complete top-level JSON object')
    return result

def recorded_synthetic_tf(verified):
    """Inspect recorded evidence only; current verifier uses last_transforms."""
    header=verified.get('headers',{}).get('/tf_static',{})
    transforms=header.get('last_transforms',header.get('transforms',[]))
    return any(t.get('parent')=='body' and t.get('child')=='synthetic_sensor' for t in transforms)

def owned_sigint(child,identity):
    if child.poll() is not None:return
    if not identity or not process_matches(identity):raise ValueError('owned child identity changed; not signalling')
    descriptor=os.pidfd_open(child.pid)
    try:
        if not process_matches(identity):raise ValueError('owned child changed after pidfd open')
        signal.pidfd_send_signal(descriptor,signal.SIGINT)
    finally:os.close(descriptor)

def publisher(config_path,domain,authorized):
    validate_environment(os.environ,domain,authorized)
    config=read_json(config_path);output=Path(config['output']).resolve();fixture=Path(config['fixture']).resolve()
    if config['source_provenance']!='synthetic':raise ValueError('publisher can only use synthetic fixture')
    os.environ['ROS_LOG_DIR']=str(output/'ros_logs/publisher');os.environ['ROS_HOME']=str(output/'ros_home/publisher')
    Path(os.environ['ROS_LOG_DIR']).mkdir(parents=True,exist_ok=True);Path(os.environ['ROS_HOME']).mkdir(parents=True,exist_ok=True)
    meta,imu,scans,extrinsic=load_fixture(fixture)
    if round(imu[0][0]*1e9)!=scans[0][0]:raise ValueError('this smoke requires common IMU/point/header frozen epoch')
    import array
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy,qos_profile_sensor_data
    from sensor_msgs.msg import Imu,PointCloud2,PointField
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from a2w_fastlio_msgs.msg import RegistrationStatus
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('controlled_capture_synthetic_'+str(os.getpid()))
    imu_pub=node.create_publisher(Imu,'/lidar_imu',qos_profile_sensor_data)
    cloud_pub=node.create_publisher(PointCloud2,'/lidar_points',qos_profile_sensor_data)
    static_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tf_pub=node.create_publisher(TFMessage,'/tf_static',static_qos)
    state=dict(schema='controlled_capture_smoke_publisher_v1',source_provenance='synthetic',robot_connected=False,
        gravity_alignment=False,fixture_manifest_sha256=sha256(fixture/'manifest.json'),
        prelude_scope='independently synthesized static repeats of first cloud/first IMU; NOT original frozen samples',
        frozen_scope='one delta_ns applies to original IMU/header integer ns and ALL float64 point times; values unchanged',
        point_time_precision='float64 absolute seconds at current wall epoch, rounding retained; not exact integer point timestamps',
        prelude_imu=0,prelude_scans=0,frozen_imu=0,frozen_scans=0,backend_processed_max=0,
        frozen_started=False,frozen_done=False,normal_control_stop=False)
    def backend_status(message):
        if message.stage=='backend':
            found=re.search(r'\bprocessed=(\d+)\b',message.reason)
            if found:state['backend_processed_max']=max(state['backend_processed_max'],int(found.group(1)))
            state['last_backend_reason']=message.reason
    subscription=node.create_subscription(RegistrationStatus,'/mapping/registration_status',backend_status,1000)
    stopped=False
    def interrupt(signum,frame):
        nonlocal stopped
        stopped=True;state['signal_received']=signum
    previous={s:signal.signal(s,interrupt) for s in [signal.SIGINT,signal.SIGTERM]}
    fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in [
        ('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),('z',8,PointField.FLOAT32),('intensity',12,PointField.FLOAT32),
        ('ring',16,PointField.UINT16),('timestamp',24,PointField.FLOAT64)]]
    def send_imu(row):
        ns=row['stamp_ns'];msg=Imu();msg.header.frame_id='imu';msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
        msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=row['accel']
        msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=row['gyro'];imu_pub.publish(msg)
    def send_scan(scan):
        begin,end,packed=scan;msg=PointCloud2();msg.header.frame_id='synthetic_sensor'
        msg.header.stamp.sec=begin//10**9;msg.header.stamp.nanosec=begin%10**9
        msg.height=1;msg.width=len(packed)//32;msg.fields=fields;msg.is_bigendian=False;msg.point_step=32
        msg.row_step=len(packed);msg.is_dense=True;msg.data=array.array('B',packed);cloud_pub.publish(msg)
    status_path=output/'publisher_status.json'
    try:
        transform=TransformStamped();now=time.time_ns();transform.header.stamp.sec=now//10**9;transform.header.stamp.nanosec=now%10**9
        transform.header.frame_id='body';transform.child_frame_id='synthetic_sensor'
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=extrinsic[9:]
        q=matrix_quaternion(extrinsic[:9]);transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=q
        tf_pub.publish(TFMessage(transforms=[transform]));state.update(tf_static_published_unix_ns=now,tf_static_publish_count=1,
            tf_static_scope='synthetic known fixture T_I_L body->synthetic_sensor; NOT real hardware calibration')
        write_json(status_path,state)
        deadline=time.monotonic()+120
        while not stopped and not (imu_pub.get_subscription_count() and cloud_pub.get_subscription_count()):
            if time.monotonic()>deadline:raise ValueError('no raw subscriptions within120s')
            if (output/'publisher.stop.json').exists():stopped=True;state['normal_control_stop']=True;break
            rclpy.spin_once(node,timeout_sec=.02)
        prelude_begin=time.time_ns();state['prelude_started_unix_ns']=prelude_begin
        imu_index=0;scan_index=0;frozen_anchor=None;delta=None;frozen_imu_index=0;frozen_scan_index=0;last_report=0.
        while not stopped:
            rclpy.spin_once(node,timeout_sec=0.)
            if (output/'publisher.stop.json').exists():state['normal_control_stop']=True;break
            if frozen_anchor is None and (output/'publisher.freeze.json').exists():
                request=read_json(output/'publisher.freeze.json')
                if not request.get('actual_runtime_ready'):raise ValueError('frozen release requires parent actual runtime ready evidence')
                frozen_anchor=choose_frozen_anchor(prelude_begin,time.time_ns());delta=frozen_anchor-scans[0][0]
                state.update(delta_ns=delta,frozen_anchor_ns=frozen_anchor,freeze_request=request,
                    minimum_frozen_start_unix_ns=frozen_anchor)
            now=time.time_ns()
            # Prefix is generated independently on a continuous5ms/100ms source grid.
            next_imu=prelude_begin+imu_index*5_000_000
            while next_imu<=now and (frozen_anchor is None or next_imu<frozen_anchor):
                send_imu(dict(stamp_ns=next_imu,accel=imu[0][1:4],gyro=imu[0][4:7]));imu_index+=1;state['prelude_imu']+=1
                next_imu=prelude_begin+imu_index*5_000_000
            next_cloud_begin=prelude_begin+scan_index*100_000_000
            while next_cloud_begin+100_000_000<=now and (frozen_anchor is None or next_cloud_begin<frozen_anchor):
                send_scan(shift_scan(scans[0],next_cloud_begin-scans[0][0]));scan_index+=1;state['prelude_scans']+=1
                next_cloud_begin=prelude_begin+scan_index*100_000_000
            if frozen_anchor is not None and now>=frozen_anchor:
                if not state['frozen_started']:
                    state['frozen_started']=True;state['frozen_started_actual_unix_ns']=now
                while frozen_imu_index<len(imu) and round(imu[frozen_imu_index][0]*1e9)+delta<=now:
                    send_imu(shift_imu(imu[frozen_imu_index],delta));frozen_imu_index+=1;state['frozen_imu']+=1
                while frozen_scan_index<len(scans) and scans[frozen_scan_index][1]+delta<=now:
                    send_scan(shift_scan(scans[frozen_scan_index],delta));frozen_scan_index+=1;state['frozen_scans']+=1
                if frozen_imu_index==len(imu) and frozen_scan_index==len(scans) and not state['frozen_done']:
                    state['frozen_done']=True;state['frozen_done_unix_ns']=time.time_ns()
            if time.monotonic()-last_report>.1:
                state['updated_unix_ns']=time.time_ns();write_json(status_path,state);last_report=time.monotonic()
            time.sleep(.001)
    except BaseException as error:
        state.update(error=str(error),failed=True);raise
    finally:
        state['closed']=True;state['closed_unix_ns']=time.time_ns();write_json(status_path,state)
        node.destroy_node();rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)
    return 0 if state['normal_control_stop'] else 2

class Runner:
    def __init__(self,args):
        self.args=args;self.output=args.output.resolve();self.session=None;self.children={};self.ledger=[];self.sequence=0
        self.env=os.environ.copy();self.env['ROS_LOG_DIR']=str(self.output/'ros_logs/root');self.env['ROS_HOME']=str(self.output/'ros_home/root')
    def persist(self):write_json(self.output/'owned_processes.json',dict(processes=self.ledger))
    def spawn(self,role,command):
        self.sequence+=1;log=self.output/'logs'/f'{self.sequence:02d}_{role}.log'
        with log.open('x') as stream:
            child=subprocess.Popen([str(x) for x in command],stdout=stream,stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,env=self.env,start_new_session=True)
        try:identity=process_identity(child.pid)
        except OSError:identity=None
        item=dict(role=role,command=[str(x) for x in command],log=str(log),identity=identity,started_unix_ns=time.time_ns())
        self.ledger.append(item);self.children[role]=(child,identity,item);self.persist();return child,item
    def cli(self,role,*arguments,timeout=60,allow_failure=False):
        child,item=self.spawn(role,['/usr/bin/python3',ROOT/'scripts/mapping_capture.py',*arguments])
        end=time.monotonic()+timeout
        while child.poll() is None and time.monotonic()<end:time.sleep(.05)
        if child.poll() is None:
            owned_sigint(child,item['identity'])
            try:child.wait(timeout=20)
            except subprocess.TimeoutExpired:raise RuntimeError(role+' timeout; own CLI still alive, no SIGKILL issued')
        item.update(exit_code=child.returncode,finished_unix_ns=time.time_ns());self.persist()
        if child.returncode and not allow_failure:raise RuntimeError(role+' CLI failed: '+item['log'])
        text=Path(item['log']).read_text(errors='replace')
        try:result=parse_cli_json(text)
        except ValueError:
            if allow_failure:return dict(exit_code=child.returncode,log=item['log'])
            raise
        result['_CLI_exit_code']=child.returncode;return result
    def wait_publisher(self,predicate,timeout,reason):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            child,_,_=self.children['publisher']
            if child.poll() is not None:raise RuntimeError('publisher exited early; '+reason)
            path=self.output/'publisher_status.json'
            if path.exists():
                state=read_json(path)
                if state.get('failed'):raise RuntimeError('publisher failed: '+str(state))
                if predicate(state):return state
            time.sleep(.05)
        raise RuntimeError(reason+' timeout')
    def stop_publisher(self):
        if 'publisher' not in self.children:return
        child,identity,item=self.children['publisher']
        if child.poll() is None:
            if not (self.output/'publisher.stop.json').exists():write_json(self.output/'publisher.stop.json',dict(requested_unix_ns=time.time_ns(),mode='normal_control_file'),exclusive=True)
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                owned_sigint(child,identity)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:item['still_alive_no_SIGKILL']=True
        item['exit_code']=child.poll();self.persist()
    def cleanup(self):
        errors=[]
        if self.session and (self.session/'processes.json').exists():
            try:
                args=['stop',self.session,'--timeout','60']
                if not (self.session/'save_success.json').exists():args+=['--discard-unsaved']
                self.cli('cleanup_stop',*args,timeout=260,allow_failure=True)
            except BaseException as error:errors.append(str(error))
        self.stop_publisher()
        if 'record_manager' in self.children:
            child,identity,item=self.children['record_manager']
            if child.poll() is None:
                try:child.wait(timeout=70)
                except subprocess.TimeoutExpired:
                    try:owned_sigint(child,identity);child.wait(timeout=60)
                    except BaseException as error:errors.append('record manager: '+str(error))
            item['exit_code']=child.poll();self.persist()
        return errors

def run(args):
    validate_environment(os.environ,args.domain,args.allow_localhost_ros)
    output=args.output.resolve();fixture=args.fixture.resolve();build=args.build_root.resolve()
    if output.exists() or any(output==p or output in p.parents or p in output.parents for p in [fixture,build]):
        raise ValueError('new output separate from input/build required')
    # No ROS: validate immutable inputs before spawning anything.
    fixture_meta,imu,scans,extrinsic=load_fixture(fixture)
    if not (build/'build_identity.json').is_file():raise ValueError('controlled build not complete: build_identity.json missing')
    output.mkdir(parents=True)
    for name in ['setup','logs','ros_logs/root','ros_home/root']:(output/name).mkdir(parents=True)
    runner=Runner(args);result=dict(schema='controlled_capture_smoke_result_v1',source_provenance='synthetic',passed=False,
        robot_connected=False,motion_commands=False,gravity_alignment=False,domain=args.domain,
        input_fixture_manifest_sha256=sha256(fixture/'manifest.json'),input_fixture_manifest=fixture_meta,
        script_sha256=sha256(__file__),controlled_build_identity_sha256=sha256(build/'build_identity.json'),
        phases={},scope='real localhost ROS nodes/rosbag/MapBundle with synthetic data, not physical accuracy/loop/hardware acceptance')
    try:
        import yaml
        base=copy.deepcopy(parameters(ROOT/'src/a2w_fastlio2_bringup/config/jt128.yaml'))
        base.setdefault('common',{}).update(lid_topic='/lidar_points',imu_topic='/lidar_imu',imu_gyr_unit='rad',time_sync_en=False,time_offset_lidar_to_imu=0.)
        base.setdefault('mapping',{}).update(extrinsic_R=extrinsic[:9],extrinsic_T=extrinsic[9:],extrinsic_est_en=False)
        base.setdefault('preprocess',{}).update(lidar_type=2,scan_line=128,timestamp_unit=0,blind=.5)
        base['point_filter_num']=6;base['runtime_pos_log_enable']=False
        base.setdefault('pcd_save',{})['pcd_save_en']=False;base.setdefault('trajectory_save',{})['tum_en']=False
        base_path=output/'setup/synthetic_jt128.yaml'
        with base_path.open('x') as stream:yaml.safe_dump({'/**':{'ros__parameters':base}},stream)
        evidence=dict(source=str(fixture/'manifest.json'),version=fixture_meta['schema'],config='frozen synthetic SI/time/frame generator; NOT real driver',sha256=sha256(fixture/'manifest.json'))
        contract=dict(schema='mapping_input_interface_v1',source_provenance='synthetic',gyro_unit='rad/s',accel_unit='m/s^2',
            imu_frame='imu',lidar_frame='synthetic_sensor',imu_minus_lidar_offset_ns=0,
            offset_definition='t_imu_raw_minus_t_lidar_same_event',
            gyro_unit_evidence=evidence,accel_unit_evidence=evidence,time_offset_evidence=evidence,frame_evidence=evidence,
            extrinsic_evidence={**evidence,'base_config_sha256':sha256(base_path)})
        interface=output/'setup/interface.json';write_json(interface,contract,exclusive=True)
        initialized=runner.cli('init','init','synthetic_smoke','--sessions-root',output/'sessions','--build-root',build,
            '--synthetic-test','--base-config',base_path,'--interface',interface,timeout=120)
        runner.session=Path(initialized['session']).resolve();result['session']=str(runner.session);result['phases']['init']=initialized
        config_path=output/'setup/publisher.json';write_json(config_path,dict(output=str(output),fixture=str(fixture),source_provenance='synthetic'),exclusive=True)
        runner.spawn('publisher',['/usr/bin/python3',Path(__file__).resolve(),'--run','--allow-localhost-ros','--domain',args.domain,'--publisher-config',config_path])
        tf=runner.wait_publisher(lambda state:state.get('tf_static_publish_count')==1,30,'pre-bag static transform')
        result['tf_static_pre_bag_publish_unix_ns']=tf['tf_static_published_unix_ns']
        runner.spawn('record_manager',['/usr/bin/python3',ROOT/'scripts/mapping_capture.py','record',runner.session,'--cache-size-mib','1'])
        end=time.monotonic()+60
        while time.monotonic()<end:
            record,_,_=runner.children['record_manager']
            if record.poll() is not None:raise RuntimeError('record manager exited before real raw SQL commits')
            committed=committed_raw_evidence(runner.session/'bag')
            state_path=output/'publisher_status.json';state=read_json(state_path) if state_path.exists() else {}
            report_path=runner.session/'raw_input_report.json';report=read_json(report_path) if report_path.exists() else {}
            observed=all(report.get('recorder_subscriptions',{}).get(t) and report.get('topics',{}).get(t,{}).get('count',0)>=2 for t in ['/lidar_points','/lidar_imu'])
            prelude_old_enough=time.time_ns()-state.get('prelude_started_unix_ns',time.time_ns())>=2_000_000_000
            if committed['valid'] and observed and prelude_old_enough:break
            time.sleep(.1)
        else:raise RuntimeError('actual two-raw-topic SQLite commits/prelude/subscriptions not ready')
        write_json(output/'pre_start_actual_sql_evidence.json',committed,exclusive=True);result['phases']['pre_start_actual_sql']=committed
        started=runner.cli('start','start',runner.session,timeout=120);result['phases']['start']=started
        # Keep static even if INIT is already committed; never leak moving data before ready.
        time.sleep(1.25)
        ready=runner.cli('ready','ready',runner.session,'--timeout','60',timeout=100)
        if not ready.get('ready'):raise RuntimeError('actual runtime INIT ready did not pass')
        result['phases']['ready']=ready
        write_json(output/'publisher.freeze.json',dict(actual_runtime_ready=True,ready_observed_unix_ns=time.time_ns(),
            ready_state_stamp_ns=ready.get('state_stamp_ns'),session=str(runner.session)),exclusive=True)
        finished=runner.wait_publisher(lambda state:state.get('frozen_done'),40,'complete frozen30scan sequence')
        if finished['frozen_scans']!=30 or finished['frozen_imu']!=len(imu):raise RuntimeError('frozen stream incomplete')
        result['phases']['frozen_publisher']=finished
        saved=runner.cli('save','save',runner.session,'--timeout','120',timeout=150)
        result['phases']['save']=saved
        if not saved.get('success') or saved.get('capture_degraded'):raise RuntimeError('MapBundle save failed or raw capture degraded')
        stopped=runner.cli('stop','stop',runner.session,'--timeout','60',timeout=260);result['phases']['stop']=stopped
        record,_,record_item=runner.children['record_manager'];record.wait(timeout=70);record_item['exit_code']=record.returncode;runner.persist()
        if record.returncode!=0:raise RuntimeError('record manager did not flush/close normally')
        runner.stop_publisher()
        verified=runner.cli('verify','verify',runner.session,timeout=120);result['phases']['verify']=verified
        with (runner.session/'runtime_state.csv').open() as stream:
            import csv
            runtime=list(csv.DictReader(stream))
        post=[row for row in runtime if row['stage']=='POST_LIDAR_UPDATE']
        effective=sum(int(row['lidar_effective_features'])>0 for row in post)
        status=runtime_shutdown(runner.session/'runtime_state.csv',runner.session/'logs/frontend.log')
        bundle=verify_bundle(runner.session/'bundles/map_bundle')
        pub=read_json(output/'publisher_status.json')
        processed=max(pub.get('backend_processed_max',0),int(re.search(r'\bprocessed=(\d+)\b',verified.get('last_backend_status',{}).get('reason','')).group(1)) if re.search(r'\bprocessed=(\d+)\b',verified.get('last_backend_status',{}).get('reason','')) else 0)
        if not post or not effective or not status['valid'] or processed<1 or bundle['keyframe_count']<1:
            raise RuntimeError('actual POST/features/runtime/backend processed/Bundle proof incomplete')
        counts=verified['topic_counts']
        if any(counts.get(topic,0)<1 for topic in TOPICS):raise RuntimeError('one of exact6 requested bag topics is empty')
        if not recorded_synthetic_tf(verified):raise RuntimeError('pre-bag transient-local synthetic tf_static not actually recorded')
        if not pub.get('normal_control_stop') or not pub.get('closed'):raise RuntimeError('publisher normal control-file shutdown missing')
        if sha256(fixture/'manifest.json')!=result['input_fixture_manifest_sha256']:raise RuntimeError('original fixture manifest changed')
        result.update(passed=True,post_lidar_update_rows=len(post),post_with_effective_features=effective,backend_processed_max=processed,
            bundle=bundle,runtime=status,publisher=pub,bag_topic_counts=counts,
            note='Some POST invocations may have zero effective features; no loop/accuracy/real-device claim.')
    except BaseException as error:
        result.update(passed=False,error=type(error).__name__+': '+str(error))
    finally:
        result['cleanup_errors']=runner.cleanup()
        if result['cleanup_errors']:result['passed']=False
        result['finished_unix_ns']=time.time_ns();write_json(output/'result.json',result)
    return result

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true');parser.add_argument('--allow-localhost-ros',action='store_true')
    parser.add_argument('--domain',type=int,default=218)
    parser.add_argument('--build-root',type=Path,default=Path(os.environ.get('A2W_CONTROLLED_BUILD_ROOT',ROOT/'.controlled_mapping')))
    parser.add_argument('--fixture',type=Path,default=ROOT/'artifacts/loop_v4_20261021/frontend/frozen_raw_prefix_v2')
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/mapping_capture_20261021/full_capture_smoke')
    parser.add_argument('--publisher-config',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args(argv)
    if not args.run:
        print(json.dumps(make_plan(ROOT,args.output,args.build_root,args.fixture,args.domain),indent=2));return 0
    if args.publisher_config:return publisher(args.publisher_config,args.domain,args.allow_localhost_ros)
    result=run(args);print(json.dumps(result,indent=2));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
