#!/usr/bin/env python3
"""Separate operator-authorized field profile. No driver, motion or production writes."""
import argparse
import copy
import csv
from collections import Counter
import json
import io
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
sys.dont_write_bytecode=True

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
sys.path.insert(0, str(WS/'scripts'))
import mapping_capture as core
import mapping_capture_lib as lib
import mapping_initial_conditions as initials

OUT = WS/'captures/field_fixed_20261023_prepare_001'
BUILD = OUT/'build_003'
INTERFACE_SOURCE = WS/'captures/v4_static_cache128_20261021_205825/interface.json'
PROFILE = HERE/'fixed_extrinsic.yaml'
CACHE_MIB = 128
EXPECTED = dict(extrinsic_R=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                extrinsic_T=[-.00862,.00914,.03916], offset_s=0., extrinsic_est_en=False)
OLD_FRONTEND_PARAMETERS = core.frontend_parameters
OLD_ENV = core.session_env


def policy_errors(policy, contract, params, base_hash):
    errors = []
    if not isinstance(policy, dict) or policy.get('enabled') is not True:
        return ['explicit user-specified fixed-extrinsic authorization required; default OFF']
    if policy.get('schema')!='field_user_fixed_extrinsic_v1': errors.append('wrong fixed policy schema')
    if policy.get('base_config_sha256')!=base_hash: errors.append('fixed policy base hash mismatch')
    if not policy.get('authorization_note'): errors.append('authorization note missing')
    if initials.initial_values(params)!=EXPECTED or policy.get('initial_conditions')!=EXPECTED:
        errors.append('only user-specified false / T[-.00862,.00914,.03916] / R=I / offset=0 is authorized')
    if params.get('common',{}).get('time_sync_en') is not False: errors.append('automatic clock correction forbidden')
    if policy.get('physical_calibration_verified') is not False: errors.append('not physical calibration proof')
    if policy.get('calibration_status')!=initials.STATUS or contract.get('calibration_status')!=initials.STATUS:
        errors.append('unverified label required')
    for key, expected in [('source_provenance','real'), ('gyro_unit','rad/s'), ('accel_unit','m/s^2'),
                          ('imu_frame','hesai_imu_aligned'), ('lidar_frame','hesai_lidar_aligned'),
                          ('imu_minus_lidar_offset_ns',0)]:
        if contract.get(key)!=expected: errors.append('input contract mismatch: '+key)
    for key in ['time_offset_evidence','extrinsic_evidence']:
        e = contract.get(key,{})
        if e.get('physical_calibration_verified') is not False or e.get('sha256')!=base_hash or e.get('evidence_scope')!='configured_value_only':
            errors.append('configuration-only evidence missing: '+key)
    return errors


def authorize(source, base, params, digest, note, authorized=False):
    if authorized is not True: raise ValueError('explicit --authorize-user-fixed-extrinsic required')
    c = copy.deepcopy(source)
    policy = dict(schema='field_user_fixed_extrinsic_v1', enabled=True, authorization_note=note,
                  base_config_sha256=digest, calibration_status=initials.STATUS,
                  physical_calibration_verified=False, source='user_specified_fixed_configuration',
                  initial_conditions=copy.deepcopy(EXPECTED))
    e = dict(source=str(Path(base).resolve()), config=str(Path(base).resolve()), sha256=digest,
             version='user-specified fixed configuration SHA256 '+digest, evidence_scope='configured_value_only',
             physical_calibration_verified=False, authorization_note=note,
             note='R/T direction follows p_I = R_I_L p_L + t_I_L. User values; physical accuracy unverified.')
    c.update(calibration_status=initials.STATUS, imu_minus_lidar_offset_ns=0,
             extrinsic_evidence=dict(e, base_config_sha256=digest), time_offset_evidence=dict(e))
    c['audit_observations'] = dict(c.get('audit_observations',{}),
                                 frontend_config_extrinsic_est_en=False, actual_capture_base_config=str(base),
                                 source='historical driver archive; current live frames/times checked per new session')
    errors = policy_errors(policy,c,params,digest)
    if errors: raise ValueError('; '.join(errors))
    return c, policy


def frontend_parameters(base, contract, session):
    p = OLD_FRONTEND_PARAMETERS(base, contract, session)
    p['field_capture'] = dict(imu_depth=100)
    # P / R / J are absent from this binary, not simply disabled in YAML.
    return p


def session_env(session):
    env = OLD_ENV(session)
    metadata = lib.read_json(Path(session)/'session.json')
    env.update(A2W_HANDOFF_DIRECTORY=str(Path(session).resolve()),
               A2W_FIELD_MODE=metadata.get('field_mode','live'))
    return env


def build_identity(root):
    root = Path(root).resolve()
    if root!=BUILD: raise ValueError('field profile requires selected field B build: '+str(BUILD))
    manifest = lib.read_json(root/'build_manifest.json')
    install = WS/'.controlled_mapping/backend_install'
    binaries = {role:core.binary_identity(e['path'],e['sha256'],role,install)
                for role,e in manifest['binaries'].items()}
    for source, e in manifest['sources'].items():
        if lib.sha256(source)!=e['sha256'] or lib.sha256(e['generated'])!=e['generated_sha256']:
            raise ValueError('build source changed: '+source)
    if lib.sha256(root/'source/startup_trace.hpp')!=manifest['trace_header_sha256']:
        raise ValueError('compiled trace header identity changed')
    for source,digest in manifest['unchanged_physics'].items():
        if lib.sha256(source)!=digest: raise ValueError('filter/IMU/preprocessing source changed: '+source)
    plugins={str(p.resolve()):lib.sha256(p.resolve()) for p in (install/'a2w_fastlio_msgs/lib').glob('liba2w_fastlio_msgs__*.so')}
    return dict(build_root=str(root),backend_install=str(install),binaries=binaries,runtime_plugins=plugins,
                manifests={str(root/'build_manifest.json'):lib.sha256(root/'build_manifest.json')})


def fixed_state_report(path,closed=False):
    rows = 0; init = 0; post = 0; errors = []; max_t = 0.; max_q = 0.
    partial=False
    try:
        contents=Path(path).read_text()
        if contents and not contents.endswith('\n'):
            partial=True
            if closed: errors.append('closed state CSV has incomplete final row')
            contents=contents[:contents.rfind('\n')+1]
        # A bounded writer may be in the middle of its last row. Read a stable,
        # newline-complete prefix; final closure separately requires all rows.
        with io.StringIO(contents) as stream:
            for row in csv.DictReader(stream):
                if row.get('stage') not in ['INIT_COMMIT','POST_LIDAR_UPDATE']: continue
                t = [float(row['t_I_L_'+k]) for k in 'xyz']
                q = [float(row['q_I_L_'+k]) for k in 'xyzw']
                if not all(math.isfinite(x) for x in t+q): raise ValueError('nonfinite fixed state')
                terr = max(abs(a-b) for a,b in zip(t,EXPECTED['extrinsic_T']))
                qerr = min(max(abs(a-b) for a,b in zip(q,sign)) for sign in [[0,0,0,1],[0,0,0,-1]])
                max_t=max(max_t,terr); max_q=max(max_q,qerr); rows+=1
                init += row['stage']=='INIT_COMMIT'; post += row['stage']=='POST_LIDAR_UPDATE'
                if max(terr,qerr)>1e-10: errors.append('fixed state changed at seq '+str(row['seq']))
    except (OSError,ValueError,TypeError,KeyError) as e: errors.append(str(e))
    if init!=1 or post<1: errors.append('one real INIT_COMMIT plus POST required')
    return dict(valid=not errors, rows=rows, init=init, post=post, max_translation_error_m=max_t,
                max_quaternion_component_error=max_q, tolerance=1e-10,partial_row_pending=partial,errors=errors[:20])


def committed_line(line):
    # The backend prints this audit only after OptimizationFinished / accepted.
    return bool(re.search(r'\[LoopCommitted\](?:\s|$)', line))


def backend_receiving(progress,now):
    reason=progress.get('reason',''); processed=re.search(r'(?:^|\s)processed=(\d+)(?:\s|$)',reason)
    return (0<=now-progress.get('steady_ns',0)<15_000_000_000 and
            bool(processed and int(processed.group(1))>0) and
            bool(re.search(r'(?:^|\s)state=receiving(?:\s|$)',reason)))


def matches_valid(records, observer, now):
    for role, topics, required in [('frontend',['/Odometry','/cloud_registered_body'],'mapping_ingress_node'),
                                  ('ingress',['/mapping/keyframe_odom','/mapping/keyframe_cloud'],'mapping_backend_node')]:
        r = records.get(role,{})
        if r.get('schema')!='publisher_matched_edges_v1' or r.get('ready') is not True: return False
        if not 0<=now-r.get('steady_ns',0)<2_000_000_000: return False
        edges = r.get('edges',[])
        if len(edges)!=2 or {e['topic'] for e in edges}!=set(topics): return False
        for e in edges:
            names={required,observer}|({'rosbag2_recorder'} if e['topic']=='/Odometry' else set())
            if e.get('matched_count')!=len(names) or len(e.get('subscribers',[]))!=len(names) or set(e['subscribers'])!=names: return False
    return True


def check_environment(session=None):
    mode = lib.read_json(Path(session)/'session.json').get('field_mode','live') if session else 'live'
    expected = ('87','1') if mode=='offline-test' else ('0','0')
    if (os.environ.get('ROS_DOMAIN_ID'),os.environ.get('ROS_LOCALHOST_ONLY'))!=expected:
        raise ValueError('wrong domain/localhost; use this session env.sh')
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp': raise ValueError('CycloneDDS required')
    if mode=='live' and '127.0.0.1' in os.environ.get('CYCLONEDDS_URI',''):
        raise ValueError('offline DDS profile forbidden in live field mode')
    if session and mode=='live':
        directory=Path(session)/'field_environment'
        expected_uri='file://'+str(directory/'cyclonedds.xml')
        if os.environ.get('CYCLONEDDS_URI')!=expected_uri: raise ValueError('source exact session.env before this command')
        if lib.sha256(directory/'cyclonedds.xml')!=lib.read_json(directory/'network.json')['dds_sha256']:
            raise ValueError('session DDS profile changed')


def configure():
    # Extension applies only in this independent process; production entry remains unchanged.
    core.authorize_existing_initials = authorize
    lib.experimental_policy_errors = policy_errors
    core.frontend_parameters = frontend_parameters
    core.capture_build_identity = build_identity
    core.session_env = session_env
    core.dump_parameters = dump_actual_parameters


def dump_actual_parameters(session):
    """Query exact parameter services directly; never reuse a CLI graph daemon."""
    import rclpy
    from rcl_interfaces.srv import ListParameters, GetParameters
    from rclpy.parameter import parameter_value_to_python
    session=Path(session)
    rclpy.init(); node=rclpy.create_node('field_parameter_audit_'+str(os.getpid()))
    result={}
    try:
        for role,name in core.NODE_NAMES.items():
            clients=[]
            try:
                listing=node.create_client(ListParameters,name+'/list_parameters'); clients.append(listing)
                values=node.create_client(GetParameters,name+'/get_parameters'); clients.append(values)
                if not listing.wait_for_service(timeout_sec=10) or not values.wait_for_service(timeout_sec=10):
                    raise ValueError('actual parameter services unavailable for '+name)
                request=ListParameters.Request(); request.depth=0
                future=listing.call_async(request); rclpy.spin_until_future_complete(node,future,timeout_sec=10)
                if not future.done(): raise ValueError('list parameter timeout: '+name)
                names=sorted(future.result().result.names)
                request2=GetParameters.Request(); request2.names=names
                future=values.call_async(request2); rclpy.spin_until_future_complete(node,future,timeout_sec=10)
                if not future.done(): raise ValueError('get parameter timeout: '+name)
                flat=dict(zip(names,[parameter_value_to_python(v) for v in future.result().values]))
                nested={}
                for key,value in flat.items():
                    cursor=nested; parts=key.split('.')
                    for part in parts[:-1]: cursor=cursor.setdefault(part,{})
                    cursor[parts[-1]]=value
                target=session/'snapshots'/('actual_'+role+'.yaml')
                with target.open('x') as stream: yaml.safe_dump({name:{'ros__parameters':nested}},stream,sort_keys=False)
                result[role]=dict(path=str(target),sha256=lib.sha256(target))
            except (ValueError,OSError) as e: result[role]=dict(error=str(e))
            finally:
                for client in clients: node.destroy_client(client)
        qos={}
        for topic in ['/lidar_imu','/lidar_points']:
            rows=[]
            for e in node.get_subscriptions_info_by_topic(topic):
                if e.node_name!='laser_mapping': continue
                q=e.qos_profile
                rows.append(dict(node=e.node_name,namespace=e.node_namespace,depth=q.depth,
                                 reliability=q.reliability.name,history=q.history.name,durability=q.durability.name))
            qos[topic]=rows
        lib.write_json(session/'actual_input_qos.json',qos)
    finally:
        node.destroy_node(); rclpy.shutdown()
    lib.write_json(session/'actual_parameters.json',result)
    return result


def observer(session):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from a2w_fastlio_msgs.msg import RegistrationStatus
    session = Path(session); metadata = lib.read_json(session/'session.json')
    rclpy.init(); node=rclpy.create_node('field_capture_observer_'+metadata['uuid'])
    counts={}; subs=[]; last={}; backend={}
    def message(topic, m):
        stamp=core.stamp_ns(m.header); counts[topic]=counts.get(topic,0)+1
        last[topic]=stamp
    for topic, type_ in [('/Odometry',Odometry),('/cloud_registered_body',PointCloud2),
                         ('/mapping/keyframe_odom',Odometry),('/mapping/keyframe_cloud',PointCloud2)]:
        subs.append(node.create_subscription(type_,topic,lambda m,t=topic:message(t,m),100))
    # Actual loop notifications come from the backend's post-optimization audit log.
    # A RegistrationStatus.accepted by itself is not a committed-loop proof.
    def status(message):
        if message.stage=='backend':
            backend.update(reason=message.reason,steady_ns=time.monotonic_ns())
    subs.append(node.create_subscription(RegistrationStatus,'/mapping/registration_status',status,100))
    def snapshot():
        lib.write_json(session/'observer_live.json',dict(node=node.get_name(),counts=counts,last_stamp_ns=last,backend=backend,steady_ns=time.monotonic_ns()))
    timer=node.create_timer(.5,snapshot); snapshot()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        snapshot(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def release(session, timeout=40):
    session=Path(session); deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        processes=lib.read_json(session/'processes.json')
        if not all(lib.process_matches(processes.get(r,{})) for r in ['frontend','ingress','backend','field_observer']):
            raise ValueError('owned startup process exited; no release')
        try:
            records={r:lib.read_json(session/(r+'_matched.json')) for r in ['frontend','ingress']}
            obs=lib.read_json(session/'observer_live.json')
            if matches_valid(records,obs['node'],time.monotonic_ns()):
                result=dict(verified=True,mode='live_sensor_intake_gate',
                            checked_steady_ns=time.monotonic_ns(),edges=records,observer=obs['node'])
                lib.write_json(session/'live_release.json',result,exclusive=True)
                return result
        except (OSError,ValueError,KeyError): pass
        time.sleep(.05)
    raise ValueError('actual endpoint handshake timeout; no release; inspect matched JSON/logs')


def mapped_identities(session):
    metadata=lib.read_json(Path(session)/'identity.json')['build']
    processes=lib.read_json(Path(session)/'processes.json'); result={}
    for role in core.NODE_NAMES:
        e=processes[role]
        if not lib.process_matches(e): raise ValueError('owned process changed: '+role)
        executable=(Path('/proc')/str(e['pid'])/'exe').resolve(strict=True)
        expected=metadata['binaries'][role]
        if executable!=Path(expected['path']).resolve() or lib.sha256(executable)!=expected['sha256']:
            raise ValueError('actual executable mismatch: '+role)
        allowed={str(Path(item['path']).resolve()) for item in expected['dsos']}
        project=[]
        for line in (Path('/proc')/str(e['pid'])/'maps').read_text().splitlines():
            parts=line.split(maxsplit=5)
            if len(parts)<6 or not parts[5].startswith('/'): continue
            path=Path(parts[5])
            if path.name.startswith('liba2w_'):
                if str(path.resolve()) not in allowed:
                    expected_plugin=metadata.get('runtime_plugins',{}).get(str(path.resolve()))
                    if not expected_plugin or lib.sha256(path)!=expected_plugin:
                        raise ValueError('actual unexpected project library: '+str(path))
                project.append(str(path.resolve()))
        result[role]=dict(pid=e['pid'],executable=str(executable),sha256=expected['sha256'],project_libraries=sorted(set(project)))
    lib.write_json(Path(session)/'runtime_libraries.json',result)
    return result


def start(session):
    check_environment(session)
    # Validate before adding an observer to the graph.
    status=core.check_session(session)
    if not status['start_allowed']: raise ValueError(json.dumps(status))
    session=Path(session)
    child,identity=core.spawn_owned(session,'field_observer',[sys.executable,str(HERE/'field.py'),'_observer',str(session)])
    try:
        result=core.start_session(session)
        result['endpoint_release']=release(session)
        mapped_identities(session)
        print(json.dumps(dict(started=True, fixed=EXPECTED,
            frontend=result['startup_summary']['binaries']['frontend']['path'],
            configuration=str(session/'snapshots/frontend.yaml'),
            startup_summary=str(session/'startup_summary.json'),endpoint_release=str(session/'live_release.json'),
            instruction='Keep robot stationary. Run params and ready; started is not INIT_COMMIT.'),indent=2))
        return result
    except BaseException:
        # No SIGKILL or global pkill. Preserve recorder and any state already collected.
        for role in ['frontend','ingress','backend','field_observer']:
            e=lib.read_json(session/'processes.json').get(role,{})
            if lib.process_matches(e): core.signal_owned(e,signal.SIGINT)
        raise


def effective(session):
    target=Path(session)/'snapshots/actual_frontend.yaml'
    result=core.dump_parameters(session) if not target.exists() else lib.read_json(Path(session)/'actual_parameters.json')
    document=yaml.safe_load(target.read_text())
    candidates=[v['ros__parameters'] for k,v in document.items() if k in ['laser_mapping','/laser_mapping'] and isinstance(v,dict)]
    if len(candidates)!=1: raise ValueError('actual laser_mapping node parameter dump missing/ambiguous')
    p=candidates[0]
    if initials.initial_values(p)!=EXPECTED: raise ValueError('actual ROS parameter values differ from user fixed profile')
    if p.get('field_capture',{}).get('imu_depth')!=100: raise ValueError('actual IMU depth differs')
    qos=lib.read_json(Path(session)/'actual_input_qos.json').get('/lidar_imu',[])
    if len(qos)!=1 or qos[0]['depth']!=100 or qos[0]['history']!='KEEP_LAST' or qos[0]['reliability']!='BEST_EFFORT':
        raise ValueError('actual DDS frontend IMU QoS differs: '+str(qos))
    result['fixed_values']=EXPECTED; lib.write_json(Path(session)/'fixed_actual_parameters.json',result)
    print(json.dumps(result,indent=2)); return result


def bind_network(session):
    check_environment()
    session=Path(session)
    if lib.read_json(session/'session.json').get('field_mode')!='live': raise ValueError('only live sessions may bind robot LAN')
    uri=os.environ.get('CYCLONEDDS_URI','')
    if not uri.startswith('file://'): raise ValueError('source env.sh to generate explicit local DDS file')
    import ipaddress
    ip=ipaddress.IPv4Address(os.environ.get('A2W_PC_IP',''))
    if ip not in ipaddress.IPv4Network('192.168.123.0/24') or ip==ipaddress.IPv4Address('192.168.123.164'):
        raise ValueError('robot-LAN local address must be verified 192.168.123.x and distinct from164')
    import shutil
    environment=session/'field_environment'; environment.mkdir()
    shutil.copy2(uri[7:],environment/'cyclonedds.xml')
    lib.write_json(environment/'network.json',dict(domain=0,localhost_only=False,
        interface=os.environ.get('A2W_NETWORK_INTERFACE'),pc_ip=str(ip),peer='192.168.123.164',
        dds_sha256=lib.sha256(environment/'cyclonedds.xml')))
    return dict(bound=True,environment=str(environment),domain=0,pc_ip=str(ip))


def ready(session,timeout):
    end=time.monotonic()+timeout
    while True:
        r=core.ready_session(session,0); fixed=fixed_state_report(Path(session)/'runtime_state.csv')
        r['fixed_state']=fixed; r['ready']=bool(r['ready'] and fixed['valid'] and (Path(session)/'live_release.json').exists())
        try:
            obs=lib.read_json(Path(session)/'observer_live.json'); progress=obs.get('backend',{})
            r['backend_progress']=progress
            backend_ok=backend_receiving(progress,time.monotonic_ns())
            r['ready']=r['ready'] and backend_ok
            if not backend_ok: r['backend_ready_reason']='need fresh receiving backend with actual processed keyframe'
        except (ValueError,OSError): r['ready']=False
        if r['ready']:
            if not (Path(session)/'fixed_actual_parameters.json').exists(): effective(session)
            break
        if time.monotonic()>=end: break
        time.sleep(.2)
    lib.write_json(Path(session)/'field_ready.json',r); print(json.dumps(r,indent=2)); return 0 if r['ready'] else 2


def handoff_report(session):
    import importlib.util
    path=WS/'experiments/offline_mapping_optimization_20261022/replay/analyze_handoff.py'
    spec=importlib.util.spec_from_file_location('field_handoff_accounting',path)
    audit=importlib.util.module_from_spec(spec); spec.loader.exec_module(audit)
    events={}; writers={}; errors=[]; all_counts={}
    for role in ['frontend','ingress','backend']:
        rows=audit.csv_rows(Path(session)/(role+'_handoff.csv'))
        status=lib.read_json(Path(session)/(role+'_handoff_trace_status.json')); writers[role]=status
        if not (status.get('closed') and status.get('flush_ok') and status.get('errors')==0 and status.get('attempted')==status.get('written')==len(rows)):
            errors.append(role+' handoff writer not fully closed/complete')
        if [int(r['seq']) for r in rows]!=list(range(len(rows))): errors.append(role+' sequence discontinuity')
        for r in rows: events.setdefault(r['stage'],[]).append(int(r['stamp_ns']))
    saved=audit.csv_rows(Path(session)/'bundles/map_bundle/keyframes/index.csv')
    stamps=[int(r['stamp_ns']) for r in saved]; cutoff=max(stamps)
    runtime=[int(r['state_stamp_ns']) for r in audit.csv_rows(Path(session)/'runtime_state.csv') if r['stage']=='POST_LIDAR_UPDATE']
    stages={}
    def check(stage,expected,tolerance=0):
        seen=[t for t in events.get(stage,[]) if t<=cutoff+1000]
        all_counts[stage]=len(events.get(stage,[]))
        stages[stage]=audit.account(expected,seen,tolerance)
        if not stages[stage]['complete']: errors.append(stage+' missing/extra in saved cutoff')
        return seen
    post=[t for t in runtime if t<=cutoff+1000]
    check('post_lidar_update',post)
    pub=check('frontend_odom_publish_return',post,1000)
    for stage in ['frontend_cloud_publish_return','ingress_odom_received','ingress_cloud_received','ingress_synchronized','keyframe_decision']:
        check(stage,pub)
    for stage in ['keyframe_odom_publish_return','keyframe_cloud_publish_return','backend_odom_received','backend_cloud_received','backend_synchronized','backend_accepted','backend_processed']:
        check(stage,stamps)
    if not pub or not stamps or min(pub)!=min(stamps): errors.append('first POST not covered by first saved keyframe')
    release=lib.read_json(Path(session)/'live_release.json')
    if release.get('verified') is not True: errors.append('endpoint release evidence invalid')
    for role in ['frontend','ingress','backend']:
        rows=audit.csv_rows(Path(session)/(role+'_handoff.csv'))
        if role=='frontend':
            first=min((int(r['steady_ns']) for r in rows if r['stage']=='post_lidar_update'),default=0)
            if first<=release['checked_steady_ns']: errors.append('POST before actual endpoint release')
    return dict(valid=not errors,errors=errors,stages=stages,trace_writers=writers,all_stage_counts=all_counts,
                saved_keyframes=len(stamps),saved_cutoff_stamp_ns=cutoff,total_runtime_post=len(runtime),
                post_after_saved_cutoff=sum(t>cutoff+1000 for t in runtime),
                scope='Integer stamps, derived POST tolerance 1000 ns only. Saved prefix is frozen; continuing sensor tail is reported separately.')


def verify(session):
    # Reuse the normal scan of the Bag, collecting only integer IMU identities for a
    # callback-coverage comparison. No second full raw-cloud decode is needed.
    original=core.read_bag_messages; bag_imu=Counter()
    def counted(path):
        for topic,type_,message,stamp in original(path):
            if topic=='/lidar_imu': bag_imu[core.stamp_ns(message.header)]+=1
            yield topic,type_,message,stamp
    core.read_bag_messages=counted
    try: result=core.verify_session(session)
    finally: core.read_bag_messages=original
    fixed=fixed_state_report(Path(session)/'runtime_state.csv',closed=True); result['fixed_state']=fixed
    # The first column is status, followed by exact raw_stamp_ns in the existing v4 writer.
    callback=Counter()
    with (Path(session)/'input.csv').open() as stream:
        reader=csv.reader(stream); header=next(reader)
        for row in reader:
            if len(row)>1 and row[0] in ['adapted','rejected']: callback[int(row[1])]+=1
    if callback:
        lo,hi=min(callback),max(callback); recorded=Counter({t:n for t,n in bag_imu.items() if lo<=t<=hi})
        missing=recorded-callback; extra=callback-recorded
        result['imu_callback_vs_bag']=dict(valid=not missing and not extra,begin_ns=lo,end_ns=hi,
            bag_count=sum(recorded.values()),callback_count=sum(callback.values()),
            bag_not_in_callback_count=sum(missing.values()),callback_not_in_bag_count=sum(extra.values()),
            bag_not_in_callback_stamps=list(missing.elements())[:100],callback_not_in_bag_stamps=list(extra.elements())[:100],
            scope='Only callback-covered sensor interval, integer stamp multiplicities. Not driver upstream loss proof.')
    else: result['imu_callback_vs_bag']=dict(valid=False,error='no callback trace')
    try: result['handoff']=handoff_report(session)
    except (ValueError,OSError,KeyError) as e: result['handoff']=dict(valid=False,error=str(e))
    if not fixed['valid'] or not result['imu_callback_vs_bag']['valid'] or not result['handoff']['valid']:
        result['capture_degraded']=True
    lib.write_json(Path(session)/'field_verification.json',result)
    print(json.dumps(result,indent=2)); return 2 if result['capture_degraded'] else 0


def main():
    configure()
    p=argparse.ArgumentParser(description=__doc__); commands=p.add_subparsers(dest='command',required=True)
    for name in ['init','prepare']:
        q=commands.add_parser(name); q.add_argument('name'); q.add_argument('--authorize-user-fixed-extrinsic',action='store_true'); q.add_argument('--authorization-note',required=True)
    for name in ['check','inputs','record','start','params','ready','save','stop','verify','loops','bind-network','_observer','_test-recorder']:
        q=commands.add_parser(name); q.add_argument('session',type=Path)
        if name in ['ready','save']: q.add_argument('--timeout',type=float,default=120 if name=='ready' else 600)
        if name=='stop': q.add_argument('--discard-unsaved',action='store_true')
    a=p.parse_args()
    try:
        if a.command in ['init','prepare']:
            if a.command=='init': check_environment()
            _,policy=authorize(lib.read_json(INTERFACE_SOURCE),PROFILE,lib.parameters(PROFILE),lib.sha256(PROFILE),a.authorization_note,a.authorize_user_fixed_extrinsic)
            session=core.init_session(a.name,WS/'captures/field_fixed_sessions',BUILD,INTERFACE_SOURCE,base=PROFILE,experimental_policy=policy)
            metadata=lib.read_json(session/'session.json'); metadata.update(field_mode='live',profile='user_specified_fixed_configuration')
            lib.write_json(session/'session.json',metadata)
            # prepare is a file-only operation; a fresh field bind is mandatory before any ROS action.
            if a.command=='init': bind_network(session)
            import shlex
            (session/'session.env').write_text('export SESSION='+shlex.quote(str(session))+'\nsource '+shlex.quote(str(HERE/'env.sh'))+' "$SESSION"\n')
            lib.write_json(OUT/'current_session.json',dict(session=str(session)))
            (OUT/'current_session.env').write_text('source '+shlex.quote(str(session/'session.env'))+'\n')
            print(json.dumps(dict(session=str(session),binary=build_identity(BUILD)['binaries']['frontend']['path'],config=str(session/'snapshots/frontend.yaml'),fixed=EXPECTED),indent=2))
            return 0
        session=a.session.resolve(strict=True)
        if a.command=='bind-network':
            print(json.dumps(bind_network(session),indent=2)); return 0
        check_environment(session)
        if a.command=='_observer': observer(session); return 0
        if a.command=='_test-recorder':
            if lib.read_json(session/'session.json').get('field_mode')!='offline-test': raise ValueError('test recorder only allowed in isolated regression')
            return core.bag_child(session,'none',CACHE_MIB)
        if a.command=='check':
            r=core.check_session(session); print(json.dumps(r,indent=2)); return 0 if r['start_allowed'] else 2
        if a.command=='inputs':
            r=lib.read_json(session/'raw_input_report.json')
            errors=core.capture_degradation(session)
            settings=lib.read_json(session/'record_settings.json')
            if settings.get('cache_size_bytes')!=134217728 or settings.get('compression')!='none' or settings.get('topics')!=lib.TOPICS:
                errors.append('actual recorder settings are not this field profile')
            print(json.dumps(dict(valid=not errors,errors=errors,report=r,settings=settings),indent=2))
            return 2 if errors else 0
        if a.command=='record': return core.record_session(session,'none',CACHE_MIB)
        if a.command=='start': start(session); return 0
        if a.command=='params': effective(session); return 0
        if a.command=='ready': return ready(session,a.timeout)
        if a.command=='save':
            r=core.save_session(session,a.timeout); print(json.dumps(r,indent=2)); return 0
        if a.command=='stop':
            r=core.stop_session(session,a.discard_unsaved)
            e=lib.read_json(session/'processes.json').get('field_observer',{})
            if lib.process_matches(e): core.assert_owned_session_process(session,'field_observer',e); core.signal_owned(e,signal.SIGINT)
            print(json.dumps(r,indent=2)); return 0
        if a.command=='verify':
            return verify(session)
        if a.command=='loops':
            log=session/'logs/backend.log'
            with log.open() as stream:
                while True:
                    line=stream.readline()
                    if line:
                        if committed_line(line): print(line.rstrip(),flush=True)
                    else: time.sleep(.2)
    except (ValueError,OSError,KeyError,subprocess.SubprocessError) as e:
        print(json.dumps(dict(error=str(e))),file=sys.stderr); return 2


if __name__=='__main__': raise SystemExit(main())
