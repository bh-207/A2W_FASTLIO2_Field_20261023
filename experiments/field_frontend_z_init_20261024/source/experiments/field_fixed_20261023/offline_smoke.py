#!/usr/bin/env python3
"""Short real-Bag wiring/state regression. Domain87 TCP loopback only, no live start bypass."""
import json
import csv
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import shutil

import field as f
from replay_drain import wait_frontend


def smoke_passed(result):
    return (not result.get('error') and not result.get('cleanup_error') and
            result.get('frontend_eof_drain',{}).get('barrier_passed') is True and
            result['frontend_eof_drain'].get('callback_counts_exact') is True and
            set(result.get('exit_codes',{}))=={'frontend','ingress','backend','recorder','field_observer'} and
            all(c==0 for c in result['exit_codes'].values()) and
            result.get('fixed_state_closed',{}).get('valid') is True and
            result['fixed_state_closed'].get('post',0)>=150 and
            result.get('runtime_shutdown',{}).get('valid') is True and
            result.get('handoff',{}).get('valid') is True and
            result.get('backend_receiving') is True and
            result.get('recorder_log',{}).get('checked') is True and
            not result['recorder_log'].get('loss_reported',True) and not result['recorder_log'].get('errors'))


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('this regression requires explicit domain87 / localhost-only')
    f.configure()
    duration=float(os.environ.get('A2W_REPLAY_SECONDS','20'))
    rate=float(os.environ.get('A2W_REPLAY_RATE','1'))
    if duration==0 and shutil.disk_usage(f.OUT).free<35*2**30:
        raise ValueError('full replay requires >=35GiB independent output space; no old data deleted')
    bag=Path(os.environ.get('A2W_REPLAY_BAG','/home/bh/A2W_FastLio2_controlled/captures/field_fixed_sessions/fixed_short_20261023_001/bag')).resolve()
    name=sys.argv[1] if len(sys.argv)>1 else 'offline_smoke_001'
    if not name.startswith('offline_smoke_') or not name[14:].isdigit(): raise ValueError('fresh versioned test name required')
    root=f.OUT/'tests'/name;root.parent.mkdir(parents=True,exist_ok=True);root.mkdir()
    base=38100+int(name[14:])*10
    ports={r:base+i+1 for i,r in enumerate(['frontend','ingress','backend','field_observer','player','helper','recorder'])}
    peers=''.join('<Peer Address="127.0.0.1:'+str(p)+'"/>' for p in ports.values())
    for role,port in ports.items():
        (root/(role+'.xml')).write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="any"><General><AllowMulticast>false</AllowMulticast><Transport>tcp</Transport></General><TCP><Port>'+str(port)+'</Port><NoDelay>true</NoDelay></TCP><Discovery><Peers>'+peers+'</Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    _,policy=f.authorize(f.lib.read_json(f.INTERFACE_SOURCE),f.PROFILE,f.lib.parameters(f.PROFILE),f.lib.sha256(f.PROFILE),'Offline field-entry regression; user fixed R/T; not hardware evidence',True)
    guard=os.environ.get('A2W_OFFLINE_GEOMETRY_GUARD','0')=='1'
    review=os.environ.get('A2W_OFFLINE_REVOCABLE_REVIEW','0')=='1'
    async_input_trace=os.environ.get('A2W_OFFLINE_ASYNC_INPUT_LOG','0')=='1'
    sqlite_sync_off=os.environ.get('A2W_OFFLINE_SQLITE_SYNC_OFF','0')=='1'
    session=f.core.init_session('session',root,f.BUILD,f.INTERFACE_SOURCE,base=f.PROFILE,experimental_policy=policy,causal_geometry_guard=guard,revocable_review=review,async_input_trace=async_input_trace,offline_sqlite_sync_off=sqlite_sync_off)
    metadata=f.lib.read_json(session/'session.json'); metadata['field_mode']='offline-test'; f.lib.write_json(session/'session.json',metadata)
    f.lib.write_json(session/'offline_script_identity.json',dict(files={str(path):f.lib.sha256(path) for path in
        [Path(__file__).resolve(),f.HERE/'field.py',Path(f.core.__file__).resolve(),Path(f.lib.__file__).resolve(),f.HERE/'replay_drain.py']}),exclusive=True)
    children={}; result=dict(offline_only=True,live_input_gate_exercised=False,recorder_exercised=True,record_manager_exercised=False,
        source_bag=str(bag),requested_duration_s=duration,rate=rate,causal_geometry_guard=guard,revocable_review=review,async_input_trace=async_input_trace,offline_sqlite_sync_off=sqlite_sync_off)
    def environment(role):
        os.environ['CYCLONEDDS_URI']='file://'+str(root/(role+'.xml'))
    def launch(role,cmd):
        environment(role); children[role]=f.core.spawn_owned(session,role,cmd)[0]
    names={'frontend':['frontend.yaml'],'ingress':['ingress.yaml'],
           'backend':['mapping_topics.yaml','pose_graph.yaml','scan_context.yaml','registration.yaml','loop_validation.yaml','map.yaml','backend_runtime.yaml']}
    try:
        launch('recorder',[sys.executable,str(f.HERE/'field.py'),'_test-recorder',str(session)])
        launch('field_observer',[sys.executable,str(f.HERE/'field.py'),'_observer',str(session)])
        identity=f.lib.read_json(session/'identity.json')['build']
        for role in ['backend','ingress','frontend']:
            command=[identity['binaries'][role]['path'],'--ros-args']
            for name in names[role]: command+=['--params-file',str(session/'snapshots'/name)]
            launch(role,command)
        result['release']=f.release(session)
        result['runtime_identities']=f.mapped_identities(session)
        environment('helper'); result['parameters']=f.effective(session)
        actual=f.yaml.safe_load((session/'snapshots/actual_frontend.yaml').read_text())
        selected=next(v['ros__parameters'] for k,v in actual.items() if k in ['laser_mapping','/laser_mapping'])
        trace_parameters=selected.get('frontend_init_v4',{})
        if trace_parameters.get('async_input_trace') is not async_input_trace or trace_parameters.get('input_trace_queue_capacity')!=4096:
            raise ValueError('actual async input trace parameters differ from requested independent logging ablation')
        print('PARAMETERS_VERIFIED',flush=True)
        # The original old sensor epoch is preserved. The live host-vs-sensor gate is
        # intentionally not invoked here: this is a distinctly labelled offline test.
        environment('player')
        import rclpy
        from rclpy.serialization import deserialize_message
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,HistoryPolicy,ReliabilityPolicy,DurabilityPolicy,LivelinessPolicy,qos_check_compatible,QoSCompatibility
        from rclpy.duration import Duration
        from sensor_msgs.msg import PointCloud2,Imu
        import rosbag2_py
        rclpy.init(); node=rclpy.create_node('field_capture_observer_debug_extra')
        print('PLAYER_NODE_CREATED',flush=True)
        bag_metadata=f.yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        source_profiles={}
        for entry in bag_metadata['topics_with_message_count']:
            topic=entry['topic_metadata']['name']
            if topic not in ['/lidar_points','/lidar_imu']:continue
            profiles=f.yaml.safe_load(entry['topic_metadata']['offered_qos_profiles'])
            if len(profiles)!=1:raise ValueError('unambiguous original publisher QoS required')
            source_profiles[topic]=profiles[0]
        def original_qos(topic):
            q=source_profiles[topic]
            kwargs=dict(history=HistoryPolicy(q['history']),depth=q['depth'],
                reliability=ReliabilityPolicy(q['reliability']),durability=DurabilityPolicy(q['durability']),
                liveliness=LivelinessPolicy(q['liveliness']),avoid_ros_namespace_conventions=q['avoid_ros_namespace_conventions'])
            for name in ['deadline','lifespan','liveliness_lease_duration']:
                kwargs[name]=Duration(seconds=q[name]['sec'],nanoseconds=q[name]['nsec'])
            profile=QoSProfile(**kwargs)
            if qos_check_compatible(profile,qos_profile_sensor_data)[0]==QoSCompatibility.ERROR:
                raise ValueError('original publisher QoS incompatible with unchanged BEST_EFFORT consumer')
            return profile
        pubs={t:node.create_publisher(c,t,original_qos(t)) for t,c in [('/lidar_points',PointCloud2),('/lidar_imu',Imu)]}
        f.lib.write_json(session/'playback_original_publisher_qos.json',dict(source_metadata=str(bag/'metadata.yaml'),
            metadata_sha256=f.lib.sha256(bag/'metadata.yaml'),profiles=source_profiles,
            frontend_imu_subscription_unchanged='KEEP_LAST100_BEST_EFFORT',compatibility_checked=True),exclusive=True)
        from nav_msgs.msg import Odometry
        debug_subscribers=[node.create_subscription(c,t,lambda _m:None,100) for t,c in
            [('/Odometry',Odometry),('/cloud_registered_body',PointCloud2),
             ('/mapping/keyframe_odom',Odometry),('/mapping/keyframe_cloud',PointCloud2)]]
        result['extra_debug_subscriber_enabled']=True
        end=time.monotonic()+20
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if all(p.get_subscription_count()==2 for p in pubs.values()): break
        else: raise ValueError('test input publishers did not match the single frontend')
        reader=rosbag2_py.SequentialReader()
        print('READER_CREATED',flush=True)
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(pubs)))
        print('READER_OPENED',flush=True)
        begin=None; clock=None; counts={t:0 for t in pubs};disconnect_injected=False
        disconnect_after=float(os.environ.get('A2W_TEST_DROP_OBSERVER_AFTER_S','0'))
        published_stream=(session/'published_raw.csv').open('x')
        published_writer=csv.writer(published_stream)
        published_writer.writerow(['topic','header_ns','bag_record_ns','publish_steady_ns','publish_before_steady_ns'])
        resources=(session/'process_resources.jsonl').open('x');last_probe=0
        process_ids=f.lib.read_json(session/'processes.json')
        while reader.has_next():
            topic,data,stamp=reader.read_next()
            if begin is None: begin=stamp; clock=time.monotonic()
            if duration and stamp-begin>duration*1e9: break
            while time.monotonic()<clock+(stamp-begin)/1e9/rate:
                rclpy.spin_once(node,timeout_sec=min(.005,max(0,clock+(stamp-begin)/1e9/rate-time.monotonic())))
            message=deserialize_message(data,PointCloud2 if topic=='/lidar_points' else Imu)
            if disconnect_after and not disconnect_injected and stamp-begin>=disconnect_after*1e9:
                owned=f.lib.read_json(session/'processes.json')['field_observer']
                f.core.signal_owned(owned,signal.SIGINT);disconnect_injected=True
                result['required_observer_disconnect_injected']=True
            publish_before=time.monotonic_ns()
            pubs[topic].publish(message); counts[topic]+=1
            published_writer.writerow([topic,f.core.stamp_ns(message.header),stamp,time.monotonic_ns(),publish_before])
            if time.monotonic()-last_probe>=1:
                sample=dict(steady_ns=time.monotonic_ns(),source_record_elapsed_ns=stamp-begin,roles={})
                for role,identity_row in process_ids.items():
                    try:
                        stat=(Path('/proc')/str(identity_row['pid'])/'stat').read_text().rsplit(')',1)[1].split()
                        sample['roles'][role]=dict(cpu_ticks=int(stat[11])+int(stat[12]),rss_bytes=int(stat[21])*os.sysconf('SC_PAGE_SIZE'))
                        sample['roles'][role]['io']={k:int(v) for k,v in (line.split(':',1) for line in (Path('/proc')/str(identity_row['pid'])/'io').read_text().splitlines())}
                        sample['roles'][role]['log_bytes']=(session/'logs'/(role+'.log')).stat().st_size
                    except (OSError,ValueError,IndexError): sample['roles'][role]=dict(unavailable=True)
                sample['output_free_bytes']=shutil.disk_usage(session).free
                sample['memory_kB']={k:int(v.split()[0]) for k,v in (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines()) if k in ['MemAvailable','Dirty','Writeback']}
                resources.write(json.dumps(sample)+'\n');last_probe=time.monotonic()
            if sum(counts.values())%6000==0:
                print(json.dumps(dict(progress_s=(stamp-begin)/1e9,published=counts)),flush=True)
        result['published_counts']=counts
        published_stream.flush();published_stream.close()
        resources.flush();resources.close()
        result['source_record_span_s']=(stamp-begin)/1e9
        node.destroy_node(); rclpy.shutdown(); del reader
        result['frontend_eof_drain']=wait_frontend(session,counts)
        f.lib.write_json(session/'frontend_eof_drain.json',result['frontend_eof_drain'],exclusive=True)
        if not result['frontend_eof_drain']['barrier_passed']:
            raise ValueError('frontend EOF queues were not drained; failed evidence retained')
        end=time.monotonic()+10
        while time.monotonic()<end:
            state=f.fixed_state_report(session/'runtime_state.csv')
            if state['valid'] and state['post']>=150: break
            if any(p.poll() is not None for p in children.values()): raise ValueError('test node exited')
            time.sleep(.1)
        result['fixed_state']=state
        if not state['valid']: raise ValueError('actual fixed state regression failed')
        progress=f.lib.read_json(session/'observer_live.json').get('backend',{})
        result['backend_receiving']=f.backend_receiving(progress,time.monotonic_ns())
        result['observed_backend_progress']=progress
        if not result['backend_receiving']: raise ValueError('actual backend has no fresh processed keyframe')
        environment('helper')
        result['save']=f.core.save_session(session,120)
    except BaseException as error:
        result['error']=repr(error)
        import traceback
        result['traceback']=traceback.format_exc()
    finally:
        try: result['ordered_stop']=f.core.stop_session(session,discard_unsaved='save' not in result,timeout=60)
        except Exception as error: result['cleanup_error']=repr(error)
        for role,child in children.items():
            try: child.wait(timeout=20)
            except subprocess.TimeoutExpired: result['cleanup_error']=role+' still alive; no kill issued'
        result['exit_codes']={r:p.poll() for r,p in children.items()}
        result['fixed_state_closed']=f.fixed_state_report(session/'runtime_state.csv',closed=True)
        result['runtime_shutdown']=f.lib.runtime_shutdown(session/'runtime_state.csv',session/'logs/frontend.log')
        result['recorder_log']=f.core.recorder_log_report(session/'logs/recorder.log')
        if (session/'bundles/map_bundle/manifest.sha256').exists():
            try: result['handoff']=f.handoff_report(session)
            except Exception as e: result['handoff']=dict(valid=False,error=repr(e))
        result['valid']=smoke_passed(result) and result.get('ordered_stop',{}).get('success') is True
        f.lib.write_json(root/'result.json',result,exclusive=True)
        print(json.dumps(dict(result=str(root/'result.json'),valid=result['valid'],error=result.get('error')),indent=2))
    return 0 if result['valid'] else 2


if __name__=='__main__': raise SystemExit(main())
