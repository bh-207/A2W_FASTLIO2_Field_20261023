#!/usr/bin/env python3
"""Short real-Bag wiring/state regression. Domain87 TCP loopback only, no live start bypass."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import field as f


def smoke_passed(result):
    return (not result.get('error') and not result.get('cleanup_error') and
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
    name=sys.argv[1] if len(sys.argv)>1 else 'offline_smoke_001'
    if not name.startswith('offline_smoke_') or not name[14:].isdigit(): raise ValueError('fresh versioned test name required')
    root=f.OUT/'tests'/name; root.mkdir()
    base=38100+int(name[14:])*10
    ports={r:base+i+1 for i,r in enumerate(['frontend','ingress','backend','field_observer','player','helper','recorder'])}
    peers=''.join('<Peer Address="127.0.0.1:'+str(p)+'"/>' for p in ports.values())
    for role,port in ports.items():
        (root/(role+'.xml')).write_text('<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="any"><General><AllowMulticast>false</AllowMulticast><Transport>tcp</Transport></General><TCP><Port>'+str(port)+'</Port><NoDelay>true</NoDelay></TCP><Discovery><Peers>'+peers+'</Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    _,policy=f.authorize(f.lib.read_json(f.INTERFACE_SOURCE),f.PROFILE,f.lib.parameters(f.PROFILE),f.lib.sha256(f.PROFILE),'Offline field-entry regression; user fixed R/T; not hardware evidence',True)
    session=f.core.init_session('session',root,f.BUILD,f.INTERFACE_SOURCE,base=f.PROFILE,experimental_policy=policy)
    metadata=f.lib.read_json(session/'session.json'); metadata['field_mode']='offline-test'; f.lib.write_json(session/'session.json',metadata)
    children={}; result=dict(offline_only=True,live_input_gate_exercised=False,recorder_exercised=True,record_manager_exercised=False)
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
        print('PARAMETERS_VERIFIED',flush=True)
        # The original old sensor epoch is preserved. The live host-vs-sensor gate is
        # intentionally not invoked here: this is a distinctly labelled offline test.
        environment('player')
        import rclpy
        from rclpy.serialization import deserialize_message
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2,Imu
        import rosbag2_py
        rclpy.init(); node=rclpy.create_node('field_offline_test_input')
        print('PLAYER_NODE_CREATED',flush=True)
        pubs={t:node.create_publisher(c,t,qos_profile_sensor_data) for t,c in [('/lidar_points',PointCloud2),('/lidar_imu',Imu)]}
        end=time.monotonic()+20
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if all(p.get_subscription_count()==2 for p in pubs.values()): break
        else: raise ValueError('test input publishers did not match the single frontend')
        reader=rosbag2_py.SequentialReader()
        print('READER_CREATED',flush=True)
        reader.open(rosbag2_py.StorageOptions(uri='/home/bh/A2W_FastLio2/bags/mapping_20261021_152846',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(pubs)))
        print('READER_OPENED',flush=True)
        begin=None; clock=None; counts={t:0 for t in pubs}
        while reader.has_next():
            topic,data,stamp=reader.read_next()
            if begin is None: begin=stamp; clock=time.monotonic()
            if stamp-begin>20_000_000_000: break
            while time.monotonic()<clock+(stamp-begin)/1e9:
                rclpy.spin_once(node,timeout_sec=min(.005,max(0,clock+(stamp-begin)/1e9-time.monotonic())))
            message=deserialize_message(data,PointCloud2 if topic=='/lidar_points' else Imu)
            pubs[topic].publish(message); counts[topic]+=1
        result['published_counts']=counts
        node.destroy_node(); rclpy.shutdown(); del reader
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
        for role in ['frontend','ingress','backend','field_observer','recorder']:
            if role in children and children[role].poll() is None:
                e=f.lib.read_json(session/'processes.json')[role]
                if f.lib.process_matches(e): f.core.signal_owned(e,signal.SIGINT)
                try: children[role].wait(timeout=20)
                except subprocess.TimeoutExpired: result['cleanup_error']=role+' still alive; no kill issued'
        result['exit_codes']={r:p.poll() for r,p in children.items()}
        result['fixed_state_closed']=f.fixed_state_report(session/'runtime_state.csv',closed=True)
        result['runtime_shutdown']=f.lib.runtime_shutdown(session/'runtime_state.csv',session/'logs/frontend.log')
        result['recorder_log']=f.core.recorder_log_report(session/'logs/recorder.log')
        if (session/'bundles/map_bundle/manifest.sha256').exists():
            try: result['handoff']=f.handoff_report(session)
            except Exception as e: result['handoff']=dict(valid=False,error=repr(e))
        result['valid']=smoke_passed(result)
        f.lib.write_json(root/'result.json',result,exclusive=True)
        print(json.dumps(dict(result=str(root/'result.json'),valid=result['valid'],error=result.get('error')),indent=2))
    return 0 if result['valid'] else 2


if __name__=='__main__': raise SystemExit(main())
