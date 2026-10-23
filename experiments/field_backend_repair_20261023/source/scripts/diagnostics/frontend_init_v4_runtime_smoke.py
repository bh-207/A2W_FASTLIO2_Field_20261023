#!/usr/bin/env python3
"""ROOT-coordinated localhost-only synthetic real-node smoke. Never a robot test.

Does not start ROS without --allow-localhost-ros and an explicit nonzero domain.
Pure fixture loading/review functions do not import ROS. Root owns run scheduling.
"""
import argparse
import array
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_fixture(path):
    path=Path(path).resolve();meta=json.loads((path/'manifest.json').read_text())
    if meta.get('source_provenance')!='synthetic' or meta.get('scans')!=30:
        raise ValueError('only frozen synthetic30 scan fixture is allowed; never real recording')
    required=['imu.csv','extrinsic.txt']+[f'scan_{i:04d}.csv' for i in range(30)]
    for name in required:
        item=(path/name).resolve()
        if item.parent!=path or meta.get('exported_sha256',{}).get(name)!=sha(item):
            raise ValueError('fixture integrity mismatch '+name)
    with (path/'imu.csv').open() as stream:
        imu=[[float(row[key]) for key in ['stamp_s','ax','ay','az','gx','gy','gz']] for row in csv.DictReader(stream)]
    scans=[];packer=struct.Struct('<ffffH6xd')
    for index in range(30):
        packed=bytearray();first=None;last=None
        with (path/f'scan_{index:04d}.csv').open() as stream:
            for row in csv.DictReader(stream):
                values=[float(row[key]) for key in ['x','y','z','intensity','ring','timestamp']]
                if not all(math.isfinite(x) for x in values):raise ValueError('nonfinite fixture point')
                if first is None:first=values[5]
                last=values[5];packed.extend(packer.pack(*values[:4],int(values[4]),values[5]))
        if first is None or last<first:raise ValueError('empty or reversed scan')
        scans.append((round(first*1e9),round(last*1e9),bytes(packed)))
    extrinsic=[float(x) for x in (path/'extrinsic.txt').read_text().split()]
    if len(extrinsic)!=12:raise ValueError('extrinsic must be9 rotation+3 translation')
    return meta,imu,scans,extrinsic

def review_outputs(output,odometry):
    output=Path(output);runtime=output/'runtime_state.csv'
    if not runtime.is_file():raise ValueError('RUNTIME STATE LOG MISSING on actual laserMapping node')
    with runtime.open() as stream:rows=list(csv.DictReader(stream))
    state_columns=[p+'_'+a for p in ['pos_O_I','vel_O_I','g_O','bg_I','ba_I','t_I_L'] for a in 'xyz']
    state_columns += [p+'_'+a for p in ['q_O_I','q_I_L'] for a in 'xyzw']+['anti_g_O_to_plus_Z_O_rad']
    if any(not math.isfinite(float(row[k])) for row in rows for k in state_columns):
        raise ValueError('nonfinite actual runtime state/angle')
    initial=[r for r in rows if r['stage']=='INIT_COMMIT']
    post=[r for r in rows if r['stage']=='POST_LIDAR_UPDATE']
    if len(initial)!=1 or not post:raise ValueError('actual node must produce exactly1 INIT and nonempty POST_LIDAR_UPDATE')
    init=initial[0]
    if not (init['init_stamp_consistent']=='1' and int(init['state_stamp_ns'])==int(init['accepted_window_last_imu_ns'])==int(init['group_last_imu_ns'])):
        raise ValueError('INIT exact accepted source stamp is not verified')
    with (output/'quality.csv').open() as stream:
        committed=[r for r in csv.DictReader(stream) if r['stage']=='initialized']
    if len(committed)!=1 or int(committed[0]['last_stamp_ns'])!=int(init['state_stamp_ns']):
        raise ValueError('runtime INIT does not bind exact actual Process committed window')
    quality=committed[0]
    pairs=[('g_O_'+a,'g_W'+a) for a in 'xyz']+[(p+'_I_'+a,p+a) for p in ['bg','ba'] for a in 'xyz']
    pairs += [('t_I_L_'+a,'ext_t'+a) for a in 'xyz']+[('q_I_L_'+a,'ext_q'+a) for a in 'xyzw']
    pairs += [('q_O_I_'+a,'q_WI'+a) for a in 'xyzw']
    if any(abs(float(init[a])-float(quality[b]))>1e-12 for a,b in pairs):
        raise ValueError('runtime INIT state differs from actual Process committed state')
    by_stamp={int(o['stamp_ns']):o for o in odometry};matched=0
    for row in post:
        stamp=int(row['state_stamp_ns'])
        if stamp!=int(row['scan_end_ns']):raise ValueError('post state attached to wrong scan source time')
        # FAST-LIO get_ros_time casts double seconds; observation uses llround.
        match=next((by_stamp[k] for k in [stamp,stamp-1,stamp+1] if k in by_stamp),None)
        if match is None:raise ValueError('no same-scan actual Odometry for runtime POST row')
        actual=[float(row[k]) for k in ['pos_O_I_x','pos_O_I_y','pos_O_I_z','q_O_I_x','q_O_I_y','q_O_I_z','q_O_I_w']]
        if max(abs(a-b) for a,b in zip(actual,match['pose']))>1e-10:
            raise ValueError('runtime snapshot differs from actual same-scan post-update Odometry')
        matched+=1
    status=json.loads(Path(str(runtime)+'.status.json').read_text())
    if not status.get('closed') or not status.get('flush_ok') or status.get('dropped') or status.get('write_errors'):
        raise ValueError('runtime writer did not close/flush completely without drops/errors')
    if status['enqueued']!=status['written'] or status['written']!=len(rows):raise ValueError('row count differs from closed writer accounting')
    effective=sum(int(r['lidar_effective_features'])>0 for r in post)
    if not effective:raise ValueError('update invocation exists but no effective lidar features demonstrated')
    return dict(init_rows=len(initial),post_lidar_update_rows=len(post),same_scan_odometry_matches=matched,
        post_rows_with_effective_lidar_features=effective,init_state_stamp_ns=int(init['state_stamp_ns']),
        init_anti_g_to_camera_init_Z_rad=float(init['anti_g_O_to_plus_Z_O_rad']),writer_status=status,
        source_provenance='synthetic',gravity_alignment=False,robot_connected=False,
        scope='actual laserMapping node, DDS localhost synthetic input, real iterated lidar update path; no real sensor/robot accuracy certification')

def run(isolation,fixture,output,domain,authorized=False):
    if not authorized or not 1<=domain<=232:raise ValueError('explicit --allow-localhost-ros and nonzero unique domain1..232 required')
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!=str(domain):
        raise ValueError('caller must explicitly set ROS_LOCALHOST_ONLY=1 and matching ROS_DOMAIN_ID')
    # Known discovery/profile variables can override the intended local scope.
    for name in ['ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE']:
        if os.environ.get(name):raise ValueError('remove externally configured discovery/transport '+name)
    isolation,fixture,output=map(lambda p:Path(p).resolve(),[isolation,fixture,output])
    if output.exists() or any(p==output or p in output.parents or output in p.parents for p in [isolation,fixture]):
        raise ValueError('new nonoverlapping smoke output required')
    build=json.loads((isolation/'build_manifest.json').read_text());binary=isolation/'build/fastlio_mapping'
    if sha(binary)!=build['binaries']['fastlio_mapping']['sha256']:raise ValueError('node binary provenance mismatch')
    meta,imu,scans,extrinsic=load_fixture(fixture)
    output.mkdir(parents=True)
    # Keep rclpy/rclcpp logging and optional ROS caches inside this smoke session,
    # including failures before the estimator child process exists.
    os.environ['ROS_LOG_DIR']=str(output/'ros_logs')
    os.environ['ROS_HOME']=str(output/'ros_home')
    (output/'ros_logs').mkdir();(output/'ros_home').mkdir()
    # ROS imports below this explicit authorization boundary only.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,PointCloud2,PointField
    from nav_msgs.msg import Odometry
    topic=f'/frontend_runtime_v4_smoke_{os.getpid()}'
    parameters={'common.lid_topic':topic+'/points','common.imu_topic':topic+'/imu',
        'common.imu_gyr_unit':'rad','common.time_sync_en':False,'common.time_offset_lidar_to_imu':0.,
        'preprocess.lidar_type':2,'preprocess.scan_line':128,'preprocess.timestamp_unit':0,'preprocess.blind':.5,
        'point_filter_num':6,'feature_extract_enable':False,'max_iteration':3,'filter_size_surf':.5,'filter_size_map':.3,
        'mapping.extrinsic_R':extrinsic[:9],'mapping.extrinsic_T':extrinsic[9:],'mapping.extrinsic_est_en':False,
        'runtime_pos_log_enable':False,'pcd_save.pcd_save_en':False,'trajectory_save.tum_en':False,
        'publish.path_en':False,'publish.scan_publish_en':False,'publish.scan_bodyframe_pub_en':False,
        'frontend_init_v4.input_interface_enabled':True,'frontend_init_v4.quality_gate_enabled':True,
        'frontend_init_v4.input_gyro_unit':'rad/s','frontend_init_v4.input_accel_unit':'m/s^2','frontend_init_v4.input_imu_frame':'imu',
        'frontend_init_v4.source_provenance':'synthetic','frontend_init_v4.gyro_unit_evidence':'frozen terrain fixture manifest',
        'frontend_init_v4.accel_unit_evidence':'frozen terrain fixture manifest','frontend_init_v4.time_offset_evidence':'same frozen source clock,offset0',
        'frontend_init_v4.quality_trace_path':str(output/'quality.csv'),'frontend_init_v4.input_trace_path':str(output/'input.csv'),
        'frontend_init_v4.runtime_state_log_enabled':True,'frontend_init_v4.runtime_state_log_path':str(output/'runtime_state.csv'),
        'frontend_init_v4.runtime_state_log_capacity':1024}
    params=output/'parameters.json';params.write_text(json.dumps({'/**':{'ros__parameters':parameters}},indent=2)+'\n')
    command=[str(binary),'--ros-args','--params-file',str(params),'-r','/Odometry:='+topic+'/odom']
    identity=dict(binary_sha256=sha(binary),source_patch_sha256=build['patch_sha256'],fixture_manifest_sha256=sha(fixture/'manifest.json'),
        source_provenance='synthetic',command=command,ros_domain_id=domain,ROS_LOCALHOST_ONLY='1',
        ROS_LOG_DIR=os.environ['ROS_LOG_DIR'],ROS_HOME=os.environ['ROS_HOME'],gravity_alignment=False,robot_connected=False)
    (output/'identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    rclpy.init();node=rclpy.create_node('frontend_runtime_v4_synthetic_smoke')
    imu_pub=node.create_publisher(Imu,topic+'/imu',qos_profile_sensor_data)
    cloud_pub=node.create_publisher(PointCloud2,topic+'/points',qos_profile_sensor_data)
    odometry=[]
    def observe(msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        odometry.append(dict(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]))
    subscription=node.create_subscription(Odometry,topic+'/odom',observe,1000)
    process=None;failure=None;exit_code=None
    def service(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=min(.01,max(0.,until-time.monotonic())))
    try:
        with (output/'node.log').open('x') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+15
            while (imu_pub.get_subscription_count()<1 or cloud_pub.get_subscription_count()<1) and time.monotonic()<deadline:
                if process.poll() is not None:raise ValueError('actual node exited before subscription discovery')
                service(.05)
            if not imu_pub.get_subscription_count() or not cloud_pub.get_subscription_count():raise ValueError('local node subscriptions not discovered')
            fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in [
                ('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),('z',8,PointField.FLOAT32),('intensity',12,PointField.FLOAT32),
                ('ring',16,PointField.UINT16),('timestamp',24,PointField.FLOAT64)]]
            cursor=0;epoch=time.monotonic();source_epoch=imu[0][0]
            def publish_imu(values):
                ns=round(values[0]*1e9);msg=Imu();msg.header.frame_id='imu';msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=values[1:4]
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=values[4:7];imu_pub.publish(msg)
            for begin,end,packed in scans:
                # Deliver original source times once, without retiming or new motion.
                while cursor<len(imu) and round(imu[cursor][0]*1e9)<=end+5_000_000:
                    due=epoch+imu[cursor][0]-source_epoch
                    if due>time.monotonic():service(due-time.monotonic())
                    publish_imu(imu[cursor]);cursor+=1
                cloud=PointCloud2();cloud.header.frame_id='lidar';cloud.header.stamp.sec=begin//10**9;cloud.header.stamp.nanosec=begin%10**9
                cloud.height=1;cloud.width=len(packed)//32;cloud.fields=fields;cloud.is_bigendian=False;cloud.point_step=32
                cloud.row_step=len(packed);cloud.is_dense=True;cloud.data=array.array('B',packed);cloud_pub.publish(cloud);service(.015)
            while cursor<len(imu):publish_imu(imu[cursor]);cursor+=1;service(.005)
            service(3.)
            process.send_signal(signal.SIGINT);exit_code=process.wait(timeout=30);service(.2)
            if exit_code!=0:raise ValueError('actual node SIGINT exit was nonzero '+str(exit_code))
    except Exception as error:failure=str(error)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
        (output/'actual_odometry.json').write_text(json.dumps(odometry,indent=2)+'\n')
    result={**identity,'node_exit_code':exit_code,'SIGINT_requested':exit_code is not None,'odometry_rows':len(odometry)}
    try:
        if failure:raise ValueError(failure)
        result.update(review_outputs(output,odometry));result['passed']=True
    except Exception as error:result.update(passed=False,error=str(error))
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n');return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('isolation',type=Path);p.add_argument('fixture',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--domain',type=int,required=True);p.add_argument('--allow-localhost-ros',action='store_true');a=p.parse_args()
    result=run(a.isolation,a.fixture,a.output,a.domain,a.allow_localhost_ros);print(json.dumps(result,indent=2));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
