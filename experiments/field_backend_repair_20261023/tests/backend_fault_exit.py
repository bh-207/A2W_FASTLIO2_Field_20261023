#!/usr/bin/env python3
"""Actual backend invalid-frame -> SIGINT regression, loopback only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('explicit offline domain87/localhost-only required')
    out=Path(sys.argv[1]).resolve();out.mkdir()
    xml=out/'dds.xml'
    xml.write_text('<CycloneDDS><Domain Id="any"><General><AllowMulticast>false</AllowMulticast><Transport>tcp</Transport></General><TCP><Port>0</Port></TCP><Discovery><Peers><Peer Address="127.0.0.1:39901"/><Peer Address="127.0.0.1:39902"/></Peers></Discovery></Domain></CycloneDDS>')
    env=dict(os.environ,A2W_FIELD_MODE='offline-test',A2W_HANDOFF_DIRECTORY=str(out),CYCLONEDDS_URI=xml.as_uri())
    # Fixed per-process TCP ports; no multicast or non-local peer.
    backend_xml=out/'backend.xml';backend_xml.write_text(xml.read_text().replace('<Port>0</Port>','<Port>39901</Port>'))
    driver_xml=out/'driver.xml';driver_xml.write_text(xml.read_text().replace('<Port>0</Port>','<Port>39902</Port>'))
    env['CYCLONEDDS_URI']=backend_xml.as_uri();os.environ['CYCLONEDDS_URI']=driver_xml.as_uri()
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from a2w_fastlio_msgs.msg import RegistrationStatus
    rclpy.init();node=rclpy.create_node('backend_fault_regression');states=[]
    node.create_subscription(RegistrationStatus,'/mapping/registration_status',lambda m:states.append(m.reason) if m.stage=='backend' else None,100)
    pubs=[node.create_publisher(Odometry,'/mapping/keyframe_odom',100),node.create_publisher(PointCloud2,'/mapping/keyframe_cloud',100)]
    def wait(predicate,timeout=10):
        limit=time.monotonic()+timeout
        while time.monotonic()<limit:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    process=None
    try:
        with (out/'backend.log').open('x') as stream:
            process=subprocess.Popen([sys.argv[2],'--ros-args','-p','status_period_ms:=100'],env=env,stdout=stream,stderr=subprocess.STDOUT)
            assert wait(lambda:all(p.get_subscription_count()>0 for p in pubs)), 'DDS matching failed'
            odom=Odometry();cloud=PointCloud2();odom.header.stamp.sec=cloud.header.stamp.sec=1
            odom.header.frame_id='deliberately_invalid_frame';odom.child_frame_id=cloud.header.frame_id='body';odom.pose.pose.orientation.w=1.
            pubs[0].publish(odom);pubs[1].publish(cloud)
            assert wait(lambda:any('state=fault' in s for s in states)), 'invalid input was not diagnosed'
            process.send_signal(signal.SIGINT);code=process.wait(timeout=10)
        text=(out/'backend.log').read_text();result=dict(actual_exit_code=code,fault_observed=True,shutdown_incomplete='shutdown_incomplete' in text,passed=code==2 and 'shutdown_incomplete' in text)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return 0 if result['passed'] else 1
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGTERM);process.wait(timeout=10)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
