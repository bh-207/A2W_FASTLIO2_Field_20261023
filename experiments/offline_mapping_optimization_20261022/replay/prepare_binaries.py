#!/usr/bin/env python3
"""Compile isolated handoff instrumentation against frozen existing objects.

No CMake reconfiguration, installation, algorithm/header or production edits.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
WS=HERE.parents[2]
OUTPUT=WS/'captures/offline_mapping_optimization_20261022_023147'
FRONT=WS/'captures/offline_gravity_imu_20261021/imu/frontend/source'
FRONT_BUILD=WS/'captures/offline_gravity_imu_20261021/imu/build'
BACK_BUILD=WS/'.controlled_mapping/backend_build/a2w_fastlio_mapping'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb')as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def once(s,old,new):
    if s.count(old)!=1:raise ValueError('source anchor count changed: '+old[:90])
    return s.replace(old,new,1)


def prefix(s,role):
    s='#define A2W_TRACE_ROS 1\n#include <iostream>\n#include <rclcpp/rclcpp.hpp>\n#include "startup_trace.hpp"\n'+s
    at=s.index('int main(');p=s.index('{',at)+1
    return s[:p]+'\n    try { offline_handoff::start("'+role+'"); } catch(const std::exception& e) { std::cerr<<e.what()<<"\\n"; return 2; }\n'+s[p:]


def frontend(s):
    s=prefix(s,'frontend')
    marker='        RCLCPP_INFO(this->get_logger(), "Node init finished.");'
    s=once(s,marker,'''        handoff_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this] {
          if(offline_handoff::report_ready("frontend",{
            offline_handoff::edge(*this,pubOdomAftMapped_,"/Odometry",{"mapping_ingress_node","offline_mapping_save_observer_*"}),
            offline_handoff::edge(*this,pubLaserCloudFull_body_,"/cloud_registered_body",{"mapping_ingress_node","offline_mapping_save_observer_*"})}))handoff_timer_->cancel();
        });
'''+marker)
    s=once(s,'    rclcpp::TimerBase::SharedPtr timer_;','    rclcpp::TimerBase::SharedPtr handoff_timer_;\n    rclcpp::TimerBase::SharedPtr timer_;')
    marker='            /******* Publish odometry *******/'
    s=once(s,marker,'''            offline_handoff::event("post_lidar_update",std::llround(Measures.lidar_end_time*1e9),effct_feat_num,"derived_measuregroup_ns");
            offline_handoff::require_two(pubOdomAftMapped_);
            offline_handoff::require_two(pubLaserCloudFull_body_);
'''+marker)
    marker='    pubOdomAftMapped->publish(odomAftMapped);'
    s=once(s,marker,marker+'\n    offline_handoff::event("frontend_odom_publish_return",offline_handoff::stamp_ns(odomAftMapped.header.stamp));')
    marker='    pubLaserCloudFull_body->publish(laserCloudmsg);'
    return once(s,marker,marker+'\n    offline_handoff::event("frontend_cloud_publish_return",offline_handoff::stamp_ns(laserCloudmsg.header.stamp));')


def ingress(s):
    s=prefix(s,'ingress')
    marker='    synchronizer_ = std::make_shared<Synchronizer>('
    s=once(s,marker,'''    odometry_subscriber_.registerCallback([](const nav_msgs::msg::Odometry::ConstSharedPtr& m){offline_handoff::event("ingress_odom_received",offline_handoff::stamp_ns(m->header.stamp));});
    body_cloud_subscriber_.registerCallback([](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){offline_handoff::event("ingress_cloud_received",offline_handoff::stamp_ns(m->header.stamp));});
    handoff_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
      if(offline_handoff::report_ready("ingress",{
        offline_handoff::edge(*this,keyframe_odom_publisher_,"/mapping/keyframe_odom",{"mapping_backend_node","offline_mapping_save_observer_*"}),
        offline_handoff::edge(*this,keyframe_cloud_publisher_,"/mapping/keyframe_cloud",{"mapping_backend_node","offline_mapping_save_observer_*"})}))handoff_timer_->cancel();
    });
'''+marker)
    marker='    const auto adapted = makeFrontendFrame('
    s=once(s,marker,'    offline_handoff::event("ingress_synchronized",offline_handoff::stamp_ns(odometry->header.stamp));\n'+marker)
    marker='      ++adapter_rejection_count_;'
    s=once(s,marker,'      offline_handoff::event("ingress_adapter_rejected",offline_handoff::stamp_ns(odometry->header.stamp));\n'+marker)
    marker='    const auto decision = keyframe_manager_->consider(adapted.frame);'
    s=once(s,marker,marker+'\n    offline_handoff::event("keyframe_decision",adapted.frame.stamp_ns,decision.keyframe?1:0,std::to_string(static_cast<int>(decision.reason)));')
    marker='    keyframe_odom_publisher_->publish(*odometry);'
    s=once(s,marker,'''    offline_handoff::require_two(keyframe_odom_publisher_);
    offline_handoff::require_two(keyframe_cloud_publisher_);
'''+marker+'\n    offline_handoff::event("keyframe_odom_publish_return",adapted.frame.stamp_ns);')
    marker='    keyframe_cloud_publisher_->publish(*body_cloud);'
    s=once(s,marker,marker+'\n    offline_handoff::event("keyframe_cloud_publish_return",adapted.frame.stamp_ns);')
    return once(s,'  std::string odom_frame_;','  rclcpp::TimerBase::SharedPtr handoff_timer_;\n  std::string odom_frame_;')


def backend(s):
    s=prefix(s,'backend')
    marker='    synchronizer_ = std::make_shared<Synchronizer>('
    s=once(s,marker,'''    odometry_subscriber_.registerCallback([](const nav_msgs::msg::Odometry::ConstSharedPtr& m){offline_handoff::event("backend_odom_received",offline_handoff::stamp_ns(m->header.stamp));});
    cloud_subscriber_.registerCallback([](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){offline_handoff::event("backend_cloud_received",offline_handoff::stamp_ns(m->header.stamp));});
'''+marker)
    marker='    std::lock_guard<std::mutex> intake_lock{progress_mutex_};'
    s=once(s,marker,'    offline_handoff::event("backend_synchronized",offline_handoff::stamp_ns(odometry->header.stamp));\n'+marker)
    s=once(s,'      ++ignored_after_freeze_;','      offline_handoff::event("backend_ignored_after_freeze",offline_handoff::stamp_ns(odometry->header.stamp));\n      ++ignored_after_freeze_;')
    marker='    provider_.append(std::move(keyframe));'
    s=once(s,marker,marker+'\n    offline_handoff::event("backend_accepted",adapted.frame.stamp_ns,static_cast<std::int64_t>(id));')
    marker='        ++processed_keyframes_;'
    return once(s,marker,'        offline_handoff::event("backend_processed",current->stamp_ns,static_cast<std::int64_t>(id));\n'+marker)


def flags(path):
    values={}
    for line in path.read_text().splitlines():
        if line.startswith('CXX_'):
            name,value=line.split('=',1);values[name.strip()]=shlex.split(value.strip())
    return values['CXX_DEFINES']+values['CXX_INCLUDES']+values['CXX_FLAGS']


def run():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name',default='build_handoff_002')
    args=parser.parse_args()
    if not args.name.startswith('build_handoff_') or not args.name.removeprefix('build_handoff_').isdigit():raise ValueError('plain versioned build name required')
    target=OUTPUT/args.name
    if target.exists():raise FileExistsError(target)
    target.mkdir();(target/'source').mkdir();manifest={'sources':{},'builds':[]}
    jobs=[('ingress',WS/'src/a2w_fastlio_mapping/src/mapping_ingress_node.cpp',BACK_BUILD,'mapping_ingress_node',ingress),
          ('backend',WS/'src/a2w_fastlio_mapping/src/mapping_backend_node.cpp',BACK_BUILD,'mapping_backend_node',backend),
          ('frontend',FRONT/'src/laserMapping.cpp',FRONT_BUILD,'fastlio_mapping',frontend)]
    for role,source,build,name,adapt in jobs:
        original=source.read_text();changed=adapt(original)
        generated=target/'source'/source.name;generated.write_text(changed)
        (target/'source'/(role+'.patch')).write_text(''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),fromfile=str(source),tofile=str(generated))))
        obj=target/(role+'.o');binary=target/name
        options=flags(build/f'CMakeFiles/{name}.dir/flags.make')
        # Quoted includes originally searched the translation unit's src/ folder.
        # Moving only that unit must preserve its actual original include search.
        compile_cmd=['/usr/bin/c++',*options,'-std=c++17','-I'+str(HERE),'-I'+str(source.parent),'-c',str(generated),'-o',str(obj)]
        link_cmd=shlex.split((build/f'CMakeFiles/{name}.dir/link.txt').read_text())
        oldobj=f'CMakeFiles/{name}.dir/src/{source.name}.o'
        if link_cmd.count(oldobj)!=1:raise ValueError('link object contract changed')
        link_cmd[link_cmd.index(oldobj)]=str(obj);link_cmd[link_cmd.index('-o')+1]=str(binary)
        manifest['sources'][str(source)]={'sha256':sha(source),'generated_sha256':sha(generated)}
        for label,command in [('compile',compile_cmd),('link',link_cmd)]:
            with (target/f'{role}_{label}.log').open('x')as log:
                p=subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT)
            manifest['builds'].append({'role':role,'stage':label,'argv':command,'cwd':str(build),'exit_code':p.returncode})
            (target/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
            if p.returncode:raise RuntimeError(role+' '+label+' failed; inspect first compiler diagnostic')
        manifest[role]={'path':str(binary),'sha256':sha(binary)}
    manifest['trace_header_sha256']=sha(HERE/'startup_trace.hpp')
    # Mathematical state, IMU deskew and point preprocessing files remain byte-identical.
    manifest['unchanged_physics']={str(p):sha(p)for p in [FRONT/'src/IMU_Processing.hpp',FRONT/'src/preprocess.cpp',FRONT/'include/use-ikfom.hpp']if p.is_file()}
    (target/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'directory':str(target),'status':'compiled_private_handoff_binaries'}))


if __name__=='__main__':run()
