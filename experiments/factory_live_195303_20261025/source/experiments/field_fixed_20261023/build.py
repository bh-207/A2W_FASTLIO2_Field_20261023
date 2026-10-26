#!/usr/bin/env python3
"""Build only three changed translation units; original filter objects stay read-only."""
import difflib
import importlib.util
import json
from pathlib import Path
import shlex
import subprocess
import sys

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
OUT = WS/'captures/field_fixed_20261023_prepare_001'
OLD = WS/'experiments/offline_mapping_optimization_20261022/replay'
spec = importlib.util.spec_from_file_location('original_b_builder', OLD/'prepare_binaries.py')
b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)


def generate(target):
    target.mkdir(); (target/'source').mkdir()
    header = (OLD/'startup_trace.hpp').read_text()
    begin = header.index('  const char*domain=', header.index('inline void start('))
    end = header.index('  trace=std::make_unique', begin)
    header = header[:begin] + '''  const char* mode=std::getenv("A2W_FIELD_MODE"),*directory=std::getenv("A2W_HANDOFF_DIRECTORY");
  const char* domain=std::getenv("ROS_DOMAIN_ID"),*local=std::getenv("ROS_LOCALHOST_ONLY"),*rmw=std::getenv("RMW_IMPLEMENTATION");
  bool real=mode&&std::string(mode)=="live"&&domain&&std::string(domain)=="0"&&local&&std::string(local)=="0";
  bool test=mode&&std::string(mode)=="offline-test"&&domain&&std::string(domain)=="87"&&local&&std::string(local)=="1";
  if((!real&&!test)||!directory||!rmw||std::string(rmw)!="rmw_cyclonedds_cpp")throw std::invalid_argument("explicit field session/domain required");
  auto path=std::filesystem::canonical(directory);
  if(path.string().find("/home/bh/A2W_FastLio2_controlled/captures/field_fixed_")!=0)throw std::invalid_argument("field trace outside controlled sessions");
''' + header[end:]
    header=header.replace('template<class Pub>inline void require_two', 'template<class Pub>inline void require_two')
    (target/'source/startup_trace.hpp').write_text(header)
    front = WS/'.controlled_mapping/frontend/source'
    jobs = [('frontend', front/'src/laserMapping.cpp', WS/'.controlled_mapping/frontend/build', 'fastlio_mapping', b.frontend),
            ('ingress', WS/'src/a2w_fastlio_mapping/src/mapping_ingress_node.cpp', b.BACK_BUILD, 'mapping_ingress_node', b.ingress),
            ('backend', WS/'src/a2w_fastlio_mapping/src/mapping_backend_node.cpp', b.BACK_BUILD, 'mapping_backend_node', b.backend)]
    manifest = dict(schema='field_fixed_b_build_v1', baseline='v4 + B endpoint instrumentation; no P/R/J code', commands=[], binaries={}, sources={}, unchanged_physics={})
    for role, source, build, name, adapt in jobs:
        original = source.read_text(); changed = adapt(original)
        changed = changed.replace('offline_mapping_save_observer_*', 'field_capture_observer_*')
        changed = changed.replace('}))handoff_timer_->cancel();', '} ) && std::filesystem::exists(offline_handoff::trace->directory()/"live_release.json"))handoff_timer_->cancel();')
        if role == 'frontend':
            changed = changed.replace('"/Odometry",{"mapping_ingress_node","field_capture_observer_*"}', '"/Odometry",{"mapping_ingress_node","field_capture_observer_*","rosbag2_recorder"}')
            changed = changed.replace('offline_handoff::require_two(pubOdomAftMapped_);', 'if(pubOdomAftMapped_->get_subscription_count()!=3)throw std::runtime_error("Odometry ingress/observer/recorder edge disconnected");')
            cloud = '        sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);'
            imu = '        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);'
            changed = b.once(changed, cloud, '')
            changed = b.once(changed, imu, '')
            marker = '        RCLCPP_INFO(this->get_logger(), "Node init finished.");'
            gate = '''        // Live source continues recording; do not create sensor subscriptions until all output edges match.
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(45);
        bool field_released=false;
        while(rclcpp::ok()&&std::chrono::steady_clock::now()<deadline) {
          bool matched=offline_handoff::report_ready("frontend",{
            offline_handoff::edge(*this,pubOdomAftMapped_,"/Odometry",{"mapping_ingress_node","field_capture_observer_*","rosbag2_recorder"}),
            offline_handoff::edge(*this,pubLaserCloudFull_body_,"/cloud_registered_body",{"mapping_ingress_node","field_capture_observer_*"})});
          if(matched&&std::filesystem::exists(offline_handoff::trace->directory()/"live_release.json")){field_released=true;break;}
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if(!field_released)throw std::runtime_error("live startup endpoint handshake timeout; no sensor intake started");
        offline_handoff::event("live_sensor_intake_release",0);
        const int field_depth=this->declare_parameter<int>("field_capture.imu_depth",100);
        if(field_depth!=100)throw std::invalid_argument("field IMU depth must be 100");
''' + cloud + '\n' + imu.replace('rclcpp::SensorDataQoS(), imu_cbk', 'rclcpp::SensorDataQoS().keep_last(field_depth), imu_cbk') + '\n'
            changed = b.once(changed, marker, gate+marker)
            # A diagnostic assertion, never state clamping or a different filter update.
            marker = '            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;'
            check = '''            if(!extrinsic_est_en && p_imu->frontend_init_v4_initialized()) {
              const auto field_x=kf.get_x();
              if((field_x.offset_T_L_I-Lidar_T_wrt_IMU).norm()>1e-10 ||
                 (field_x.offset_R_L_I.toRotationMatrix()-Lidar_R_wrt_IMU).norm()>1e-10)
                throw std::runtime_error("fixed extrinsic state changed; refusing continued mapping");
            }
'''
            # This anchor occurs once after IMU processing and once after the laser update.
            if changed.count(marker)!=2: raise ValueError('state assertion anchors changed')
            changed = changed.replace(marker, check+marker)
        generated = target/'source'/source.name; generated.write_text(changed)
        (target/'source'/(role+'.patch')).write_text(''.join(difflib.unified_diff(original.splitlines(True), changed.splitlines(True), fromfile=str(source), tofile=str(generated))))
        options = b.flags(build/f'CMakeFiles/{name}.dir/flags.make')
        obj = target/(role+'.o'); binary = target/name
        compile_ = ['/usr/bin/c++', *options, '-std=c++17', '-I'+str(target/'source'), '-I'+str(source.parent), '-c', str(generated), '-o', str(obj)]
        link = shlex.split((build/f'CMakeFiles/{name}.dir/link.txt').read_text())
        old_obj = f'CMakeFiles/{name}.dir/src/{source.name}.o'
        if link.count(old_obj)!=1: raise ValueError('link contract changed')
        link[link.index(old_obj)] = str(obj); link[link.index('-o')+1] = str(binary)
        manifest['sources'][str(source)] = dict(sha256=b.sha(source), generated=str(generated), generated_sha256=b.sha(generated))
        for label, argv in [('compile', compile_), ('link', link)]:
            with (target/(role+'_'+label+'.log')).open('x') as log:
                result = subprocess.run(argv, cwd=build, stdout=log, stderr=subprocess.STDOUT)
            manifest['commands'].append(dict(role=role, stage=label, argv=argv, cwd=str(build), exit_code=result.returncode))
            (target/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            if result.returncode: raise RuntimeError(role+' '+label+' failed')
        manifest['binaries'][role] = dict(path=str(binary), sha256=b.sha(binary))
    for relative in ['src/IMU_Processing.hpp', 'src/preprocess.cpp', 'include/use-ikfom.hpp', 'include/IKFoM_toolkit/esekfom/esekfom.hpp']:
        p = front/relative
        manifest['unchanged_physics'][str(p)] = b.sha(p)
    manifest['trace_header_sha256'] = b.sha(target/'source/startup_trace.hpp')
    (target/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(build=str(target), valid=True)))


if __name__=='__main__':
    name = sys.argv[1] if len(sys.argv)>1 else 'build_001'
    if not name.startswith('build_') or not name[6:].isdigit(): raise ValueError('versioned build name required')
    generate(OUT/name)
