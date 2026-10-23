"""Generate a base-anchored root-repository patch; never edit submodule files."""
import difflib
from pathlib import Path


def once(text, old, new):
    if text.count(old)!=1:
        raise ValueError('unexpected FAST-LIO base context: '+old[:80])
    return text.replace(old,new,1)


IMU_METHODS=r'''
void ImuProcess::set_frontend_init_v4(const frontend_init_v4::GateConfig &config,
  const frontend_init_v4::InputConfig &input_config, const std::string &trace_path)
{
  if(config.enabled && !input_config.enabled)
    throw std::invalid_argument("v4 quality gate requires explicit SI input contract");
  init_gate_v4_=frontend_init_v4::Gate(config);
  if(!config.enabled)return;
  if(trace_path.empty() || std::ifstream(trace_path).good() || std::ifstream(trace_path+".metadata.json").good())
    throw std::invalid_argument("v4 gate trace must be a new file; refusing overwrite");
  std::ofstream effective_config_v4(trace_path+".metadata.json");
  if(!effective_config_v4)throw std::runtime_error("cannot open effective v4 configuration sidecar");
  auto quoted=[](const std::string &value){std::ostringstream out;out << '"';
    for(unsigned char c:value){if(c=='"'||c=='\\')out << '\\' << c;
      else if(c<32)out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(c);
      else out << c;}out << '"';return out.str();};
  effective_config_v4 << std::setprecision(17)
    << "{\"schema\":\"frontend_init_v4_effective_config_v1\",\"base_commit\":\"16e97cdcc4260b9ba6241518a19ad8384224ba48\","
    << "\"frames\":\"W=camera_init,I=body,L=LiDAR; no axis remap/world leveling\",\"source_provenance\":" << quoted(input_config.source_provenance)
    << ",\"input\":{\"enabled\":" << (input_config.enabled?"true":"false")
    << ",\"gyro_unit\":" << quoted(input_config.gyro_unit) << ",\"accel_unit\":" << quoted(input_config.accel_unit)
    << ",\"frame_id\":" << quoted(input_config.frame_id) << ",\"gyro_evidence\":" << quoted(input_config.gyro_evidence)
    << ",\"accel_evidence\":" << quoted(input_config.accel_evidence) << ",\"time_evidence\":" << quoted(input_config.time_evidence)
    << ",\"imu_minus_lidar_offset_ns\":" << input_config.imu_minus_lidar_offset_ns
    << ",\"offset_rule\":\"aligned=raw-offset exactly once; no self-sync\",\"internal_acc_normalization_retained\":true},\"gate\":{\"enabled\":true"
    << ",\"minimum_samples\":" << config.minimum_samples << ",\"maximum_samples\":" << config.maximum_samples
    << ",\"minimum_duration_s\":" << config.minimum_duration_s << ",\"maximum_gap_s\":" << config.maximum_gap_s
    << ",\"maximum_gyro_norm_rad_s\":" << config.maximum_gyro_norm_rad_s
    << ",\"maximum_mean_gyro_norm_rad_s\":" << config.maximum_mean_gyro_norm_rad_s
    << ",\"gyro_variance_limit\":" << config.gyro_variance_limit << ",\"accel_variance_limit\":" << config.accel_variance_limit
    << ",\"minimum_accel_norm_m_s2\":" << config.minimum_accel_norm_m_s2 << ",\"maximum_accel_norm_m_s2\":" << config.maximum_accel_norm_m_s2
    << ",\"maximum_direction_change_rad\":" << config.maximum_direction_change_rad
    << "},\"warning\":\"Explicit caller declarations are not independent device-unit certification; IMU-only stationarity proxy, not calibration\"}\n";
  init_trace_v4_.open(trace_path);
  if(!init_trace_v4_)throw std::runtime_error("cannot open v4 initialization trace");
  init_trace_v4_ << std::setprecision(17);
  init_trace_v4_ << "stage,reason,first_stamp_ns,last_stamp_ns,accepted_samples,span_s,max_gap_s,max_direction_rad,reset_count,mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,var_ax,var_ay,var_az,var_gx,var_gy,var_gz,state_initialized,g_Wx,g_Wy,g_Wz,bgx,bgy,bgz,bax,bay,baz,ext_tx,ext_ty,ext_tz,ext_qx,ext_qy,ext_qz,ext_qw,q_WIx,q_WIy,q_WIz,q_WIw,window_member_stamps_ns\n";
}

void ImuProcess::trace_frontend_init_v4(const char *stage,
  const frontend_init_v4::Stats &stats, const state_ikfom &state, bool initialized)
{
  if(!init_trace_v4_)return;
  init_trace_v4_ << stage << ',' << stats.reason << ',' << stats.first_stamp_ns << ','
    << stats.last_stamp_ns << ',' << stats.count << ',' << stats.span_s << ',' << stats.maximum_gap_s
    << ',' << stats.maximum_direction_change_rad << ',' << stats.reset_count;
  for(const auto &v:{stats.mean_acc,stats.mean_gyro,stats.variance_acc,stats.variance_gyro})
    for(double value:v)init_trace_v4_ << ',' << value;
  init_trace_v4_ << ',' << initialized;
  for(int k=0;k<3;++k)init_trace_v4_ << ',' << state.grav[k];
  for(const auto &v:std::array<V3D,3>{V3D(state.bg),V3D(state.ba),V3D(state.offset_T_L_I)})
    for(int k=0;k<3;++k)init_trace_v4_ << ',' << v[k];
  const auto eq=state.offset_R_L_I.coeffs(),q=state.rot.coeffs();
  for(int k=0;k<4;++k)init_trace_v4_ << ',' << eq[k];
  for(int k=0;k<4;++k)init_trace_v4_ << ',' << q[k];
  init_trace_v4_ << ',';
  if(std::string(stage)!="sample_rejected")
    for(auto stamp:init_gate_v4_.member_stamps_ns())init_trace_v4_ << stamp << ';';
  init_trace_v4_ << std::endl;
}
'''

GATED_PROCESS=r'''
    if(init_gate_v4_.enabled())
    {
      if(b_first_frame_){first_lidar_time=meas.lidar_beg_time;b_first_frame_=false;}
      const auto consume_rejection=[&](){
        if(!input_rejections_v4_.exchange(0))return false;
        init_gate_v4_.invalidate_input();
        trace_frontend_init_v4("input_group_invalidated",init_gate_v4_.last_rejected_stats(),kf_state.get_x(),false);
        return true;
      };
      // A callback-rejected suffix must not turn the valid prefix into a
      // supposedly stationary group. Discard the whole affected group.
      if(consume_rejection()){last_imu_=meas.imu.back();return;}
      // Evaluate every sample in this synchronized group. Ready is not latched
      // mid-group: later motion/invalid timing cancels an earlier valid prefix.
      for(const auto &imu:meas.imu)
      {
        frontend_init_v4::Sample input;
        input.stamp_ns=static_cast<std::int64_t>(imu->header.stamp.sec)*1000000000LL+imu->header.stamp.nanosec;
        input.acc={imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z};
        input.gyro={imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z};
        input.frame_id=imu->header.frame_id;input.gyro_unit="rad/s";input.accel_unit="m/s^2";
        const auto resets=init_gate_v4_.stats().reset_count;
        init_gate_v4_.feed(input);
        if(init_gate_v4_.stats().reset_count!=resets)
          trace_frontend_init_v4("sample_rejected",init_gate_v4_.last_rejected_stats(),kf_state.get_x(),false);
      }
      last_imu_=meas.imu.back();
      if(consume_rejection())return;
      trace_frontend_init_v4("group_evaluated",init_gate_v4_.stats(),kf_state.get_x(),false);
      if(!init_gate_v4_.ready())return;
      const auto &stats=init_gate_v4_.stats();
      mean_acc=V3D(stats.mean_acc[0],stats.mean_acc[1],stats.mean_acc[2]);
      mean_gyr=V3D(stats.mean_gyro[0],stats.mean_gyro[1],stats.mean_gyro[2]);
      // Actual estimator assignment from the exact accepted window, NOT a
      // parallel helper statistic or a fabricated legacy internal N.
      auto initial=kf_state.get_x();
      initial.grav=S2(-mean_acc/mean_acc.norm()*G_m_s2);
      initial.bg=mean_gyr;
      initial.offset_T_L_I=Lidar_T_wrt_IMU;initial.offset_R_L_I=Lidar_R_wrt_IMU;
      kf_state.change_x(initial);
      auto initial_P=kf_state.get_P();initial_P.setIdentity();
      initial_P(6,6)=initial_P(7,7)=initial_P(8,8)=.00001;
      initial_P(9,9)=initial_P(10,10)=initial_P(11,11)=.00001;
      initial_P(15,15)=initial_P(16,16)=initial_P(17,17)=.0001;
      initial_P(18,18)=initial_P(19,19)=initial_P(20,20)=.001;
      initial_P(21,21)=initial_P(22,22)=.00001;kf_state.change_P(initial_P);
      cov_acc=cov_acc_scale;cov_gyr=cov_gyr_scale;imu_need_init_=false;
      // Establish the real deskew/prediction boundary at the accepted scan.
      last_lidar_end_time_=meas.lidar_end_time;
      angvel_last=V3D(last_imu_->angular_velocity.x,last_imu_->angular_velocity.y,last_imu_->angular_velocity.z)-initial.bg;
      const V3D last_acc(last_imu_->linear_acceleration.x,last_imu_->linear_acceleration.y,last_imu_->linear_acceleration.z);
      acc_s_last=initial.rot*(last_acc*G_m_s2/mean_acc.norm()-initial.ba)+V3D(initial.grav);
      trace_frontend_init_v4("initialized",stats,kf_state.get_x(),true);
      std::cout << "IMU Initial Done [v4 accepted_samples=" << stats.count << " span_s=" << stats.span_s << "]" << std::endl;
      return;
    }
'''

NODE_CONFIG=r'''
        // Default-off experimental contract. No numeric inference of corrected
        // driver units; all claims require explicit source evidence.
        frontend_input_config_v4.enabled=this->declare_parameter<bool>("frontend_init_v4.input_interface_enabled",false);
        frontend_input_config_v4.gyro_unit=this->declare_parameter<string>("frontend_init_v4.input_gyro_unit","unknown");
        frontend_input_config_v4.accel_unit=this->declare_parameter<string>("frontend_init_v4.input_accel_unit","unknown");
        frontend_input_config_v4.frame_id=this->declare_parameter<string>("frontend_init_v4.input_imu_frame","unknown");
        frontend_input_config_v4.gyro_evidence=this->declare_parameter<string>("frontend_init_v4.gyro_unit_evidence","");
        frontend_input_config_v4.accel_evidence=this->declare_parameter<string>("frontend_init_v4.accel_unit_evidence","");
        frontend_input_config_v4.time_evidence=this->declare_parameter<string>("frontend_init_v4.time_offset_evidence","");
        frontend_input_config_v4.source_provenance=this->declare_parameter<string>("frontend_init_v4.source_provenance","unknown");
        if(frontend_input_config_v4.enabled)
        {
          if(!std::isfinite(time_diff_lidar_to_imu)||std::abs(time_diff_lidar_to_imu)>1e6)
            throw std::invalid_argument("invalid v4 fixed offset range");
          frontend_input_config_v4.imu_minus_lidar_offset_ns=std::llround(time_diff_lidar_to_imu*1e9);
          if(frontend_input_config_v4.source_provenance!="synthetic"&&frontend_input_config_v4.source_provenance!="real")
            throw std::invalid_argument("v4 source_provenance must explicitly be synthetic or real");
          frontend_input_adapter_v4.reset(new frontend_init_v4::InputAdapter(frontend_input_config_v4));
          const string legacy_mode=frontend_input_config_v4.gyro_unit=="rad/s"?"rad":"deg";
          if(imu_gyr_unit_mode!=legacy_mode||time_sync_en)
            throw std::invalid_argument("v4 explicit gyro mode mismatch or second time synchronization enabled");
          // Converted messages are SI; never run the legacy conversion again.
          imu_gyr_is_deg=false;imu_gyr_unit_resolved=true;
          const auto path=this->declare_parameter<string>("frontend_init_v4.input_trace_path",DEBUG_FILE_DIR("input_interface_v4.csv"));
          if(std::ifstream(path).good())throw std::invalid_argument("v4 input trace exists; refusing overwrite");
          frontend_input_trace_v4.open(path);
          if(!frontend_input_trace_v4)throw std::runtime_error("cannot open v4 input trace");
          frontend_input_trace_v4 << std::setprecision(17)
            << "status,raw_stamp_ns,aligned_stamp_ns,input_frame,source_gyro_unit,source_accel_unit,conversion_count,time_adjustment_count,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,si_ax,si_ay,si_az,si_gx,si_gy,si_gz\n";
          RCLCPP_INFO(this->get_logger(),"v4 source gyro=%s [%s] acc=%s [%s] offset=t_IMU_raw-t_LiDAR %.9f s applied ONCE by subtraction [%s]; axes unchanged; internal9.81/mean_acc scaling remains",
            frontend_input_config_v4.gyro_unit.c_str(),frontend_input_config_v4.gyro_evidence.c_str(),
            frontend_input_config_v4.accel_unit.c_str(),frontend_input_config_v4.accel_evidence.c_str(),
            time_diff_lidar_to_imu,frontend_input_config_v4.time_evidence.c_str());
        }
        frontend_init_v4::GateConfig v4_gate;
        v4_gate.enabled=this->declare_parameter<bool>("frontend_init_v4.quality_gate_enabled",false);
        const int v4_min_samples=this->declare_parameter<int>("frontend_init_v4.minimum_samples",200);
        if(v4_min_samples<2)throw std::invalid_argument("v4 minimum_samples must be>=2");
        v4_gate.minimum_samples=static_cast<std::size_t>(v4_min_samples);
        v4_gate.minimum_duration_s=this->declare_parameter<double>("frontend_init_v4.minimum_duration_s",1.);
        v4_gate.maximum_gap_s=this->declare_parameter<double>("frontend_init_v4.maximum_gap_s",.02);
        v4_gate.maximum_gyro_norm_rad_s=this->declare_parameter<double>("frontend_init_v4.maximum_gyro_norm_rad_s",.05);
        v4_gate.maximum_mean_gyro_norm_rad_s=this->declare_parameter<double>("frontend_init_v4.maximum_mean_gyro_norm_rad_s",.02);
        v4_gate.gyro_variance_limit=this->declare_parameter<double>("frontend_init_v4.gyro_variance_limit",1e-5);
        v4_gate.accel_variance_limit=this->declare_parameter<double>("frontend_init_v4.accel_variance_limit",.04);
        v4_gate.minimum_accel_norm_m_s2=this->declare_parameter<double>("frontend_init_v4.minimum_accel_norm_m_s2",9.);
        v4_gate.maximum_accel_norm_m_s2=this->declare_parameter<double>("frontend_init_v4.maximum_accel_norm_m_s2",10.6);
        v4_gate.maximum_direction_change_rad=this->declare_parameter<double>("frontend_init_v4.maximum_direction_change_rad",.02);
        const auto v4_trace=this->declare_parameter<string>("frontend_init_v4.quality_trace_path",DEBUG_FILE_DIR("init_quality_v4.csv"));
        p_imu->set_frontend_init_v4(v4_gate,frontend_input_config_v4,v4_trace);
'''

CALLBACK=r'''
    if(frontend_input_config_v4.enabled)
    {
      frontend_init_v4::Sample raw;
      raw.stamp_ns=static_cast<std::int64_t>(msg_in->header.stamp.sec)*1000000000LL+msg_in->header.stamp.nanosec;
      raw.frame_id=msg_in->header.frame_id;raw.gyro_unit=frontend_input_config_v4.gyro_unit;raw.accel_unit=frontend_input_config_v4.accel_unit;
      raw.acc={msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z};
      raw.gyro={msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z};
      frontend_init_v4::Sample si;
      try{si=frontend_input_adapter_v4->adapt(raw);}
      catch(const std::invalid_argument &error)
      {
        p_imu->notify_frontend_input_rejection_v4();
        frontend_input_trace_v4 << "rejected," << raw.stamp_ns << ",," << raw.frame_id << ',' << raw.gyro_unit << ',' << raw.accel_unit << ",0,0";
        for(const auto &v:{raw.acc,raw.gyro})for(double x:v)frontend_input_trace_v4 << ',' << x;
        frontend_input_trace_v4 << ",,,,,," << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("laserMapping"),"v4 IMU rejected: %s",error.what());return;
      }
      if(si.stamp_ns/1000000000LL>std::numeric_limits<std::int32_t>::max())
      {p_imu->notify_frontend_input_rejection_v4();RCLCPP_ERROR(rclcpp::get_logger("laserMapping"),"v4 aligned IMU stamp exceeds ROS sec range");return;}
      msg->header.stamp.sec=static_cast<std::int32_t>(si.stamp_ns/1000000000LL);
      msg->header.stamp.nanosec=static_cast<std::uint32_t>(si.stamp_ns%1000000000LL);
      msg->linear_acceleration.x=si.acc[0];msg->linear_acceleration.y=si.acc[1];msg->linear_acceleration.z=si.acc[2];
      msg->angular_velocity.x=si.gyro[0];msg->angular_velocity.y=si.gyro[1];msg->angular_velocity.z=si.gyro[2];
      frontend_input_trace_v4 << "adapted," << raw.stamp_ns << ',' << si.stamp_ns << ',' << raw.frame_id << ','
        << raw.gyro_unit << ',' << raw.accel_unit << ',' << si.conversion_count << ',' << si.time_adjustment_count;
      for(const auto &v:{raw.acc,raw.gyro,si.acc,si.gyro})for(double x:v)frontend_input_trace_v4 << ',' << x;
      frontend_input_trace_v4 << std::endl;
      mtx_buffer.lock();
    }
    else
    {
LEGACY_CALLBACK_BLOCK
    }
'''


def modified_sources(source):
    result={}
    path='src/IMU_Processing.hpp';old=(source/path).read_text();text=old
    text=once(text,'#include <cmath>','#include <cmath>\n#include <atomic>\n#include <sstream>\n#include <iomanip>\n#include "frontend_init_v4.hpp"')
    text=once(text,'  Eigen::Matrix<double, 12, 12> Q;', '''  void set_frontend_init_v4(const frontend_init_v4::GateConfig &, const frontend_init_v4::InputConfig &, const std::string &trace_path);
  void notify_frontend_input_rejection_v4(){if(init_gate_v4_.enabled())++input_rejections_v4_;}
  bool frontend_init_v4_initialized()const{return !imu_need_init_;}
  const frontend_init_v4::Stats &frontend_init_v4_stats()const{return init_gate_v4_.stats();}
  Eigen::Matrix<double, 12, 12> Q;''')
    text=once(text,' private:\n',''' private:
  frontend_init_v4::Gate init_gate_v4_;
  std::atomic<std::size_t> input_rejections_v4_{0};
  std::ofstream init_trace_v4_;
  void trace_frontend_init_v4(const char *, const frontend_init_v4::Stats &, const state_ikfom &, bool);
''')
    text=once(text,'  v_imu_.clear();','  init_gate_v4_.reset();\n  v_imu_.clear();')
    text=once(text,'void ImuProcess::IMU_init(',IMU_METHODS+'\nvoid ImuProcess::IMU_init(')
    text=once(text,'    /// The very first lidar frame',GATED_PROCESS+'\n    /// The very first lidar frame')
    result[path]=(old,text)
    path='src/laserMapping.cpp';old=(source/path).read_text();text=old
    text=once(text,'vector<double> imu_acc_norm_samples;', '''vector<double> imu_acc_norm_samples;
frontend_init_v4::InputConfig frontend_input_config_v4;
std::unique_ptr<frontend_init_v4::InputAdapter> frontend_input_adapter_v4;
std::ofstream frontend_input_trace_v4;''')
    start=text.index('    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);')
    end=text.index('    double timestamp = get_time_sec(msg->header.stamp);',start)
    oldblock=text[start:end]
    text=text[:start]+CALLBACK.replace('LEGACY_CALLBACK_BLOCK',oldblock)+text[end:]
    text=once(text,'        std::cerr << "imu loop back, clear buffer" << std::endl;',
        '        if(frontend_input_config_v4.enabled)p_imu->notify_frontend_input_rejection_v4();\n        std::cerr << "imu loop back, clear buffer" << std::endl;')
    anchor='        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);'
    text=once(text,anchor,NODE_CONFIG+'\n'+anchor)
    result[path]=(old,text)
    path='CMakeLists.txt';old=(source/path).read_text();text=old
    anchor='# ---------------- Install --------------- #'
    extra='''# Explicit experiment-only offline Process harness; no ROS init/publisher.
option(FRONTEND_INIT_V4_BUILD_HARNESS "Build offline initialization mechanism harness" OFF)
if(FRONTEND_INIT_V4_BUILD_HARNESS)
  if(NOT EXISTS "${FRONTEND_INIT_V4_HARNESS_SOURCE}")
    message(FATAL_ERROR "Provide FRONTEND_INIT_V4_HARNESS_SOURCE")
  endif()
  add_executable(frontend_init_v4_harness "${FRONTEND_INIT_V4_HARNESS_SOURCE}" src/preprocess.cpp)
  target_include_directories(frontend_init_v4_harness PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include" ${PCL_INCLUDE_DIRS})
  target_link_libraries(frontend_init_v4_harness ${PCL_LIBRARIES} Eigen3::Eigen ${cpp_typesupport_target})
  ament_target_dependencies(frontend_init_v4_harness ${dependencies})
endif()

'''
    text=once(text,anchor,extra+anchor);result[path]=(old,text)
    header=Path(__file__).resolve().parents[2]/'patches/fast_lio_v4/frontend_init_v4_core_v8.hpp'
    result['include/frontend_init_v4.hpp']=('',header.read_text())
    return result


def render_patch(source):
    chunks=[]
    for name,(old,new) in modified_sources(Path(source)).items():
        chunks.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
            fromfile='a/'+name if old else '/dev/null',tofile='b/'+name))
    return ''.join(chunks)


if __name__=='__main__':
    import sys
    print(render_patch(Path(__file__).resolve().parents[2]/'src/FAST_LIO_Hesai'),end='')
