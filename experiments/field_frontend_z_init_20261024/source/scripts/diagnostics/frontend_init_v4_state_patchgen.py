#!/usr/bin/env python3
"""State-guard full patch generator. Never builds, runs ROS or edits source.

Emits the base16e97cd default-off frontend patch with pre-commit state validation.
Historical v8 generator/core inputs stay frozen for RED reproduction.
"""
import difflib
import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('frontend_v4_base_patchgen',Path(__file__).with_name('frontend_init_v4_patchgen_v8.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
once=base.once

SETUP=r'''
  state_guard_config_v4_=state_config;
  if(config.enabled)state_guard_config_v4_.validate();
'''

OPEN_TRACE=r'''
  const auto state_trace_path=trace_path+".state_checks.csv";
  const auto state_meta_path=trace_path+".state_checks.metadata.json";
  if(std::ifstream(state_trace_path).good()||std::ifstream(state_meta_path).good())
    throw std::invalid_argument("state guard trace exists; refusing overwrite");
  state_trace_v4_.open(state_trace_path);
  if(!state_trace_v4_)throw std::runtime_error("cannot open state guard trace");
  state_trace_v4_ << std::setprecision(17);
  state_trace_v4_ << "phase,accepted,reason,first_stamp_ns,last_stamp_ns,window_count,mean_checked,last_checked,mean_rx,mean_ry,mean_rz,mean_norm,last_rx,last_ry,last_rz,last_norm,pre_P_symmetry,pre_P_min_pivot,candidate_P_symmetry,candidate_P_min_pivot,raw_ext_det,raw_ext_orthogonality,candidate_present";
  for(const char *prefix:{"before","candidate"}){
    for(const char *vector:{"pos","vel","bg","ba","ext_t","g_W"})
      for(const char *axis:{"x","y","z"})state_trace_v4_ << ',' << prefix << '_' << vector << '_' << axis;
    for(const char *rotation:{"q_WI","ext_q"})
      for(const char *axis:{"x","y","z","w"})state_trace_v4_ << ',' << prefix << '_' << rotation << '_' << axis;
  }
  state_trace_v4_ << '\n';
  std::ofstream state_metadata(state_meta_path);
  if(!state_metadata)throw std::runtime_error("cannot open effective state guard metadata");
  const auto &c=state_guard_config_v4_;
  state_metadata << std::setprecision(17)
    << "{\"schema\":\"frontend_v4_state_guard_v1\",\"enabled_by\":\"quality_gate_enabled only\",\"input_and_window_metadata\":\"adjacent quality CSV.metadata.json\","
    << "\"policy\":\"preserve original fresh candidate; nonidentity KF requires explicit consistent restart; no bias or world repair\","
    << "\"hardware_threshold_acceptance\":\"NOT_MEASURED_ON164\",\"expected_S2_norm_m_s2\":9.809,\"internal_acc_scale_numerator\":9.81,"
    << "\"quaternion_norm_tolerance\":" << c.quaternion_norm_tolerance
    << ",\"rotation_orthogonality_tolerance\":" << c.rotation_orthogonality_tolerance
    << ",\"rotation_determinant_tolerance\":" << c.rotation_determinant_tolerance
    << ",\"initial_rotation_identity_tolerance\":" << c.initial_rotation_identity_tolerance
    << ",\"covariance_symmetry_tolerance\":" << c.covariance_symmetry_tolerance
    << ",\"minimum_covariance_pivot\":" << c.minimum_covariance_pivot
    << ",\"maximum_mean_accel_residual_m_s2\":" << c.maximum_mean_accel_residual_m_s2
    << ",\"maximum_last_accel_residual_m_s2\":" << c.maximum_last_accel_residual_m_s2
    << ",\"gravity_norm_tolerance_m_s2\":" << c.gravity_norm_tolerance_m_s2
    << ",\"candidate_preservation_tolerance\":" << c.candidate_preservation_tolerance << "}\n";
'''

LOGGER=r'''
void ImuProcess::trace_frontend_state_v4(const char *phase,
  const frontend_init_v4::StateGuardReport &report,const frontend_init_v4::Stats &stats,
  const state_ikfom &before,const state_ikfom *candidate)
{
  if(!state_trace_v4_)return;
  state_trace_v4_ << phase << ',' << report.accepted << ',' << report.reason << ','
    << stats.first_stamp_ns << ',' << stats.last_stamp_ns << ',' << stats.count
    << ',' << report.mean_checked << ',' << report.last_checked;
  const auto scalar=[&](double value){state_trace_v4_ << ',';if(std::isfinite(value))state_trace_v4_ << value;};
  for(const auto &r:{report.mean_residual,report.last_residual}){
    for(int k=0;k<3;++k)scalar(r[k]);scalar(r.norm());
  }
  for(double v:{report.pre_covariance_symmetry,report.pre_covariance_minimum_pivot,
    report.candidate_covariance_symmetry,report.candidate_covariance_minimum_pivot,
    report.raw_extrinsic_determinant,report.raw_extrinsic_orthogonality})scalar(v);
  state_trace_v4_ << ',' << static_cast<bool>(candidate);
  const auto write_state=[&](const state_ikfom *state){
    if(!state){for(int k=0;k<26;++k)state_trace_v4_ << ',';return;}
    // Actual invalid inputs retain nan/inf in their state fields; absent computed
    // metrics above are empty, never a fabricated residual zero/PASS.
    for(const auto &v:std::array<V3D,6>{V3D(state->pos),V3D(state->vel),V3D(state->bg),V3D(state->ba),V3D(state->offset_T_L_I),V3D(state->grav)})
      for(int k=0;k<3;++k)state_trace_v4_ << ',' << v[k];
    for(int k=0;k<4;++k)state_trace_v4_ << ',' << state->rot.coeffs()[k];
    for(int k=0;k<4;++k)state_trace_v4_ << ',' << state->offset_R_L_I.coeffs()[k];
  };
  write_state(&before);write_state(candidate);state_trace_v4_ << std::endl;
}
'''

COMMIT=r'''
      const auto stats=init_gate_v4_.stats();
      const V3D ready_mean_acc(stats.mean_acc[0],stats.mean_acc[1],stats.mean_acc[2]);
      const V3D ready_mean_gyr(stats.mean_gyro[0],stats.mean_gyro[1],stats.mean_gyro[2]);
      const auto before=kf_state.get_x();const auto before_P=kf_state.get_P();
      auto state_report=frontend_init_v4::check_state_inputs(before,before_P,
        Lidar_R_wrt_IMU,Lidar_T_wrt_IMU,
        std::array<V3D,4>{V3D(cov_acc_scale),V3D(cov_gyr_scale),V3D(cov_bias_acc),V3D(cov_bias_gyr)},
        state_guard_config_v4_);
      trace_frontend_state_v4("preconditions",state_report,stats,before,nullptr);
      const auto reject_state=[&](){
        auto rejected_stats=stats;rejected_stats.reason=state_report.reason;
        trace_frontend_init_v4("state_check_rejected",rejected_stats,kf_state.get_x(),false);
        init_gate_v4_.invalidate_state(state_report.reason);
      };
      if(!state_report.accepted){reject_state();return;}
      // Keep original fresh numerical construction. All validation occurs on
      // copies before either actual KF state or covariance can change.
      auto initial=before;
      initial.grav=S2(-ready_mean_acc/ready_mean_acc.norm()*G_m_s2);
      initial.bg=ready_mean_gyr;
      initial.offset_T_L_I=Lidar_T_wrt_IMU;initial.offset_R_L_I=Lidar_R_wrt_IMU;
      auto initial_P=before_P;initial_P.setIdentity();
      initial_P(6,6)=initial_P(7,7)=initial_P(8,8)=.00001;
      initial_P(9,9)=initial_P(10,10)=initial_P(11,11)=.00001;
      initial_P(15,15)=initial_P(16,16)=initial_P(17,17)=.0001;
      initial_P(18,18)=initial_P(19,19)=initial_P(20,20)=.001;
      initial_P(21,21)=initial_P(22,22)=.00001;
      const V3D state_last_acc(last_imu_->linear_acceleration.x,last_imu_->linear_acceleration.y,last_imu_->linear_acceleration.z);
      state_report=frontend_init_v4::check_state_candidate(before,initial,initial_P,
        Lidar_R_wrt_IMU,Lidar_T_wrt_IMU,ready_mean_acc,ready_mean_gyr,state_last_acc,state_guard_config_v4_,state_report);
      trace_frontend_state_v4("candidate",state_report,stats,before,&initial);
      if(!state_report.accepted){reject_state();return;}
      mean_acc=ready_mean_acc;mean_gyr=ready_mean_gyr;
      kf_state.change_x(initial);kf_state.change_P(initial_P);
      cov_acc=cov_acc_scale;cov_gyr=cov_gyr_scale;imu_need_init_=false;
      const auto committed_state=kf_state.get_x();
      trace_frontend_state_v4("committed",state_report,stats,before,&committed_state);
'''

PARAMETERS=r'''
        frontend_init_v4::StateGuardConfig v4_state;
        v4_state.quaternion_norm_tolerance=this->declare_parameter<double>("frontend_init_v4.state_quaternion_norm_tolerance",1e-6);
        v4_state.rotation_orthogonality_tolerance=this->declare_parameter<double>("frontend_init_v4.state_rotation_orthogonality_tolerance",1e-6);
        v4_state.rotation_determinant_tolerance=this->declare_parameter<double>("frontend_init_v4.state_rotation_determinant_tolerance",1e-6);
        v4_state.initial_rotation_identity_tolerance=this->declare_parameter<double>("frontend_init_v4.state_initial_rotation_identity_tolerance",1e-8);
        v4_state.covariance_symmetry_tolerance=this->declare_parameter<double>("frontend_init_v4.state_covariance_symmetry_tolerance",1e-10);
        v4_state.minimum_covariance_pivot=this->declare_parameter<double>("frontend_init_v4.state_minimum_covariance_pivot",0.);
        v4_state.maximum_mean_accel_residual_m_s2=this->declare_parameter<double>("frontend_init_v4.state_maximum_mean_accel_residual_m_s2",.05);
        v4_state.maximum_last_accel_residual_m_s2=this->declare_parameter<double>("frontend_init_v4.state_maximum_last_accel_residual_m_s2",.25);
        v4_state.gravity_norm_tolerance_m_s2=this->declare_parameter<double>("frontend_init_v4.state_gravity_norm_tolerance_m_s2",1e-6);
        v4_state.candidate_preservation_tolerance=this->declare_parameter<double>("frontend_init_v4.state_candidate_preservation_tolerance",1e-12);
'''

def modified_sources(source):
    result=base.modified_sources(source)
    path='src/IMU_Processing.hpp';old,text=result[path]
    text=once(text,'#include "use-ikfom.hpp"','#include "use-ikfom.hpp"\n#include "frontend_init_state_v4.hpp"')
    text=once(text,'  if(meas.imu.empty()) {return;};',
        '  // Reset/reused output must never release stale points while gated initialization waits/rejects.\n  if(init_gate_v4_.enabled() && imu_need_init_ && cur_pcl_un_)cur_pcl_un_->clear();\n  if(meas.imu.empty()) {return;};')
    text=once(text,'const frontend_init_v4::InputConfig &, const std::string &trace_path);',
        'const frontend_init_v4::InputConfig &, const std::string &trace_path, const frontend_init_v4::StateGuardConfig &state_config=frontend_init_v4::StateGuardConfig{});')
    text=once(text,'  std::ofstream init_trace_v4_;','''  std::ofstream init_trace_v4_;
  frontend_init_v4::StateGuardConfig state_guard_config_v4_;
  std::ofstream state_trace_v4_;
  void trace_frontend_state_v4(const char *, const frontend_init_v4::StateGuardReport &, const frontend_init_v4::Stats &, const state_ikfom &, const state_ikfom *);''')
    text=once(text,'const frontend_init_v4::InputConfig &input_config, const std::string &trace_path)',
        'const frontend_init_v4::InputConfig &input_config, const std::string &trace_path, const frontend_init_v4::StateGuardConfig &state_config)')
    text=once(text,'  init_gate_v4_=frontend_init_v4::Gate(config);',SETUP+'\n  init_gate_v4_=frontend_init_v4::Gate(config);')
    text=once(text,'  init_trace_v4_.open(trace_path);',OPEN_TRACE+'\n  init_trace_v4_.open(trace_path);')
    text=once(text,'void ImuProcess::trace_frontend_init_v4(',LOGGER+'\nvoid ImuProcess::trace_frontend_init_v4(')
    start=text.index('      const auto &stats=init_gate_v4_.stats();')
    end=text.index('      // Establish the real deskew/prediction boundary',start)
    text=text[:start]+COMMIT+'\n'+text[end:];result[path]=(old,text)
    path='src/laserMapping.cpp';old,text=result[path]
    needle='        p_imu->set_frontend_init_v4(v4_gate,frontend_input_config_v4,v4_trace);'
    text=once(text,needle,PARAMETERS+'\n'+needle.replace('v4_trace);','v4_trace,v4_state);'));result[path]=(old,text)
    old,header=result['include/frontend_init_v4.hpp']
    header=once(header,'  void invalidate_input(){if(enabled())reject("input_interface_rejection");}',
        '  void invalidate_input(){if(enabled())reject("input_interface_rejection");}\n  void invalidate_state(const std::string &reason){if(enabled())reject(reason);}')
    result['include/frontend_init_v4.hpp']=(old,header)
    result['include/frontend_init_state_v4.hpp']=('',(ROOT/'patches/fast_lio_v4/frontend_init_state_v4.hpp').read_text())
    return result

def render_patch(source):
    chunks=[]
    for name,(old,new) in modified_sources(Path(source)).items():
        chunks.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name if old else '/dev/null',tofile='b/'+name))
    return ''.join(chunks)

if __name__=='__main__':print(render_patch(ROOT/'src/FAST_LIO_Hesai'),end='')
