// Actual state/output contract tests. Old-v8 named REDs are archived in
// state_RED_v1 and state_RED_v2; current-version results are versioned separately.
// Actual ImuProcess::Process / actual ESKF; no ROS init, node or publisher.
#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include <array>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace fs=std::filesystem;
using Filter=esekfom::esekf<state_ikfom,12,input_ikfom>;
void no_measurement(state_ikfom &,esekfom::dyn_share_datastruct<double> &data){data.valid=false;}
void check(bool ok,const std::string &reason){if(!ok)throw std::runtime_error(reason);}
std::vector<double> flatten(const state_ikfom &state){
  std::vector<double> out;
  for(const auto &v:std::array<V3D,6>{V3D(state.pos),V3D(state.vel),V3D(state.bg),V3D(state.ba),V3D(state.offset_T_L_I),V3D(state.grav)})
    for(int i=0;i<3;++i)out.push_back(v[i]);
  for(int i=0;i<4;++i){out.push_back(state.rot.coeffs()[i]);out.push_back(state.offset_R_L_I.coeffs()[i]);}
  return out;
}
bool same(double a,double b){return a==b||(std::isnan(a)&&std::isnan(b));}
bool same_state(const std::vector<double> &a,const std::vector<double> &b){
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)if(!same(a[i],b[i]))return false;
  return true;
}
bool same_cov(const Filter::cov &a,const Filter::cov &b){
  for(int i=0;i<a.rows();++i)for(int j=0;j<a.cols();++j)if(!same(a(i,j),b(i,j)))return false;
  return true;
}
template<class Signature>struct FourthConfig{using type=void;};
template<class Result,class Owner,class A,class B,class C,class D>
struct FourthConfig<Result(Owner::*)(A,B,C,D)>{using type=std::remove_cv_t<std::remove_reference_t<D>>;};
template<class Process>
void configure(Process &process,const frontend_init_v4::GateConfig &gate,const frontend_init_v4::InputConfig &input,
  const std::string &path,const std::string &name){
  using Config=typename FourthConfig<decltype(&Process::set_frontend_init_v4)>::type;
  if constexpr(std::is_void_v<Config>)process.set_frontend_init_v4(gate,input,path);
  else {Config config;
    // Deliberately loose SYNTHETIC thresholds test configurability, not hardware acceptance.
    if(name=="configured_mean_tolerance"){
      config.maximum_mean_accel_residual_m_s2=2.;
      // Constant preserved ba affects both residuals; keep the last-sample
      // threshold nonbinding while checking the configured mean threshold.
      config.maximum_last_accel_residual_m_s2=2.;
    }
    if(name=="configured_last_tolerance")config.maximum_last_accel_residual_m_s2=1.;
    process.set_frontend_init_v4(gate,input,path,config);
  }
}

int main(int argc,char **argv){try{
  check(argc==3,"CASE NEW_OUTPUT");const std::string name=argv[1];const fs::path output=argv[2];
  check(!fs::exists(output),"refuse existing output");fs::create_directories(output);
  const bool configured=name=="configured_mean_tolerance"||name=="configured_last_tolerance";
  const bool fresh=name=="fresh_unchanged"||name=="stale_output_commit_scan"||configured,off=name=="off_unchanged"||name=="off_reused_rotated_legacy"||name=="off_sentinel_legacy";
  const bool sentinel=name=="stale_output_waiting"||name=="stale_output_rejected"||name=="stale_output_commit_scan"||name=="off_sentinel_legacy";
  const bool known=fresh||off||sentinel||name=="reused_rotated_state"||name=="reused_yaw_state"||name=="preserved_ba_residual"||name=="last_accel_residual"||
    name=="nonfinite_position"||name=="nonfinite_ba"||name=="invalid_state_quaternion"||
    name=="nonfinite_ext_translation"||name=="nonproper_ext_scale"||name=="nonproper_ext_reflection"||
    name=="nonfinite_covariance"||name=="asymmetric_covariance"||name=="negative_covariance"||name=="singular_covariance"||
    name=="nonfinite_acc_noise"||name=="negative_gyro_noise"||name=="negative_bias_noise"||name=="negative_gyro_bias_noise";
  check(known,"unknown state contract test");
  frontend_init_v4::GateConfig gate;gate.enabled=!off;
  frontend_init_v4::InputConfig input;input.enabled=true;input.gyro_unit="rad/s";input.accel_unit="m/s^2";
  input.frame_id="imu";input.source_provenance="synthetic";
  input.gyro_evidence=input.accel_evidence=input.time_evidence="hand-specified static state-consistency fixture";
  ImuProcess process;configure(process,gate,input,(output/"quality.csv").string(),name);
  V3D ext(.06,-.02,.04);M3D rotation=Eye3d;
  const double nan=std::numeric_limits<double>::quiet_NaN();
  if(name=="nonfinite_ext_translation")ext[0]=nan;
  if(name=="nonproper_ext_scale")rotation(0,0)=2.;
  if(name=="nonproper_ext_reflection")rotation(2,2)=-1.;
  process.set_extrinsic(ext,rotation);
  process.set_acc_cov(V3D(name=="nonfinite_acc_noise"?nan:.1,.1,.1));
  process.set_gyr_cov(V3D(name=="negative_gyro_noise"?-.1:.1,.1,.1));
  process.set_acc_bias_cov(V3D(name=="negative_bias_noise"?-.0001:.0001,.0001,.0001));
  process.set_gyr_bias_cov(V3D(name=="negative_gyro_bias_noise"?-.0001:.0001,.0001,.0001));
  Filter filter;double eps[23];std::fill(eps,eps+23,.001);
  filter.init_dyn_share(get_f,df_dx,df_dw,no_measurement,4,eps);
  auto state=filter.get_x();auto covariance=filter.get_P();covariance.setIdentity();
  if(name=="reused_rotated_state"||name=="off_reused_rotated_legacy"){
    // Literal proper R_y(+90deg), not an Euler/quaternion helper under test.
    M3D r;r<<0,0,1, 0,1,0, -1,0,0;state.rot=r;
  }
  if(name=="reused_yaw_state"){M3D r;r<<0,-1,0, 1,0,0, 0,0,1;state.rot=r;}
  if(name=="preserved_ba_residual"||name=="stale_output_rejected"||name=="configured_mean_tolerance")state.ba[0]=1.;
  if(name=="nonfinite_position")state.pos[0]=nan;
  if(name=="nonfinite_ba")state.ba[0]=nan;
  if(name=="invalid_state_quaternion")state.rot.coeffs()*=2.;
  if(name=="nonfinite_covariance")covariance(0,0)=nan;
  if(name=="asymmetric_covariance")covariance(0,1)=.1;
  if(name=="negative_covariance")covariance(0,0)=-1.;
  if(name=="singular_covariance")covariance(0,0)=0.;
  filter.change_x(state);filter.change_P(covariance);
  if(name=="reused_rotated_state"||name=="reused_yaw_state"||name=="off_reused_rotated_legacy")process.Reset();
  const auto before=flatten(filter.get_x());const auto before_p=filter.get_P();
  bool initialized=false;
  bool sentinel_retained=false;
  for(int group_index=0;group_index<(name=="stale_output_waiting"?1:12);++group_index){
    MeasureGroup group;group.lidar_beg_time=1000.+group_index*.1;group.lidar_end_time=group.lidar_beg_time+.0999;
    group.lidar=std::make_shared<PointCloudXYZI>();PointType p{};p.x=2.;p.curvature=0.;group.lidar->push_back(p);p.curvature=99.9;group.lidar->push_back(p);
    for(int k=0;k<20;++k){const std::int64_t ns=1000000000000LL+(group_index*20+k)*5000000LL;
      auto imu=std::make_shared<sensor_msgs::msg::Imu>();imu->header.frame_id="imu";
      imu->header.stamp.sec=ns/1000000000LL;imu->header.stamp.nanosec=ns%1000000000LL;
      imu->linear_acceleration.z=((name=="last_accel_residual"||name=="configured_last_tolerance")&&group_index==10&&k==19)?10.31:9.81;group.imu.push_back(imu);}
    auto cloud=std::make_shared<PointCloudXYZI>();
    if(sentinel&&(name=="stale_output_waiting"||off||group_index==10)){PointType old{};old.x=123.;cloud->push_back(old);}
    process.Process(group,filter,cloud);sentinel_retained=!cloud->empty();
    initialized=process.frontend_init_v4_initialized();
    if(sentinel&&!off)check(cloud->empty(),"OUTPUT GUARD MISSING: enabled initialization retained stale mapping cloud");
    if(!fresh&&!off){
      check(!initialized,"STATE GUARD MISSING: stationary samples enabled inconsistent/invalid estimator");
      check(cloud->empty(),"mapping/deskew released despite failed state validation");
      check(same_state(before,flatten(filter.get_x())),"failed candidate partially mutated estimator state");
      check(same_cov(before_p,filter.get_P()),"failed candidate partially mutated covariance");
    }
    if(initialized)break; // Test only the allow/deny boundary, no unsupported propagation with invalid state.
  }
  if(fresh||off){
    check(initialized,"fresh/default-off existing behavior changed");
    const auto actual=filter.get_x();
    check(std::abs(actual.grav[0])<1e-12&&std::abs(actual.grav[1])<1e-12&&std::abs(actual.grav[2]+9.809)<1e-12,"fresh gravity numerics changed");
    if(name=="configured_mean_tolerance")check(std::abs(actual.ba[0]-1.)<1e-12&&actual.ba[1]==0.&&actual.ba[2]==0.,"existing bias was repaired instead of checking configured tolerance");
    else check(actual.ba.norm()==0.,"bias was silently invented");
    check(actual.bg.norm()==0.,"gyro bias was silently invented");
    if(name=="off_reused_rotated_legacy"){
      M3D original;original<<0,0,1, 0,1,0, -1,0,0;
      check((actual.rot.toRotationMatrix()-original).norm()<1e-12,"disabled guard changed legacy reused rotation");
    }else check((actual.rot.toRotationMatrix()-Eye3d).norm()<1e-12,"world rotation was forced/changed");
    check((actual.offset_T_L_I-ext).norm()<1e-12,"configured extrinsic changed");
  }
  if(name=="off_sentinel_legacy")check(sentinel_retained,"disabled path changed legacy output behavior");
  std::cout<<name<<" PASS (actual Process state allow/deny contract)\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
