#include <cmath>
#include <atomic>
#include <sstream>
#include <iomanip>
#include "frontend_init_v4.hpp"
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"
#include "frontend_init_state_v4.hpp"
#include "frontend_joint_candidate.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_frontend_init_v4(const frontend_init_v4::GateConfig &, const frontend_init_v4::InputConfig &, const std::string &trace_path, const frontend_init_v4::StateGuardConfig &state_config=frontend_init_v4::StateGuardConfig{});
  void notify_frontend_input_rejection_v4(){if(init_gate_v4_.enabled())++input_rejections_v4_;}
  bool frontend_init_v4_initialized()const{return !imu_need_init_;}
  void configure_joint(const std::string&path,const std::string&receipt){joint_.load(path,receipt);}
  bool joint_committed()const{return joint_.committed;}
  double state_time()const{return imu_need_init_?0:last_lidar_end_time_;}
  void joint_context(std::int64_t id,std::int64_t header,sensor_msgs::msg::Imu::ConstSharedPtr future){joint_scan_=id;joint_header_=header;joint_future_=future;}
  V3D normalization_mean()const{return mean_acc;}
  double acceleration_scale()const{return G_m_s2/mean_acc.norm();}
  const frontend_init_v4::Stats &frontend_init_v4_stats()const{return init_gate_v4_.stats();}
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  frontend_joint::Candidate joint_;
  bool joint_static_ready_{false},joint_first_deskew_{false};
  std::int64_t joint_scan_{-1},joint_header_{0};
  sensor_msgs::msg::Imu::ConstSharedPtr joint_future_;
  bool commit_joint(const MeasureGroup&,esekfom::esekf<state_ikfom,12,input_ikfom>&);
  frontend_init_v4::Gate init_gate_v4_;
  std::atomic<std::size_t> input_rejections_v4_{0};
  std::ofstream init_trace_v4_;
  frontend_init_v4::StateGuardConfig state_guard_config_v4_;
  std::ofstream state_trace_v4_;
  void trace_frontend_state_v4(const char *, const frontend_init_v4::StateGuardReport &, const frontend_init_v4::Stats &, const state_ikfom &, const state_ikfom *);
  void trace_frontend_init_v4(const char *, const frontend_init_v4::Stats &, const state_ikfom &, bool);
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  init_gate_v4_.reset();
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}


void ImuProcess::set_frontend_init_v4(const frontend_init_v4::GateConfig &config,
  const frontend_init_v4::InputConfig &input_config, const std::string &trace_path, const frontend_init_v4::StateGuardConfig &state_config)
{
  if(config.enabled && !input_config.enabled)
    throw std::invalid_argument("v4 quality gate requires explicit SI input contract");

  state_guard_config_v4_=state_config;
  if(config.enabled)state_guard_config_v4_.validate();

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

  init_trace_v4_.open(trace_path);
  if(!init_trace_v4_)throw std::runtime_error("cannot open v4 initialization trace");
  init_trace_v4_ << std::setprecision(17);
  init_trace_v4_ << "stage,reason,first_stamp_ns,last_stamp_ns,accepted_samples,span_s,max_gap_s,max_direction_rad,reset_count,mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,var_ax,var_ay,var_az,var_gx,var_gy,var_gz,state_initialized,g_Wx,g_Wy,g_Wz,bgx,bgy,bgz,bax,bay,baz,ext_tx,ext_ty,ext_tz,ext_qx,ext_qy,ext_qz,ext_qw,q_WIx,q_WIy,q_WIz,q_WIw,window_member_stamps_ns\n";
}


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

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  if(joint_first_deskew_&&pcl_beg_time<last_lidar_end_time_-2.56e-7)
    throw std::runtime_error("joint first scan overlaps commit boundary; no precommit poses are available");
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(joint_first_deskew_?last_lidar_end_time_-pcl_beg_time:0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  joint_first_deskew_=false;
  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
    double head_stamp = rclcpp::Time(head->header.stamp).seconds();

    if (tail_stamp < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head_stamp < last_lidar_end_time_)
    {
      dt = tail_stamp - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail_stamp - head_stamp;
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail_stamp - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

// Explicit experiment mode: batch observations end before this one-time commit.
// The commit scan itself is not released to the ordinary point-to-plane update.
bool ImuProcess::commit_joint(const MeasureGroup&meas,esekfom::esekf<state_ikfom,12,input_ikfom>&kf)
{
 if(joint_scan_<joint_.scan_id)return false;
 if(joint_scan_!=joint_.scan_id||joint_header_!=joint_.header_ns||
    std::abs(std::llround(meas.lidar_end_time*1e9)-joint_.stamp_ns)>256||
    std::abs(acceleration_scale()-joint_.acc_scale)>1e-12)
   throw std::runtime_error("joint commit source/time/normalization differs from validated candidate");
 const auto past=meas.imu.back();const auto future=joint_future_;
 const auto ns=[](const auto&m){return static_cast<std::int64_t>(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;};
 const auto end=std::llround(meas.lidar_end_time*1e9);
 if(ns(past)>end)throw std::runtime_error("joint last IMU after commit end");
 if(ns(past)<end&&(!future||ns(future)<end||ns(future)<=ns(past)||ns(future)-ns(past)>20000000))throw std::runtime_error("joint commit needs <=20ms real IMU bracket");
 const auto upper=ns(past)==end?past:future;
 const double w=ns(past)==end?0.:double(end-ns(past))/double(ns(upper)-ns(past));
 auto endpoint=std::make_shared<sensor_msgs::msg::Imu>(*past);
 endpoint->header.stamp.sec=end/1000000000LL;endpoint->header.stamp.nanosec=end%1000000000LL;
 for(int k=0;k<3;++k){double*a=k==0?&endpoint->linear_acceleration.x:k==1?&endpoint->linear_acceleration.y:&endpoint->linear_acceleration.z;
 const double a0=k==0?past->linear_acceleration.x:k==1?past->linear_acceleration.y:past->linear_acceleration.z;
 const double a1=k==0?upper->linear_acceleration.x:k==1?upper->linear_acceleration.y:upper->linear_acceleration.z;*a=a0+(a1-a0)*w;
 double*b=k==0?&endpoint->angular_velocity.x:k==1?&endpoint->angular_velocity.y:&endpoint->angular_velocity.z;
 const double b0=k==0?past->angular_velocity.x:k==1?past->angular_velocity.y:past->angular_velocity.z;
 const double b1=k==0?upper->angular_velocity.x:k==1?upper->angular_velocity.y:upper->angular_velocity.z;*b=b0+(b1-b0)*w;}
 auto x=kf.get_x();x.pos=joint_.p;x.rot=SO3(joint_.R);x.vel=joint_.v;x.grav=S2(joint_.g);x.ba=joint_.ba;x.bg=joint_.bg;
 x.offset_R_L_I=Lidar_R_wrt_IMU;x.offset_T_L_I=Lidar_T_wrt_IMU;
 Eigen::Matrix<double,2,3>N;x.grav.S2_Nx_yy(N);auto P=frontend_joint::map_covariance(joint_.covariance,N);
 Eigen::LDLT<Eigen::Matrix<double,23,23>> factor(P);
 if(!P.allFinite()||factor.info()!=Eigen::Success||!factor.vectorD().allFinite()||factor.vectorD().minCoeff()<=state_guard_config_v4_.minimum_covariance_pivot)throw std::runtime_error("joint SO3/S2 mapped covariance invalid");
 kf.change_x(x);kf.change_P(P);last_imu_=endpoint;last_lidar_end_time_=meas.lidar_end_time;
 angvel_last=V3D(endpoint->angular_velocity.x,endpoint->angular_velocity.y,endpoint->angular_velocity.z)-x.bg;
 acc_s_last=x.rot*(acceleration_scale()*V3D(endpoint->linear_acceleration.x,endpoint->linear_acceleration.y,endpoint->linear_acceleration.z)-x.ba)+V3D(x.grav);
 cov_acc=cov_acc_scale;cov_gyr=cov_gyr_scale;imu_need_init_=false;joint_first_deskew_=true;joint_.committed=true;
 joint_.reason="joint_committed_validated_motion_batch";joint_.save(joint_scan_,end,acceleration_scale());
 std::ofstream bracket(joint_.receipt+".boundary.json");bracket<<std::setprecision(17)<<"{\"past_raw_ns\":"<<ns(past)<<",\"future_raw_ns\":"<<ns(upper)<<",\"endpoint_ns\":"<<end<<",\"interpolation_weight\":"<<w<<",\"endpoint_is_source_message\":false,\"ordinary_update_on_commit_scan\":false}\n";
 if(!bracket)throw std::runtime_error("joint endpoint receipt write failed");
 trace_frontend_init_v4("joint_initialized",init_gate_v4_.stats(),x,true);return true;
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  // Reset/reused output must never release stale points while gated initialization waits/rejects.
  if(init_gate_v4_.enabled() && imu_need_init_ && cur_pcl_un_)cur_pcl_un_->clear();
  if(meas.imu.empty()) {return;};
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    if(joint_.active&&joint_static_ready_){
      if(input_rejections_v4_.exchange(0))throw std::runtime_error("joint initialization observed rejected source input");
      commit_joint(meas,kf_state);return;
    }
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
      if(joint_.active){
        mean_acc=ready_mean_acc;mean_gyr=ready_mean_gyr;joint_static_ready_=true;
        trace_frontend_init_v4("joint_scale_window_ready",stats,kf_state.get_x(),false);
        commit_joint(meas,kf_state);return;
      }
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

      // Establish the real deskew/prediction boundary at the accepted scan.
      last_lidar_end_time_=meas.lidar_end_time;
      angvel_last=V3D(last_imu_->angular_velocity.x,last_imu_->angular_velocity.y,last_imu_->angular_velocity.z)-initial.bg;
      const V3D last_acc(last_imu_->linear_acceleration.x,last_imu_->linear_acceleration.y,last_imu_->linear_acceleration.z);
      acc_s_last=initial.rot*(last_acc*G_m_s2/mean_acc.norm()-initial.ba)+V3D(initial.grav);
      trace_frontend_init_v4("initialized",stats,kf_state.get_x(),true);
      std::cout << "IMU Initial Done [v4 accepted_samples=" << stats.count << " span_s=" << stats.span_s << "]" << std::endl;
      return;
    }

    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      std::cout << "IMU Initial Done" << std::endl;
      if (mean_gyr.norm() > 0.5)
        std::cout << "[WARN] estimated gyro bias " << mean_gyr.norm()
                  << " rad/s is implausibly large for a stationary IMU; "
                  << "check common.imu_gyr_unit (auto/deg/rad)" << std::endl;
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      // fout_imu is never written to (the write below is commented out); do not create Log/imu.txt
      // fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
