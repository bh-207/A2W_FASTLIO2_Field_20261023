#pragma once
// Default-off experimental quality/unit contract. No ROS dependency or axis remap.
// Thresholds require device calibration; low variance cannot rule out constant
// linear acceleration. This is NOT motion-excited extrinsic/time calibration.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace frontend_init_v4
{
using Vec3=std::array<double,3>;
inline double norm(const Vec3 &v) {return std::hypot(v[0],v[1],v[2]);}
inline bool finite(const Vec3 &v) {return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});}

struct Sample
{
  std::int64_t stamp_ns=0;
  Vec3 acc{},gyro{};
  std::string frame_id,gyro_unit,accel_unit;
  unsigned conversion_count=0,time_adjustment_count=0;
};

struct InputConfig
{
  bool enabled=false;
  std::string gyro_unit="unknown",accel_unit="unknown",frame_id="unknown";
  std::string gyro_evidence,accel_evidence,time_evidence;
  std::string source_provenance="unknown";
  // Positive delta means the same physical event has t_IMU_raw > t_LiDAR.
  // Apply ONCE: aligned IMU timestamp = raw IMU timestamp - delta.
  std::int64_t imu_minus_lidar_offset_ns=0;
};

class InputAdapter
{
 public:
  explicit InputAdapter(InputConfig config):config_(std::move(config))
  {
    if(!config_.enabled)return;
    if((config_.gyro_unit!="rad/s"&&config_.gyro_unit!="deg/s")||
       (config_.accel_unit!="m/s^2"&&config_.accel_unit!="g")||
       config_.frame_id.empty()||config_.frame_id=="unknown"||
       !evidence(config_.gyro_evidence)||!evidence(config_.accel_evidence)||!evidence(config_.time_evidence))
      throw std::invalid_argument("explicit unit/frame/time evidence required; auto/unknown is not verified");
  }
  const InputConfig &config()const {return config_;}
  Sample adapt(const Sample &raw)const
  {
    if(!config_.enabled)return raw;
    if(raw.conversion_count||raw.time_adjustment_count)
      throw std::invalid_argument("repeated unit conversion/time offset application");
    if(raw.gyro_unit!=config_.gyro_unit||raw.accel_unit!=config_.accel_unit||raw.frame_id!=config_.frame_id)
      throw std::invalid_argument("source unit/frame metadata mismatch; numeric unit guessing is forbidden");
    if(raw.stamp_ns<0||!finite(raw.acc)||!finite(raw.gyro))
      throw std::invalid_argument("invalid raw IMU timestamp or vector");
    auto out=raw;
    const auto delta=config_.imu_minus_lidar_offset_ns;
    if((delta>0&&raw.stamp_ns<delta)||(delta<0&&raw.stamp_ns>std::numeric_limits<std::int64_t>::max()+delta))
      throw std::invalid_argument("aligned timestamp outside nonnegative int64 range");
    out.stamp_ns=raw.stamp_ns-delta;
    const double gyro_scale=config_.gyro_unit=="deg/s"?3.14159265358979323846/180.:1.;
    const double acc_scale=config_.accel_unit=="g"?9.81:1.;
    for(std::size_t k=0;k<3;++k){out.gyro[k]*=gyro_scale;out.acc[k]*=acc_scale;}
    if(!finite(out.acc)||!finite(out.gyro))throw std::invalid_argument("converted IMU overflow");
    out.gyro_unit="rad/s";out.accel_unit="m/s^2";
    out.conversion_count=1;out.time_adjustment_count=1;
    return out;
  }
 private:
  static bool evidence(const std::string &value){return !value.empty()&&value!="unknown";}
  InputConfig config_;
};

struct GateConfig
{
  bool enabled=false;
  std::size_t minimum_samples=200,maximum_samples=20000;
  double minimum_duration_s=1.,maximum_gap_s=.02;
  double maximum_gyro_norm_rad_s=.05,maximum_mean_gyro_norm_rad_s=.02;
  double gyro_variance_limit=1e-5,accel_variance_limit=.04;
  double minimum_accel_norm_m_s2=9.,maximum_accel_norm_m_s2=10.6;
  double maximum_direction_change_rad=.02;
};

struct Stats
{
  std::size_t count=0,reset_count=0;
  std::int64_t first_stamp_ns=-1,last_stamp_ns=-1;
  Vec3 mean_acc{},mean_gyro{},variance_acc{},variance_gyro{};
  double span_s=0,maximum_gap_s=0,maximum_direction_change_rad=0;
  std::string reason="collecting";
};

class Gate
{
 public:
  explicit Gate(GateConfig config=GateConfig{}):config_(config)
  {
    const auto positive=[](double v){return std::isfinite(v)&&v>0;};
    if(!positive(config_.minimum_duration_s)||!positive(config_.maximum_gap_s)||
       !positive(config_.maximum_gyro_norm_rad_s)||!positive(config_.maximum_mean_gyro_norm_rad_s)||
       !positive(config_.gyro_variance_limit)||!positive(config_.accel_variance_limit)||
       !positive(config_.minimum_accel_norm_m_s2)||!positive(config_.maximum_accel_norm_m_s2)||
       !positive(config_.maximum_direction_change_rad)||config_.maximum_direction_change_rad>3.141592653589793||
       config_.minimum_accel_norm_m_s2>=config_.maximum_accel_norm_m_s2||
       config_.minimum_samples<2||config_.maximum_samples<config_.minimum_samples||config_.maximum_samples>1000000)
      throw std::invalid_argument("invalid frontend initialization quality thresholds");
  }
  bool enabled()const{return config_.enabled;}
  bool ready()const{return enabled()&&stats_.count>=config_.minimum_samples&&stats_.span_s+1e-12>=config_.minimum_duration_s;}
  const GateConfig &config()const{return config_;}
  const Stats &stats()const{return stats_;}
  const Stats &last_rejected_stats()const{return rejected_;}
  const std::vector<std::int64_t>&member_stamps_ns()const{return members_;}
  void invalidate_input(){if(enabled())reject("input_interface_rejection");}
  void invalidate_state(const std::string &reason){if(enabled())reject(reason);}
  void reset()
  {
    stats_=Stats{};rejected_=Stats{};members_.clear();m2_acc_={};m2_gyro_={};
    have_last_=false;last_seen_=0;anchor_={};
  }
  void feed(const Sample &sample)
  {
    if(!enabled())return;
    if(sample.gyro_unit!="rad/s"||sample.accel_unit!="m/s^2")
      throw std::invalid_argument("quality gate requires explicitly adapted SI input");
    if(sample.stamp_ns<0||!finite(sample.acc)||!finite(sample.gyro)){reject("nonfinite_or_negative_stamp");return;}
    double gap=0.;
    const bool monotonic=!have_last_||sample.stamp_ns>last_seen_;
    if(have_last_)gap=static_cast<double>(sample.stamp_ns-last_seen_)/1e9;
    last_seen_=sample.stamp_ns;have_last_=true;
    if(!monotonic){reject("non_monotonic_stamp");return;}
    if(gap>config_.maximum_gap_s){reject("imu_gap");return;}
    const double acc_norm=norm(sample.acc);
    if(acc_norm<config_.minimum_accel_norm_m_s2||acc_norm>config_.maximum_accel_norm_m_s2)
      {reject("accel_norm");return;}
    if(norm(sample.gyro)>config_.maximum_gyro_norm_rad_s){reject("gyro_norm_motion");return;}
    if(stats_.count>=config_.maximum_samples){reject("bounded_window_sample_capacity");return;}
    if(stats_.count==0){stats_.first_stamp_ns=sample.stamp_ns;anchor_=sample.acc;gap=0.;}
    double cosine=0.;for(std::size_t k=0;k<3;++k)cosine+=anchor_[k]*sample.acc[k];
    cosine=std::clamp(cosine/(norm(anchor_)*acc_norm),-1.,1.);
    const double angle=std::acos(cosine);
    stats_.maximum_direction_change_rad=std::max(stats_.maximum_direction_change_rad,angle);
    stats_.maximum_gap_s=std::max(stats_.maximum_gap_s,gap);
    ++stats_.count;stats_.last_stamp_ns=sample.stamp_ns;
    stats_.span_s=static_cast<double>(stats_.last_stamp_ns-stats_.first_stamp_ns)/1e9;
    members_.push_back(sample.stamp_ns);
    for(std::size_t k=0;k<3;++k) {
      const double da=sample.acc[k]-stats_.mean_acc[k],dg=sample.gyro[k]-stats_.mean_gyro[k];
      stats_.mean_acc[k]+=da/stats_.count;stats_.mean_gyro[k]+=dg/stats_.count;
      m2_acc_[k]+=da*(sample.acc[k]-stats_.mean_acc[k]);m2_gyro_[k]+=dg*(sample.gyro[k]-stats_.mean_gyro[k]);
      stats_.variance_acc[k]=std::max(0.,m2_acc_[k]/stats_.count);
      stats_.variance_gyro[k]=std::max(0.,m2_gyro_[k]/stats_.count);
    }
    if(angle>config_.maximum_direction_change_rad){reject("accel_direction_instability");return;}
    if(norm(stats_.mean_gyro)>config_.maximum_mean_gyro_norm_rad_s){reject("mean_gyro_motion_or_bias");return;}
    if(*std::max_element(stats_.variance_acc.begin(),stats_.variance_acc.end())>config_.accel_variance_limit)
      {reject("accel_variance");return;}
    if(*std::max_element(stats_.variance_gyro.begin(),stats_.variance_gyro.end())>config_.gyro_variance_limit)
      {reject("gyro_variance");return;}
    stats_.reason=ready()?"window_ready_not_yet_committed":"collecting";
  }
 private:
  void reject(const std::string &reason)
  {
    rejected_=stats_;rejected_.reason=reason;rejected_.reset_count=stats_.reset_count+1;
    stats_=Stats{};stats_.reset_count=rejected_.reset_count;stats_.reason=reason;
    m2_acc_={};m2_gyro_={};members_.clear();anchor_={};
  }
  GateConfig config_;
  Stats stats_,rejected_;
  Vec3 m2_acc_{},m2_gyro_{},anchor_{};
  bool have_last_=false;
  std::int64_t last_seen_=0;
  std::vector<std::int64_t>members_;
};
} // namespace frontend_init_v4
