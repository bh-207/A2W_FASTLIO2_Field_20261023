#pragma once
// Observation only. Fixed-capacity queue; never changes estimator state/time.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace frontend_runtime_v4 {
enum class Stage {InitCommit,JointInitCommit,PostLidarUpdate};
struct Row {
  Stage stage{Stage::PostLidarUpdate};
  std::uint64_t seq{0};
  std::int64_t scan_begin_ns{0},scan_end_ns{0},state_stamp_ns{0};
  std::int64_t accepted_window_last_imu_ns{0},group_last_imu_ns{0};
  bool init_stamp_consistent{false};int lidar_effective_features{-1};
  // pos3,vel3,q_O_I4,g_O3,bg_I3,ba_I3,t_I_L3,q_I_L4 (xyzw).
  std::array<double,26> state{};
};
struct Config {
  bool enabled{false};std::string path,source_provenance{"unknown"};std::size_t capacity{1024};
};
struct Stats {
  std::uint64_t attempted{0},enqueued{0},written{0},dropped{0},write_errors{0};
  std::size_t maximum_pending{0};bool closed{false},flush_ok{false};
};
class Writer {
public:
  explicit Writer(const Config &config):config_(config) {
    if(!config_.enabled)return;
    if(config_.path.empty()||config_.capacity==0||config_.capacity>1048576||
       (config_.source_provenance!="synthetic"&&config_.source_provenance!="real"))
      throw std::invalid_argument("runtime state log requires new path, bounded capacity and synthetic/real provenance");
    // Reserve every output exclusively before starting the worker. No path is
    // truncated, including a dangling symlink. The status file is reserved now
    // but populated only on close; an empty status is NOT a successful shutdown.
    csv_=exclusive(config_.path);
    try {
      metadata_=exclusive(config_.path+".metadata.json");
      status_=exclusive(config_.path+".status.json");
      const auto &p=config_.source_provenance;
      if(std::fprintf(metadata_,
        "{\"schema\":\"frontend_runtime_state_v4_v1\",\"source_provenance\":\"%s\","
        "\"frames\":{\"O\":\"camera_init\",\"I\":\"body\",\"L\":\"LiDAR\"},"
        "\"gravity_alignment\":false,\"quaternion_order\":\"xyzw\","
        "\"units\":{\"time\":\"source nanoseconds\",\"position\":\"m\",\"velocity\":\"m/s\",\"gravity_and_ba\":\"m/s^2\",\"bg\":\"rad/s\",\"angle\":\"rad\"},"
        "\"state_time\":\"POST_LIDAR_UPDATE: same scan end after real iterated update returns; INIT_COMMIT: exact accepted last IMU, not lidar-updated\","
        "\"scan_time_precision\":\"MeasureGroup double seconds llround to ns; not original raw header integer precision. Only INIT accepted window stamp retains exact integer ns\","
        "\"stage_scope\":\"POST_LIDAR_UPDATE denotes update invocation completed, not proof of effective constraints, convergence or independent accuracy\","
        "\"queue_capacity\":%zu,\"queue_policy\":\"fixed ring; try-lock/drop-new on contention or full; counters and alarm\","
        "\"flush_policy\":\"INIT_COMMIT flushed by worker immediately; other rows every16; drain, fflush/fclose on graceful close; no fsync durability guarantee; SIGKILL cannot flush\"}\n",p.c_str(),config_.capacity)<0||std::fflush(metadata_)!=0)
        throw std::runtime_error("runtime state metadata write failed");
      const bool metadata_closed=std::fclose(metadata_)==0;metadata_=nullptr;
      if(!metadata_closed)throw std::runtime_error("runtime state metadata close failed");
      if(std::fprintf(csv_,"seq,stage,scan_begin_ns,scan_end_ns,state_stamp_ns,pos_O_I_x,pos_O_I_y,pos_O_I_z,vel_O_I_x,vel_O_I_y,vel_O_I_z,q_O_I_x,q_O_I_y,q_O_I_z,q_O_I_w,g_O_x,g_O_y,g_O_z,bg_I_x,bg_I_y,bg_I_z,ba_I_x,ba_I_y,ba_I_z,t_I_L_x,t_I_L_y,t_I_L_z,q_I_L_x,q_I_L_y,q_I_L_z,q_I_L_w,anti_g_O_to_plus_Z_O_rad,accepted_window_last_imu_ns,group_last_imu_ns,init_stamp_consistent,lidar_effective_features\n")<0||std::fflush(csv_)!=0)
        throw std::runtime_error("runtime state CSV header write failed");
      ring_.resize(config_.capacity);
      worker_=std::thread([this]{consume();});
    }catch(...){if(csv_)std::fclose(csv_);if(metadata_)std::fclose(metadata_);if(status_)std::fclose(status_);csv_=metadata_=status_=nullptr;throw;}
  }
  Writer(const Writer &)=delete;Writer &operator=(const Writer &)=delete;
  ~Writer(){close();}
  bool push(Row row) noexcept {
    if(!config_.enabled||closed_.load(std::memory_order_acquire))return false;
    row.seq=attempted_.fetch_add(1,std::memory_order_relaxed);
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock()||stopping_||pending_==ring_.size()){
      if(dropped_.fetch_add(1,std::memory_order_relaxed)==0)
        std::fprintf(stderr,"[frontend_runtime_v4] QUEUE_DROP (first); inspect final status counters\n");
      return false;
    }
    ring_[(head_+pending_)%ring_.size()]=row;++pending_;
    maximum_pending_=std::max(maximum_pending_,pending_);enqueued_.fetch_add(1,std::memory_order_relaxed);
    lock.unlock();ready_.notify_one();return true;
  }
  void close() noexcept {
    // Owner calls close after callbacks stop; destructor is an idempotent fallback.
    // No worker joins while holding queue mutex. Not a signal-handler function.
    if(!config_.enabled||closed_.exchange(true,std::memory_order_acq_rel))return;
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}ready_.notify_one();
    if(worker_.joinable())worker_.join();
    bool flushed=std::fflush(csv_)==0;
    if(std::fclose(csv_)!=0)flushed=false;csv_=nullptr;
    if(!flushed)write_error();flush_ok_.store(flushed&&write_errors_.load()==0);
    const auto s=stats();
    const int count=std::fprintf(status_,"{\"schema\":\"frontend_runtime_state_v4_status_v1\",\"capacity\":%zu,\"attempted\":%llu,\"enqueued\":%llu,\"written\":%llu,\"dropped\":%llu,\"write_errors\":%llu,\"maximum_pending\":%zu,\"closed\":true,\"flush_ok\":%s}\n",
      config_.capacity,ull(s.attempted),ull(s.enqueued),ull(s.written),ull(s.dropped),ull(s.write_errors),s.maximum_pending,s.flush_ok?"true":"false");
    bool status_ok=count>=0&&std::fflush(status_)==0;
    if(std::fclose(status_)!=0)status_ok=false;status_=nullptr;
    if(!status_ok){write_error();flush_ok_.store(false);std::fprintf(stderr,"[frontend_runtime_v4] STATUS_WRITE_ERROR; status is not valid shutdown evidence\n");}
    std::fprintf(stderr,"[frontend_runtime_v4] CLOSED attempted=%llu enqueued=%llu written=%llu dropped=%llu write_errors=%llu flush_ok=%d\n",
      ull(s.attempted),ull(s.enqueued),ull(s.written),ull(s.dropped),ull(write_errors_.load()),static_cast<int>(flush_ok_.load()));
  }
  Stats stats() const noexcept {
    Stats s;s.attempted=attempted_.load();s.enqueued=enqueued_.load();s.written=written_.load();
    s.dropped=dropped_.load();s.write_errors=write_errors_.load();
    {std::lock_guard<std::mutex> lock(mutex_);s.maximum_pending=maximum_pending_;}
    s.closed=closed_.load();s.flush_ok=flush_ok_.load();return s;
  }
private:
  static unsigned long long ull(std::uint64_t v){return static_cast<unsigned long long>(v);}
  static FILE *exclusive(const std::string &path){
    const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
    if(fd<0)throw std::runtime_error("cannot exclusively create runtime state file: "+path);
    FILE *out=::fdopen(fd,"w");if(!out){::close(fd);throw std::runtime_error("fdopen runtime state file failed");}return out;
  }
  void write_error() noexcept {if(write_errors_.fetch_add(1)==0)std::fprintf(stderr,"[frontend_runtime_v4] WRITE_ERROR; state log is incomplete\n");}
  void consume() noexcept {
    while(true){
      Row row;
      {std::unique_lock<std::mutex> lock(mutex_);ready_.wait(lock,[this]{return pending_||stopping_;});
       if(!pending_&&stopping_)break;row=ring_[head_];head_=(head_+1)%ring_.size();--pending_;}
      bool ok=std::fprintf(csv_,"%llu,%s,%lld,%lld,%lld",ull(row.seq),row.stage==Stage::InitCommit?"INIT_COMMIT":row.stage==Stage::JointInitCommit?"JOINT_INIT_COMMIT":"POST_LIDAR_UPDATE",
        static_cast<long long>(row.scan_begin_ns),static_cast<long long>(row.scan_end_ns),static_cast<long long>(row.state_stamp_ns))>=0;
      for(double v:row.state)if(std::fprintf(csv_,",%.17g",v)<0)ok=false;
      const double gx=row.state[10],gy=row.state[11],gz=row.state[12],norm=std::sqrt(gx*gx+gy*gy+gz*gz);
      const double angle=norm>0&&std::isfinite(norm)?std::acos(std::max(-1.,std::min(1.,-gz/norm))):NAN;
      if(std::fprintf(csv_,",%.17g,%lld,%lld,%d,%d\n",angle,
        static_cast<long long>(row.accepted_window_last_imu_ns),static_cast<long long>(row.group_last_imu_ns),
        static_cast<int>(row.init_stamp_consistent),row.lidar_effective_features)<0)ok=false;
      if(row.stage!=Stage::PostLidarUpdate||(written_.load()+1)%16==0)
        if(std::fflush(csv_)!=0)ok=false;
      if(ok)written_.fetch_add(1);else write_error();
    }
  }
  Config config_;FILE *csv_{nullptr},*metadata_{nullptr},*status_{nullptr};std::vector<Row> ring_;
  mutable std::mutex mutex_;std::condition_variable ready_;std::thread worker_;
  std::size_t head_{0},pending_{0},maximum_pending_{0};bool stopping_{false};
  std::atomic<std::uint64_t> attempted_{0},enqueued_{0},written_{0},dropped_{0},write_errors_{0};
  std::atomic<bool> closed_{false},flush_ok_{false};
};
template<class State>
Row capture(Stage stage,std::int64_t scan_begin_ns,std::int64_t scan_end_ns,std::int64_t state_stamp_ns,const State &state){
  Row row;row.stage=stage;row.scan_begin_ns=scan_begin_ns;row.scan_end_ns=scan_end_ns;row.state_stamp_ns=state_stamp_ns;
  for(int k=0;k<3;++k){row.state[k]=state.pos[k];row.state[3+k]=state.vel[k];row.state[10+k]=state.grav[k];
    row.state[13+k]=state.bg[k];row.state[16+k]=state.ba[k];row.state[19+k]=state.offset_T_L_I[k];}
  for(int k=0;k<4;++k){row.state[6+k]=state.rot.coeffs()[k];row.state[22+k]=state.offset_R_L_I.coeffs()[k];}
  return row;
}
} // namespace frontend_runtime_v4
