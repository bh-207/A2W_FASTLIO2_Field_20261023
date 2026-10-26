#pragma once
// Experiment-only observation. No estimator, queue, timestamp or QoS decisions.
// A bounded compact memory ledger is written only after intake stops.
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <vector>

namespace frontend_integrity {
inline std::int64_t steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
enum class Stage:unsigned {ImuRaw,ImuAdapted,ImuRejected,ImuBuffered,ImuSynced,ImuCleared,
  CloudRaw,CloudPreprocessed,CloudBuffered,CloudSynced,CloudCleared,Group,
  ComputeBegin,FirstScan,InitSkip,EmptyCloud,TreeInit,FewFeatures,Post,ComputeEnd,ImuLogged,Count};
inline const char* name(Stage s){static const char* names[]={"imu_raw","imu_adapted","imu_rejected","imu_buffered","imu_synced","imu_cleared","cloud_raw","cloud_preprocessed","cloud_buffered","cloud_synced","cloud_cleared","sync_group","compute_begin","first_scan_skip","initialization_skip","empty_cloud_skip","tree_initialization","few_features_skip","post_lidar_update","compute_end","imu_logged"};return names[static_cast<unsigned>(s)];}
struct Event {std::int64_t stamp_ns,steady_ns;std::uint64_t id,group;std::int64_t a,b;Stage stage;};
static_assert(sizeof(Event)<=64,"bounded compact trace");
struct Snapshot {
  std::uint64_t lidar_buffer_size=0,imu_buffer_size=0;
  std::int64_t imu_buffer_front_ns=0,imu_buffer_back_ns=0,lidar_buffer_front_stamp_ns=0;
  bool computing=false;
};
class Trace {
public:
  explicit Trace(std::filesystem::path directory,std::size_t capacity=2500000):directory_(std::move(directory)),capacity_(capacity){
    if(!capacity_)throw std::invalid_argument("positive integrity capacity required");
    std::filesystem::create_directories(directory_);
    if(std::filesystem::exists(directory_/"frontend_input_stages.csv"))throw std::invalid_argument("fresh integrity output required");
    events_.reserve(capacity_);
  }
  std::uint64_t record(Stage stage,std::int64_t stamp=0,std::uint64_t id=0,std::uint64_t group=0,std::int64_t a=0,std::int64_t b=0){
    std::lock_guard<std::mutex> lock(mutex_);if(closed_)return 0;
    const auto index=static_cast<unsigned>(stage);const auto sequence=++counts_[index];const auto now=steady_ns();
    if(stage==Stage::ImuRaw||stage==Stage::CloudRaw){id=sequence;last_callback_steady_=now;}
    if(stage==Stage::ImuRaw)last_raw_imu_=stamp;
    if(stage==Stage::CloudRaw)last_raw_cloud_=stamp;
    if(stage==Stage::Group){last_sync_cloud_=stamp;last_group_=group;first_imu_=a;last_imu_=b;}
    if(stage==Stage::Post)last_post_group_=group;
    if(events_.size()>=capacity_){++overflow_;return id;}
    events_.push_back({stamp,now,id,group,a,b,stage});return id;
  }
  void set_input_csv_mode(const std::string& mode){std::lock_guard<std::mutex> lock(mutex_);input_csv_mode_=mode;}
  std::uint64_t count(Stage stage)const{std::lock_guard<std::mutex> lock(mutex_);return counts_[static_cast<unsigned>(stage)];}
  std::uint64_t overflow()const{std::lock_guard<std::mutex> lock(mutex_);return overflow_;}
  // Atomic replace, <=1Hz normally. force pairs a computing begin/end snapshot.
  bool status(const Snapshot& snapshot,bool force=false){
    std::lock_guard<std::mutex> lock(mutex_);const auto now=steady_ns();
    if(!force&&now-last_status_ns_<1000000000LL)return false;
    try {write_status(snapshot,now);last_status_ns_=now;return true;}
    catch(...){++status_errors_;throw;}
  }
  bool close(const Snapshot& snapshot){
    std::lock_guard<std::mutex> lock(mutex_);if(closed_)return flush_ok_;
    closed_=true;
    try {
      std::ofstream out(directory_/"frontend_input_stages.csv");
      out<<"stage,stamp_ns,steady_ns,id,group,a,b\n";
      for(const auto& e:events_)out<<name(e.stage)<<','<<e.stamp_ns<<','<<e.steady_ns<<','<<e.id<<','<<e.group<<','<<e.a<<','<<e.b<<'\n';
      out.flush();out.close();flush_ok_=bool(out)&&overflow_==0&&status_errors_==0;
      write_status(snapshot,steady_ns());
    }catch(...){flush_ok_=false;}
    return flush_ok_;
  }
private:
  void write_status(const Snapshot& snapshot,std::int64_t now){
    const auto tmp=directory_/"frontend_processing_status.tmp";std::ofstream out(tmp);
    out<<"{\"schema\":\"frontend_processing_status_v1\",\"input_csv_mode\":\""<<input_csv_mode_<<"\",\"stage_trace_mode\":\"bounded_memory_exit_write\",\"steady_ns\":"<<now
      <<",\"closed\":"<<(closed_?"true":"false")<<",\"flush_ok\":"<<(flush_ok_?"true":"false")
      <<",\"computing\":"<<(snapshot.computing?"true":"false")
      <<",\"imu_raw_callbacks\":"<<counts_[0]<<",\"imu_adapted\":"<<counts_[1]<<",\"imu_buffer_push\":"<<counts_[3]<<",\"imu_sync_consumed\":"<<counts_[4]
      <<",\"cloud_raw_callbacks\":"<<counts_[6]<<",\"cloud_buffer_push\":"<<counts_[8]<<",\"cloud_sync_consumed\":"<<counts_[9]
      <<",\"post_count\":"<<counts_[18]<<",\"last_raw_imu_ns\":"<<last_raw_imu_<<",\"last_raw_cloud_ns\":"<<last_raw_cloud_
      <<",\"last_sync_cloud_ns\":"<<last_sync_cloud_<<",\"last_group\":"<<last_group_<<",\"last_group_first_imu_ns\":"<<first_imu_<<",\"last_group_last_imu_ns\":"<<last_imu_
      <<",\"last_post_group\":"<<last_post_group_<<",\"last_callback_steady_ns\":"<<last_callback_steady_
      <<",\"lidar_buffer_size\":"<<snapshot.lidar_buffer_size<<",\"imu_buffer_size\":"<<snapshot.imu_buffer_size
      <<",\"imu_buffer_front_ns\":"<<snapshot.imu_buffer_front_ns<<",\"imu_buffer_back_ns\":"<<snapshot.imu_buffer_back_ns<<",\"lidar_buffer_front_stamp_ns\":"<<snapshot.lidar_buffer_front_stamp_ns
      <<",\"trace_capacity\":"<<capacity_<<",\"trace_records\":"<<events_.size()<<",\"trace_overflow\":"<<overflow_<<",\"status_write_errors\":"<<status_errors_<<",\"counts\":{";
    for(unsigned i=0;i<static_cast<unsigned>(Stage::Count);++i){if(i)out<<',';out<<'"'<<name(static_cast<Stage>(i))<<"\":"<<counts_[i];}
    out<<"}}\n";out.flush();out.close();if(!out)throw std::runtime_error("integrity status write failed");
    std::filesystem::rename(tmp,directory_/"frontend_processing_status.json");
  }
  std::string input_csv_mode_{"legacy_sync_endl"};
  std::filesystem::path directory_;std::size_t capacity_;std::vector<Event> events_;
  mutable std::mutex mutex_;std::array<std::uint64_t,static_cast<unsigned>(Stage::Count)> counts_{};
  std::uint64_t overflow_=0,status_errors_=0,last_group_=0,last_post_group_=0;
  std::int64_t last_raw_imu_=0,last_raw_cloud_=0,last_sync_cloud_=0,first_imu_=0,last_imu_=0,last_callback_steady_=0,last_status_ns_=0;
  bool closed_=false,flush_ok_=false;
};
} // namespace frontend_integrity
