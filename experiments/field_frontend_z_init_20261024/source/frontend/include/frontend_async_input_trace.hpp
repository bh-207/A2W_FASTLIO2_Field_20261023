#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <fcntl.h>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <functional>
#include <string>
namespace frontend_input_csv {
inline const char* header(){return "status,raw_stamp_ns,aligned_stamp_ns,input_frame,source_gyro_unit,source_accel_unit,conversion_count,time_adjustment_count,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,si_ax,si_ay,si_az,si_gx,si_gy,si_gz\n";}
struct Row {
 bool adapted{true};std::int64_t raw_stamp_ns{0},aligned_stamp_ns{0};std::uint64_t callback_id{0};
 std::string input_frame,gyro_unit,accel_unit;unsigned conversion_count{0},time_adjustment_count{0};
 std::array<double,12> values{};
};
struct Config {std::string path;std::size_t capacity{4096};};
struct Stats {std::uint64_t attempted{0},enqueued{0},written{0},overflow{0},io_errors{0},maximum_pending{0};bool closed{false},flush_ok{false};bool healthy()const{return closed&&flush_ok&&!overflow&&!io_errors&&enqueued==written;}};
using WrittenObserver=std::function<void(const Row&)>;
// Exactly one producer: the existing SingleThreadedExecutor IMU callback.
// The consumer exclusively owns both files. No producer mutex, file write,
// flush, wait or join. Queue occupancy, not enqueue, defines accepted work.
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,"input trace requires lock-free 64-bit indices");
class AsyncWriter {
public:
 AsyncWriter(Config config,WrittenObserver observer={}):config_(std::move(config)),observer_(std::move(observer)) {
  if(config_.path.empty()||!config_.capacity||config_.capacity>1048576)
   throw std::invalid_argument("async input trace needs fresh path and capacity in [1,1048576]");
  ring_.resize(config_.capacity);
  std::promise<bool> ready;auto started=ready.get_future();
  worker_=std::thread([this,promise=std::move(ready)]()mutable{consume(std::move(promise));});
  if(!started.get()){worker_.join();throw std::runtime_error("cannot create/initialize async input trace and status");}
 }
 AsyncWriter(const AsyncWriter&)=delete;AsyncWriter&operator=(const AsyncWriter&)=delete;
 ~AsyncWriter(){close();}
 bool enqueue(Row&& row)noexcept {
  if(!accepting_.load(std::memory_order_acquire)||io_errors_.load(std::memory_order_acquire))return false;
  attempted_.fetch_add(1,std::memory_order_relaxed);
  const auto write=write_.load(std::memory_order_relaxed),read=read_.load(std::memory_order_acquire);
  if(write-read>=ring_.size()){overflow_.fetch_add(1,std::memory_order_relaxed);return false;}
  ring_[write%ring_.size()]=std::move(row);
  write_.store(write+1,std::memory_order_release);enqueued_.fetch_add(1,std::memory_order_relaxed);
  const auto pending=write+1-read;auto maximum=maximum_pending_.load(std::memory_order_relaxed);
  if(maximum<pending)maximum_pending_.store(pending,std::memory_order_relaxed);
  ready_.notify_one();return true;
 }
 void close()noexcept {
  // Owner must first stop the producer. Destructor is an idempotent fallback.
  accepting_.store(false,std::memory_order_release);stopping_.store(true,std::memory_order_release);ready_.notify_one();
  if(worker_.joinable())worker_.join();
 }
 Stats stats()const noexcept {
  Stats s;s.attempted=attempted_.load();s.enqueued=enqueued_.load();s.written=written_.load();
  s.overflow=overflow_.load();s.io_errors=io_errors_.load();s.maximum_pending=maximum_pending_.load();
  s.closed=closed_.load(std::memory_order_acquire);s.flush_ok=flush_ok_.load();return s;
 }
private:
 static unsigned long long ull(std::uint64_t v){return static_cast<unsigned long long>(v);}
 static FILE* exclusive(const std::string& path) {
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
  if(fd<0)return nullptr;
  FILE* out=::fdopen(fd,"w");
  if(!out)::close(fd);
  return out;
 }
 void io_error()noexcept {io_errors_.fetch_add(1,std::memory_order_release);}
 static bool write_row(FILE* out,const Row& r)noexcept {
  int n;
  if(r.adapted)n=std::fprintf(out,"adapted,%lld,%lld,%s,%s,%s,%u,%u",static_cast<long long>(r.raw_stamp_ns),static_cast<long long>(r.aligned_stamp_ns),r.input_frame.c_str(),r.gyro_unit.c_str(),r.accel_unit.c_str(),r.conversion_count,r.time_adjustment_count);
  else n=std::fprintf(out,"rejected,%lld,,%s,%s,%s,0,0",static_cast<long long>(r.raw_stamp_ns),r.input_frame.c_str(),r.gyro_unit.c_str(),r.accel_unit.c_str());
  bool ok=n>=0;
  for(unsigned i=0;i<(r.adapted?12U:6U);++i)if(std::fprintf(out,",%.17g",r.values[i])<0)ok=false;
  if(std::fputs(r.adapted?"\n":",,,,,,\n",out)<0)ok=false;
  // Preserve the original per-row endl flush semantics on the worker.
  if(std::fflush(out)!=0)ok=false;
  return ok;
 }
 void consume(std::promise<bool> started)noexcept {
  FILE* csv=exclusive(config_.path);FILE* status=csv?exclusive(config_.path+".status.json"):nullptr;
  bool initial=csv&&status;
  if(initial)initial=std::fputs(header(),csv)>=0&&std::fflush(csv)==0;
  if(!initial){io_error();if(csv)std::fclose(csv);if(status)std::fclose(status);closed_.store(true,std::memory_order_release);started.set_value(false);return;}
  accepting_.store(true,std::memory_order_release);started.set_value(true);
  while(true){
   const auto read=read_.load(std::memory_order_relaxed),write=write_.load(std::memory_order_acquire);
   if(read==write){
    if(stopping_.load(std::memory_order_acquire))break;
    std::unique_lock<std::mutex> lock(wait_mutex_);
    ready_.wait_for(lock,std::chrono::milliseconds(2),[this]{return stopping_.load(std::memory_order_acquire)||read_.load(std::memory_order_relaxed)!=write_.load(std::memory_order_acquire);});
    continue;
   }
   Row row=std::move(ring_[read%ring_.size()]);read_.store(read+1,std::memory_order_release);
   if(write_row(csv,row)){
    written_.fetch_add(1,std::memory_order_relaxed);
    try{if(observer_)observer_(row);}catch(...){io_error();}
   }else io_error();
  }
  bool csv_ok=std::fflush(csv)==0;if(std::fclose(csv)!=0)csv_ok=false;
  if(!csv_ok)io_error();
  flush_ok_.store(csv_ok&&io_errors_.load()==0,std::memory_order_relaxed);
  auto s=stats();
  const int n=std::fprintf(status,"{\"schema\":\"frontend_async_input_csv_status_v1\",\"input_csv_mode\":\"async_spsc_bounded_flush_worker\",\"capacity\":%zu,\"attempted\":%llu,\"enqueued\":%llu,\"written\":%llu,\"overflow\":%llu,\"io_errors\":%llu,\"maximum_pending\":%llu,\"closed\":true,\"flush_ok\":%s}\n",config_.capacity,ull(s.attempted),ull(s.enqueued),ull(s.written),ull(s.overflow),ull(s.io_errors),ull(s.maximum_pending),s.flush_ok?"true":"false");
  bool status_ok=n>=0&&std::fflush(status)==0;if(std::fclose(status)!=0)status_ok=false;
  if(!status_ok){io_error();flush_ok_.store(false);}
  closed_.store(true,std::memory_order_release);
  s=stats();std::fprintf(stderr,"[async_input_csv] CLOSED enqueued=%llu written=%llu overflow=%llu io_errors=%llu flush_ok=%d\n",ull(s.enqueued),ull(s.written),ull(s.overflow),ull(s.io_errors),static_cast<int>(s.flush_ok));
 }
 Config config_;WrittenObserver observer_;std::vector<Row> ring_;std::thread worker_;
 std::condition_variable ready_;std::mutex wait_mutex_;
 std::atomic<std::uint64_t> write_{0},read_{0},attempted_{0},enqueued_{0},written_{0},overflow_{0},io_errors_{0},maximum_pending_{0};
 std::atomic<bool> accepting_{false},stopping_{false},closed_{false},flush_ok_{false};
};
}
