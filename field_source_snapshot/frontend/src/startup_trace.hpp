#pragma once
// Experimental evidence only: no estimator/registration mathematics.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace offline_handoff {
inline std::int64_t steady_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline bool matched(std::size_t count,const std::vector<std::string>& names,const std::vector<std::string>& expected) {
  if(count!=expected.size()||names.size()!=expected.size()||std::set<std::string>(names.begin(),names.end()).size()!=names.size())return false;
  std::set<std::string> used;
  for(const auto& pattern:expected) {
    std::size_t n=0;std::string chosen;
    for(const auto& name:names)if(pattern.back()=='*'?name.rfind(pattern.substr(0,pattern.size()-1),0)==0:name==pattern){++n;chosen=name;}
    if(n!=1||!used.insert(chosen).second)return false;
  }
  return true;
}

class Trace {
public:
  Trace(const std::filesystem::path& directory,const std::string& role,std::size_t capacity=4096):directory_(directory),role_(role),capacity_(capacity) {
    if(!capacity||!std::filesystem::is_directory(directory)||std::filesystem::exists(directory/(role+"_handoff.csv")))throw std::invalid_argument("fresh bounded trace directory required");
    file_.open(directory/(role+"_handoff.csv"));
    if(!file_)throw std::runtime_error("trace open failed");
    file_<<"seq,stage,stamp_ns,steady_ns,value,reason\n";
    worker_=std::thread([this]{work();});
  }
  ~Trace(){try{close();}catch(...){}}
  void record(const std::string& stage,std::int64_t stamp,std::int64_t value=0,const std::string& reason="ok") {
    std::lock_guard<std::mutex> lock(mutex_);
    if(closed_||closing_||errors_||pending_.size()>=capacity_) {++errors_;throw std::runtime_error("bounded handoff trace closed/overflow/write failure");}
    std::ostringstream row;row<<attempted_++<<','<<stage<<','<<stamp<<','<<steady_ns()<<','<<value<<','<<reason<<'\n';
    pending_.push_back(row.str());peak_=std::max(peak_,pending_.size());changed_.notify_one();
  }
  void close() {
    {std::lock_guard<std::mutex> lock(mutex_);if(closed_)return;closing_=true;changed_.notify_all();}
    if(worker_.joinable())worker_.join();
    file_.flush();if(!file_)++errors_;file_.close();closed_=true;
    std::ofstream status(directory_/(role_+"_handoff_trace_status.json"));
    status<<"{\"closed\":true,\"flush_ok\":"<<(errors_?"false":"true")<<",\"attempted\":"<<attempted_<<",\"written\":"<<written_<<",\"errors\":"<<errors_<<",\"peak\":"<<peak_<<",\"capacity\":"<<capacity_<<"}\n";
    if(!status)throw std::runtime_error("trace status write failed");
  }
  const std::filesystem::path& directory()const{return directory_;}
private:
  void work() {
    while(true) {
      std::string row;
      {std::unique_lock<std::mutex> lock(mutex_);changed_.wait(lock,[this]{return closing_||!pending_.empty();});
       if(pending_.empty()&&closing_)break;row=std::move(pending_.front());pending_.pop_front();}
      file_<<row;if(!file_)++errors_;else ++written_;
    }
  }
  std::filesystem::path directory_;std::string role_;std::size_t capacity_,peak_{0};
  std::ofstream file_;std::mutex mutex_;std::condition_variable changed_;std::deque<std::string> pending_;std::thread worker_;
  std::atomic<std::uint64_t> errors_{0},written_{0};std::uint64_t attempted_{0};bool closing_{false},closed_{false};
};
inline std::unique_ptr<Trace> trace;
inline void start(const std::string& role) {
  const char* mode=std::getenv("A2W_FIELD_MODE"),*directory=std::getenv("A2W_HANDOFF_DIRECTORY");
  const char* domain=std::getenv("ROS_DOMAIN_ID"),*local=std::getenv("ROS_LOCALHOST_ONLY"),*rmw=std::getenv("RMW_IMPLEMENTATION");
  bool real=mode&&std::string(mode)=="live"&&domain&&std::string(domain)=="0"&&local&&std::string(local)=="0";
  bool test=mode&&std::string(mode)=="offline-test"&&domain&&std::string(domain)=="87"&&local&&std::string(local)=="1";
  if((!real&&!test)||!directory||!rmw||std::string(rmw)!="rmw_cyclonedds_cpp")throw std::invalid_argument("explicit field session/domain required");
  auto path=std::filesystem::canonical(directory);
  if(path.string().find("/home/bh/A2W_FastLio2_controlled/captures/field_fixed_")!=0)throw std::invalid_argument("field trace outside controlled sessions");
  trace=std::make_unique<Trace>(path,role);
}
template<class Stamp>inline std::int64_t stamp_ns(const Stamp& stamp){return std::int64_t(stamp.sec)*1000000000LL+stamp.nanosec;}
inline void event(const std::string& stage,std::int64_t stamp,std::int64_t value=0,const std::string& reason="ok"){trace->record(stage,stamp,value,reason);}

#ifdef A2W_TRACE_ROS
struct Edge {std::string topic;std::size_t matched_count;std::vector<std::string> names,expected;};
template<class Publisher>inline Edge edge(rclcpp::Node& node,const Publisher& pub,const std::string& topic,std::vector<std::string> expected) {
  Edge result{topic,pub->get_subscription_count(),{},std::move(expected)};
  for(const auto& info:node.get_subscriptions_info_by_topic(topic))result.names.push_back(info.node_name());
  return result;
}
inline bool report_ready(const std::string& role,const std::vector<Edge>& edges) {
  bool ready=true;std::ostringstream out;out<<"{\"schema\":\"publisher_matched_edges_v1\",\"steady_ns\":"<<steady_ns()<<",\"edges\":[";
  bool first=true;for(const auto& e:edges){ready=ready&&matched(e.matched_count,e.names,e.expected);if(!first)out<<',';first=false;
    out<<"{\"topic\":"<<std::quoted(e.topic)<<",\"matched_count\":"<<e.matched_count<<",\"subscribers\":[";
    bool f=true;for(const auto& name:e.names){if(!f)out<<',';f=false;out<<std::quoted(name);}out<<"]}";}
  out<<"],\"ready\":"<<(ready?"true":"false")<<"}\n";
  auto target=trace->directory()/(role+"_matched.json"),temporary=trace->directory()/(role+"_matched.tmp");
  {std::ofstream file(temporary);file<<out.str();if(!file)throw std::runtime_error("match evidence write failed");}
  std::filesystem::rename(temporary,target);return ready;
}
template<class Pub>inline void require_two(const Pub& pub){if(pub->get_subscription_count()!=2)throw std::runtime_error("output edge disconnected/unexpected consumer; no silent startup loss");}
#endif
}
