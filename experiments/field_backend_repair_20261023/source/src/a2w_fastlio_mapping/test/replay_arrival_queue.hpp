#pragma once
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
namespace replay_queue {
struct Event {std::size_t id, waiting; std::int64_t wall_ns, due_ns; const char * action;};
struct Stats {std::size_t produced{0}, consumed{0}, max_waiting{0}; bool overflow{false};};
class ArrivalQueue {
public:
  ArrivalQueue(const std::vector<std::int64_t> & stamps, std::size_t capacity,
    std::chrono::steady_clock::time_point epoch = std::chrono::steady_clock::now())
  : capacity_(capacity), epoch_(epoch)
  {
    if (stamps.empty() || capacity == 0 || stamps.front() < 0) {
      throw std::invalid_argument("queue requires positive capacity and valid stamps");
    }
    due_.reserve(stamps.size()); events_.reserve(stamps.size()*2+1);
    for (std::size_t i=0;i<stamps.size();++i) {
      if (i && stamps[i] <= stamps[i-1]) {throw std::invalid_argument("nonmonotonic source stamp");}
      due_.push_back(stamps[i]-stamps.front());
    }
    producer_=std::thread([this] {produce();});
  }
  ArrivalQueue(const ArrivalQueue &)=delete;
  ArrivalQueue & operator=(const ArrivalQueue &)=delete;
  ~ArrivalQueue() {cancel(); if (producer_.joinable()) {producer_.join();}}
  std::size_t pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock,[this] {return done_ || !queue_.empty();});
    if (stats_.overflow) {throw std::runtime_error("offline arrival queue overflow; input archive preserved");}
    if (queue_.empty()) {throw std::out_of_range("arrival queue complete or cancelled");}
    auto id=queue_.front(); queue_.pop_front(); ++stats_.consumed;
    events_.push_back({id,queue_.size(),wall(),due_[id],"dequeue"});
    return id;
  }
  void waitDone() {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock,[this] {return done_;});
  }
  void cancel() {
    {std::lock_guard<std::mutex> lock(mutex_); cancelled_=true;}
    changed_.notify_all();
  }
  Stats stats() const {std::lock_guard<std::mutex> lock(mutex_); return stats_;}
  std::vector<Event> events() const {std::lock_guard<std::mutex> lock(mutex_); return events_;}
private:
  std::int64_t wall() const {return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now()-epoch_).count();}
  void produce() {
    std::unique_lock<std::mutex> lock(mutex_);
    for (std::size_t id=0;id<due_.size();++id) {
      if (changed_.wait_until(lock,epoch_+std::chrono::nanoseconds(due_[id]),
        [this] {return cancelled_;})) {break;}
      if (queue_.size() >= capacity_) {
        stats_.overflow=true;
        events_.push_back({id,queue_.size(),wall(),due_[id],"overflow"});
        break;
      }
      queue_.push_back(id); ++stats_.produced;
      stats_.max_waiting=std::max(stats_.max_waiting,queue_.size());
      events_.push_back({id,queue_.size(),wall(),due_[id],"arrival"});
      changed_.notify_all();
    }
    done_=true; changed_.notify_all();
  }
  const std::size_t capacity_;
  const std::chrono::steady_clock::time_point epoch_;
  std::vector<std::int64_t> due_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::deque<std::size_t> queue_;
  std::vector<Event> events_;
  Stats stats_;
  bool done_{false}, cancelled_{false};
  std::thread producer_;
};
}
