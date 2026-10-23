#include "a2w_fastlio_localization/normal_matching_worker.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_localization
{
namespace
{

bool validFrame(const a2w_fastlio_common::FrontendFrame & frame)
{
  return frame.stamp_ns >= 0 && a2w_fastlio_common::isFinitePose(frame.odom_pose) &&
         frame.body_cloud && !frame.body_cloud->empty();
}

double rotationDistance(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  return Eigen::AngleAxisd{lhs.rotation.conjugate() * rhs.rotation}.angle();
}

}  // namespace

NormalResultPublication normalResultPublication(
  const LocalizationOutput & output, const bool current_generation, const LocalizationState state)
{
  if (output.result_stale) {
    return {false, false, output.reason};
  }
  if (!current_generation) {
    return {false, false, "discarded_normal_result:" + output.reason};
  }
  if (state == LocalizationState::kLost || state == LocalizationState::kRelocalizing) {
    return {false, false, "untrusted_normal_result:" + output.reason};
  }
  return {output.match_accepted, true, output.reason};
}

NormalMatchMotionGate::NormalMatchMotionGate(NormalMatchMotionGateConfig config)
: config_{std::move(config)}
{
  if (!std::isfinite(config_.translation_threshold_m) ||
    config_.translation_threshold_m <= 0.0 ||
    !std::isfinite(config_.rotation_threshold_rad) || config_.rotation_threshold_rad <= 0.0 ||
    config_.maximum_elapsed_ns <= 0)
  {
    throw std::invalid_argument{"invalid normal match motion gate configuration"};
  }
}

bool NormalMatchMotionGate::shouldSubmit(
  const a2w_fastlio_common::FrontendFrame & frame) const
{
  if (!validFrame(frame)) {
    return false;
  }
  if (!last_pose_ || !last_stamp_ns_) {
    return true;
  }
  if (frame.stamp_ns <= *last_stamp_ns_) {
    return false;
  }
  return (frame.odom_pose.translation - last_pose_->translation).norm() >=
         config_.translation_threshold_m ||
         rotationDistance(*last_pose_, frame.odom_pose) >= config_.rotation_threshold_rad ||
         frame.stamp_ns - *last_stamp_ns_ >= config_.maximum_elapsed_ns;
}

void NormalMatchMotionGate::recordSubmission(
  const a2w_fastlio_common::FrontendFrame & frame)
{
  if (!validFrame(frame) || (last_stamp_ns_ && frame.stamp_ns <= *last_stamp_ns_)) {
    throw std::invalid_argument{"normal match submission frame is invalid or non-monotonic"};
  }
  last_pose_ = frame.odom_pose;
  last_stamp_ns_ = frame.stamp_ns;
}

NormalMatchingWorker::NormalMatchingWorker(
  EvaluateFunction evaluator, ResultCallback callback)
: evaluator_{std::move(evaluator)}, callback_{std::move(callback)}
{
  if (!evaluator_ || !callback_) {
    throw std::invalid_argument{"normal matching worker requires callbacks"};
  }
  thread_ = std::thread{&NormalMatchingWorker::run, this};
}

NormalMatchingWorker::~NormalMatchingWorker()
{
  {
    std::lock_guard<std::mutex> lock{mutex_};
    stopping_ = true;
    job_.reset();
  }
  condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool NormalMatchingWorker::trySubmit(const a2w_fastlio_common::FrontendFrame & frame)
{
  if (!validFrame(frame)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (stopping_ || busy_) {
      return false;
    }
    job_ = frame;
    submitted_ = std::chrono::steady_clock::now();
    busy_ = true;
  }
  condition_.notify_one();
  return true;
}

bool NormalMatchingWorker::busy() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return busy_;
}

void NormalMatchingWorker::run()
{
  while (true) {
    a2w_fastlio_common::FrontendFrame frame;
    std::chrono::steady_clock::time_point submitted;
    {
      std::unique_lock<std::mutex> lock{mutex_};
      condition_.wait(lock, [this]() {return stopping_ || job_.has_value();});
      if (stopping_) {
        return;
      }
      frame = std::move(*job_);
      submitted = submitted_;
      job_.reset();
    }

    const auto started = std::chrono::steady_clock::now();
    const auto queue_wait_ms = std::chrono::duration<double, std::milli>(started - submitted).count();
    LocalizationOutput output;
    try {
      output = evaluator_(frame);
    } catch (const std::exception & error) {
      output.stamp_ns = frame.stamp_ns;
      output.match_attempted = true;
      output.reason = std::string{"normal_match_worker_failed:"} + error.what();
    } catch (...) {
      output.stamp_ns = frame.stamp_ns;
      output.match_attempted = true;
      output.reason = "normal_match_worker_failed:unknown";
    }
    try {
      const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      callback_(NormalMatchingWorkResult{frame, std::move(output), elapsed_ms, queue_wait_ms});
    } catch (...) {
      // A result consumer must not terminate the worker thread.
    }
    {
      std::lock_guard<std::mutex> lock{mutex_};
      busy_ = false;
    }
  }
}

}  // namespace a2w_fastlio_localization
