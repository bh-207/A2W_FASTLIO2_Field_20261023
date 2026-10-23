#include "a2w_fastlio_localization/localization_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_common::Pose3d compose(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = (lhs.rotation * rhs.rotation).normalized();
  result.translation = lhs.rotation * rhs.translation + lhs.translation;
  return result;
}

a2w_fastlio_common::Pose3d inverse(const a2w_fastlio_common::Pose3d & pose)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation.normalized().conjugate();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

double rotationDistance(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  return Eigen::AngleAxisd(lhs.rotation.conjugate() * rhs.rotation).angle();
}

}  // namespace

LocalizationManager::LocalizationManager(
  LocalizationManagerConfig config, MatchFunction matcher, SteadyNow steady_now)
: config_{config}, matcher_{std::move(matcher)}, steady_now_{std::move(steady_now)}
{
  if (!steady_now_) {
    steady_now_ = []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      };
  }
  if (!std::isfinite(config_.maximum_correction_translation_jump_m) ||
    config_.maximum_correction_translation_jump_m <= 0.0 ||
    !std::isfinite(config_.maximum_correction_rotation_jump_rad) ||
    config_.maximum_correction_rotation_jump_rad <= 0.0 ||
    config_.maximum_match_result_age_ns <= 0 || !matcher_)
  {
    throw std::invalid_argument{"invalid LocalizationManager configuration"};
  }
}

LocalizationOutput LocalizationManager::propagate(
  const a2w_fastlio_common::FrontendFrame & frame)
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (frame.stamp_ns < 0 || (last_frame_ns_ && frame.stamp_ns <= *last_frame_ns_) ||
    !a2w_fastlio_common::isFinitePose(frame.odom_pose))
  {
    throw std::invalid_argument{"Localization frame is invalid or non-monotonic"};
  }
  last_frame_ns_ = frame.stamp_ns;
  LocalizationOutput output;
  output.stamp_ns = frame.stamp_ns;
  output.reason = "continuous_from_last_correction";
  if (correction_) {
    output.valid = true;
    output.map_camera_init = *correction_;
    output.map_body = compose(*correction_, frame.odom_pose);
    output.correction_stamp_ns = *correction_stamp_ns_;
    output.correction_source_age_ns = std::max<std::int64_t>(
      0, frame.stamp_ns - *correction_stamp_ns_);
    output.correction_age_ns = std::max<std::int64_t>(
      0, steady_now_() - correction_accepted_steady_ns_);
  }
  latest_ = output;
  latest_odom_pose_ = frame.odom_pose;
  return output;
}

LocalizationOutput LocalizationManager::matchAndUpdate(
  const a2w_fastlio_common::FrontendFrame & frame,
  const MapMatchingMode mode)
{
  a2w_fastlio_common::Pose3d predicted;
  LocalizationOutput output;
  std::uint64_t generation;
  std::int64_t started_steady_ns;
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (frame.stamp_ns < 0 ||
      (last_match_frame_ns_ && frame.stamp_ns <= *last_match_frame_ns_) ||
      !a2w_fastlio_common::isFinitePose(frame.odom_pose))
    {
      throw std::invalid_argument{"Localization match frame is invalid or non-monotonic"};
    }
    last_match_frame_ns_ = frame.stamp_ns;
    generation = correction_generation_;
    output.result_generation = generation;
    started_steady_ns = steady_now_();
    predicted = correction_ ? compose(*correction_, frame.odom_pose) : frame.odom_pose;
    output.predicted_map_body = predicted;
    output.prediction_correction_stamp_ns = correction_stamp_ns_.value_or(-1);
    output.prediction_correction_accepted_steady_ns =
      correction_ ? correction_accepted_steady_ns_ : -1;
  }

  const auto match = matcher_(frame, predicted, mode);

  std::lock_guard<std::mutex> lock{mutex_};
  output.stamp_ns = frame.stamp_ns;
  output.match_attempted = true;
  output.matching_mode = mode;
  output.candidate_id = match.candidate_id;
  output.source_points = match.source_points;
  output.target_points = match.target_points;
  output.match_target = match.target_cloud;
  output.registration = match.registration;
  output.timings = match.timings;
  if (correction_stamp_ns_ && frame.stamp_ns <= *correction_stamp_ns_) {
    output.result_stale = true;
    output.reason = "stale_match_result";
  } else if (generation != correction_generation_ || !normal_match_acceptance_enabled_) {
    output.result_stale = true;
    output.reason = "invalidated_match_result";
  } else if (steady_now_() - started_steady_ns > config_.maximum_match_result_age_ns) {
    output.result_stale = true;
    output.reason = "expired_match_result";
  } else if (match.success && a2w_fastlio_common::isFinitePose(match.map_body)) {
    const auto candidate = compose(match.map_body, inverse(frame.odom_pose));
    bool jump = correction_ &&
      ((match.map_body.translation - predicted.translation).norm() >
      config_.maximum_correction_translation_jump_m ||
      rotationDistance(predicted, match.map_body) > config_.maximum_correction_rotation_jump_rad);
    if (!jump && correction_ && latest_ && latest_odom_pose_ &&
      latest_->stamp_ns > frame.stamp_ns)
    {
      const auto current_body = compose(*correction_, *latest_odom_pose_);
      const auto corrected_body = compose(candidate, *latest_odom_pose_);
      jump = (corrected_body.translation - current_body.translation).norm() >
        config_.maximum_correction_translation_jump_m ||
        rotationDistance(current_body, corrected_body) >
        config_.maximum_correction_rotation_jump_rad;
    }
    if (!jump) {
      correction_ = candidate;
      correction_stamp_ns_ = frame.stamp_ns;
      correction_accepted_steady_ns_ = steady_now_();
      output.match_accepted = true;
      output.reason = "match_accepted";
    } else {
      output.reason = "correction_jump_rejected";
    }
  } else {
    output.reason = match.reason.empty() ? "match_failed" : match.reason;
  }
  if (correction_) {
    output.valid = true;
    output.map_camera_init = *correction_;
    output.map_body = compose(*correction_, frame.odom_pose);
    output.correction_stamp_ns = *correction_stamp_ns_;
    output.correction_source_age_ns = std::max<std::int64_t>(
      0, frame.stamp_ns - *correction_stamp_ns_);
    output.correction_age_ns = std::max<std::int64_t>(
      0, steady_now_() - correction_accepted_steady_ns_);
  }
  if (!latest_ || output.stamp_ns >= latest_->stamp_ns) {
    latest_ = output;
    latest_odom_pose_ = frame.odom_pose;
  } else if (output.match_accepted) {
    refreshLatestCorrection();
  }
  return output;
}

void LocalizationManager::restoreCorrection(
  const std::int64_t stamp_ns, const a2w_fastlio_common::Pose3d & map_camera_init)
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (stamp_ns < 0 || !a2w_fastlio_common::isFinitePose(map_camera_init) ||
    (correction_stamp_ns_ && stamp_ns <= *correction_stamp_ns_))
  {
    throw std::invalid_argument{"invalid or non-monotonic restored correction"};
  }
  correction_ = map_camera_init;
  correction_stamp_ns_ = stamp_ns;
  correction_accepted_steady_ns_ = steady_now_();
  normal_match_acceptance_enabled_ = true;
  ++correction_generation_;
  refreshLatestCorrection();
}

void LocalizationManager::refreshLatestCorrection()
{
  if (!latest_ || !latest_odom_pose_ || !correction_) {
    return;
  }
  latest_->valid = true;
  latest_->map_camera_init = *correction_;
  latest_->map_body = compose(*correction_, *latest_odom_pose_);
  latest_->correction_stamp_ns = *correction_stamp_ns_;
  latest_->correction_source_age_ns = std::max<std::int64_t>(
    0, latest_->stamp_ns - *correction_stamp_ns_);
  latest_->correction_age_ns = std::max<std::int64_t>(
    0, steady_now_() - correction_accepted_steady_ns_);
}

std::int64_t LocalizationManager::correctionAcceptedAgeNs() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return correction_ ? std::max<std::int64_t>(
    0, steady_now_() - correction_accepted_steady_ns_) : 0;
}

void LocalizationManager::invalidatePendingMatches()
{
  std::lock_guard<std::mutex> lock{mutex_};
  normal_match_acceptance_enabled_ = false;
  ++correction_generation_;
}

bool LocalizationManager::isCurrentResult(const LocalizationOutput & output) const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return output.match_attempted && !output.result_stale && normal_match_acceptance_enabled_ &&
         output.result_generation == correction_generation_;
}

std::optional<LocalizationOutput> LocalizationManager::latest() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return latest_;
}

}  // namespace a2w_fastlio_localization
