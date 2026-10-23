#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "a2w_fastlio_localization/map_matcher.hpp"

namespace a2w_fastlio_localization
{

struct LocalizationManagerConfig
{
  // Bounds on the body-pose change at the matched scan, independent of the
  // arbitrary odometry origin.
  double maximum_correction_translation_jump_m{5.0};
  double maximum_correction_rotation_jump_rad{1.0};
  // Steady elapsed time from matcher dispatch through result completion. Source
  // age is preserved separately and is never substituted for this duration.
  std::int64_t maximum_match_result_age_ns{3'000'000'000LL};
};

struct LocalizationOutput
{
  bool valid{false};
  bool match_attempted{false};
  bool match_accepted{false};
  bool result_stale{false};
  // Matcher-start generation accompanies the result through asynchronous inboxes.
  std::uint64_t result_generation{0U};
  std::string reason;
  std::int64_t stamp_ns{0};
  // Sensor/source stamp remains the ordering key for rejecting old results.
  std::int64_t correction_stamp_ns{0};
  // Monotonic elapsed time since the correction was actually accepted.
  std::int64_t correction_age_ns{0};
  std::int64_t correction_source_age_ns{0};
  a2w_fastlio_common::Pose3d predicted_map_body{};
  std::int64_t prediction_correction_stamp_ns{-1};
  std::int64_t prediction_correction_accepted_steady_ns{-1};
  std::uint64_t candidate_id{0U};
  std::size_t source_points{0U};
  std::size_t target_points{0U};
  a2w_fastlio_common::CloudConstPtr match_target{};
  MapMatchingMode matching_mode{MapMatchingMode::kFull};
  a2w_fastlio_common::Pose3d map_body{};
  a2w_fastlio_common::Pose3d map_camera_init{};
  a2w_fastlio_common::RegistrationResult registration{};
  MapMatchTimings timings{};
};

using MatchFunction = std::function<MapMatchResult(
    const a2w_fastlio_common::FrontendFrame &,
    const a2w_fastlio_common::Pose3d &,
    MapMatchingMode)>;

class LocalizationManager
{
public:
  using SteadyNow = std::function<std::int64_t()>;
  LocalizationManager(
    LocalizationManagerConfig config, MatchFunction matcher, SteadyNow steady_now = {});
  LocalizationOutput propagate(const a2w_fastlio_common::FrontendFrame & frame);
  LocalizationOutput matchAndUpdate(
    const a2w_fastlio_common::FrontendFrame & frame,
    MapMatchingMode mode);
  void restoreCorrection(
    std::int64_t stamp_ns, const a2w_fastlio_common::Pose3d & map_camera_init);
  // Call when tracking loses trust. Pending and late-dispatched normal solves
  // remain invalid until restoreCorrection confirms a new global correction.
  void invalidatePendingMatches();
  // Recheck just before consuming a completed asynchronous result: LOST or a
  // global restore may have invalidated it after the solve finished.
  bool isCurrentResult(const LocalizationOutput & output) const;
  std::optional<LocalizationOutput> latest() const;
  std::int64_t correctionAcceptedAgeNs() const;

private:
  // Caller holds mutex_. Retain the latest pose's source timestamp and odometry
  // while applying a newly accepted historical correction.
  void refreshLatestCorrection();
  LocalizationManagerConfig config_;
  MatchFunction matcher_;
  SteadyNow steady_now_;
  mutable std::mutex mutex_;
  std::optional<a2w_fastlio_common::Pose3d> correction_;
  std::optional<std::int64_t> correction_stamp_ns_;
  std::int64_t correction_accepted_steady_ns_{0};
  std::optional<std::int64_t> last_frame_ns_;
  std::optional<std::int64_t> last_match_frame_ns_;
  std::optional<LocalizationOutput> latest_;
  std::optional<a2w_fastlio_common::Pose3d> latest_odom_pose_;
  std::uint64_t correction_generation_{0U};
  bool normal_match_acceptance_enabled_{true};
};

}  // namespace a2w_fastlio_localization
