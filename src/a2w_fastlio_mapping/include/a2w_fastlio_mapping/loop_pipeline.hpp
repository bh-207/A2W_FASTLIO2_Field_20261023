#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_common/place_recognition.hpp"
#include "a2w_fastlio_common/registration_pipeline.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{

struct LoopPipelineConfig
{
  std::size_t top_k{5U};
  std::size_t exclude_recent{30U};
  std::size_t local_map_before{5U};
  std::size_t local_map_after{5U};
  std::size_t minimum_keyframes_between_accepted_loops{10U};
  std::size_t queue_capacity{32U};
  double maximum_descriptor_distance{0.2};
  // Opt-in; existing production YAML and legacy acceptance stay unchanged.
  bool revisit_verification_enabled{false};
  std::size_t origin_keyframe_count{31U};
  std::size_t origin_top_k{5U};
  std::size_t hypothesis_group_id_span{10U};
  double hypothesis_group_distance_m{3.0};
  double solution_merge_distance_m{0.5};
  double solution_merge_rotation_rad{0.1};
  // A new, explicitly metre-valued geometric margin, NOT the SC score gap.
  double distinct_solution_fitness_margin_m{0.05};
  std::size_t confirmation_frames{2U};
  std::size_t confirmation_max_gap{10U};
  std::size_t retry_interval_keyframes{2U};
  std::size_t retry_max_attempts{3U};
  std::size_t retry_reset_keyframes{30U};
  // Offline opt-in scheduling experiment. Defaults preserve the v2 policy.
  std::size_t discovery_interval_keyframes{1U};
  double confirmation_min_translation_m{0.0};
  double confirmation_min_rotation_rad{0.0};
};

struct LoopPoseCheck
{
  a2w_fastlio_common::Pose3d predicted_pose{};  // T_map_body at current source stamp
  a2w_fastlio_common::Pose3d corrected_pose{};  // D_map_map * predicted_pose
  double translation_difference_m{0.0};
  double rotation_difference_rad{0.0};
  Eigen::Vector3d relative_rpy_rad{Eigen::Vector3d::Zero()};
  double translation_limit_m{0.0};
  double rotation_limit_rad{0.0};
  bool position_pass{false};
  bool rotation_pass{false};
};

struct LoopPipelineEvent
{
  std::uint64_t current_id{0U};
  std::uint64_t candidate_id{0U};
  std::size_t candidate_rank{0U};
  bool accepted{false};
  std::string reason{};
  a2w_fastlio_common::RegistrationResult registration{};
  std::size_t graph_factor_count{0U};
  std::size_t candidate_count{0U};
  std::size_t source_points{0U};
  std::size_t target_points{0U};
  double descriptor_ms{0.0};
  double scan_context_ms{0.0};
  double source_local_map_ms{0.0};
  double target_local_map_ms{0.0};
  double quatro_ms{0.0};
  double nano_gicp_ms{0.0};
  double evidence_ms{0.0};
  double validator_ms{0.0};
  double gtsam_ms{0.0};
  double registration_total_ms{0.0};
  double frame_total_ms{0.0};
  // Current keyframe pose change relative to its pre-update pose, not GICP fitness.
  std::optional<a2w_fastlio_common::Pose3d> optimized_pose_change{};
  std::optional<LoopPoseCheck> pose_check{};
  double descriptor_distance{std::numeric_limits<double>::infinity()};
  double yaw_hint_rad{0.0};
  bool origin_candidate{false};
  std::size_t confirmation_count{0U};
  std::string rejection_stage{};
};

enum class LoopProgressStage
{
  CandidateDetected, RegistrationStarted, CandidateFinished,
  OptimizationStarted, OptimizationFinished
};

// Synchronous, diagnostic-only observer. No algorithm decisions depend on it.
using LoopProgressObserver = std::function<void(LoopProgressStage, const LoopPipelineEvent &)>;

std::string loopRejectionChinese(const std::string & reason);
std::string loopPoseCheckChinese(
  const LoopPipelineEvent & event, const std::string & map_frame,
  const std::string & tracking_frame);

class LoopPipeline
{
public:
  LoopPipeline(
    std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition,
    std::shared_ptr<a2w_fastlio_common::DescriptorIndex> descriptor_index,
    std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
    std::shared_ptr<PoseGraphOptimizer> graph,
    a2w_fastlio_common::LocalMapBuilder local_map_builder,
    LoopPipelineConfig config = {});

  std::vector<LoopPipelineEvent> process(
    const a2w_fastlio_common::KeyFrame & current,
    const LoopProgressObserver & observer = {});
  bool enqueue(a2w_fastlio_common::KeyFrame keyframe);
  std::optional<std::vector<LoopPipelineEvent>> processNext();
  std::size_t pending() const noexcept;

private:
  class StoredKeyFrameProvider;

  bool confirmationMotionSufficient(const a2w_fastlio_common::Pose3d & raw_odom) const;

  std::vector<LoopPipelineEvent> processRevisit(
    const a2w_fastlio_common::KeyFrame & current,
    const a2w_fastlio_common::Pose3d & predicted_pose,
    const std::vector<a2w_fastlio_common::LoopCandidate> & candidates,
    const LoopProgressObserver & observer,
    double & source_local_map_ms, std::size_t & source_points);

  struct RetryState
  {
    std::uint64_t first_frame{0U};
    std::uint64_t last_attempt{0U};
    std::size_t attempts{0U};
  };
  struct PendingLoop
  {
    a2w_fastlio_common::Pose3d correction{};  // D_map_map, never a body pose
    std::uint64_t last_frame{0U};
    std::size_t count{0U};
    a2w_fastlio_common::Pose3d odom_pose{};  // Last counted raw observation, not optimized map pose.
  };
  std::map<std::uint64_t, RetryState> retry_states_;
  std::optional<PendingLoop> pending_confirmation_;
  std::optional<std::uint64_t> last_discovery_keyframe_;

  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition_;
  std::shared_ptr<a2w_fastlio_common::DescriptorIndex> descriptor_index_;
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration_;
  std::shared_ptr<PoseGraphOptimizer> graph_;
  a2w_fastlio_common::LocalMapBuilder local_map_builder_;
  LoopPipelineConfig config_;
  std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> keyframes_;
  std::optional<std::uint64_t> last_accepted_loop_id_{};
  mutable std::mutex queue_mutex_;
  std::deque<a2w_fastlio_common::KeyFrame> queue_;
};

}  // namespace a2w_fastlio_mapping
