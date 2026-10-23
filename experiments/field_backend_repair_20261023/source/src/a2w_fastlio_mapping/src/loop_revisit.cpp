#include "a2w_fastlio_mapping/loop_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

namespace a2w_fastlio_mapping
{
namespace
{
using a2w_fastlio_common::KeyFrame;
using a2w_fastlio_common::LoopCandidate;
using a2w_fastlio_common::Pose3d;

double elapsedMs(const std::chrono::steady_clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
}

Pose3d compose(const Pose3d & lhs, const Pose3d & rhs)
{
  Pose3d result;
  result.rotation = (lhs.rotation * rhs.rotation).normalized();
  result.translation = lhs.rotation * rhs.translation + lhs.translation;
  return result;
}

Pose3d between(const Pose3d & from, const Pose3d & to)
{
  Pose3d result;
  result.rotation = (from.rotation.inverse() * to.rotation).normalized();
  result.translation = from.rotation.inverse() * (to.translation - from.translation);
  return result;
}

bool samePose(const Pose3d & lhs, const Pose3d & rhs, double metres, double radians)
{
  if (!a2w_fastlio_common::isFinitePose(lhs) || !a2w_fastlio_common::isFinitePose(rhs)) {
    return false;
  }
  const double angle = Eigen::AngleAxisd(
    lhs.rotation.normalized().inverse() * rhs.rotation.normalized()).angle();
  return (lhs.translation - rhs.translation).norm() <= metres &&
         std::isfinite(angle) && angle <= radians;
}

class RevisitKeyFrameProvider final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  explicit RevisitKeyFrameProvider(const std::map<std::uint64_t, KeyFrame> & frames)
  : frames_{frames} {}

  std::optional<KeyFrame> get(std::uint64_t id) const override
  {
    const auto found = frames_.find(id);
    return found == frames_.end() ? std::nullopt : std::optional<KeyFrame>{found->second};
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    if (frames_.empty()) {return std::nullopt;}
    return std::pair<std::uint64_t, std::uint64_t>{
      frames_.begin()->first, frames_.rbegin()->first};
  }

private:
  const std::map<std::uint64_t, KeyFrame> & frames_;
};

struct HypothesisGroup
{
  LoopCandidate representative;
  std::vector<LoopCandidate> members;
};

struct ValidHypothesis
{
  std::size_t event_index;
  Pose3d correction;
  Pose3d corrected_pose;
};
}  // namespace

std::vector<LoopPipelineEvent> LoopPipeline::processRevisit(
  const KeyFrame & current, const Pose3d & predicted_pose,
  const std::vector<LoopCandidate> & candidates, const LoopProgressObserver & observer,
  double & source_local_map_ms, std::size_t & source_points)
{
  const auto notify = [&observer](LoopProgressStage stage, const LoopPipelineEvent & event) {
      if (observer) {
        try {observer(stage, event);} catch (...) {
          // Observers are diagnostic sinks, never part of acceptance.
        }
      }
    };
  const auto eventFor = [&current, &candidates, this](const LoopCandidate & candidate) {
      LoopPipelineEvent event;
      event.current_id = current.id;
      event.candidate_id = candidate.keyframe_id;
      event.candidate_rank = candidate.rank;
      event.descriptor_distance = candidate.distance;
      event.yaw_hint_rad = candidate.yaw_hint_rad;
      event.origin_candidate = candidate.keyframe_id < config_.origin_keyframe_count;
      event.candidate_count = candidates.size();
      return event;
    };
  std::vector<LoopPipelineEvent> events;
  std::map<std::uint64_t, bool> evaluated_candidates;
  const auto finish = [&events, &notify, &evaluated_candidates]() {
      for (auto & event : events) {
        const auto evaluated = evaluated_candidates.find(event.candidate_id);
        if (evaluated != evaluated_candidates.end()) {
          event.rejection_stage = std::string{evaluated->second ?
            "mandatory_competitor_verification" : "blind_discovery"} +
            (event.rejection_stage.empty() ? "" : ":" + event.rejection_stage);
        }
        notify(LoopProgressStage::CandidateFinished, event);
      }
      return events;
    };

  std::vector<HypothesisGroup> groups;
  for (const auto & candidate : candidates) {
    notify(LoopProgressStage::CandidateDetected, eventFor(candidate));
    const auto frame = keyframes_.find(candidate.keyframe_id);
    if (frame == keyframes_.end() || candidate.keyframe_id >= current.id) {
      auto event = eventFor(candidate);
      event.reason = "loop_keyframe_out_of_range";
      event.rejection_stage = "candidate";
      events.push_back(std::move(event));
      continue;
    }
    auto group = std::find_if(groups.begin(), groups.end(), [&](const auto & existing) {
        const auto id = existing.representative.keyframe_id;
        const auto difference = id > candidate.keyframe_id ?
          id - candidate.keyframe_id : candidate.keyframe_id - id;
        return difference <= config_.hypothesis_group_id_span &&
               (frame->second.optimized_pose.translation -
               keyframes_.at(id).optimized_pose.translation).norm() <=
               config_.hypothesis_group_distance_m;
      });
    if (group == groups.end()) {
      groups.push_back({candidate, {candidate}});
    } else {
      // Always compare against the original representative: never chain groups.
      group->members.push_back(candidate);
    }
  }

  const RevisitKeyFrameProvider provider{keyframes_};
  a2w_fastlio_common::CloudConstPtr source;
  std::vector<ValidHypothesis> valid;
  struct DeferredGroup
  {
    const HypothesisGroup * group;
    std::uint64_t retry_key;
  };
  std::vector<DeferredGroup> deferred_groups;
  bool unassessed_competitor = false;
  const a2w_fastlio_common::ValidationContext geometry_context{};
  for (auto state = retry_states_.begin(); state != retry_states_.end();) {
    if (keyframes_.find(state->first) == keyframes_.end() ||
      current.id < state->second.first_frame ||
      current.id - state->second.first_frame >= config_.retry_reset_keyframes)
    {
      state = retry_states_.erase(state);
    } else {
      ++state;
    }
  }
  const auto evaluateGroup = [&](const HypothesisGroup & group, std::uint64_t key, bool mandatory) {
    bool group_valid = false;
    bool geometry_attempted = false;
    bool group_evaluated = true;
    for (const auto & candidate : group.members) {
      auto event = eventFor(candidate);
      evaluated_candidates.emplace(candidate.keyframe_id, mandatory);
      event.rejection_stage = mandatory ? "mandatory_competitor_verification" : "blind_discovery";
      try {
        if (!source) {
          const auto started = std::chrono::steady_clock::now();
          const auto built = local_map_builder_.build(
            current.id, config_.local_map_before, config_.local_map_after, provider);
          source_local_map_ms = elapsedMs(started);
          source = built.cloud;
          source_points = source->size();
        }
        const auto target_started = std::chrono::steady_clock::now();
        const auto target = local_map_builder_.build(
          candidate.keyframe_id, config_.local_map_before, config_.local_map_after, provider);
        event.target_local_map_ms = elapsedMs(target_started);
        event.target_points = target.cloud->size();
        notify(LoopProgressStage::RegistrationStarted, event);
        geometry_attempted = true;
        // Both submaps are expressed in map. A body-frame SC yaw is not D_map_map.
        const auto result = registration_->run(source, target.cloud, std::nullopt, geometry_context);
        event.registration = result.fine;
        event.quatro_ms = result.coarse.elapsed_ms;
        event.nano_gicp_ms = result.fine.elapsed_ms;
        event.evidence_ms = result.coarse_evidence_ms + result.fine_evidence_ms;
        event.validator_ms = result.validation_ms;
        event.registration_total_ms = result.total_ms;
        event.reason = result.validation.reason;
        event.rejection_stage = "registration";
        bool pose_valid = false;
        Pose3d corrected;
        if (a2w_fastlio_common::isUsableRegistration(result.fine)) {
          corrected = compose(result.fine.transform, predicted_pose);
          auto pose_result = result.fine;
          pose_result.transform = corrected;
          a2w_fastlio_common::ValidationContext pose_context;
          pose_context.predicted_transform = predicted_pose;
          const auto pose_started = std::chrono::steady_clock::now();
          const auto validation = registration_->validateResult(pose_result, pose_context);
          event.validator_ms += elapsedMs(pose_started);
          pose_valid = validation.accepted;
          if (result.success) {
            event.reason = validation.reason;
            event.rejection_stage = pose_valid ? "" : "pose_guard";
          }
          const auto relative = between(predicted_pose, corrected);
          const auto rotation = relative.rotation.toRotationMatrix();
          const auto & limits = registration_->validationConfig();
          LoopPoseCheck check;
          check.predicted_pose = predicted_pose;
          check.corrected_pose = corrected;
          check.translation_difference_m = (corrected.translation - predicted_pose.translation).norm();
          check.rotation_difference_rad = Eigen::AngleAxisd(relative.rotation).angle();
          check.relative_rpy_rad = Eigen::Vector3d{
            std::atan2(rotation(2, 1), rotation(2, 2)),
            std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
            std::atan2(rotation(1, 0), rotation(0, 0))};
          check.translation_limit_m = limits.maximum_translation_jump_m;
          check.rotation_limit_rad = limits.maximum_rotation_jump_rad;
          check.position_pass = std::isfinite(check.translation_difference_m) &&
            check.translation_difference_m <= check.translation_limit_m;
          check.rotation_pass = std::isfinite(check.rotation_difference_rad) &&
            check.rotation_difference_rad <= check.rotation_limit_rad;
          event.pose_check = check;
        }
        if (result.success && pose_valid) {
          event.reason = "loop_geometry_valid";
          event.rejection_stage.clear();
          valid.push_back({events.size(), result.fine.transform, corrected});
          group_valid = true;
        }
      } catch (const std::exception & error) {
        event.reason = std::string{"candidate_processing_failed:"} + error.what();
        event.rejection_stage = "registration";
        group_evaluated = false;
      }
      events.push_back(std::move(event));
      // Historical proximity only shares the retry budget; it does not prove
      // that registrations agree. Evaluate every member of the bounded pool
      // before merging or rejecting their actual corrected-pose solutions.
    }
    if (group_valid) {
      retry_states_.erase(key);
    } else if (geometry_attempted) {
      // One failed observation of this group, independent of its member count.
      auto inserted = retry_states_.try_emplace(key, RetryState{current.id, current.id, 0U});
      inserted.first->second.last_attempt = current.id;
      if (inserted.first->second.attempts < config_.retry_max_attempts) {
        ++inserted.first->second.attempts;
      }
    }
    return group_evaluated;
  };
  for (const auto & group : groups) {
    // Fixed historical anchors keep retry budgets stable when ranks rotate.
    // This proximity check controls scheduling, never geometric equivalence.
    auto state = std::find_if(retry_states_.begin(), retry_states_.end(), [&](const auto & entry) {
        const auto id = group.representative.keyframe_id;
        const auto difference = id > entry.first ? id - entry.first : entry.first - id;
        return difference <= config_.hypothesis_group_id_span &&
               (keyframes_.at(id).optimized_pose.translation -
               keyframes_.at(entry.first).optimized_pose.translation).norm() <=
               config_.hypothesis_group_distance_m;
      });
    const auto key = state == retry_states_.end() ?
      group.representative.keyframe_id : state->first;
    const bool retry_blocked = state != retry_states_.end() &&
      (state->second.attempts >= config_.retry_max_attempts ||
      current.id <= state->second.last_attempt ||
      current.id - state->second.last_attempt < config_.retry_interval_keyframes);
    if (retry_blocked) {
      deferred_groups.push_back({&group, key});
    } else if (!evaluateGroup(group, key, false)) {
      unassessed_competitor = true;
    }
  }
  if (!valid.empty()) {
    // A positive hypothesis makes every deferred competitor mandatory on this
    // same source frame. Budget limits blind discovery, never this safety check.
    for (const auto & deferred : deferred_groups) {
      if (!evaluateGroup(*deferred.group, deferred.retry_key, true)) {
        unassessed_competitor = true;
      }
    }
  } else {
    for (const auto & deferred : deferred_groups) {
      for (const auto & candidate : deferred.group->members) {
        auto event = eventFor(candidate);
        event.reason = "loop_hypothesis_retry_deferred";
        event.rejection_stage = "retry";
        events.push_back(std::move(event));
      }
    }
  }

  if (valid.empty()) {
    const bool only_deferred = !events.empty() && std::all_of(
      events.begin(), events.end(), [](const auto & event) {
        return event.rejection_stage == "retry";
      });
    if (!only_deferred || (pending_confirmation_ &&
      (current.id < pending_confirmation_->last_frame ||
      current.id - pending_confirmation_->last_frame > config_.confirmation_max_gap)))
    {
      pending_confirmation_.reset();
    }
    return finish();
  }
  std::sort(valid.begin(), valid.end(), [&events](const auto & lhs, const auto & rhs) {
      const auto & left = events[lhs.event_index];
      const auto & right = events[rhs.event_index];
      if (left.registration.fitness != right.registration.fitness) {
        return left.registration.fitness < right.registration.fitness;
      }
      return left.candidate_id < right.candidate_id;
    });
  std::vector<ValidHypothesis> solutions;
  for (const auto & hypothesis : valid) {
    const auto existing = std::find_if(solutions.begin(), solutions.end(), [&](const auto & other) {
        return samePose(hypothesis.corrected_pose, other.corrected_pose,
          config_.solution_merge_distance_m, config_.solution_merge_rotation_rad);
      });
    const bool new_solution = existing == solutions.end();
    if (new_solution) {solutions.push_back(hypothesis);}
    auto & event = events[hypothesis.event_index];
    event.reason = new_solution ? "loop_alternative_not_selected" : "loop_same_solution";
    event.rejection_stage = "selection";
  }
  const auto & winner = solutions.front();
  const double best_fitness = events[winner.event_index].registration.fitness;
  const bool ambiguous = solutions.size() > 1U &&
    events[solutions[1].event_index].registration.fitness - best_fitness <
    config_.distinct_solution_fitness_margin_m;
  // An exception while assessing a competitor is not negative geometric
  // evidence; no unknown solution may be silently excluded from confirmation.
  if (ambiguous || unassessed_competitor) {
    if (ambiguous || (pending_confirmation_ &&
      (current.id < pending_confirmation_->last_frame ||
      current.id - pending_confirmation_->last_frame > config_.confirmation_max_gap)))
    {
      pending_confirmation_.reset();
    }
    for (const auto & hypothesis : valid) {
      auto & event = events[hypothesis.event_index];
      event.reason = ambiguous ? "loop_distinct_solutions_ambiguous" : "loop_competing_hypothesis_deferred";
      event.rejection_stage = "geometric_ambiguity";
    }
    return finish();
  }

  std::size_t confirmations = 1U;
  bool motion_pending = false;
  if (pending_confirmation_ && current.id > pending_confirmation_->last_frame &&
    current.id - pending_confirmation_->last_frame <= config_.confirmation_max_gap &&
    samePose(compose(pending_confirmation_->correction, predicted_pose),
    winner.corrected_pose, config_.solution_merge_distance_m, config_.solution_merge_rotation_rad))
  {
    const bool enough_motion = confirmationMotionSufficient(current.odom_pose);
    confirmations = pending_confirmation_->count + (enough_motion ? 1U : 0U);
    motion_pending = !enough_motion;
  }
  if (!motion_pending) {
    pending_confirmation_ = PendingLoop{winner.correction, current.id, confirmations, current.odom_pose};
  }
  auto & winner_event = events[winner.event_index];
  winner_event.confirmation_count = confirmations;
  winner_event.reason = motion_pending ? "loop_confirmation_motion_pending" : "loop_confirmation_pending";
  winner_event.rejection_stage = "confirmation";
  if (confirmations >= config_.confirmation_frames) {
    LoopConstraint constraint;
    constraint.from_id = current.id;
    constraint.to_id = winner_event.candidate_id;
    constraint.relative_pose = between(
      winner.corrected_pose, keyframes_.at(winner_event.candidate_id).optimized_pose);
    constraint.accepted = true;
    const auto queued = graph_->addLoopConstraint(constraint);
    winner_event.accepted = queued.accepted;
    winner_event.reason = queued.accepted ? "loop_accepted" : queued.reason;
    winner_event.rejection_stage = queued.accepted ? "" : "graph";
    pending_confirmation_.reset();
  }
  // Keep the selected frame's final result last, after all alternatives have
  // been evaluated. accepted means queued; process() owns the graph commit.
  if (winner.event_index + 1U != events.size()) {
    auto selected = std::move(winner_event);
    events.erase(events.begin() + static_cast<std::ptrdiff_t>(winner.event_index));
    events.push_back(std::move(selected));
  }
  return finish();
}

}  // namespace a2w_fastlio_mapping
