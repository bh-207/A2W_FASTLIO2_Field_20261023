#include "a2w_fastlio_mapping/loop_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_mapping
{
namespace
{

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

Pose3d yawHint(const double yaw_rad)
{
  Pose3d result;
  result.rotation = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());
  return result;
}

}  // namespace

std::string loopPoseCheckChinese(
  const LoopPipelineEvent & event, const std::string & map_frame,
  const std::string & tracking_frame)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(6)
       << "[回环验证] " << event.current_id << " -> " << event.candidate_id
       << "；双方均为 " << map_frame << " <- " << tracking_frame << " 的当前帧位姿";
  if (event.pose_check) {
    const auto & check = *event.pose_check;
    const double degrees = 180.0 / std::acos(-1.0);
    text << "\npredicted translation=[" << check.predicted_pose.translation.transpose()
         << "] loop translation=[" << check.corrected_pose.translation.transpose()
         << "] translation difference=" << check.translation_difference_m << " m"
         << "\npredicted rotation(xyzw)=[" << check.predicted_pose.rotation.coeffs().transpose()
         << "] loop rotation(xyzw)=[" << check.corrected_pose.rotation.coeffs().transpose()
         << "] rotation difference=" << check.rotation_difference_rad << " rad ("
         << check.rotation_difference_rad * degrees << " deg)"
         << " relative roll/pitch/yaw=[" << (check.relative_rpy_rad * degrees).transpose()
         << "] deg"
         << "\nposition_jump_check=" << (check.position_pass ? "PASS" : "REJECT")
         << " (limit=" << check.translation_limit_m << " m) rotation_jump_check="
         << (check.rotation_pass ? "PASS" : "REJECT")
         << " (limit=" << check.rotation_limit_rad << " rad)";
  } else {
    text << "\n位姿差值及跳变检查=N/A（配准未产生可用结果，禁止作为 PASS）";
  }
  text << "\nfitness=" << event.registration.fitness << " overlap=" << event.registration.overlap
       << " 最终结果=" << (event.accepted ? "ACCEPT（待图优化提交）" : "REJECT")
       << " reason=" << event.reason;
  return text.str();
}

std::string loopRejectionChinese(const std::string & reason)
{
  const std::pair<const char *, const char *> descriptions[] = {
    {"coarse_registration_failed", "Quatro 粗配准失败"},
    {"fine_registration_failed", "Nano-GICP 精配准失败（未收敛或结果无效）"},
    {"fitness_above_limit", "配准误差超过门限"},
    {"overlap_below_limit", "重叠率不足"},
    {"correspondences_below_limit", "有效对应点不足"},
    {"translation_jump_above_limit", "位置跳变超过门限"},
    {"rotation_jump_above_limit", "旋转跳变超过门限"},
    {"candidate_separation_below_limit", "候选歧义，最佳与次佳区分不足"},
    {"candidate_distances_invalid", "候选描述子距离无效"},
    {"registration_not_usable", "配准结果无效"},
    {"prediction_not_finite", "预测位姿无效"},
    {"candidate_processing_failed", "候选处理异常"},
    {"loop_keyframe_out_of_range", "回环关键帧编号越界"},
    {"loop_keyframes_not_distinct", "回环两端关键帧相同"},
    {"loop_pose_not_finite", "回环约束位姿无效"},
  };
  for (const auto & [prefix, description] : descriptions) {
    if (reason.rfind(prefix, 0U) == 0U) {
      return std::string{description} + "：" + reason;
    }
  }
  return "候选未通过：" + reason;
}

class LoopPipeline::StoredKeyFrameProvider final :
  public a2w_fastlio_common::KeyFrameProvider
{
public:
  explicit StoredKeyFrameProvider(
    const std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> & keyframes)
  : keyframes_{keyframes} {}

  std::optional<a2w_fastlio_common::KeyFrame> get(std::uint64_t id) const override
  {
    const auto iterator = keyframes_.find(id);
    return iterator == keyframes_.end() ? std::nullopt :
           std::optional<a2w_fastlio_common::KeyFrame>{iterator->second};
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    if (keyframes_.empty()) {
      return std::nullopt;
    }
    return std::pair<std::uint64_t, std::uint64_t>{
      keyframes_.begin()->first, keyframes_.rbegin()->first};
  }

private:
  const std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> & keyframes_;
};

LoopPipeline::LoopPipeline(
  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition,
  std::shared_ptr<a2w_fastlio_common::DescriptorIndex> descriptor_index,
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
  std::shared_ptr<PoseGraphOptimizer> graph,
  a2w_fastlio_common::LocalMapBuilder local_map_builder,
  LoopPipelineConfig config)
: place_recognition_{std::move(place_recognition)}, descriptor_index_{std::move(descriptor_index)},
  registration_{std::move(registration)}, graph_{std::move(graph)},
  local_map_builder_{std::move(local_map_builder)}, config_{config}
{
  if (!place_recognition_ || !descriptor_index_ || !registration_ || !graph_ ||
    config_.top_k == 0U || config_.minimum_keyframes_between_accepted_loops == 0U ||
    config_.queue_capacity == 0U || !std::isfinite(config_.maximum_descriptor_distance) ||
    config_.maximum_descriptor_distance <= 0.0 || config_.origin_keyframe_count == 0U ||
    config_.origin_top_k == 0U || config_.confirmation_frames == 0U ||
    config_.confirmation_max_gap == 0U || config_.retry_interval_keyframes == 0U ||
    config_.retry_max_attempts == 0U || config_.retry_reset_keyframes == 0U ||
    !std::isfinite(config_.hypothesis_group_distance_m) || config_.hypothesis_group_distance_m <= 0.0 ||
    !std::isfinite(config_.solution_merge_distance_m) || config_.solution_merge_distance_m <= 0.0 ||
    !std::isfinite(config_.solution_merge_rotation_rad) || config_.solution_merge_rotation_rad <= 0.0 ||
    !std::isfinite(config_.distinct_solution_fitness_margin_m) ||
    config_.distinct_solution_fitness_margin_m < 0.0 || config_.discovery_interval_keyframes == 0U ||
    !std::isfinite(config_.confirmation_min_translation_m) || config_.confirmation_min_translation_m < 0.0 ||
    !std::isfinite(config_.confirmation_min_rotation_rad) || config_.confirmation_min_rotation_rad < 0.0 ||
    config_.confirmation_min_rotation_rad > std::acos(-1.0))
  {
    throw std::invalid_argument{"invalid loop-pipeline dependencies or configuration"};
  }
  if (config_.revocable_review_enabled) {
    if (config_.causal_geometry_guard_enabled || config_.revisit_verification_enabled) {
      throw std::invalid_argument{"revocable review requires conservative path without legacy guard"};
    }
    if (config_.revocable_review_ledger_path.empty() || std::filesystem::exists(config_.revocable_review_ledger_path)) {
      throw std::invalid_argument{"revocable review requires a fresh ledger path"};
    }
    std::ofstream ledger(config_.revocable_review_ledger_path);
    if (!ledger) {throw std::runtime_error{"cannot create revocable review ledger"};}
  }
  if (config_.causal_geometry_guard_enabled) {
    if (config_.revisit_verification_enabled) {
      throw std::invalid_argument{"geometry guard is validated only on conservative loop path"};
    }
    if (config_.causal_geometry_guard_ledger_path.empty() ||
        std::filesystem::exists(config_.causal_geometry_guard_ledger_path)) {
      throw std::invalid_argument{"geometry guard requires a fresh ledger path"};
    }
    std::ofstream ledger(config_.causal_geometry_guard_ledger_path);
    if (!ledger) {throw std::runtime_error{"cannot create geometry guard ledger"};}
  }
}

bool LoopPipeline::confirmationMotionSufficient(const Pose3d & raw_odom) const
{
  if (!pending_confirmation_ || (config_.confirmation_min_translation_m == 0.0 &&
    config_.confirmation_min_rotation_rad == 0.0)) {return true;}
  const auto & previous = pending_confirmation_->odom_pose;
  const double translation = (raw_odom.translation - previous.translation).norm();
  const double angle = Eigen::AngleAxisd(previous.rotation.normalized().inverse() *
    raw_odom.rotation.normalized()).angle();
  // A zero threshold disables that axis. The two enabled axes use OR.
  return (config_.confirmation_min_translation_m > 0.0 && translation >= config_.confirmation_min_translation_m) ||
         (config_.confirmation_min_rotation_rad > 0.0 && angle >= config_.confirmation_min_rotation_rad);
}

std::vector<LoopPipelineEvent> LoopPipeline::process(
  const a2w_fastlio_common::KeyFrame & current, const LoopProgressObserver & observer)
{
  const auto notify = [&observer](LoopProgressStage stage, const LoopPipelineEvent & event) {
      if (observer) {
        try {observer(stage, event);} catch (...) {
          // A diagnostic sink must never alter registration/graph processing.
        }
      }
    };
  const auto frame_started = std::chrono::steady_clock::now();
  std::vector<LoopPipelineEvent> events;
  if (current.id != keyframes_.size()) {
    const auto graph_state = graph_->addKeyFrame(current.id, current.odom_pose);
    events.push_back({
      current.id, 0U, 0U, false, graph_state.reason, {}, graph_state.factor_count});
    return events;
  }

  a2w_fastlio_common::ScanDescriptor descriptor{1U, 1U, {0.0F}};
  const auto descriptor_started = std::chrono::steady_clock::now();
  double descriptor_ms = 0.0;
  try {
    descriptor = place_recognition_->describe(current.body_cloud);
    descriptor_ms = elapsedMs(descriptor_started);
  } catch (const std::exception & error) {
    LoopPipelineEvent event{
      current.id, 0U, 0U, false, std::string{"descriptor_failed:"} + error.what(), {},
      keyframes_.size()};
    event.descriptor_ms = elapsedMs(descriptor_started);
    event.frame_total_ms = elapsedMs(frame_started);
    events.push_back(std::move(event));
    return events;
  }

  const auto graph_add = graph_->addKeyFrame(current.id, current.odom_pose);
  if (!graph_add.accepted) {
    LoopPipelineEvent event{
      current.id, 0U, 0U, false, graph_add.reason, {}, graph_add.factor_count};
    event.descriptor_ms = descriptor_ms;
    event.frame_total_ms = elapsedMs(frame_started);
    events.push_back(std::move(event));
    return events;
  }
  // O = T_camera_init_body, P = T_map_body. Propagate the latest committed
  // optimized pose with the same-source-stamp odometry increment. Never put
  // raw camera_init coordinates into a LocalMap made of optimized map poses.
  auto predicted_current = current;
  if (!keyframes_.empty()) {
    const auto & previous = keyframes_.rbegin()->second;
    predicted_current.optimized_pose = compose(
      previous.optimized_pose, between(previous.odom_pose, current.odom_pose));
  } else {
    predicted_current.optimized_pose = current.odom_pose;
  }
  const Pose3d predicted_pose = predicted_current.optimized_pose;
  keyframes_.emplace(current.id, std::move(predicted_current));

  std::optional<GraphUpdate> replacement_update;
  const bool cooldown = last_accepted_loop_id_ &&
    current.id < *last_accepted_loop_id_ + config_.minimum_keyframes_between_accepted_loops;
  if (config_.revisit_verification_enabled && pending_confirmation_ &&
    current.id > pending_confirmation_->last_frame &&
    current.id - pending_confirmation_->last_frame > config_.confirmation_max_gap)
  {
    pending_confirmation_.reset();
    // An expired pending observation starts a fresh discovery opportunity;
    // motion-wait frames never refreshed either anchor.
    last_discovery_keyframe_.reset();
  }
  const bool confirmation_motion_waiting = config_.revisit_verification_enabled &&
    pending_confirmation_ && !confirmationMotionSufficient(current.odom_pose);
  const bool discovery_deferred = config_.revisit_verification_enabled &&
    !pending_confirmation_ && last_discovery_keyframe_ &&
    current.id - *last_discovery_keyframe_ < config_.discovery_interval_keyframes;
  std::vector<a2w_fastlio_common::LoopCandidate> candidates;
  double scan_context_ms = 0.0;
  if (!cooldown && !confirmation_motion_waiting && !discovery_deferred && current.id > 0U) {
    if (config_.revisit_verification_enabled) {last_discovery_keyframe_ = current.id;}
    const auto query_started = std::chrono::steady_clock::now();
    a2w_fastlio_common::CandidateFilter filter;
    filter.max_inclusive_id = current.id - 1U;
    filter.exclude_recent = config_.exclude_recent;
    candidates = descriptor_index_->queryTopK(descriptor, config_.top_k, filter);
    if (config_.revisit_verification_enabled &&
      current.id >= config_.origin_keyframe_count &&
      current.id - config_.origin_keyframe_count >= config_.exclude_recent)
    {
      // Independent descriptor retrieval; NOT an odometry-based "back at start" trigger.
      a2w_fastlio_common::CandidateFilter origin_filter;
      origin_filter.max_inclusive_id = std::min<std::uint64_t>(
        config_.origin_keyframe_count - 1U, current.id - 1U - config_.exclude_recent);
      origin_filter.exclude_recent = 0U;
      for (const auto & candidate : descriptor_index_->queryTopK(
          descriptor, config_.origin_top_k, origin_filter))
      {
        const auto present = std::any_of(candidates.begin(), candidates.end(),
          [&candidate](const auto & value) {return value.keyframe_id == candidate.keyframe_id;});
        if (!present && candidate.keyframe_id <= origin_filter.max_inclusive_id) {
          candidates.push_back(candidate);
        }
      }
      std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
        return a.distance == b.distance ? a.keyframe_id < b.keyframe_id : a.distance < b.distance;
      });
    }
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [this](const auto & candidate) {
          return !std::isfinite(candidate.distance) || candidate.distance < 0.0 ||
                 candidate.distance > config_.maximum_descriptor_distance;
        }),
      candidates.end());
    scan_context_ms = elapsedMs(query_started);
  }

  double source_local_map_ms = 0.0;
  std::size_t source_points = 0U;

  if (cooldown) {
    events.push_back({current.id, 0U, 0U, false, "loop_policy_cooldown"});
  } else if (confirmation_motion_waiting) {
    events.push_back({current.id, 0U, 0U, false, "loop_confirmation_motion_waiting"});
    events.back().confirmation_count = pending_confirmation_->count;
    events.back().rejection_stage = "confirmation";
  } else if (discovery_deferred) {
    events.push_back({current.id, 0U, 0U, false, "loop_discovery_interval_pending"});
  } else if (candidates.empty()) {
    events.push_back({current.id, 0U, 0U, false, "no_loop_candidates"});
  } else if (config_.revisit_verification_enabled) {
    events = processRevisit(
      current, predicted_pose, candidates, observer, source_local_map_ms, source_points);
  } else {
    for (const auto & candidate : candidates) {
      notify(LoopProgressStage::CandidateDetected,
        {current.id, candidate.keyframe_id, candidate.rank});
    }
    const StoredKeyFrameProvider provider{keyframes_};
    a2w_fastlio_common::ValidationContext validation_context;
    validation_context.best_descriptor_distance = candidates.front().distance;
    if (candidates.size() > 1U) {
      validation_context.second_descriptor_distance = candidates[1].distance;
    }
    const auto context_validation = registration_->validateCandidateContext(validation_context);
    if (!context_validation.accepted) {
      for (const auto & candidate : candidates) {
        events.push_back({
          current.id, candidate.keyframe_id, candidate.rank, false, context_validation.reason});
        notify(LoopProgressStage::CandidateFinished, events.back());
      }
    } else {
      const auto source_started = std::chrono::steady_clock::now();
      const auto source = local_map_builder_.build(
        current.id, config_.local_map_before, config_.local_map_after, provider);
      source_local_map_ms = elapsedMs(source_started);
      source_points = source.cloud->size();
      for (const auto & candidate : candidates) {
        LoopPipelineEvent event;
        event.current_id = current.id;
        event.candidate_id = candidate.keyframe_id;
        event.candidate_rank = candidate.rank;
        try {
          const auto target_started = std::chrono::steady_clock::now();
          const auto target = local_map_builder_.build(
            candidate.keyframe_id, config_.local_map_before, config_.local_map_after, provider);
          event.target_local_map_ms = elapsedMs(target_started);
          event.target_points = target.cloud->size();
          event.source_submap_ids = source.included_ids;
          event.target_submap_ids = target.included_ids;
          event.registration_initial_guess = yawHint(candidate.yaw_hint_rad);
          notify(LoopProgressStage::RegistrationStarted, event);
          const auto registration = registration_->run(
            source.cloud, target.cloud, yawHint(candidate.yaw_hint_rad), validation_context);
          event.registration = registration.fine;
          event.coarse_transform = registration.coarse.transform;
          event.quatro_ms = registration.coarse.elapsed_ms;
          event.nano_gicp_ms = registration.fine.elapsed_ms;
          event.evidence_ms = registration.coarse_evidence_ms + registration.fine_evidence_ms;
          event.validator_ms = registration.validation_ms;
          event.registration_total_ms = registration.total_ms;
          event.reason = registration.validation.reason;
          bool pose_accepted = false;
          Pose3d corrected_current;
          if (a2w_fastlio_common::isUsableRegistration(registration.fine)) {
            // Both LocalMaps are in map. Registration D maps source-map points
            // to target-map points; D itself is NOT a robot pose.
            corrected_current = compose(registration.fine.transform, predicted_pose);
            auto pose_result = registration.fine;
            pose_result.transform = corrected_current;
            auto pose_context = validation_context;
            pose_context.predicted_transform = predicted_pose;
            const auto pose_validation = registration_->validateResult(pose_result, pose_context);
            pose_accepted = pose_validation.accepted;
            if (registration.success) {event.reason = pose_validation.reason;}

            const auto relative = between(predicted_pose, corrected_current);
            const auto rotation = relative.rotation.toRotationMatrix();
            const auto & limits = registration_->validationConfig();
            LoopPoseCheck check;
            check.predicted_pose = predicted_pose;
            check.corrected_pose = corrected_current;
            check.translation_difference_m =
              (corrected_current.translation - predicted_pose.translation).norm();
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
          if (registration.success && pose_accepted) {
            LoopConstraint constraint;
            constraint.from_id = current.id;
            constraint.to_id = candidate.keyframe_id;
            constraint.relative_pose = between(
              corrected_current, keyframes_.at(candidate.keyframe_id).optimized_pose);
            constraint.accepted = true;
            event.evaluated_constraint = constraint;
            if (config_.revocable_review_enabled) {
              ReviewEdge edge;edge.constraint=constraint;
              edge.registration_ids.insert(event.source_submap_ids.begin(),event.source_submap_ids.end());
              edge.registration_ids.insert(event.target_submap_ids.begin(),event.target_submap_ids.end());
              auto pool=review_pool_;
              std::ofstream ledger(config_.revocable_review_ledger_path,std::ios::app);
              if(!ledger)throw LoopReviewPersistenceError{"review_ledger_open_failed"};
              if(!appendReviewEdgeBounded(pool,edge,active_review_edges_,current.id,ledger)){
                if(!ledger)throw LoopReviewPersistenceError{"review_pending_ledger_write_failed"};
                event.reason="revocable_review_pending_capacity_or_ledger_unknown";
                events.push_back(event);notify(LoopProgressStage::CandidateFinished,event);break;
              }
              review_pool_=pool; // bounded deferred measurements, never graph factors
              const auto reviewed=reviewLoopSets(*graph_,keyframes_,current.id,pool,active_review_edges_,ledger);
              if(!ledger)throw LoopReviewPersistenceError{"review_assessment_ledger_write_failed"};
              event.reason=reviewed.reason;
              if(reviewed.selected){
                std::vector<LoopConstraint> selected;
                for(const auto&e:reviewed.selected_edges)selected.push_back(e.constraint);
                auto same_set=[&]{if(selected.size()!=active_review_edges_.size())return false;
                  for(const auto&e:active_review_edges_)if(std::none_of(selected.begin(),selected.end(),[&](const auto&c){return c.from_id==e.constraint.from_id&&c.to_id==e.constraint.to_id;}))return false;return true;};
                if(!same_set()){
                  event.active_constraints=selected;
                  for(const auto&e:active_review_edges_)if(std::none_of(selected.begin(),selected.end(),[&](const auto&c){return c.from_id==e.constraint.from_id&&c.to_id==e.constraint.to_id;}))event.withdrawn_constraints.push_back(e.constraint);
                  const auto committed=commitReviewLoopSet(*graph_,active_review_edges_,reviewed.selected_edges,current.id,ledger);
                  if(committed.accepted){
                    replacement_update=committed;event.replaced_loop_set=true;
                    event.accepted=std::any_of(selected.begin(),selected.end(),[&](const auto&c){return c.from_id==constraint.from_id&&c.to_id==constraint.to_id;});
                    if(event.accepted)event.queued_constraint=constraint;
                    event.reason="revocable_review_set_committed";
                  }else event.reason=committed.reason;
                }else event.reason="revocable_review_retained_existing_set";
              }
              events.push_back(event);notify(LoopProgressStage::CandidateFinished,event);
              // The conservative path stops after its first validated measurement,
              // including deferred measurements; do not refill the review pool
              // with alternate matches from the same source observation.
              break;
            }
            if (config_.causal_geometry_guard_enabled) {
              const auto trial = graph_->previewLoopConstraint(constraint);
              if (!trial.update.accepted) {
                event.reason = "geometry_trial_failed:"+trial.update.reason;
                events.push_back(event);notify(LoopProgressStage::CandidateFinished,event);continue;
              }
              auto before = graph_->optimizedPoses();
              before.poses.push_back({current.id,predicted_pose});
              std::set<std::uint64_t> excluded(event.source_submap_ids.begin(),event.source_submap_ids.end());
              excluded.insert(event.target_submap_ids.begin(),event.target_submap_ids.end());
              const auto check=CausalGeometryGuard{}.assess(keyframes_,before,trial.snapshot,
                current.id,candidate.keyframe_id,excluded);
              std::ofstream ledger(config_.causal_geometry_guard_ledger_path,std::ios::app);
              writeCausalGeometryGuardLedger(ledger,check);ledger.flush();
              if (!ledger) {throw std::runtime_error{"geometry guard ledger write failed"};}
              if (!check.accepted()) {
                event.reason="causal_geometry_"+std::string(causalGeometryGuardVerdictName(check.verdict))+":"+check.reason;
                events.push_back(event);notify(LoopProgressStage::CandidateFinished,event);continue;
              }
            }
            const auto loop_add = graph_->addLoopConstraint(constraint);
            if (loop_add.accepted) {event.queued_constraint = constraint;}
            event.accepted = loop_add.accepted;
            event.reason = loop_add.accepted ? "loop_accepted" : loop_add.reason;
            events.push_back(event);
            notify(LoopProgressStage::CandidateFinished, event);
            if (loop_add.accepted) {
              break;
            }
            continue;
          }
        } catch (const LoopReviewPersistenceError &) {
          throw; // backend marks failure_ and refuses save; never swallow durable-ledger failure
        } catch (const std::exception & error) {
          event.reason = std::string{"candidate_processing_failed:"} + error.what();
        }
        events.push_back(event);
        notify(LoopProgressStage::CandidateFinished, event);
      }
    }
  }

  descriptor_index_->add(current.id, descriptor);
  const auto accepted_loop = std::find_if(events.begin(), events.end(),
    [](const auto & event) {return event.accepted;});
  if (accepted_loop != events.end()) {
    notify(LoopProgressStage::OptimizationStarted, *accepted_loop);
  }
  const auto graph_started = std::chrono::steady_clock::now();
  const auto update = replacement_update ? *replacement_update : graph_->update();
  const double gtsam_ms = elapsedMs(graph_started);
  for (auto & event : events) {
    event.graph_factor_count = update.factor_count;
    event.candidate_count = candidates.size();
    event.source_points = source_points;
    event.descriptor_ms = descriptor_ms;
    event.scan_context_ms = scan_context_ms;
    event.source_local_map_ms = source_local_map_ms;
    event.gtsam_ms = gtsam_ms;
    const auto candidate = std::find_if(candidates.begin(), candidates.end(),
      [&event](const auto & value) {return value.keyframe_id == event.candidate_id;});
    if (candidate != candidates.end()) {
      event.descriptor_distance = candidate->distance;
      event.yaw_hint_rad = candidate->yaw_hint_rad;
      event.origin_candidate = candidate->keyframe_id < config_.origin_keyframe_count;
    }
    if (event.rejection_stage.empty()) {
      if (event.accepted) {event.rejection_stage = "graph_commit";}
      else if (event.reason.rfind("coarse_registration_failed", 0U) == 0U) {event.rejection_stage = "quatro";}
      else if (event.reason.rfind("fine_registration_failed", 0U) == 0U) {event.rejection_stage = "nano_gicp";}
      else if (event.target_points > 0U) {event.rejection_stage = "validation";}
      else {event.rejection_stage = "retrieval_or_policy";}
    }
  }
  if (update.accepted) {
    for (const auto & optimized : graph_->optimizedPoses().poses) {
      keyframes_.at(optimized.id).optimized_pose = optimized.pose;
    }
    if (accepted_loop != events.end()) {
      last_accepted_loop_id_ = current.id;
    }
  } else if (!events.empty() && events.back().reason.empty()) {
    events.back().reason = update.reason;
  }
  if (accepted_loop != events.end()) {
    auto completion = *accepted_loop;
    completion.accepted = update.accepted;
    if (update.accepted) {
      completion.optimized_pose_change = between(
        predicted_pose, keyframes_.at(current.id).optimized_pose);
    } else {
      completion.reason = update.reason;
    }
    notify(LoopProgressStage::OptimizationFinished, completion);
    // Queuing a factor is not the same as committing it to iSAM2.
    // Return no accepted event (and apply no cooldown) if the update failed.
    if (!update.accepted) {
      accepted_loop->accepted = false;
      accepted_loop->reason = update.reason;
    }
  }
  const double frame_total_ms = elapsedMs(frame_started);
  for (auto & event : events) {
    event.frame_total_ms = frame_total_ms;
  }
  return events;
}

bool LoopPipeline::enqueue(a2w_fastlio_common::KeyFrame keyframe)
{
  std::lock_guard<std::mutex> lock{queue_mutex_};
  if (queue_.size() >= config_.queue_capacity) {
    return false;
  }
  queue_.push_back(std::move(keyframe));
  return true;
}

std::optional<std::vector<LoopPipelineEvent>> LoopPipeline::processNext()
{
  a2w_fastlio_common::KeyFrame keyframe;
  {
    std::lock_guard<std::mutex> lock{queue_mutex_};
    if (queue_.empty()) {
      return std::nullopt;
    }
    keyframe = std::move(queue_.front());
    queue_.pop_front();
  }
  return process(keyframe);
}

std::size_t LoopPipeline::pending() const noexcept
{
  std::lock_guard<std::mutex> lock{queue_mutex_};
  return queue_.size();
}

}  // namespace a2w_fastlio_mapping
