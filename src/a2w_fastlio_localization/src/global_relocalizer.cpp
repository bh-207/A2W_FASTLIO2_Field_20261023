#include "a2w_fastlio_localization/global_relocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

namespace a2w_fastlio_localization
{
namespace
{

using a2w_fastlio_common::Pose3d;

void validate(const RelocalizationConfig & config)
{
  const auto finite_nonnegative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (config.top_k < 2U || !finite_nonnegative(config.maximum_descriptor_distance) ||
    !finite_nonnegative(config.minimum_score_margin) ||
    !finite_nonnegative(config.descriptor_score_weight) ||
    !finite_nonnegative(config.fitness_score_weight) ||
    !finite_nonnegative(config.overlap_score_weight) ||
    config.fitness_score_weight + config.overlap_score_weight <= 0.0 ||
    !finite_nonnegative(config.strong_maximum_fitness) ||
    !finite_nonnegative(config.maximum_translation_difference_m) ||
    !finite_nonnegative(config.maximum_rotation_difference_rad) ||
    config.maximum_rotation_difference_rad > M_PI ||
    !std::isfinite(config.strong_minimum_overlap) || config.strong_minimum_overlap < 0.0 ||
    config.strong_minimum_overlap > 1.0 || config.strong_minimum_correspondences == 0U ||
    (config.candidate_region.enabled &&
    (!std::isfinite(config.candidate_region.center_x) ||
    !std::isfinite(config.candidate_region.center_y) ||
    !std::isfinite(config.candidate_region.size_x) ||
    !std::isfinite(config.candidate_region.size_y) ||
    config.candidate_region.size_x <= 0.0 || config.candidate_region.size_y <= 0.0 ||
    config.candidate_region.keyframe_window == 0U)))
  {
    throw std::invalid_argument{"invalid global relocalization configuration"};
  }
}

bool inCandidateRegion(
  const a2w_fastlio_common::LoopCandidate & candidate,
  const CandidateSearchRegion & region,
  const a2w_fastlio_common::KeyFrameProvider & keyframes)
{
  const auto lower_id = region.anchor_keyframe_id > region.keyframe_window ?
    region.anchor_keyframe_id - region.keyframe_window : 0U;
  const auto maximum_id = std::numeric_limits<std::uint64_t>::max();
  const auto upper_id = region.anchor_keyframe_id > maximum_id - region.keyframe_window ?
    maximum_id : region.anchor_keyframe_id + region.keyframe_window;
  if (candidate.keyframe_id < lower_id || candidate.keyframe_id > upper_id) {
    return false;
  }
  const auto keyframe = keyframes.get(candidate.keyframe_id);
  if (!keyframe) {
    return false;
  }
  const auto half_x = region.size_x * 0.5;
  const auto half_y = region.size_y * 0.5;
  const auto x = keyframe->optimized_pose.translation.x();
  const auto y = keyframe->optimized_pose.translation.y();
  return std::isfinite(x) && std::isfinite(y) &&
         std::abs(x - region.center_x) <= half_x &&
         std::abs(y - region.center_y) <= half_y;
}

bool strong(const CandidateAudit & audit, const RelocalizationConfig & config)
{
  return audit.registration.fitness <= config.strong_maximum_fitness &&
         audit.registration.overlap >= config.strong_minimum_overlap &&
         audit.registration.correspondence_count >= config.strong_minimum_correspondences;
}

bool samePose(
  const Pose3d & lhs, const Pose3d & rhs,
  const double translation_limit, const double rotation_limit)
{
  if (!a2w_fastlio_common::isFinitePose(lhs) || !a2w_fastlio_common::isFinitePose(rhs)) {
    return false;
  }
  const double rotation = Eigen::AngleAxisd{lhs.rotation.inverse() * rhs.rotation}.angle();
  return std::isfinite(rotation) && rotation <= rotation_limit &&
         (lhs.translation - rhs.translation).norm() <= translation_limit;
}

std::string rejectionLayer(
  const RelocalizationResult & result,
  const RelocalizationSessionSnapshot & session)
{
  if (session.confirmed_correction) {
    return "无（多帧确认完成）";
  }
  if (result.reason == "candidate_score_ambiguous") {
    return "ambiguity / best-second margin";
  }
  if (result.success) {
    return "multi-frame confirmation";
  }
  if (result.audits.empty() || std::all_of(
      result.audits.begin(), result.audits.end(), [](const auto & audit) {
        return !audit.evaluated && audit.reason != "candidate_local_map_invalid";
      }))
  {
    return "Scan Context 无有效候选";
  }
  if (std::none_of(result.audits.begin(), result.audits.end(), [](const auto & audit) {
      return audit.evaluated;
    }))
  {
    return "candidate Local Map";
  }
  if (std::none_of(result.audits.begin(), result.audits.end(), [](const auto & audit) {
      return a2w_fastlio_common::isUsableRegistration(audit.coarse);
    }))
  {
    return "Quatro";
  }
  if (std::none_of(result.audits.begin(), result.audits.end(), [](const auto & audit) {
      return audit.registration.converged;
    }))
  {
    return "Nano-GICP";
  }
  return "MatchValidator";
}

}  // namespace

GlobalRelocalizer::GlobalRelocalizer(
  std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index,
  std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes,
  std::shared_ptr<const a2w_fastlio_common::CoarseRegistration> coarse,
  std::shared_ptr<const a2w_fastlio_common::FineRegistration> fine,
  a2w_fastlio_common::MatchValidator validator,
  a2w_fastlio_common::LocalMapConfig local_map_config,
  a2w_fastlio_common::RegistrationPipelineConfig pipeline_config)
: descriptor_index_{std::move(descriptor_index)}, keyframes_{std::move(keyframes)},
  local_map_builder_{local_map_config},
  registration_{std::make_shared<a2w_fastlio_common::RegistrationPipeline>(
      std::move(coarse), std::move(fine), std::move(validator), pipeline_config)}
{
  if (!descriptor_index_ || !keyframes_) {
    throw std::invalid_argument{"global relocalizer requires descriptor and keyframe interfaces"};
  }
}

GlobalRelocalizer::GlobalRelocalizer(
  std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index,
  std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes,
  std::shared_ptr<const a2w_fastlio_common::RegistrationPipeline> registration,
  a2w_fastlio_common::LocalMapConfig local_map_config)
: descriptor_index_{std::move(descriptor_index)}, keyframes_{std::move(keyframes)},
  local_map_builder_{local_map_config}, registration_{std::move(registration)}
{
  if (!descriptor_index_ || !keyframes_ || !registration_) {
    throw std::invalid_argument{"global relocalizer requires common algorithm interfaces"};
  }
}

RelocalizationResult GlobalRelocalizer::evaluate(
  const a2w_fastlio_common::CloudConstPtr & current_local_map,
  const a2w_fastlio_common::ScanDescriptor & query_descriptor,
  const RelocalizationConfig & config) const
{
  validate(config);
  if (!current_local_map || current_local_map->empty()) {
    throw std::invalid_argument{"global relocalization query cloud must not be empty"};
  }
  RelocalizationResult result;
  result.source_points = current_local_map->size();
  auto candidates = descriptor_index_->queryTopK(
    query_descriptor,
    config.candidate_region.enabled ?
    std::numeric_limits<std::size_t>::max() : config.top_k,
    a2w_fastlio_common::CandidateFilter{});
  if (config.candidate_region.enabled) {
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [this, &config](const auto & candidate) {
          return !inCandidateRegion(candidate, config.candidate_region, *keyframes_);
        }),
      candidates.end());
    if (candidates.size() > config.top_k) {
      candidates.resize(config.top_k);
    }
    for (std::size_t rank = 0U; rank < candidates.size(); ++rank) {
      candidates[rank].rank = rank;
    }
  }
  std::vector<std::size_t> accepted;
  result.audits.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    CandidateAudit audit;
    audit.candidate = candidate;
    std::optional<a2w_fastlio_common::KeyFrame> center;
    try {
      center = keyframes_->get(candidate.keyframe_id);
      if (center) {
        audit.candidate_map_pose = center->optimized_pose;
      }
    } catch (const std::exception &) {
      // The candidate will retain its normal validation outcome below.
    }
    if (!std::isfinite(candidate.distance) || candidate.distance < 0.0 ||
      candidate.distance > config.maximum_descriptor_distance)
    {
      audit.reason = "descriptor_distance_above_limit";
      result.audits.push_back(std::move(audit));
      continue;
    }
    try {
      if (!center) {
        throw std::runtime_error{"candidate keyframe is missing"};
      }
      const auto target = local_map_builder_.build(
        candidate.keyframe_id, config.neighbor_keyframes_before,
        config.neighbor_keyframes_after, *keyframes_);
      audit.target_points = target.cloud ? target.cloud->size() : 0U;
      Pose3d hint = center->optimized_pose;
      hint.rotation = (hint.rotation * Eigen::Quaterniond{
        Eigen::AngleAxisd{candidate.yaw_hint_rad, Eigen::Vector3d::UnitZ()}}).normalized();
      const auto pipeline = registration_->run(current_local_map, target.cloud, hint, {});
      audit.evaluated = true;
      audit.accepted = pipeline.success;
      audit.reason = pipeline.validation.reason;
      audit.coarse = pipeline.coarse;
      audit.registration = pipeline.fine;
      audit.map_body = pipeline.final_transform;
      if (audit.accepted) {
        audit.score = config.descriptor_score_weight * candidate.distance +
          config.fitness_score_weight * audit.registration.fitness +
          config.overlap_score_weight * (1.0 - audit.registration.overlap);
      }
    } catch (const std::exception &) {
      audit.reason = "candidate_local_map_invalid";
    }
    result.audits.push_back(std::move(audit));
    if (result.audits.back().accepted) {
      accepted.push_back(result.audits.size() - 1U);
    }
  }
  if (accepted.empty()) {
    result.reason = candidates.empty() ? "no_candidates" : "no_geometrically_valid_candidates";
    return result;
  }
  std::stable_sort(accepted.begin(), accepted.end(), [&result](const auto lhs, const auto rhs) {
      const auto & left = result.audits[lhs];
      const auto & right = result.audits[rhs];
      return left.score != right.score ? left.score < right.score :
             left.candidate.keyframe_id < right.candidate.keyframe_id;
    });
  const auto & best = result.audits[accepted.front()];
  result.best_candidate_id = best.candidate.keyframe_id;
  result.best_score = best.score;
  // Compare every final map->body pose directly to the winner. Do not cluster
  // by keyframe origin or transitively merge chains of nearby hypotheses.
  for (const auto index : accepted) {
    const auto & alternative = result.audits[index];
    if (samePose(best.map_body, alternative.map_body,
      config.maximum_translation_difference_m, config.maximum_rotation_difference_rad))
    {
      ++result.best_hypothesis_candidates;
    } else {
      result.second_best_score = std::min(result.second_best_score, alternative.score);
    }
  }
  if (result.second_best_score - result.best_score < config.minimum_score_margin) {
    result.reason = "candidate_score_ambiguous";
    return result;
  }
  result.success = true;
  result.strong = strong(best, config);
  result.reason = result.strong ? "strong_candidate_accepted" : "candidate_accepted";
  result.candidate_id = best.candidate.keyframe_id;
  result.map_body = best.map_body;
  result.registration = best.registration;
  return result;
}

std::string formatInitializationDiagnostic(
  const RelocalizationResult & result,
  const RelocalizationSessionSnapshot & session)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << "【全局初始化摘要】source_points=" << result.source_points << '\n';
  stream << "【Top-K候选】";
  if (result.audits.empty()) {
    stream << "无";
  }
  for (const auto & audit : result.audits) {
    stream << " id=" << audit.candidate.keyframe_id << " SC=" << audit.candidate.distance <<
      " yaw=" << audit.candidate.yaw_hint_rad << "rad map_xy=";
    if (audit.candidate_map_pose) {
      stream << '(' << audit.candidate_map_pose->translation.x() << ',' <<
        audit.candidate_map_pose->translation.y() << ')';
    } else {
      stream << "unknown";
    }
    stream << ';';
  }
  stream << '\n' << "【各候选结果】";
  if (result.audits.empty()) {
    stream << "无";
  }
  for (const auto & audit : result.audits) {
    stream << " id=" << audit.candidate.keyframe_id << " target=" << audit.target_points <<
      " Quatro=" << (a2w_fastlio_common::isUsableRegistration(audit.coarse) ? "成功" : "失败") <<
      " Nano-GICP=" << (audit.registration.converged ? "收敛" : "未收敛") <<
      " fitness=" << audit.registration.fitness <<
      " overlap=" << audit.registration.overlap <<
      " correspondences=" << audit.registration.correspondence_count <<
      " matched_xyz=(" << audit.map_body.translation.x() << ',' <<
      audit.map_body.translation.y() << ',' << audit.map_body.translation.z() << ')' <<
      " reason=" << audit.reason << ';';
  }
  stream << '\n' << "【最终拒绝层级】" << rejectionLayer(result, session) <<
    " best=";
  if (result.best_candidate_id) {
    stream << *result.best_candidate_id;
  } else {
    stream << "none";
  }
  stream << " result=" << result.reason << " session=" << session.reason <<
    " confirmation=" << session.consecutive_matches << '/' << session.required_matches <<
    " same_pose_candidates=" << result.best_hypothesis_candidates <<
    " best_score=" << result.best_score << " distinct_second_score=" << result.second_best_score <<
    " elapsed_ms=" << result.elapsed_ms << " budget_ms=" << result.time_budget_ms <<
    " 耗时告警=" << (result.time_budget_ms > 0.0 && result.elapsed_ms > result.time_budget_ms ?
    "超出预算（不覆盖匹配结果）" : "无");
  return stream.str();
}

RelocalizationSession::RelocalizationSession(RelocalizationSessionConfig config) : config_{config}
{
  if (config_.confirmation_count < 2U ||
    !std::isfinite(config_.maximum_translation_difference_m) ||
    config_.maximum_translation_difference_m < 0.0 ||
    !std::isfinite(config_.maximum_rotation_difference_rad) ||
    config_.maximum_rotation_difference_rad < 0.0 ||
    config_.maximum_rotation_difference_rad > M_PI || config_.timeout_ns <= 0)
  {
    throw std::invalid_argument{"invalid relocalization session configuration"};
  }
}

void RelocalizationSession::requireMonotonic(const std::int64_t stamp_ns) const
{
  if (stamp_ns < 0 || (last_stamp_ns_ && stamp_ns <= *last_stamp_ns_)) {
    throw std::invalid_argument{"relocalization session timestamps must be increasing"};
  }
}

RelocalizationSessionSnapshot RelocalizationSession::start(const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  last_stamp_ns_ = stamp_ns;
  last_progress_ns_ = stamp_ns;
  previous_correction_.reset();
  snapshot_ = RelocalizationSessionSnapshot{};
  snapshot_.active = true;
  snapshot_.required_matches = config_.confirmation_count;
  snapshot_.reason = "started";
  snapshot_.stamp_ns = stamp_ns;
  return snapshot_;
}

RelocalizationSessionSnapshot RelocalizationSession::observe(
  const std::int64_t stamp_ns, const RelocalizationResult & result)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  last_stamp_ns_ = stamp_ns;
  snapshot_.stamp_ns = stamp_ns;
  snapshot_.confirmed_correction.reset();
  if (!snapshot_.active || !last_progress_ns_) {
    throw std::logic_error{"cannot observe an inactive relocalization session"};
  }
  const bool valid = result.success && a2w_fastlio_common::isFinitePose(result.map_body);
  const bool consistent = previous_correction_ && valid && samePose(
    *previous_correction_, result.map_body,
    config_.maximum_translation_difference_m, config_.maximum_rotation_difference_rad);
  // Only a valid, consistent evaluation earns compute-time credit. Failed or
  // inconsistent evaluations cannot keep refreshing a dead session forever.
  const double gap_ms = static_cast<double>(stamp_ns - *last_progress_ns_) / 1.0e6;
  const double compute_ms = valid && (!previous_correction_ || consistent) &&
    std::isfinite(result.elapsed_ms) && result.elapsed_ms > 0.0 ? result.elapsed_ms : 0.0;
  const double waiting_ms = std::max(0.0, gap_ms - compute_ms);
  if (waiting_ms >= static_cast<double>(config_.timeout_ns) / 1.0e6) {
    snapshot_.active = false;
    snapshot_.consecutive_matches = 0U;
    snapshot_.reason = "session_timeout";
    previous_correction_.reset();
    return snapshot_;
  }
  if (!valid) {
    snapshot_.consecutive_matches = 0U;
    snapshot_.reason = result.reason.empty() ? "candidate_rejected" : result.reason;
    previous_correction_.reset();
    return snapshot_;
  }
  if (!previous_correction_ || consistent) {
    last_progress_ns_ = stamp_ns;
  }
  snapshot_.candidate_id = result.candidate_id;
  snapshot_.consecutive_matches = consistent ? snapshot_.consecutive_matches + 1U : 1U;
  // Keep the first correction as anchor; pairwise small drift must not accumulate.
  if (!consistent) {
    previous_correction_ = result.map_body;
  }
  snapshot_.reason = consistent ? "confirmation_pending" : "confirmation_restarted";
  if (result.strong || snapshot_.consecutive_matches >= config_.confirmation_count) {
    snapshot_.active = false;
    snapshot_.reason = result.strong ? "strong_result_confirmed" : "multiframe_confirmed";
    snapshot_.confirmed_correction = result.map_body;
  }
  return snapshot_;
}

RelocalizationSessionSnapshot RelocalizationSession::cancel(const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  last_stamp_ns_ = stamp_ns;
  snapshot_.stamp_ns = stamp_ns;
  snapshot_.active = false;
  snapshot_.reason = "cancelled";
  snapshot_.consecutive_matches = 0U;
  snapshot_.confirmed_correction.reset();
  previous_correction_.reset();
  last_progress_ns_.reset();
  return snapshot_;
}

RelocalizationSessionSnapshot RelocalizationSession::latest() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return snapshot_;
}

}  // namespace a2w_fastlio_localization
