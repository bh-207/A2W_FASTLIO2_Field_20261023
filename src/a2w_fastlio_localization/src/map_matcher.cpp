#include "a2w_fastlio_localization/map_matcher.hpp"

#include <optional>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_localization
{
namespace
{
double elapsedMs(const std::chrono::steady_clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
}
}  // namespace

class MapMatcher::Provider final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  explicit Provider(a2w_fastlio_map::MapSnapshot snapshot) : snapshot_{std::move(snapshot)} {}
  std::optional<a2w_fastlio_common::KeyFrame> get(const std::uint64_t id) const override
  {
    return id < snapshot_.keyframes.size() ?
      std::optional<a2w_fastlio_common::KeyFrame>{snapshot_.keyframes[id]} : std::nullopt;
  }
  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    return snapshot_.keyframes.empty() ? std::nullopt :
      std::optional<std::pair<std::uint64_t, std::uint64_t>>{{0U, snapshot_.keyframes.size() - 1U}};
  }
private:
  a2w_fastlio_map::MapSnapshot snapshot_;
};

MapMatcher::MapMatcher(
  a2w_fastlio_map::MapSnapshot snapshot,
  std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> registration,
  a2w_fastlio_common::LocalMapConfig local_map_config)
: provider_{std::make_shared<Provider>(std::move(snapshot))},
  registration_{std::move(registration)}, local_map_builder_{local_map_config}
{
  if (!registration_) {
    throw std::invalid_argument{"MapMatcher requires a registration pipeline"};
  }
}

MapMatchResult MapMatcher::match(
  const a2w_fastlio_common::FrontendFrame & frame,
  const SelectionResult & selection,
  const MapMatchingMode mode) const
{
  MapMatchResult result;
  result.candidate_id = selection.nearest_keyframe_id;
  result.source_points = frame.body_cloud ? frame.body_cloud->size() : 0U;
  if (!selection.success) {
    result.reason = "selection_invalid:" + selection.reason;
    return result;
  }
  try {
    const auto started = std::chrono::steady_clock::now();
    bool cache_hit = false;
    const auto target = targetFor(selection.keyframe_ids, cache_hit);
    const auto submap_ms = elapsedMs(started);
    result = matchPrepared(frame, selection, target, mode);
    result.timings.submap_ms = submap_ms;
    result.timings.target_cache_hit = cache_hit;
    result.timings.total_ms = elapsedMs(started);
    return result;
  } catch (const std::exception & error) {
    result.reason = std::string{"map_match_failed:"} + error.what();
  }
  return result;
}

a2w_fastlio_common::CloudConstPtr MapMatcher::targetFor(
  const std::vector<std::uint64_t> & ids, bool & cache_hit) const
{
  std::lock_guard<std::mutex> lock{target_mutex_};
  cache_hit = cached_target_ && ids == cached_ids_;
  if (!cache_hit) {
    const auto built = local_map_builder_.build(ids, *provider_);
    cached_ids_ = ids;
    cached_target_ = built.cloud;
  }
  return cached_target_;
}

MapMatchResult MapMatcher::matchPrepared(
  const a2w_fastlio_common::FrontendFrame & frame,
  const SelectionResult & selection,
  const a2w_fastlio_common::CloudConstPtr & target,
  const MapMatchingMode mode) const
{
  MapMatchResult result;
  result.candidate_id = selection.nearest_keyframe_id;
  result.source_points = frame.body_cloud ? frame.body_cloud->size() : 0U;
  result.target_points = target ? target->size() : 0U;
  result.target_cloud = target;
  try {
    a2w_fastlio_common::ValidationContext context;
    context.predicted_transform = selection.initial_map_body;
    const auto pipeline = mode == MapMatchingMode::kFineOnly ?
      registration_->runFineOnly(
      frame.body_cloud, target, selection.initial_map_body, context) :
      registration_->run(
      frame.body_cloud, target, selection.initial_map_body, context);
    result.success = pipeline.success;
    result.reason = pipeline.validation.reason;
    result.map_body = pipeline.final_transform;
    result.registration = pipeline.fine;
    result.timings.coarse_ms = pipeline.coarse.elapsed_ms;
    result.timings.fine_ms = pipeline.fine.elapsed_ms;
    result.timings.evidence_ms = pipeline.coarse_evidence_ms + pipeline.fine_evidence_ms;
    result.timings.validation_ms = pipeline.validation_ms;
    result.timings.total_ms = pipeline.total_ms;
  } catch (const std::exception & error) {
    result.reason = std::string{"map_match_failed:"} + error.what();
  }
  return result;
}

MapMatchComparison MapMatcher::compareModes(
  const a2w_fastlio_common::FrontendFrame & frame, const SelectionResult & selection) const
{
  MapMatchComparison result;
  if (!selection.success) {
    result.fine_only = match(frame, selection, MapMatchingMode::kFineOnly);
    result.full = result.fine_only;
    return result;
  }
  bool cache_hit = false;
  const auto target = targetFor(selection.keyframe_ids, cache_hit);
  const auto started = std::chrono::steady_clock::now();
  result.fine_only = matchPrepared(frame, selection, target, MapMatchingMode::kFineOnly);
  const auto middle = std::chrono::steady_clock::now();
  result.full = matchPrepared(frame, selection, target, MapMatchingMode::kFull);
  const auto ended = std::chrono::steady_clock::now();
  result.fine_only_latency_ms = std::chrono::duration<double, std::milli>(middle - started).count();
  result.full_latency_ms = std::chrono::duration<double, std::milli>(ended - middle).count();
  return result;
}

}  // namespace a2w_fastlio_localization
