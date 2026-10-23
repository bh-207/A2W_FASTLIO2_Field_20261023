// Manual benchmark for a complete mapping loop-candidate path using saved data.
// It does not modify the bundle and is not a JT128 hardware acceptance test.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace
{

using a2w_fastlio_common::KeyFrame;
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

class BundleProvider final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  BundleProvider(const std::vector<KeyFrame> & frames, const std::uint64_t visible_max)
  : frames_{frames}, visible_max_{visible_max} {}

  std::optional<KeyFrame> get(const std::uint64_t id) const override
  {
    if (id > visible_max_ || id >= frames_.size()) {
      return std::nullopt;
    }
    return frames_[id];
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    return frames_.empty() ? std::nullopt :
           std::optional<std::pair<std::uint64_t, std::uint64_t>>{{0U, visible_max_}};
  }

private:
  const std::vector<KeyFrame> & frames_;
  std::uint64_t visible_max_;
};

std::vector<std::uint64_t> parseIds(const int argc, char ** argv, const std::size_t count)
{
  std::vector<std::uint64_t> ids;
  for (int index = 2; index < argc; ++index) {
    const auto id = static_cast<std::uint64_t>(std::stoull(argv[index]));
    if (id >= count) {
      throw std::out_of_range{"keyframe ID is outside the bundle"};
    }
    ids.push_back(id);
  }
  if (ids.empty()) {
    for (const auto id : {33U, 40U, 50U, 58U}) {
      if (id < count) {
        ids.push_back(id);
      }
    }
  }
  return ids;
}

}  // namespace

int main(int argc, char ** argv)
try {
  if (argc < 2) {
    std::cerr << "usage: candidate_chain_benchmark MAP_BUNDLE [CURRENT_ID ...]\n";
    return 2;
  }
  const auto bundle = a2w_fastlio_map::MapBundleReader{}.read(argv[1]);
  const auto & frames = bundle.data.keyframes;
  const auto ids = parseIds(argc, argv, frames.size());
  if (ids.empty()) {
    throw std::runtime_error{"bundle has no eligible benchmark keyframes"};
  }

  constexpr std::size_t top_k = 5U;
  constexpr std::size_t exclude_recent = 30U;
  constexpr double maximum_descriptor_distance = 0.2;
  const char * maximum_candidates_text = std::getenv("A2W_BENCHMARK_MAX_CANDIDATES");
  const std::size_t maximum_candidates = maximum_candidates_text ?
    static_cast<std::size_t>(std::stoull(maximum_candidates_text)) :
    std::numeric_limits<std::size_t>::max();
  if (maximum_candidates == 0U) {
    throw std::invalid_argument{"A2W_BENCHMARK_MAX_CANDIDATES must be positive"};
  }
  const bool apply_context_precheck =
    std::getenv("A2W_BENCHMARK_APPLY_CONTEXT_PRECHECK") != nullptr;
  const a2w_fastlio_common::LocalMapBuilder local_map_builder{{0.2, 200000U}};

  std::cout << "build_mode="
#ifdef NDEBUG
            << "Release"
#else
            << "Debug"
#endif
            << " bundle_frames=" << frames.size()
            << " context_precheck=" << apply_context_precheck << '\n';

  for (const auto current_id : ids) {
    const auto frame_started = std::chrono::steady_clock::now();
    auto suite = a2w_fastlio_common::createDefaultAlgorithmSuite({});
    for (std::uint64_t id = 0U; id < current_id; ++id) {
      suite.descriptor_index->add(id, bundle.data.descriptors.at(id).descriptor);
    }

    const auto descriptor_started = std::chrono::steady_clock::now();
    const auto descriptor = suite.place_recognition->describe(frames.at(current_id).body_cloud);
    const double descriptor_ms = elapsedMs(descriptor_started);
    a2w_fastlio_common::CandidateFilter filter;
    filter.max_inclusive_id = current_id - 1U;
    filter.exclude_recent = exclude_recent;
    const auto query_started = std::chrono::steady_clock::now();
    auto candidates = suite.descriptor_index->queryTopK(descriptor, top_k, filter);
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [](const auto & candidate) {
          return candidate.distance > maximum_descriptor_distance;
        }),
      candidates.end());
    const double scan_context_ms = elapsedMs(query_started);

    a2w_fastlio_common::ValidationContext context;
    if (!candidates.empty()) {
      context.best_descriptor_distance = candidates.front().distance;
    }
    if (candidates.size() > 1U) {
      context.second_descriptor_distance = candidates[1].distance;
    }

    if (apply_context_precheck) {
      const auto context_validation = suite.registration->validateCandidateContext(context);
      if (!context_validation.accepted) {
        const auto evaluated = std::min(candidates.size(), maximum_candidates);
        for (std::size_t index = 0U; index < evaluated; ++index) {
          const auto & candidate = candidates[index];
          std::cout << "candidate current=" << current_id
                    << " id=" << candidate.keyframe_id
                    << " rank=" << candidate.rank
                    << " sc_distance=" << candidate.distance
                    << " source_points=0 target_points=0 target_local_map_ms=0"
                    << " quatro_ms=0 coarse_evidence_ms=0 nano_gicp_ms=0"
                    << " fine_evidence_ms=0 validator_ms=0 registration_total_ms=0"
                    << " gtsam_ms=0 candidate_total_ms=0 fitness=inf overlap=0"
                    << " correspondences=0 accepted=0 precheck=1 reason="
                    << context_validation.reason << '\n';
        }
        std::cout << "frame current=" << current_id
                  << " descriptor_ms=" << descriptor_ms
                  << " scan_context_ms=" << scan_context_ms
                  << " source_local_map_ms=0 candidates_returned=" << candidates.size()
                  << " candidates_evaluated=" << evaluated
                  << " accepted=0 frame_total_ms=" << elapsedMs(frame_started) << '\n';
        continue;
      }
    }

    const BundleProvider provider{frames, current_id};
    const auto source_started = std::chrono::steady_clock::now();
    const auto source = local_map_builder.build(current_id, 5U, 5U, provider);
    const double source_local_map_ms = elapsedMs(source_started);

    std::size_t evaluated = 0U;
    bool accepted = false;
    for (const auto & candidate : candidates) {
      if (evaluated >= maximum_candidates) {
        break;
      }
      ++evaluated;
      const auto candidate_started = std::chrono::steady_clock::now();
      const auto target_started = std::chrono::steady_clock::now();
      const auto target = local_map_builder.build(candidate.keyframe_id, 5U, 5U, provider);
      const double target_local_map_ms = elapsedMs(target_started);
      const auto registration = suite.registration->run(
        source.cloud, target.cloud, yawHint(candidate.yaw_hint_rad), context);

      double gtsam_ms = 0.0;
      bool graph_accepted = false;
      if (registration.success) {
        a2w_fastlio_mapping::PoseGraphOptimizer graph{{}};
        for (std::uint64_t id = 0U; id <= current_id; ++id) {
          graph.addKeyFrame(id, frames.at(id).odom_pose);
          graph.update();
        }
        const auto corrected_current = compose(
          registration.final_transform, frames.at(current_id).optimized_pose);
        a2w_fastlio_mapping::LoopConstraint constraint;
        constraint.from_id = current_id;
        constraint.to_id = candidate.keyframe_id;
        constraint.relative_pose = between(
          corrected_current, frames.at(candidate.keyframe_id).optimized_pose);
        constraint.accepted = true;
        const auto gtsam_started = std::chrono::steady_clock::now();
        graph_accepted = graph.addLoopConstraint(constraint).accepted && graph.update().accepted;
        gtsam_ms = elapsedMs(gtsam_started);
      }

      const std::string reason = registration.success ?
        (graph_accepted ? "loop_accepted" : "graph_rejected") : registration.validation.reason;
      std::cout << "candidate current=" << current_id
                << " id=" << candidate.keyframe_id
                << " rank=" << candidate.rank
                << " sc_distance=" << candidate.distance
                << " source_points=" << source.cloud->size()
                << " target_points=" << target.cloud->size()
                << " target_local_map_ms=" << target_local_map_ms
                << " quatro_ms=" << registration.coarse.elapsed_ms
                << " coarse_evidence_ms=" << registration.coarse_evidence_ms
                << " nano_gicp_ms=" << registration.fine.elapsed_ms
                << " fine_evidence_ms=" << registration.fine_evidence_ms
                << " validator_ms=" << registration.validation_ms
                << " registration_total_ms=" << registration.total_ms
                << " gtsam_ms=" << gtsam_ms
                << " candidate_total_ms=" << elapsedMs(candidate_started)
                << " fitness=" << registration.fine.fitness
                << " overlap=" << registration.fine.overlap
                << " correspondences=" << registration.fine.correspondence_count
                << " accepted=" << graph_accepted
                << " reason=" << reason << '\n';
      if (graph_accepted) {
        accepted = true;
        break;
      }
    }
    std::cout << "frame current=" << current_id
              << " descriptor_ms=" << descriptor_ms
              << " scan_context_ms=" << scan_context_ms
              << " source_local_map_ms=" << source_local_map_ms
              << " candidates_returned=" << candidates.size()
              << " candidates_evaluated=" << evaluated
              << " accepted=" << accepted
              << " frame_total_ms=" << elapsedMs(frame_started) << '\n';
  }
  return 0;
} catch (const std::exception & error) {
  std::cerr << "candidate benchmark failed: " << error.what() << '\n';
  return 1;
}
