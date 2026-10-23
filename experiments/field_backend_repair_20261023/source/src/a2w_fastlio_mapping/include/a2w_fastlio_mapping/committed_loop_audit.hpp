#pragma once

#include <cmath>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <string>

#include "a2w_fastlio_mapping/loop_pipeline.hpp"

namespace a2w_fastlio_mapping
{
struct CommittedLoopClock
{
  std::int64_t sensor_stamp_ns;
  std::int64_t commit_wall_ns;
  double process_to_commit_ms;
};

inline std::optional<std::string> committedLoopAudit(
  const LoopProgressStage stage, const LoopPipelineEvent & event, const CommittedLoopClock & clock)
{
  // Candidate validation/staging is not an iSAM2 commit. The pose change is
  // populated only on the successful post-update completion observation.
  if (stage != LoopProgressStage::OptimizationFinished ||
    !event.accepted || !event.optimized_pose_change)
  {
    return std::nullopt;
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(17)
         << "[LoopCommitted] audit_v=1 source_id=" << event.current_id
         << " target_id=" << event.candidate_id
         << " sensor_stamp_ns=" << clock.sensor_stamp_ns
         << " commit_wall_ns=" << clock.commit_wall_ns
         << " process_to_commit_ms=";
  const bool steady_valid = std::isfinite(clock.process_to_commit_ms) && clock.process_to_commit_ms >= 0.0;
  if (steady_valid) {stream << clock.process_to_commit_ms;} else {stream << "unknown";}
  // Source sensor/ROS time is not assumed synchronized with system_clock.
  stream << " process_clock_status=" << (steady_valid ? "ok" : "anomaly")
         << " sensor_wall_latency_ms=unknown clock_relation=unverified";
  const auto metric = [&stream](const char * name, const double value) {
      stream << ' ' << name << '=';
      if (std::isfinite(value) && value >= 0.0) {stream << value;} else {stream << "unknown";}
    };
  metric("fitness", event.registration.fitness);
  metric("overlap", event.registration.overlap);
  stream << " correspondences=" << event.registration.correspondence_count
         << " factors=" << event.graph_factor_count;
  metric("optimized_translation_m", event.optimized_pose_change->translation.norm());
  metric("optimized_rotation_rad", Eigen::AngleAxisd(event.optimized_pose_change->rotation).angle());
  metric("gtsam_ms", event.gtsam_ms);
  return stream.str();
}
}  // namespace a2w_fastlio_mapping
