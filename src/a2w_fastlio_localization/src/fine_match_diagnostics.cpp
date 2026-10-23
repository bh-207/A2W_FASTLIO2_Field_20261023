#include "a2w_fastlio_localization/fine_match_diagnostics.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

namespace a2w_fastlio_localization
{

FineSelectionDiagnostic captureFineSelection(
  const std::int64_t stamp_ns, const std::int64_t source_age_at_start_ns,
  const SelectionResult & selection,
  const std::vector<a2w_fastlio_common::KeyFrame> & keyframes)
{
  FineSelectionDiagnostic result;
  result.stamp_ns = stamp_ns;
  result.source_age_at_start_ns = source_age_at_start_ns;
  result.initial_guess = selection.initial_map_body;
  for (const auto id : selection.keyframe_ids) {
    const auto frame = std::find_if(keyframes.begin(), keyframes.end(),
      [id](const auto & candidate) {return candidate.id == id;});
    result.keyframes.emplace_back(id, frame == keyframes.end() ?
      std::numeric_limits<double>::quiet_NaN() :
      (frame->optimized_pose.translation - selection.initial_map_body.translation).norm());
  }
  return result;
}

std::optional<std::string> FineMatchDiagnostics::observe(const FineMatchRecord & record)
{
  const auto & output = record.output;
  if (!output.match_attempted || output.matching_mode != MapMatchingMode::kFineOnly ||
    (last_observed_stamp_ && *last_observed_stamp_ == output.stamp_ns))
  {
    return std::nullopt;
  }
  last_observed_stamp_ = output.stamp_ns;
  if (output.match_accepted && !output.result_stale) {
    previous_success_ = record;
    return std::nullopt;
  }

  std::ostringstream summary;
  summary << std::fixed << std::setprecision(6) << "[fine_only_failure] 在线精配准失败摘要";
  const auto pose = [&summary](const char * label, const a2w_fastlio_common::Pose3d & value) {
      summary << " " << label << "={xyz:[" << value.translation.transpose()
              << "],q_xyzw:[" << value.rotation.coeffs().transpose() << "]}";
    };
  const auto context = [&summary, &pose](const FineMatchRecord & value) {
      summary << "source_stamp_ns=" << value.output.stamp_ns
              << " odom_stamp_ns=" << value.output.stamp_ns
              << " source_age_ms=" << static_cast<double>(value.source_age_ns) / 1e6
              << " source_age_at_start_ms="
              << static_cast<double>(value.selection.source_age_at_start_ns) / 1e6
              << " correction_source_stamp_ns=" << value.output.correction_stamp_ns
              << " correction_accepted_age_ms="
              << static_cast<double>(value.correction_accepted_age_ns) / 1e6
              << " prediction_correction_source_stamp_ns="
              << value.output.prediction_correction_stamp_ns
              << " prediction_correction_accepted_age_ms="
              << static_cast<double>(value.prediction_correction_accepted_age_ns) / 1e6
              << " selection_stamp_ns=" << value.selection.stamp_ns
              << " selection_stamp_matches=" << (value.selection.stamp_ns == value.output.stamp_ns);
      pose("FAST_LIO_camera_init_body", value.odom_pose);
      pose("predicted_map_body", value.output.predicted_map_body);
      pose("Nano_GICP_initial_guess", value.selection.initial_guess);
      summary << " KeyFrames_ID距离m=[";
      for (const auto & [id, distance] : value.selection.keyframes) {
        summary << id << ":" << distance << " ";
      }
      summary << "] match_latency_ms=" << value.match_latency_ms
              << " Nano_GICP_latency_ms=" << value.output.registration.elapsed_ms
              << " source_points=" << value.output.source_points
              << " target_points=" << value.output.target_points
              << " stale_result=" << value.output.result_stale;
    };
  summary << "；本次={";
  context(record);
  // Preserve exactly one line even for exception messages containing newlines.
  auto reason = output.reason;
  std::replace(reason.begin(), reason.end(), '\n', ' ');
  std::replace(reason.begin(), reason.end(), '\r', ' ');
  summary << " 失败原因=" << reason << "}";
  if (previous_success_) {
    summary << "；上一成功={";
    context(*previous_success_);
    summary << "}；与上一成功比较={prediction平移差m="
            << (output.predicted_map_body.translation -
      previous_success_->output.predicted_map_body.translation).norm()
            << " prediction旋转差rad=" << output.predicted_map_body.rotation.angularDistance(
      previous_success_->output.predicted_map_body.rotation)
            << " FAST_LIO平移差m="
            << (record.odom_pose.translation - previous_success_->odom_pose.translation).norm()
            << " FAST_LIO旋转差rad=" << record.odom_pose.rotation.angularDistance(
      previous_success_->odom_pose.rotation) << "}";
  } else {
    summary << "；上一成功=无";
  }
  return summary.str();
}

}  // namespace a2w_fastlio_localization
