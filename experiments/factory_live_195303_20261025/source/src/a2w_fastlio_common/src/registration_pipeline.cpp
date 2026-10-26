#include "a2w_fastlio_common/registration_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <pcl/kdtree/kdtree_flann.h>

namespace a2w_fastlio_common
{
namespace
{

double elapsedMs(const std::chrono::steady_clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
}

}  // namespace

RegistrationPipeline::RegistrationPipeline(
  std::shared_ptr<const CoarseRegistration> coarse,
  std::shared_ptr<const FineRegistration> fine,
  MatchValidator validator,
  RegistrationPipelineConfig config)
: coarse_{std::move(coarse)}, fine_{std::move(fine)}, validator_{std::move(validator)}, config_{config}
{
  if (!coarse_ || !fine_ || !(config_.evidence_distance_m > 0.0)) {
    throw std::invalid_argument{"registration pipeline requires algorithms and positive distance"};
  }
}

RegistrationResult RegistrationPipeline::withGeometricEvidence(
  RegistrationResult result, const CloudConstPtr & source, const CloudConstPtr & target) const
{
  if (!result.success || !result.converged || !isFinitePose(result.transform) ||
    !source || !target || source->empty() || target->empty())
  {
    return result;
  }

  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  const double max_squared_distance = config_.evidence_distance_m * config_.evidence_distance_m;
  double squared_error_sum = 0.0;
  std::size_t matches = 0U;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto & input : source->points) {
    const Eigen::Vector3d xyz = result.transform.rotation *
      input.getVector3fMap().cast<double>() + result.transform.translation;
    PointT query = input;
    query.x = static_cast<float>(xyz.x());
    query.y = static_cast<float>(xyz.y());
    query.z = static_cast<float>(xyz.z());
    if (tree.nearestKSearch(query, 1, indices, squared_distances) == 1 &&
      squared_distances.front() <= max_squared_distance)
    {
      ++matches;
      squared_error_sum += squared_distances.front();
    }
  }
  result.correspondence_count = matches;
  result.overlap = static_cast<double>(matches) / static_cast<double>(source->size());
  result.fitness = matches == 0U ? 2.0 * config_.evidence_distance_m :
    std::sqrt(squared_error_sum / static_cast<double>(matches));
  return result;
}

PipelineResult RegistrationPipeline::run(
  const CloudConstPtr & source, const CloudConstPtr & target,
  const std::optional<Pose3d> & yaw_hint, const ValidationContext & context) const
{
  PipelineResult output;
  const auto pipeline_started = std::chrono::steady_clock::now();
  output.coarse = coarse_->align(source, target, yaw_hint);
  if (!isUsableRegistration(output.coarse)) {
    output.validation.reason = "coarse_registration_failed:" +
      (output.coarse.rejection_reason.empty() ? std::string{"unspecified"} :
      output.coarse.rejection_reason);
    output.total_ms = elapsedMs(pipeline_started);
    return output;
  }
  const auto coarse_evidence_started = std::chrono::steady_clock::now();
  output.coarse = withGeometricEvidence(std::move(output.coarse), source, target);
  output.coarse_evidence_ms = elapsedMs(coarse_evidence_started);

  output.fine = fine_->align(source, target, output.coarse.transform);
  if (!isUsableRegistration(output.fine)) {
    output.validation.reason = "fine_registration_failed:" +
      (output.fine.rejection_reason.empty() ? std::string{"unspecified"} :
      output.fine.rejection_reason);
    output.total_ms = elapsedMs(pipeline_started);
    return output;
  }
  const auto fine_evidence_started = std::chrono::steady_clock::now();
  output.fine = withGeometricEvidence(std::move(output.fine), source, target);
  output.fine_evidence_ms = elapsedMs(fine_evidence_started);
  const auto validation_started = std::chrono::steady_clock::now();
  output.validation = validator_.validate(output.fine, context);
  output.validation_ms = elapsedMs(validation_started);
  output.success = output.validation.accepted;
  if (output.success) {
    output.final_transform = output.fine.transform;
  }
  output.total_ms = elapsedMs(pipeline_started);
  return output;
}

PipelineResult RegistrationPipeline::runFineOnly(
  const CloudConstPtr & source, const CloudConstPtr & target,
  const Pose3d & initial_guess, const ValidationContext & context) const
{
  PipelineResult output;
  const auto pipeline_started = std::chrono::steady_clock::now();
  output.fine = fine_->align(source, target, initial_guess);
  if (!isUsableRegistration(output.fine)) {
    output.validation.reason = "fine_registration_failed:" +
      (output.fine.rejection_reason.empty() ? std::string{"unspecified"} :
      output.fine.rejection_reason);
    output.total_ms = elapsedMs(pipeline_started);
    return output;
  }
  const auto fine_evidence_started = std::chrono::steady_clock::now();
  output.fine = withGeometricEvidence(std::move(output.fine), source, target);
  output.fine_evidence_ms = elapsedMs(fine_evidence_started);
  const auto validation_started = std::chrono::steady_clock::now();
  output.validation = validator_.validate(output.fine, context);
  output.validation_ms = elapsedMs(validation_started);
  output.success = output.validation.accepted;
  if (output.success) {
    output.final_transform = output.fine.transform;
  }
  output.total_ms = elapsedMs(pipeline_started);
  return output;
}

ValidationResult RegistrationPipeline::validateCandidateContext(
  const ValidationContext & context) const
{
  return validator_.validateContext(context);
}

}  // namespace a2w_fastlio_common
