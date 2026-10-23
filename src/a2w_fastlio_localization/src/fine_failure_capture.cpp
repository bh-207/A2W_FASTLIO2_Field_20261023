#include "a2w_fastlio_localization/fine_failure_capture.hpp"

#include <fstream>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

namespace a2w_fastlio_localization
{
namespace
{

std::vector<std::uint64_t> ids(const FineSelectionDiagnostic & selection)
{
  std::vector<std::uint64_t> result;
  result.reserve(selection.keyframes.size());
  for (const auto & item : selection.keyframes) {result.push_back(item.first);}
  return result;
}

std::string joinedIds(const std::vector<std::uint64_t> & values)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {output << '_';}
    output << values[index];
  }
  return output.str();
}

void emitIds(YAML::Emitter & output, const char * key, const std::vector<std::uint64_t> & values)
{
  output << YAML::Key << key << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const auto value : values) {output << value;}
  output << YAML::EndSeq;
}

void emitPose(
  YAML::Emitter & output, const char * key,
  const a2w_fastlio_common::Pose3d & pose)
{
  output << YAML::Key << key << YAML::Value << YAML::BeginMap
         << YAML::Key << "translation" << YAML::Value << YAML::Flow << YAML::BeginSeq
         << pose.translation.x() << pose.translation.y() << pose.translation.z() << YAML::EndSeq
         << YAML::Key << "quaternion_xyzw" << YAML::Value << YAML::Flow << YAML::BeginSeq
         << pose.rotation.x() << pose.rotation.y() << pose.rotation.z() << pose.rotation.w()
         << YAML::EndSeq << YAML::EndMap;
}

void emitRegistrationConfig(
  YAML::Emitter & output,
  const a2w_fastlio_common::DefaultAlgorithmSuiteConfig & config)
{
  output << YAML::Key << "registration" << YAML::Value << YAML::BeginMap
         << YAML::Key << "coarse" << YAML::Value << YAML::BeginMap
         << YAML::Key << "normal_radius_m" << YAML::Value << config.coarse.normal_radius_m
         << YAML::Key << "feature_radius_m" << YAML::Value << config.coarse.feature_radius_m
         << YAML::Key << "noise_bound_m" << YAML::Value << config.coarse.noise_bound_m
         << YAML::Key << "rotation_gnc_factor" << YAML::Value << config.coarse.rotation_gnc_factor
         << YAML::Key << "rotation_cost_threshold" << YAML::Value
         << config.coarse.rotation_cost_threshold
         << YAML::Key << "rotation_max_iterations" << YAML::Value
         << config.coarse.rotation_max_iterations
         << YAML::Key << "estimate_scale" << YAML::Value << config.coarse.estimate_scale
         << YAML::Key << "optimized_matching" << YAML::Value << config.coarse.optimized_matching
         << YAML::Key << "descriptor_distance_threshold" << YAML::Value
         << config.coarse.descriptor_distance_threshold
         << YAML::Key << "maximum_correspondences" << YAML::Value
         << config.coarse.maximum_correspondences
         << YAML::Key << "minimum_points" << YAML::Value << config.coarse.minimum_points
         << YAML::EndMap
         << YAML::Key << "fine" << YAML::Value << YAML::BeginMap
         << YAML::Key << "maximum_correspondence_distance_m" << YAML::Value
         << config.fine.maximum_correspondence_distance_m
         << YAML::Key << "thread_count" << YAML::Value << config.fine.thread_count
         << YAML::Key << "correspondence_randomness" << YAML::Value
         << config.fine.correspondence_randomness
         << YAML::Key << "maximum_iterations" << YAML::Value << config.fine.maximum_iterations
         << YAML::Key << "transformation_epsilon" << YAML::Value
         << config.fine.transformation_epsilon
         << YAML::Key << "rotation_epsilon" << YAML::Value << config.fine.rotation_epsilon
         << YAML::Key << "regularization_method" << YAML::Value
         << config.fine.regularization_method
         << YAML::Key << "fitness_score_max_range_m" << YAML::Value
         << config.fine.fitness_score_max_range_m
         << YAML::Key << "minimum_points" << YAML::Value << config.fine.minimum_points
         << YAML::Key << "source_voxel_leaf_m" << YAML::Value << config.fine.source_voxel_leaf_m
         << YAML::EndMap
         << YAML::Key << "validation" << YAML::Value << YAML::BeginMap
         << YAML::Key << "maximum_fitness" << YAML::Value << config.validation.maximum_fitness
         << YAML::Key << "minimum_overlap" << YAML::Value << config.validation.minimum_overlap
         << YAML::Key << "minimum_correspondences" << YAML::Value
         << config.validation.minimum_correspondences
         << YAML::Key << "maximum_translation_jump_m" << YAML::Value
         << config.validation.maximum_translation_jump_m
         << YAML::Key << "maximum_rotation_jump_rad" << YAML::Value
         << config.validation.maximum_rotation_jump_rad
         << YAML::Key << "minimum_candidate_distance_separation" << YAML::Value
         << config.validation.minimum_candidate_distance_separation
         << YAML::EndMap
         << YAML::Key << "evidence_distance_m" << YAML::Value << config.evidence_distance_m
         << YAML::EndMap;
}

a2w_fastlio_common::Pose3d readPose(const YAML::Node & node)
{
  const auto translation = node["translation"].as<std::vector<double>>();
  const auto quaternion = node["quaternion_xyzw"].as<std::vector<double>>();
  if (translation.size() != 3U || quaternion.size() != 4U) {
    throw std::runtime_error{"capture pose has invalid dimensions"};
  }
  a2w_fastlio_common::Pose3d result;
  result.translation = {translation[0], translation[1], translation[2]};
  result.rotation = {quaternion[3], quaternion[0], quaternion[1], quaternion[2]};
  if (!a2w_fastlio_common::isFinitePose(result)) {
    throw std::runtime_error{"capture pose is not finite"};
  }
  result.rotation.normalize();
  return result;
}

std::filesystem::path captureFile(
  const std::filesystem::path & directory, const YAML::Node & metadata, const char * key)
{
  const std::filesystem::path relative{metadata[key].as<std::string>()};
  if (relative.empty() || relative.is_absolute() || relative.filename() != relative) {
    throw std::runtime_error{"capture file path is not confined"};
  }
  return directory / relative;
}

a2w_fastlio_common::CloudPtr loadCloud(const std::filesystem::path & path)
{
  auto cloud = std::make_shared<a2w_fastlio_common::Cloud>();
  if (pcl::io::loadPCDFile(path.string(), *cloud) != 0 || cloud->empty()) {
    throw std::runtime_error{"failed to load capture cloud: " + path.string()};
  }
  return cloud;
}

void readRegistrationConfig(
  const YAML::Node & node, a2w_fastlio_common::DefaultAlgorithmSuiteConfig & config)
{
  const auto coarse = node["coarse"];
  config.coarse.normal_radius_m = coarse["normal_radius_m"].as<double>();
  config.coarse.feature_radius_m = coarse["feature_radius_m"].as<double>();
  config.coarse.noise_bound_m = coarse["noise_bound_m"].as<double>();
  config.coarse.rotation_gnc_factor = coarse["rotation_gnc_factor"].as<double>();
  config.coarse.rotation_cost_threshold = coarse["rotation_cost_threshold"].as<double>();
  config.coarse.rotation_max_iterations = coarse["rotation_max_iterations"].as<int>();
  config.coarse.estimate_scale = coarse["estimate_scale"].as<bool>();
  config.coarse.optimized_matching = coarse["optimized_matching"].as<bool>();
  config.coarse.descriptor_distance_threshold =
    coarse["descriptor_distance_threshold"].as<double>();
  config.coarse.maximum_correspondences = coarse["maximum_correspondences"].as<int>();
  config.coarse.minimum_points = coarse["minimum_points"].as<std::size_t>();
  const auto fine = node["fine"];
  config.fine.maximum_correspondence_distance_m =
    fine["maximum_correspondence_distance_m"].as<double>();
  config.fine.thread_count = fine["thread_count"].as<int>();
  config.fine.correspondence_randomness = fine["correspondence_randomness"].as<int>();
  config.fine.maximum_iterations = fine["maximum_iterations"].as<int>();
  config.fine.transformation_epsilon = fine["transformation_epsilon"].as<double>();
  config.fine.rotation_epsilon = fine["rotation_epsilon"].as<double>();
  config.fine.regularization_method = fine["regularization_method"].as<int>();
  config.fine.fitness_score_max_range_m = fine["fitness_score_max_range_m"].as<double>();
  config.fine.minimum_points = fine["minimum_points"].as<std::size_t>();
  config.fine.source_voxel_leaf_m = fine["source_voxel_leaf_m"] ?
    fine["source_voxel_leaf_m"].as<double>() : 0.0;
  const auto validation = node["validation"];
  config.validation.maximum_fitness = validation["maximum_fitness"].as<double>();
  config.validation.minimum_overlap = validation["minimum_overlap"].as<double>();
  config.validation.minimum_correspondences =
    validation["minimum_correspondences"].as<std::size_t>();
  config.validation.maximum_translation_jump_m =
    validation["maximum_translation_jump_m"].as<double>();
  config.validation.maximum_rotation_jump_rad =
    validation["maximum_rotation_jump_rad"].as<double>();
  config.validation.minimum_candidate_distance_separation =
    validation["minimum_candidate_distance_separation"].as<double>();
  config.evidence_distance_m = node["evidence_distance_m"].as<double>();
}

}  // namespace

OfflineAbcCause classifyOfflineAbc(
  const bool a_success, const bool b_success, const bool c_success) noexcept
{
  if (a_success) {return OfflineAbcCause::kOnlineTimingOrLifetime;}
  if (b_success) {return OfflineAbcCause::kFineConvergenceBasin;}
  if (c_success) {return OfflineAbcCause::kLocalMapSwitch;}
  if (!a_success && !b_success && !c_success) {
    return OfflineAbcCause::kCorrectionPrediction;
  }
  return OfflineAbcCause::kInconclusive;
}

FineFailureReplayData loadFineFailureCapture(const std::filesystem::path & directory)
{
  const auto metadata = YAML::LoadFile((directory / "metadata.yaml").string());
  if (metadata["schema"].as<std::string>() != "a2w_fine_failure_v1") {
    throw std::runtime_error{"unsupported fine-failure capture schema"};
  }
  FineFailureReplayData result;
  result.abc.source = loadCloud(captureFile(directory, metadata, "source_file"));
  result.abc.current_target = loadCloud(captureFile(directory, metadata, "current_target_file"));
  result.abc.previous_target = loadCloud(captureFile(directory, metadata, "previous_target_file"));
  result.abc.initial_guess = readPose(metadata["initial_guess"]);
  result.current_keyframes = metadata["current_keyframes"].as<std::vector<std::uint64_t>>();
  result.previous_keyframes = metadata["previous_keyframes"].as<std::vector<std::uint64_t>>();
  result.failure_source_stamp_ns = metadata["failure_source_stamp_ns"].as<std::int64_t>();
  result.failure_odom = readPose(metadata["failure_odom"]);
  result.failure_correction = readPose(metadata["failure_correction"]);
  result.previous_prediction_before_update = readPose(
    metadata["previous_prediction_before_update"]);
  result.previous_pose_after_update = readPose(metadata["previous_pose_after_update"]);
  result.previous_odom = readPose(metadata["previous_odom"]);
  result.previous_correction_after_update = readPose(
    metadata["previous_correction_after_update"]);
  readRegistrationConfig(metadata["registration"], result.algorithms);
  return result;
}

OfflineAbcResult runOfflineAbc(
  const OfflineAbcInput & input,
  const std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> & pipeline)
{
  if (!pipeline || !input.source || input.source->empty() ||
    !input.current_target || input.current_target->empty() ||
    !input.previous_target || input.previous_target->empty() ||
    !a2w_fastlio_common::isFinitePose(input.initial_guess))
  {
    throw std::invalid_argument{"offline A/B/C requires complete finite inputs"};
  }
  a2w_fastlio_common::ValidationContext context;
  context.predicted_transform = input.initial_guess;
  OfflineAbcResult output;
  auto started = std::chrono::steady_clock::now();
  output.a = pipeline->runFineOnly(
    input.source, input.current_target, input.initial_guess, context);
  auto ended = std::chrono::steady_clock::now();
  output.a_latency_ms = std::chrono::duration<double, std::milli>(ended - started).count();
  started = std::chrono::steady_clock::now();
  output.b = pipeline->run(
    input.source, input.current_target, input.initial_guess, context);
  ended = std::chrono::steady_clock::now();
  output.b_latency_ms = std::chrono::duration<double, std::milli>(ended - started).count();
  started = std::chrono::steady_clock::now();
  output.c = pipeline->runFineOnly(
    input.source, input.previous_target, input.initial_guess, context);
  ended = std::chrono::steady_clock::now();
  output.c_latency_ms = std::chrono::duration<double, std::milli>(ended - started).count();
  return output;
}

FineFailureCapture::FineFailureCapture(FineFailureCaptureConfig config)
: config_{std::move(config)}
{
  if (config_.enabled && config_.output_root.empty()) {
    throw std::invalid_argument{"enabled fine-failure capture requires output_root"};
  }
}

std::optional<FineFailureCaptureResult> FineFailureCapture::observe(
  const FineMatchRecord & record,
  const a2w_fastlio_common::DefaultAlgorithmSuiteConfig & algorithms)
{
  const auto & output = record.output;
  if (!config_.enabled || captured_ || !output.match_attempted ||
    output.matching_mode != MapMatchingMode::kFineOnly)
  {
    return std::nullopt;
  }
  if (output.match_accepted && !output.result_stale) {
    previous_success_ = record;
    return std::nullopt;
  }
  if (output.result_stale || !previous_success_) {return std::nullopt;}

  FineFailureCaptureResult result;
  const auto & previous = *previous_success_;
  if (!record.source_cloud || record.source_cloud->empty() ||
    !output.match_target || output.match_target->empty() ||
    !previous.output.match_target || previous.output.match_target->empty())
  {
    result.reason = "capture_inputs_missing";
    return result;
  }
  const auto current_ids = ids(record.selection);
  const auto previous_ids = ids(previous.selection);
  if (current_ids.empty() || previous_ids.empty()) {
    result.reason = "capture_keyframes_missing";
    return result;
  }

  result.directory = config_.output_root /
    ("fine_failure_" + std::to_string(output.stamp_ns));
  if (std::filesystem::exists(result.directory)) {
    result.reason = "capture_directory_exists";
    return result;
  }
  std::filesystem::create_directories(result.directory);
  const auto current_file = "target_current_" + joinedIds(current_ids) + ".pcd";
  const auto previous_file = "target_previous_" + joinedIds(previous_ids) + ".pcd";
  if (pcl::io::savePCDFileBinary(
      (result.directory / "source_body.pcd").string(), *record.source_cloud) != 0 ||
    pcl::io::savePCDFileBinary(
      (result.directory / current_file).string(), *output.match_target) != 0 ||
    pcl::io::savePCDFileBinary(
      (result.directory / previous_file).string(), *previous.output.match_target) != 0)
  {
    result.reason = "pcd_write_failed";
    return result;
  }

  YAML::Emitter metadata;
  metadata << YAML::BeginMap
           << YAML::Key << "schema" << YAML::Value << "a2w_fine_failure_v1"
           << YAML::Key << "failure_source_stamp_ns" << YAML::Value << output.stamp_ns
           << YAML::Key << "source_file" << YAML::Value << "source_body.pcd"
           << YAML::Key << "current_target_file" << YAML::Value << current_file
           << YAML::Key << "previous_target_file" << YAML::Value << previous_file;
  emitIds(metadata, "current_keyframes", current_ids);
  emitIds(metadata, "previous_keyframes", previous_ids);
  metadata << YAML::Key << "online_failure_reason" << YAML::Value << output.reason;
  emitPose(metadata, "initial_guess", record.selection.initial_guess);
  emitPose(metadata, "failure_odom", record.odom_pose);
  emitPose(metadata, "failure_correction", output.map_camera_init);
  emitPose(metadata, "previous_prediction_before_update", previous.output.predicted_map_body);
  emitPose(metadata, "previous_pose_after_update", previous.output.map_body);
  emitPose(metadata, "previous_odom", previous.odom_pose);
  emitPose(metadata, "previous_correction_after_update", previous.output.map_camera_init);
  emitRegistrationConfig(metadata, algorithms);
  metadata << YAML::EndMap;
  std::ofstream stream{result.directory / "metadata.yaml"};
  stream << metadata.c_str() << '\n';
  if (!stream) {
    result.reason = "metadata_write_failed";
    return result;
  }
  captured_ = true;
  result.captured = true;
  result.reason = "captured";
  return result;
}

}  // namespace a2w_fastlio_localization
