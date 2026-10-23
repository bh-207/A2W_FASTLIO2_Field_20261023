// Controlled, real Quatro + Nano-GICP operator probe. No ROS or hardware.
// Synthetic gravity is independent test data, never inferred Bundle gravity.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/LU>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"

namespace
{
namespace fs = std::filesystem;
namespace common = a2w_fastlio_common;
using Clock = std::chrono::steady_clock;
using Matrix = Eigen::Matrix4d;

double elapsedMs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

Matrix rigid(const Eigen::Matrix3d & rotation, const Eigen::Vector3d & translation = Eigen::Vector3d::Zero())
{
  Matrix result = Matrix::Identity();
  result.block<3, 3>(0, 0) = rotation;
  result.block<3, 1>(0, 3) = translation;
  return result;
}

Eigen::Matrix3d rpy(double roll_degrees, double pitch_degrees, double yaw_degrees)
{
  constexpr double radians = 3.14159265358979323846 / 180.0;
  // Explicit Rz * Rx * Ry order, matching the declared synthetic experiment.
  return (Eigen::AngleAxisd(yaw_degrees * radians, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(roll_degrees * radians, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(pitch_degrees * radians, Eigen::Vector3d::UnitY())).toRotationMatrix();
}

Matrix alignment(const Eigen::Vector3d & gravity)
{
  if (!gravity.allFinite() || gravity.norm() < 8.0 || gravity.norm() > 12.0) {
    throw std::runtime_error("invalid synthetic gravity units/vector");
  }
  const Eigen::Vector3d up = -gravity.normalized();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  const double dot = std::clamp(up.dot(z), -1.0, 1.0);
  const Eigen::Vector3d cross = up.cross(z);
  const double sine = cross.norm();
  if (sine < 1e-15 && dot >= 0.0) {return Matrix::Identity();}
  if (sine < 1e-15) {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(1, 1) = rotation(2, 2) = -1.0;
    return rigid(rotation);
  }
  const Eigen::Vector3d axis = cross / sine;
  Eigen::Matrix3d skew;
  skew << 0.0, -axis.z(), axis.y(), axis.z(), 0.0, -axis.x(), -axis.y(), axis.x(), 0.0;
  return rigid(Eigen::Matrix3d::Identity() + sine * skew + (1.0 - dot) * (skew * skew));
}

common::CloudPtr transform(const common::CloudConstPtr & cloud, const Matrix & matrix)
{
  auto result = common::CloudPtr{new common::Cloud{}};
  result->reserve(cloud->size());
  for (const auto & point : cloud->points) {
    const Eigen::Vector3d xyz = matrix.block<3, 3>(0, 0) *
      point.getVector3fMap().cast<double>() + matrix.block<3, 1>(0, 3);
    auto moved = point;
    moved.x = static_cast<float>(xyz.x());
    moved.y = static_cast<float>(xyz.y());
    moved.z = static_cast<float>(xyz.z());
    result->push_back(moved);
  }
  return result;
}

Matrix matrix(const common::Pose3d & pose)
{
  return rigid(pose.rotation.normalized().toRotationMatrix(), pose.translation);
}

std::string csvReason(std::string reason)
{
  for (auto & character : reason) {
    if (character == ',' || character == '\n' || character == '\r') {character = ';';}
  }
  return reason;
}

void writeMatrix(std::ostream & stream, int scenario, const std::string & mode,
  const std::string & name, const Matrix & value)
{
  stream << scenario << ',' << mode << ',' << name;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {stream << ',' << value(row, column);}
  }
  stream << '\n';
}

common::DefaultAlgorithmSuiteConfig readConfig(const fs::path & path, int threads)
{
  auto config = common::DefaultAlgorithmSuiteConfig{};
  const auto yaml = YAML::LoadFile(path.string());
#define PROBE_CONFIG(field, key) if (yaml[key]) {config.field = yaml[key].as<decltype(config.field)>();}
  PROBE_CONFIG(coarse.normal_radius_m, "quatro.fpfh_normal_radius_m")
  PROBE_CONFIG(coarse.feature_radius_m, "quatro.fpfh_radius_m")
  PROBE_CONFIG(coarse.noise_bound_m, "quatro.noise_bound_m")
  PROBE_CONFIG(coarse.rotation_gnc_factor, "quatro.rotation_gnc_factor")
  PROBE_CONFIG(coarse.rotation_cost_threshold, "quatro.rotation_cost_threshold")
  PROBE_CONFIG(coarse.rotation_max_iterations, "quatro.rotation_max_iterations")
  PROBE_CONFIG(coarse.estimate_scale, "quatro.estimate_scale")
  PROBE_CONFIG(coarse.optimized_matching, "quatro.optimized_matching")
  PROBE_CONFIG(coarse.descriptor_distance_threshold, "quatro.descriptor_distance_threshold")
  PROBE_CONFIG(coarse.maximum_correspondences, "quatro.maximum_correspondences")
  PROBE_CONFIG(coarse.minimum_points, "quatro.minimum_points")
  PROBE_CONFIG(fine.maximum_correspondence_distance_m, "nano_gicp.maximum_correspondence_distance_m")
  PROBE_CONFIG(fine.correspondence_randomness, "nano_gicp.correspondence_randomness")
  PROBE_CONFIG(fine.maximum_iterations, "nano_gicp.maximum_iterations")
  PROBE_CONFIG(fine.transformation_epsilon, "nano_gicp.transformation_epsilon")
  PROBE_CONFIG(fine.rotation_epsilon, "nano_gicp.rotation_epsilon")
  PROBE_CONFIG(fine.regularization_method, "nano_gicp.regularization_method")
  PROBE_CONFIG(fine.fitness_score_max_range_m, "nano_gicp.fitness_score_max_range_m")
  PROBE_CONFIG(fine.minimum_points, "nano_gicp.minimum_points")
  PROBE_CONFIG(fine.source_voxel_leaf_m, "nano_gicp.source_voxel_leaf_m")
  PROBE_CONFIG(validation.maximum_fitness, "loop_validation.maximum_fitness")
  PROBE_CONFIG(validation.minimum_overlap, "loop_validation.minimum_overlap")
  PROBE_CONFIG(validation.minimum_correspondences, "loop_validation.minimum_correspondences")
  PROBE_CONFIG(validation.maximum_translation_jump_m, "loop_validation.maximum_translation_jump_m")
  PROBE_CONFIG(validation.maximum_rotation_jump_rad, "loop_validation.maximum_rotation_jump_rad")
  PROBE_CONFIG(validation.minimum_candidate_distance_separation, "loop_validation.minimum_candidate_distance_separation")
  PROBE_CONFIG(evidence_distance_m, "pipeline.evidence_distance_m")
#undef PROBE_CONFIG
  config.fine.thread_count = threads;
  return config;
}

struct Layer
{
  std::string name;
  std::vector<double> nearest;
  std::vector<double> paired;
};

void support(std::ostream & stream, int scenario, const std::string & mode,
  const common::CloudConstPtr & source, const common::CloudConstPtr & target,
  const Eigen::Vector3d & source_gravity, const Matrix & estimate)
{
  pcl::KdTreeFLANN<common::PointT> tree;
  tree.setInputCloud(target);
  const auto moved = transform(source, estimate);
  const Eigen::Vector3d up = -source_gravity.normalized();
  std::array<Layer, 4> layers{{{"all", {}, {}}, {"synthetic_reference_below_0m", {}, {}},
    {"synthetic_reference_0_to_2m", {}, {}}, {"synthetic_reference_above_2m", {}, {}}}};
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (std::size_t index = 0; index < source->size(); ++index) {
    const double height = up.dot(source->points[index].getVector3fMap().cast<double>());
    const std::size_t stratum = height <= 0.0 ? 1U : height <= 2.0 ? 2U : 3U;
    const double paired = (moved->points[index].getVector3fMap() -
      target->points[index].getVector3fMap()).norm();
    const bool found = tree.nearestKSearch(moved->points[index], 1, indices, distances) == 1;
    const double nearest = found ? std::sqrt(distances.front()) : std::numeric_limits<double>::infinity();
    for (const auto layer : std::array<std::size_t, 2>{{0U, stratum}}) {
      layers[layer].nearest.push_back(nearest);
      layers[layer].paired.push_back(paired);
    }
  }
  for (auto & layer : layers) {
    if (layer.nearest.empty()) {continue;}
    double nearest_sum = 0.0, paired_sum = 0.0;
    std::size_t matched = 0U;
    for (const double value : layer.nearest) {nearest_sum += value * value; matched += value <= 0.5;}
    for (const double value : layer.paired) {paired_sum += value * value;}
    std::sort(layer.nearest.begin(), layer.nearest.end());
    std::sort(layer.paired.begin(), layer.paired.end());
    const auto p95 = static_cast<std::size_t>(0.95 * static_cast<double>(layer.nearest.size() - 1U));
    stream << scenario << ',' << mode << ',' << layer.name << ',' << layer.nearest.size() << ','
           << matched << ',' << static_cast<double>(matched) / layer.nearest.size() << ','
           << std::sqrt(nearest_sum / layer.nearest.size()) << ',' << layer.nearest[p95] << ','
           << std::sqrt(paired_sum / layer.paired.size()) << ',' << layer.paired[p95] << ','
           << layer.paired.back() << '\n';
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 3 || argc > 4) {
      throw std::runtime_error("usage: registration_frame_probe BUNDLE EMPTY_OUTPUT_DIR [NANO_THREADS=1]");
    }
    const fs::path bundle = fs::canonical(argv[1]);
    const fs::path output = fs::weakly_canonical(argv[2]);
    const auto relative = output.lexically_relative(bundle);
    if (relative.empty() || *relative.begin() != "..") {
      throw std::runtime_error("output must be outside original Bundle");
    }
    const int threads = argc == 4 ? std::stoi(argv[3]) : 1;
    if (threads < 1 || threads > 4) {throw std::runtime_error("require 1 <= NANO_THREADS <= 4");}
    if (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output))) {
      throw std::runtime_error("output must be a new or empty directory; old evidence is preserved");
    }
    fs::create_directories(output);
    if (!fs::create_directory(output / ".probe_claim")) {
      throw std::runtime_error("probe output already reserved; use a fresh directory");
    }
    const fs::path input = bundle / "keyframes/clouds/000630.pcd";
    auto base = common::CloudPtr{new common::Cloud{}};
    if (pcl::io::loadPCDFile(input.string(), *base) != 0 || base->empty()) {
      throw std::runtime_error("cannot load representative body cloud");
    }
    const auto config = readConfig(bundle / "config/registration.yaml", threads);
    fs::copy_file(bundle / "config/registration.yaml", output / "registration_snapshot.yaml");
    fs::copy_file(bundle / "metadata.yaml", output / "bundle_metadata_snapshot.yaml");
    auto runs = std::ofstream{output / "runs.csv"};
    auto transforms = std::ofstream{output / "transforms.csv"};
    auto supports = std::ofstream{output / "full_point_support.csv"};
    auto cases = std::ofstream{output / "synthetic_references.csv"};
    auto metadata = std::ofstream{output / "run_metadata.txt"};
    if (!runs || !transforms || !supports || !cases || !metadata) {throw std::runtime_error("cannot open outputs");}
    for (auto * stream : {&runs, &transforms, &supports, &cases, &metadata}) {*stream << std::setprecision(17);}
    runs << "scenario,mode,source_points,target_points,preprocess_ms,registration_wall_ms,pipeline_total_ms,quatro_ms,nano_gicp_ms,evidence_ms,pipeline_accepted,coarse_usable,fine_usable,coarse_translation_error_m,coarse_rotation_error_rad,fitness,overlap,correspondences,translation_error_m,rotation_error_rad,synthetic_gt_pass,reason\n";
    transforms << "scenario,mode,name";
    for (int row = 0; row < 4; ++row) {for (int column = 0; column < 4; ++column) {transforms << ",m" << row << column;}}
    transforms << '\n';
    supports << "scenario,mode,synthetic_height_stratum,full_points,nn_matches_under_0_5m,nn_coverage,all_nn_rms_m,all_nn_p95_m,known_correspondence_rms_m,known_correspondence_p95_m,known_correspondence_max_m\n";
    cases << "scenario,g_source_x,g_source_y,g_source_z,g_target_x,g_target_y,g_target_z,synthetic_plane_slope_before_rad,synthetic_plane_slope_after_rad,ground_truth_restoration_error\n";
    metadata << "scope=synthetic same-cloud operator probe; not real reverse revisit; not historical gravity; not FAST-LIO initialization acceptance\n"
             << "source_body_cloud=" << input << "\nsource_frame=body\ninput_frame_id=630\n"
             << "target_generation=T_GT*source; explicit complete SE3; no scene deletion or correspondence trimming\n"
             << "gravity_source=independent synthetic base vector [0,0,-9.81] rotated consistently into each generated coordinate frame\n"
             << "normalization=minimal anti-gravity to +Z rotation; no physical extrinsic changes\n"
             << "normalization_version=gravity-rp-v3-1\n"
             << "restoration=T_original_target_source=inverse(A_target)*T_normalized_target_source*A_source\n"
             << "quatro_hint=nullopt; actual DefaultAlgorithmSuite with Bundle registration snapshot\n"
             << "pose_jump_guard_evaluated=false\ndescriptor_ambiguity_evaluated=false\n"
             << "pipeline_accepted_scope=synthetic operator geometry only; not online loop acceptance\n"
             << "unusable_estimated_transforms=omitted; consult runs.csv coarse_usable/fine_usable\n"
             << "fine_thread_count_override=" << threads << "\nOMP_NUM_THREADS="
             << (std::getenv("OMP_NUM_THREADS") ? std::getenv("OMP_NUM_THREADS") : "unset")
             << "\nevaluation=all original generated full points; indexed GT correspondences plus full-target nearest neighbours\n"
             << "synthetic_gt_pass=translation_error<=0.1m and rotation_error<=0.02rad; not a production acceptance threshold\n";
    const auto ground_truth = rigid(rpy(12.0, -8.0, 35.0), Eigen::Vector3d{2.0, -1.0, 0.3});
    const Eigen::Vector3d base_gravity{0.0, 0.0, -9.81};
    for (const double x : {0.0, 1e-5, 1e-4}) {
      const Eigen::Vector3d opposite{x, 0.0, 9.81};
      const Eigen::Matrix3d rotation = alignment(opposite).block<3, 3>(0, 0);
      if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-12 ||
        (rotation * opposite - Eigen::Vector3d{0.0, 0.0, -opposite.norm()}).norm() > 1e-12)
      {
        throw std::runtime_error("near-antipodal normalization contract failed");
      }
    }
    const Eigen::Vector3d synthetic_plane = Eigen::Vector3d{-0.1, 0.05, 1.0}.normalized();
    for (int scenario = 0; scenario < 2; ++scenario) {
      const auto startup = rigid(scenario == 0 ? Eigen::Matrix3d::Identity() : rpy(-10.0, 7.0, -25.0));
      const auto source = transform(base, startup);
      const auto target = transform(source, ground_truth);
      const Eigen::Vector3d source_gravity = startup.block<3, 3>(0, 0) * base_gravity;
      const Eigen::Vector3d target_gravity = ground_truth.block<3, 3>(0, 0) * source_gravity;
      const Matrix source_alignment = alignment(source_gravity);
      const Matrix target_alignment = alignment(target_gravity);
      const Matrix normalized_truth = target_alignment * ground_truth * source_alignment.inverse();
      const double restoration_error = (target_alignment.inverse() * normalized_truth * source_alignment - ground_truth).norm();
      const Eigen::Vector3d source_normal = startup.block<3, 3>(0, 0) * synthetic_plane;
      const double before = std::acos(std::clamp(source_normal.dot(-source_gravity.normalized()), -1.0, 1.0));
      const double after = std::acos(std::clamp((source_alignment.block<3, 3>(0, 0) * source_normal).z(), -1.0, 1.0));
      if (restoration_error > 1e-10 || std::abs(before - after) > 1e-10 ||
        (normalized_truth.block<3, 3>(0, 0) * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm() > 1e-10)
      {
        throw std::runtime_error("synthetic normalization/restoration/slope contract failed");
      }
      cases << scenario;
      for (int axis = 0; axis < 3; ++axis) {cases << ',' << source_gravity[axis];}
      for (int axis = 0; axis < 3; ++axis) {cases << ',' << target_gravity[axis];}
      cases << ',' << before << ',' << after << ',' << restoration_error << '\n';
      for (const bool normalized : {false, true}) {
        const std::string mode = normalized ? "independent_rp_normalized" : "raw";
        const auto preprocess_started = Clock::now();
        const auto registration_source = normalized ? transform(source, source_alignment) : source;
        const auto registration_target = normalized ? transform(target, target_alignment) : target;
        const double preprocess_ms = elapsedMs(preprocess_started);
        const auto suite = common::createDefaultAlgorithmSuite(config);
        const auto started = Clock::now();
        const auto result = suite.registration->run(registration_source, registration_target, std::nullopt);
        const double wall_ms = elapsedMs(started);
        const bool usable = common::isUsableRegistration(result.fine);
        const bool coarse_usable = common::isUsableRegistration(result.coarse);
        const Matrix coarse_estimated = matrix(result.coarse.transform);
        const Matrix coarse_restored = normalized ?
          (target_alignment.inverse() * coarse_estimated * source_alignment).eval() : coarse_estimated;
        const double coarse_translation_error = coarse_usable ?
          (coarse_restored.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3)).norm() : std::numeric_limits<double>::quiet_NaN();
        const double coarse_rotation_error = coarse_usable ? Eigen::AngleAxisd(
          ground_truth.block<3, 3>(0, 0).transpose() * coarse_restored.block<3, 3>(0, 0)).angle() :
          std::numeric_limits<double>::quiet_NaN();
        const Matrix estimated = matrix(result.fine.transform);
        const Matrix restored = normalized ?
          (target_alignment.inverse() * estimated * source_alignment).eval() : estimated;
        const double translation_error = usable ?
          (restored.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3)).norm() : std::numeric_limits<double>::quiet_NaN();
        const double rotation_error = usable ? Eigen::AngleAxisd(
          ground_truth.block<3, 3>(0, 0).transpose() * restored.block<3, 3>(0, 0)).angle() :
          std::numeric_limits<double>::quiet_NaN();
        runs << scenario << ',' << mode << ',' << source->size() << ',' << target->size() << ','
             << preprocess_ms << ',' << wall_ms << ',' << result.total_ms << ',' << result.coarse.elapsed_ms << ','
             << result.fine.elapsed_ms << ',' << result.coarse_evidence_ms + result.fine_evidence_ms << ','
             << result.success << ',' << coarse_usable << ',' << usable << ','
             << coarse_translation_error << ',' << coarse_rotation_error << ','
             << result.fine.fitness << ',' << result.fine.overlap << ',' << result.fine.correspondence_count << ','
             << translation_error << ',' << rotation_error << ','
             << (usable && translation_error <= 0.1 && rotation_error <= 0.02) << ','
             << csvReason(result.validation.reason) << '\n';
        std::vector<std::pair<std::string, Matrix>> matrices{
            {"startup_source_from_original_body", startup}, {"T_GT_original_target_source", ground_truth},
            {"A_source", source_alignment}, {"A_target", target_alignment},
            {"T_GT_normalized_target_source", normalized_truth}};
        if (coarse_usable) {
          matrices.emplace_back("coarse_estimated_in_registration_frames", coarse_estimated);
          matrices.emplace_back("coarse_restored_original_target_source", coarse_restored);
        }
        if (usable) {
          matrices.emplace_back("estimated_in_registration_frames", estimated);
          matrices.emplace_back("restored_original_target_source", restored);
        }
        for (const auto & item : matrices) {
          writeMatrix(transforms, scenario, mode, item.first, item.second);
        }
        if (usable) {support(supports, scenario, mode, source, target, source_gravity, restored);}
        std::cout << "scenario=" << scenario << " mode=" << mode << " pipeline=" << result.success
                  << " translation_error_m=" << translation_error << " rotation_error_rad=" << rotation_error
                  << " registration_ms=" << wall_ms << '\n';
      }
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "registration_frame_probe: " << error.what() << '\n';
    return 1;
  }
}
