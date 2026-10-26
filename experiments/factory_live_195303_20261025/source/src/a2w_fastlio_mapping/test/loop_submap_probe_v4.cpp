// Offline fixed-input diagnostic only. Never linked into the production pipeline.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace submap_v4
{
using Matrix = Eigen::Matrix4d;
std::array<std::size_t, 2> cacheExecutionOrder(bool reverse)
{return reverse ? std::array<std::size_t, 2>{16U, 0U} : std::array<std::size_t, 2>{0U, 16U};}
std::size_t differingFeatureBins(const float * a, const float * b)
{
  std::size_t mismatches = 0;
  for (int bin = 0; bin < 33; ++bin) {mismatches += std::memcmp(a + bin, b + bin, sizeof(float)) != 0;}
  return mismatches;
}
struct BenchmarkCase {std::uint64_t candidate; std::string original_reason;};
std::vector<BenchmarkCase> selectBenchmarkCases(const std::vector<BenchmarkCase> & actual)
{
  std::vector<BenchmarkCase> unique;
  for (const auto & item : actual) {
    if (std::none_of(unique.begin(), unique.end(), [&](const auto & prior) {return prior.candidate == item.candidate;})) {
      unique.push_back(item);
    }
  }
  std::vector<bool> include(unique.size(), false);
  for (std::size_t i = 0; i < std::min<std::size_t>(3, unique.size()); ++i) {include[i] = true;}
  for (const auto & reason : {"fitness_above_limit", "loop_accepted"}) {
    const auto at = std::find_if(unique.begin(), unique.end(), [&](const auto & item) {return item.original_reason == reason;});
    if (at != unique.end()) {include[std::distance(unique.begin(), at)] = true;}
  }
  std::vector<BenchmarkCase> selected;
  for (std::size_t i = 0; i < unique.size(); ++i) {if (include[i]) {selected.push_back(unique[i]);}}
  return selected;
}
std::vector<std::uint64_t> window(std::uint64_t center, std::size_t radius,
  std::uint64_t last_available)
{
  if (center > last_available) {throw std::invalid_argument("future anchor");}
  const auto first = center < radius ? 0U : center - radius;
  const auto last = radius > last_available - center ? last_available : center + radius;
  std::vector<std::uint64_t> ids;
  for (auto id = first; id <= last; ++id) {ids.push_back(id);}
  return ids;
}
std::vector<std::uint64_t> withoutCenter(std::vector<std::uint64_t> ids, std::uint64_t center)
{ids.erase(std::remove(ids.begin(), ids.end(), center), ids.end()); return ids;}
Matrix restored(const Matrix & solver, const Matrix & source_frame, const Matrix & target_frame)
{return target_frame.inverse() * solver * source_frame;}
Matrix factor(const Matrix & solver, const Matrix & source_frame, const Matrix & target_frame)
{return restored(solver, source_frame, target_frame).inverse();}
double rigidTranslationError(const Matrix & raw_anchor, const Matrix & raw_neighbor,
  const Matrix & optimized_anchor, const Matrix & optimized_neighbor)
{
  const Matrix a = raw_anchor.inverse() * raw_neighbor;
  const Matrix b = optimized_anchor.inverse() * optimized_neighbor;
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}
}  // namespace submap_v4

#ifndef LOOP_SUBMAP_PROBE_V4_NO_MAIN
#include <array>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>
#include <quatro/quatro_module.h>
#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_common/nano_gicp_registration.hpp"
#include "a2w_fastlio_common/quatro_registration.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace submap_v4
{
namespace fs = std::filesystem;
namespace common = a2w_fastlio_common;
namespace mapping = a2w_fastlio_mapping;
using Row = std::map<std::string, std::string>;
using Clock = std::chrono::steady_clock;
using Id = std::uint64_t;
double milliseconds(const Clock::time_point & start)
{return std::chrono::duration<double, std::milli>(Clock::now() - start).count();}
Matrix matrix(const common::Pose3d & pose)
{
  Matrix m = Matrix::Identity();
  m.block<3, 3>(0, 0) = pose.rotation.normalized().toRotationMatrix();
  m.block<3, 1>(0, 3) = pose.translation; return m;
}
common::Pose3d pose(const Matrix & m)
{return {Eigen::Quaterniond{m.block<3, 3>(0, 0)}.normalized(), m.block<3, 1>(0, 3)};}
double rotationError(const Matrix & a, const Matrix & b)
{return Eigen::AngleAxisd{a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0)}.angle();}

std::vector<std::string> cells(const std::string & line)
{
  std::stringstream input(line); std::vector<std::string> result; std::string cell;
  while (std::getline(input, cell, ',')) {result.push_back(cell);} return result;
}
std::vector<Row> csv(const fs::path & path)
{
  std::ifstream input(path); std::string line;
  if (!input || !std::getline(input, line)) {throw std::runtime_error("cannot read CSV " + path.string());}
  const auto names = cells(line); std::vector<Row> rows;
  while (std::getline(input, line)) {
    if (line.empty()) {continue;} const auto values = cells(line);
    if (values.size() != names.size()) {throw std::runtime_error("CSV length " + path.string());}
    Row row; for (std::size_t i = 0; i < names.size(); ++i) {row[names[i]] = values[i];}
    rows.push_back(std::move(row));
  } return rows;
}
common::Pose3d readPose(const Row & row, const std::string & prefix = "")
{
  const auto n = [&](const char * key) {
      const auto value = std::stod(row.at(prefix + key));
      if (!std::isfinite(value)) {throw std::runtime_error("nonfinite pose");} return value;
    };
  common::Pose3d p; p.translation = Eigen::Vector3d{n("tx"), n("ty"), n("tz")};
  p.rotation = Eigen::Quaterniond{n("qw"), n("qx"), n("qy"), n("qz")};
  if (p.rotation.norm() < 1e-12) {throw std::runtime_error("zero quaternion");}
  p.rotation.normalize(); return p;
}
common::DefaultAlgorithmSuiteConfig algorithmConfig(const fs::path & path, int threads)
{
  const auto yaml = YAML::LoadFile(path.string()); common::DefaultAlgorithmSuiteConfig config;
#define READ(field, key) if (yaml[key]) {config.field = yaml[key].as<decltype(config.field)>();}
  READ(coarse.normal_radius_m, "quatro.fpfh_normal_radius_m")
  READ(coarse.feature_radius_m, "quatro.fpfh_radius_m")
  READ(coarse.noise_bound_m, "quatro.noise_bound_m")
  READ(coarse.rotation_gnc_factor, "quatro.rotation_gnc_factor")
  READ(coarse.rotation_cost_threshold, "quatro.rotation_cost_threshold")
  READ(coarse.rotation_max_iterations, "quatro.rotation_max_iterations")
  READ(coarse.estimate_scale, "quatro.estimate_scale")
  READ(coarse.optimized_matching, "quatro.optimized_matching")
  READ(coarse.descriptor_distance_threshold, "quatro.descriptor_distance_threshold")
  READ(coarse.maximum_correspondences, "quatro.maximum_correspondences")
  READ(coarse.minimum_points, "quatro.minimum_points")
  READ(fine.maximum_correspondence_distance_m, "nano_gicp.maximum_correspondence_distance_m")
  READ(fine.correspondence_randomness, "nano_gicp.correspondence_randomness")
  READ(fine.maximum_iterations, "nano_gicp.maximum_iterations")
  READ(fine.transformation_epsilon, "nano_gicp.transformation_epsilon")
  READ(fine.rotation_epsilon, "nano_gicp.rotation_epsilon")
  READ(fine.regularization_method, "nano_gicp.regularization_method")
  READ(fine.fitness_score_max_range_m, "nano_gicp.fitness_score_max_range_m")
  READ(fine.minimum_points, "nano_gicp.minimum_points")
  READ(fine.source_voxel_leaf_m, "nano_gicp.source_voxel_leaf_m")
  READ(validation.maximum_fitness, "loop_validation.maximum_fitness")
  READ(validation.minimum_overlap, "loop_validation.minimum_overlap")
  READ(validation.minimum_correspondences, "loop_validation.minimum_correspondences")
  READ(validation.maximum_translation_jump_m, "loop_validation.maximum_translation_jump_m")
  READ(validation.maximum_rotation_jump_rad, "loop_validation.maximum_rotation_jump_rad")
  READ(validation.minimum_candidate_distance_separation, "loop_validation.minimum_candidate_distance_separation")
  READ(evidence_distance_m, "pipeline.evidence_distance_m")
#undef READ
  config.fine.thread_count = threads; return config;
}
mapping::PoseGraphConfig graphConfig(const fs::path & path)
{
  const auto yaml = YAML::LoadFile(path.string()); mapping::PoseGraphConfig config;
#define READ(field) config.field = yaml["pose_graph." #field].as<decltype(config.field)>()
  READ(prior_rotation_sigma_rad); READ(prior_translation_sigma_m);
  READ(odometry_rotation_sigma_rad); READ(odometry_translation_sigma_m);
  READ(loop_rotation_sigma_rad); READ(loop_translation_sigma_m); READ(robust_kernel_scale);
  READ(relinearization_threshold); READ(relinearization_skip);
#undef READ
  const auto kernel = yaml["pose_graph.robust_kernel"].as<std::string>();
  if (kernel == "huber") {config.robust_kernel = mapping::RobustKernel::HUBER;}
  else if (kernel == "cauchy") {config.robust_kernel = mapping::RobustKernel::CAUCHY;}
  else {throw std::runtime_error("unsupported graph kernel");} return config;
}
void require(const mapping::GraphUpdate & update)
{if (!update.accepted) {throw std::runtime_error("graph: " + update.reason);}}
struct Frame {Id id; std::int64_t stamp; std::string cloud; common::Pose3d raw;};
struct Measurement {Id current, history; common::Pose3d relative;};
using Snapshot = std::vector<common::Pose3d>;
std::map<Id, Snapshot> preQuerySnapshots(const std::vector<Frame> & frames,
  const std::vector<Measurement> & loops, const std::set<Id> & selected,
  const mapping::PoseGraphConfig & config)
{
  mapping::PoseGraphOptimizer graph(config); std::map<Id, Snapshot> result;
  for (const auto & frame : frames) {
    const auto before = graph.optimizedPoses();
    require(graph.addKeyFrame(frame.id, frame.raw));
    if (selected.count(frame.id)) {
      Snapshot snapshot;
      for (const auto & entry : before.poses) {
        if (entry.id != snapshot.size()) {throw std::runtime_error("noncontiguous committed graph");}
        snapshot.push_back(entry.pose);
      }
      if (snapshot.size() != frame.id) {throw std::runtime_error("stale graph snapshot");}
      const Matrix prediction = frame.id == 0 ? matrix(frame.raw) :
        matrix(snapshot.back()) * matrix(frames[frame.id - 1].raw).inverse() * matrix(frame.raw);
      snapshot.push_back(pose(prediction)); result.emplace(frame.id, std::move(snapshot));
    }
    for (const auto & loop : loops) {
      if (loop.current == frame.id) {
        require(graph.addLoopConstraint({loop.current, loop.history, loop.relative, true}));
      }
    }
    require(graph.update());
    if (frame.id == *selected.rbegin()) {break;}
  } return result;
}
void savePoses(const fs::path & path, const Snapshot & snapshot)
{
  std::ofstream output(path); output << std::setprecision(17) << "id,tx,ty,tz,qx,qy,qz,qw\n";
  for (Id id = 0; id < snapshot.size(); ++id) {
    const auto & p = snapshot[id]; output << id << ',' << p.translation.x() << ',' << p.translation.y()
      << ',' << p.translation.z() << ',' << p.rotation.x() << ',' << p.rotation.y() << ','
      << p.rotation.z() << ',' << p.rotation.w() << '\n';
  }
}
struct Provider : common::KeyFrameProvider
{
  std::map<Id, common::KeyFrame> frames;
  std::optional<common::KeyFrame> get(Id id) const override
  {const auto at = frames.find(id); return at == frames.end() ? std::nullopt : std::optional<common::KeyFrame>{at->second};}
  std::optional<std::pair<Id, Id>> bounds() const override
  {return frames.empty() ? std::nullopt : std::optional<std::pair<Id, Id>>{{frames.begin()->first, frames.rbegin()->first}};}
};
struct Ground {bool reliable; Eigen::Vector3d normal; double d;};
struct Built {common::CloudPtr cloud; Matrix anchor_to_input; std::vector<Id> ids;};
std::string idList(const std::vector<Id> & ids)
{std::string value; for (const auto id : ids) {if (!value.empty()) {value += ';';} value += std::to_string(id);} return value;}
void cloudStats(std::ostream & stream, Id current, Id history, const std::string & variant,
  const std::string & side, const std::string & stage, const common::CloudConstPtr & cloud,
  const Matrix & anchor_to_input, const Ground & ground)
{
  Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high = -low; std::size_t body_high = 0, ground_high = 0;
  const Matrix inverse = anchor_to_input.inverse();
  for (const auto & point : cloud->points) {
    const Eigen::Vector3d xyz = point.getVector3fMap().cast<double>();
    low = low.cwiseMin(xyz); high = high.cwiseMax(xyz);
    const Eigen::Vector3d body = inverse.block<3, 3>(0, 0) * xyz + inverse.block<3, 1>(0, 3);
    body_high += body.z() > 2.0;
    if (ground.reliable) {ground_high += ground.normal.dot(body) + ground.d > 3.0;}
  }
  stream << current << ',' << history << ',' << variant << ',' << side << ',' << stage << ',' << cloud->size();
  for (int k = 0; k < 3; ++k) {stream << ',' << low[k] << ',' << high[k];}
  stream << ',' << static_cast<double>(body_high) / cloud->size() << ',' << ground.reliable << ',';
  if (ground.reliable) {stream << static_cast<double>(ground_high) / cloud->size();} stream << '\n';
}
Built build(Id current, Id history, Id anchor, const std::string & variant,
  const std::vector<Id> & ids, const std::vector<Frame> & original, const Snapshot & snapshot,
  const std::map<Id, common::CloudPtr> & clouds, common::LocalMapConfig config,
  const Ground & ground, const std::string & side, std::ostream & counts, std::ostream & members)
{
  Provider provider; const bool in_map = variant == "optimized_map_window5" ||
    variant == "optimized_map_leave_centers_out";
  const bool optimized_anchor = variant == "optimized_anchor_window5";
  const Matrix anchor_to_input = in_map ? matrix(snapshot.at(anchor)) : Matrix::Identity();
  const Matrix raw_anchor = matrix(original.at(anchor).raw);
  auto merged = common::CloudPtr{new common::Cloud{}};
  for (Id id : ids) {
    common::KeyFrame frame; frame.id = id; frame.stamp_ns = original.at(id).stamp;
    frame.body_cloud = clouds.at(id); frame.odom_pose = original.at(id).raw;
    frame.optimized_pose = in_map ? snapshot.at(id) : optimized_anchor ?
      pose(matrix(snapshot.at(anchor)).inverse() * matrix(snapshot.at(id))) :
      pose(raw_anchor.inverse() * matrix(frame.odom_pose));
    provider.frames.emplace(id, frame);
    // Use the same quaternion action, double transform, float cast as LocalMapBuilder.
    for (const auto & point : frame.body_cloud->points) {
      const Eigen::Vector3d xyz = frame.optimized_pose.rotation.normalized() *
        point.getVector3fMap().cast<double>() + frame.optimized_pose.translation;
      auto output = point; output.x = static_cast<float>(xyz.x());
      output.y = static_cast<float>(xyz.y()); output.z = static_cast<float>(xyz.z()); merged->push_back(output);
    }
    const Matrix opt_relative = matrix(snapshot.at(anchor)).inverse() * matrix(snapshot.at(id));
    const Matrix raw_relative = raw_anchor.inverse() * matrix(original.at(id).raw);
    members << current << ',' << history << ',' << variant << ',' << side << ',' << anchor << ',' << id << ','
      << original.at(id).stamp << ',' << frame.body_cloud->size() << ','
      << rigidTranslationError(raw_anchor, matrix(original.at(id).raw), matrix(snapshot.at(anchor)), matrix(snapshot.at(id)))
      << ',' << rotationError(raw_relative, opt_relative) << '\n';
  }
  cloudStats(counts, current, history, variant, side, "merged", merged, anchor_to_input, ground);
  common::LocalMapBuilder voxel_builder({config.voxel_leaf_m, std::numeric_limits<std::size_t>::max()});
  const auto voxel = voxel_builder.build(ids, provider).cloud;
  cloudStats(counts, current, history, variant, side, "voxel_before_cap", voxel, anchor_to_input, ground);
  const auto final = common::LocalMapBuilder(config).build(ids, provider).cloud;
  cloudStats(counts, current, history, variant, side, "after_cap", final, anchor_to_input, ground);
  if (voxel->size() <= config.max_points && voxel->size() != final->size()) {throw std::runtime_error("unexpected cap");}
  for (std::size_t i = 0; i < final->size(); ++i) {
    const auto & a = voxel->at(i); const auto & b = final->at(i);
    if (a.x != b.x || a.y != b.y || a.z != b.z || a.intensity != b.intensity) {
      throw std::runtime_error("non-identical production builder prefix");
    }
  }
  return {final, anchor_to_input, ids};
}
common::CloudPtr transformed(const common::CloudConstPtr & cloud, const Matrix & transform)
{
  auto moved = common::CloudPtr{new common::Cloud{}}; moved->reserve(cloud->size());
  for (const auto & input : cloud->points) {
    auto output = input; const Eigen::Vector3d v = transform.block<3, 3>(0, 0) *
      input.getVector3fMap().cast<double>() + transform.block<3, 1>(0, 3);
    output.x = static_cast<float>(v.x()); output.y = static_cast<float>(v.y()); output.z = static_cast<float>(v.z());
    moved->push_back(output);
  } return moved;
}
struct Normals {std::vector<Eigen::Vector3d> values; std::vector<bool> valid;};
Normals normals(const common::CloudConstPtr & cloud)
{
  pcl::KdTreeFLANN<common::PointT> tree; tree.setInputCloud(cloud); Normals result;
  for (const auto & point : cloud->points) {
    std::vector<int> indices(12); std::vector<float> distances(12);
    const int count = tree.nearestKSearch(point, 12, indices, distances);
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int j = 0; j < count; ++j) {centroid += cloud->at(indices[j]).getVector3fMap().cast<double>();}
    centroid /= count; Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int j = 0; j < count; ++j) {
      const Eigen::Vector3d delta = cloud->at(indices[j]).getVector3fMap().cast<double>() - centroid;
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    const auto values = eigen.eigenvalues(); result.values.push_back(eigen.eigenvectors().col(0));
    result.valid.push_back(count == 12 && distances.back() <= .36F && values[1] > 1e-5 && values[0] / values[1] < .3);
  } return result;
}
std::vector<int> nearest(const common::CloudConstPtr & source, const common::CloudConstPtr & target, const Matrix & t)
{
  const auto moved = transformed(source, t); pcl::KdTreeFLANN<common::PointT> tree; tree.setInputCloud(target);
  std::vector<int> result; std::vector<int> indices(1); std::vector<float> distance(1);
  for (const auto & point : moved->points) {
    if (tree.nearestKSearch(point, 1, indices, distance) != 1) {throw std::runtime_error("nearest missing");}
    result.push_back(indices.front());
  } return result;
}
double quantile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::sort(values.begin(), values.end()); const double index = fraction * (values.size() - 1);
  const auto lower = static_cast<std::size_t>(index); const auto upper = std::min(lower + 1, values.size() - 1);
  return values[lower] + (values[upper] - values[lower]) * (index - lower);
}
struct FixedEvaluation
{
  common::CloudPtr source, target; Normals source_normals, target_normals;
  std::vector<int> st, ts; std::vector<bool> common_source, common_target;
  Ground source_ground, target_ground;
};
FixedEvaluation fixedEvaluation(common::CloudPtr source, common::CloudPtr target,
  const Matrix & prior, Ground source_ground, Ground target_ground)
{
  FixedEvaluation f{source, target, normals(source), normals(target), nearest(source, target, prior),
    nearest(target, source, prior.inverse()), {}, {}, source_ground, target_ground};
  const auto moved = transformed(source, prior); const auto back = transformed(target, prior.inverse());
  for (std::size_t i = 0; i < source->size(); ++i) {
    f.common_source.push_back((moved->at(i).getVector3fMap() - target->at(f.st[i]).getVector3fMap()).norm() <= 2.0);
  }
  for (std::size_t i = 0; i < target->size(); ++i) {
    f.common_target.push_back((back->at(i).getVector3fMap() - source->at(f.ts[i]).getVector3fMap()).norm() <= 2.0);
  } return f;
}
void evaluate(std::ostream & output, Id current, Id history, const std::string & variant,
  const std::string & mode, const FixedEvaluation & fixed, const Matrix & estimate)
{
  // Both directions, every original anchor-cloud point. The prior-selected <=2m
  // set is a frozen common-support PROXY, never actual visibility or ground truth.
  for (bool forward : {true, false}) {
    const auto & a = forward ? fixed.source : fixed.target; const auto & b = forward ? fixed.target : fixed.source;
    const auto & na = forward ? fixed.source_normals : fixed.target_normals;
    const auto & nb = forward ? fixed.target_normals : fixed.source_normals;
    const auto & match = forward ? fixed.st : fixed.ts;
    const auto & common = forward ? fixed.common_source : fixed.common_target;
    const auto & ground = forward ? fixed.source_ground : fixed.target_ground;
    const Matrix t = forward ? estimate : Matrix(estimate.inverse());
    const auto moved = transformed(a, t); const auto dynamic = nearest(a, b, t);
    for (const std::string & layer : {"all", "local_ground_low", "local_ground_mid", "local_ground_high"}) {
      if (layer != "all" && !ground.reliable) {continue;}
      for (bool common_only : {false, true}) {
        std::vector<double> nn, paired, plane, angle; std::map<int, std::size_t> multiplicity;
        std::map<std::pair<int, int>, std::size_t> cells; std::set<int> azimuths;
        std::size_t covered = 0, fixed_covered = 0;
        for (std::size_t i = 0; i < a->size(); ++i) {
          const Eigen::Vector3d original = a->at(i).getVector3fMap().cast<double>();
          const double height = ground.normal.dot(original) + ground.d;
          if ((layer == "local_ground_low" && height > .25) ||
            (layer == "local_ground_mid" && (height <= .25 || height > 3)) ||
            (layer == "local_ground_high" && height <= 3) || (common_only && !common[i])) {continue;}
          const Eigen::Vector3d x = moved->at(i).getVector3fMap().cast<double>();
          const Eigen::Vector3d fixed_delta = x - b->at(match[i]).getVector3fMap().cast<double>();
          const double distance = (x - b->at(dynamic[i]).getVector3fMap().cast<double>()).norm();
          nn.push_back(distance); paired.push_back(fixed_delta.norm());
          covered += distance <= .5; fixed_covered += fixed_delta.norm() <= .5;
          if (nb.valid[match[i]]) {plane.push_back(std::abs(nb.values[match[i]].dot(fixed_delta)));}
          if (na.valid[i] && nb.valid[match[i]]) {
            const Eigen::Vector3d normal = t.block<3, 3>(0, 0) * na.values[i];
            angle.push_back(std::atan2(normal.cross(nb.values[match[i]]).norm(),
              std::abs(normal.dot(nb.values[match[i]]))) * 180.0 / 3.14159265358979323846);
          }
          if (distance <= .5) {
            ++multiplicity[dynamic[i]];
            ++cells[{static_cast<int>(std::floor(original.x() / 2)), static_cast<int>(std::floor(original.y() / 2))}];
            azimuths.insert(static_cast<int>(std::floor((std::atan2(original.y(), original.x()) + 3.14159265358979323846) * 12 / (2 * 3.14159265358979323846))));
          }
        }
        if (nn.empty()) {continue;} std::size_t maximum = 0, concentrated = 0; double squares = 0;
        for (const auto & entry : multiplicity) {maximum = std::max(maximum, entry.second);}
        for (const auto & entry : cells) {concentrated = std::max(concentrated, entry.second); squares += entry.second * entry.second;}
        output << current << ',' << history << ',' << variant << ',' << mode << ',' << (forward ? "source_to_target" : "target_to_source")
          << ',' << layer << ',' << (common_only ? "frozen_prior_2m_common_support_proxy" : "full_original_points")
          << ',' << nn.size() << ',' << static_cast<double>(covered) / nn.size() << ',' << quantile(nn, .5) << ',' << quantile(nn, .95)
          << ',' << static_cast<double>(fixed_covered) / paired.size() << ',' << quantile(paired, .95)
          << ',' << plane.size() << ',' << quantile(plane, .95) << ',' << angle.size() << ',' << quantile(angle, .95)
          << ',' << covered << ',' << multiplicity.size() << ',' << (covered ? 1.0 - static_cast<double>(multiplicity.size()) / covered : 0)
          << ',' << maximum << ',' << cells.size() << ',' << (covered ? static_cast<double>(concentrated) / covered : 0)
          << ',' << (squares ? static_cast<double>(covered) * covered / squares : 0) << ',' << azimuths.size() << '\n';
      }
    }
  }
}
void writeTransform(std::ostream & output, Id current, Id history, const std::string & variant,
  const std::string & mode, const std::string & name, const Matrix & t)
{
  output << current << ',' << history << ',' << variant << ',' << mode << ',' << name;
  for (int row = 0; row < 4; ++row) {for (int col = 0; col < 4; ++col) {output << ',' << t(row, col);}}
  output << '\n';
}

std::uint64_t contentFingerprint(const common::CloudConstPtr & cloud)
{
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto & p : cloud->points) {
    for (const float value : {p.x, p.y, p.z, p.intensity}) {
      std::uint32_t bits; std::memcpy(&bits, &value, sizeof(bits));
      for (int byte = 0; byte < 4; ++byte) {hash ^= (bits >> (byte * 8)) & 255U; hash *= 1099511628211ULL;}
    }
  } return hash;
}
std::string clean(std::string value)
{for (auto & ch : value) {if (ch == ',' || ch == '\n' || ch == '\r') {ch = ';';}} return value;}
struct BenchInput {Id current, candidate; std::string reason; Built source, target;};
struct BenchResult {common::PipelineResult registration; common::ValidationResult pose_guard; double wall_ms;};
double difference(double a, double b)
{return a == b ? 0.0 : std::isfinite(a) && std::isfinite(b) ? std::abs(a - b) : std::numeric_limits<double>::infinity();}

int preparedAudit(const fs::path & output, const std::vector<BenchInput> & fixed,
  const common::DefaultAlgorithmSuiteConfig & config, std::ostream & metadata)
{
  using Prepared = quatro<common::PointT>::PreparedCloud;
  struct Held {std::uint64_t fingerprint; std::size_t points; Prepared prepared;};
  std::list<Held> held; std::size_t mismatches = 0, held_hits = 0, held_misses = 0;
  std::ofstream audit(output / "prepared_features.csv"); audit << std::setprecision(17);
  audit << "sequence,current,candidate,side,comparison,points,feature_bins,bit_mismatched_bins,bit_mismatched_points,max_finite_abs_difference,nonfinite_left_bins,nonfinite_right_bins\n";
  const auto compare = [&](std::size_t sequence, const BenchInput & input, const std::string & side,
      const std::string & name, const teaser::FPFHCloudPtr & a, const teaser::FPFHCloudPtr & b) {
      if (a->size() != b->size()) {throw std::runtime_error("prepared descriptor shape mismatch");}
      std::size_t bins = 0, points = 0, nonfinite_a = 0, nonfinite_b = 0; double maximum = 0;
      for (std::size_t i = 0; i < a->size(); ++i) {
        const auto count = differingFeatureBins(a->at(i).histogram, b->at(i).histogram);
        bins += count; points += count > 0;
        for (int bin = 0; bin < 33; ++bin) {
          const float x = a->at(i).histogram[bin], y = b->at(i).histogram[bin];
          nonfinite_a += !std::isfinite(x); nonfinite_b += !std::isfinite(y);
          if (std::isfinite(x) && std::isfinite(y)) {maximum = std::max(maximum, std::abs(static_cast<double>(x) - y));}
        }
      }
      mismatches += bins;
      audit << sequence << ',' << input.current << ',' << input.candidate << ',' << side << ',' << name << ',' << a->size()
        << ',' << a->size() * 33 << ',' << bins << ',' << points << ',' << maximum << ',' << nonfinite_a << ',' << nonfinite_b << '\n';
    };
  const auto cached = [&](const common::CloudPtr & cloud, const Prepared & fresh) {
      const auto fingerprint = contentFingerprint(cloud);
      const auto found = std::find_if(held.begin(), held.end(), [&](const auto & item) {
          return item.fingerprint == fingerprint && item.points == cloud->size();
        });
      if (found != held.end()) {
        ++held_hits; const auto result = found->prepared; held.splice(held.begin(), held, found); return result;
      }
      ++held_misses; held.push_front({fingerprint, cloud->size(), fresh});
      while (held.size() > 16) {held.pop_back();} return fresh;
    };
  for (std::size_t sequence = 0; sequence < fixed.size(); ++sequence) {
    const auto & input = fixed[sequence];
    quatro<common::PointT> solver(config.coarse.normal_radius_m, config.coarse.feature_radius_m, config.coarse.noise_bound_m,
      config.coarse.rotation_gnc_factor, config.coarse.rotation_cost_threshold, config.coarse.rotation_max_iterations,
      config.coarse.estimate_scale, config.coarse.optimized_matching, config.coarse.descriptor_distance_threshold, config.coarse.maximum_correspondences);
    const auto source = solver.prepare(*input.source.cloud), target = solver.prepare(*input.target.cloud);
    teaser::FPFHEstimation original;
    const auto original_source = original.computeFPFHFeatures(source.points, config.coarse.normal_radius_m, config.coarse.feature_radius_m);
    const auto original_target = original.computeFPFHFeatures(target.points, config.coarse.normal_radius_m, config.coarse.feature_radius_m);
    const auto source_repeat = solver.prepare(*input.source.cloud), target_repeat = solver.prepare(*input.target.cloud);
    const auto held_source = cached(input.source.cloud, source), held_target = cached(input.target.cloud, target);
    compare(sequence, input, "source", "original_single_estimator_vs_prepare", original_source, source.features);
    compare(sequence, input, "target", "original_single_estimator_vs_prepare", original_target, target.features);
    compare(sequence, input, "source", "prepare_vs_repeat_prepare", source.features, source_repeat.features);
    compare(sequence, input, "target", "prepare_vs_repeat_prepare", target.features, target_repeat.features);
    compare(sequence, input, "source", "prepare_vs_held_prepared_object", source.features, held_source.features);
    compare(sequence, input, "target", "prepare_vs_held_prepared_object", target.features, held_target.features);
    audit.flush();
    std::cout << "prepared audit " << sequence << ' ' << input.current << "->" << input.candidate << " mismatched_bins=" << mismatches << '\n' << std::flush;
  }
  metadata << "prepared_audit=public API only, no registration; held objects are independent diagnostic LRU not private Quatro cache introspection\n"
    << "held_cache_hits=" << held_hits << "\nheld_cache_misses=" << held_misses << "\nbit_mismatched_bins=" << mismatches << "\ncompleted=true\n";
  return mismatches ? 2 : 0;
}

int cacheBenchmark(const fs::path & output, const fs::path & bundle, const fs::path & actual,
  const std::vector<Frame> & frames, const std::map<Id, Snapshot> & snapshots,
  const std::map<Id, std::vector<BenchmarkCase>> & cases, const std::map<Id, common::CloudPtr> & clouds,
  const std::map<Id, Ground> & grounds, common::LocalMapConfig local,
  const common::DefaultAlgorithmSuiteConfig & config, bool prepared_only = false)
{
  fs::create_directories(output);
  if (!fs::create_directory(output / ".probe_claim")) {throw std::runtime_error("output reserved");}
  fs::create_directory(output / "snapshots");
  fs::copy_file(bundle / "config/registration.yaml", output / "registration_snapshot.yaml");
  fs::copy_file(bundle / "config/mapping_effective.yaml", output / "mapping_snapshot.yaml");
  fs::copy_file(actual / "loop_constraints.csv", output / "fixed_loop_constraints.csv");
  std::ofstream counts(output / "submap_counts.csv"), members(output / "submap_members.csv"),
    inputs(output / "fixed_inputs.csv"), results(output / "runs.csv"), transforms(output / "transforms.csv"),
    comparison(output / "cache_equivalence.csv"), metadata(output / "metadata.txt");
  for (auto * stream : {&counts, &members, &inputs, &results, &transforms, &comparison, &metadata}) {
    stream->exceptions(std::ios::failbit | std::ios::badbit); *stream << std::setprecision(17);
  }
  counts << "current,history,variant,side,stage,points,x_min,x_max,y_min,y_max,z_min,z_max,anchor_body_relative_z_gt2_fraction,local_ground_proxy_reliable,anchor_local_ground_proxy_gt3_fraction\n";
  members << "current,history,variant,side,anchor,id,stamp_ns,original_points,optimized_vs_raw_anchor_relative_translation_m,optimized_vs_raw_anchor_relative_rotation_rad\n";
  inputs << "sequence,current,candidate,actual_event_reason,source_ids,target_ids,source_points,target_points,source_xyzi_fnv64,target_xyzi_fnv64\n";
  results << "capacity,sequence,current,candidate,actual_event_reason,coarse_success,coarse_converged,coarse_fitness,coarse_overlap,coarse_correspondences,coarse_reason,fine_success,fine_converged,fitness,overlap,correspondences,fine_reason,geometry_valid,geometry_reason,pose_guard_valid,pose_guard_reason,operator_and_pose_valid,registration_wall_ms,quatro_ms,nano_ms,evidence_ms,hits_delta,misses_delta,total_hits,total_misses,entries\n";
  transforms << "current,history,variant,mode,name";
  for (int r = 0; r < 4; ++r) {for (int c = 0; c < 4; ++c) {transforms << ",m" << r << c;}} transforms << '\n';
  comparison << "sequence,current,candidate,decision_and_status_equal,coarse_translation_delta_m,coarse_rotation_delta_rad,fine_translation_delta_m,fine_rotation_delta_rad,coarse_fitness_delta,coarse_overlap_delta,coarse_correspondences_equal,fine_fitness_delta,fine_overlap_delta,fine_correspondences_equal,within_tolerance,uncached_wall_ms,cached_wall_ms,uncached_quatro_ms,cached_quatro_ms\n";
  metadata << "version=cache_benchmark_v4_20261021\nbundle=" << bundle << "\nactual_run=" << actual
    << "\nthreads=" << config.fine.thread_count << "\nfeature_cache_capacities=0,16\n"
    << "selection=first3 unique actually-registered events plus first actual fitness_above_limit and actual loop_accepted per selected frame, preserving CSV order\n"
    << "order=events CandidateFinished CSV order; winner notification may be deferred, not claimed online registration-start order\n"
    << "snapshots=strict pre-query actual historical factors; no current or future factor\n"
    << "inputs=build once using original LocalMapBuilder map +/-5, same cloud objects/content reused across cache variants\n"
    << "parameters=original config except explicit fine threads; SC/yaw/threshold unchanged; full pipeline nullopt hint\n"
    << "online_acceptance=false; per-candidate validator+pose guard only; all selected competitors still evaluated\n"
    << "timing=actual full registration wall including hashing/cache locks, excludes diagnostic submap builds and output IO\n"
    << "cache0_stats=disabled reports zero counters, not estimated no-cache misses\n"
    << "equivalence=all status/reason/validator/pose decisions identical; coarse/fine pose<=1e-6m/rad; fitness/overlap<=1e-8; correspondence counts identical\n";
  const char * frozen_time = std::getenv("A2W_FROZEN_MATCHER_TIME");
  const char * preload = std::getenv("LD_PRELOAD");
  metadata << "matcher_frozen_time_env=" << (frozen_time ? frozen_time : "UNSET_UNCONTROLLED")
    << "\nld_preload=" << (preload ? preload : "UNSET") << '\n';
  std::vector<BenchInput> fixed;
  for (const auto & entry : cases) {
    const Id current = entry.first; const auto & snapshot = snapshots.at(current);
    savePoses(output / "snapshots" / ("before_" + std::to_string(current) + ".csv"), snapshot);
    const auto source = build(current, current, current, "optimized_map_window5", window(current, 5, current),
      frames, snapshot, clouds, local, grounds.at(current), "source", counts, members);
    for (const auto & item : entry.second) {
      const auto target = build(current, item.candidate, item.candidate, "optimized_map_window5", window(item.candidate, 5, current),
        frames, snapshot, clouds, local, grounds.at(item.candidate), "target", counts, members);
      inputs << fixed.size() << ',' << current << ',' << item.candidate << ',' << clean(item.original_reason) << ','
        << idList(source.ids) << ',' << idList(target.ids) << ',' << source.cloud->size() << ',' << target.cloud->size()
        << ',' << contentFingerprint(source.cloud) << ',' << contentFingerprint(target.cloud) << '\n';
      fixed.push_back({current, item.candidate, item.original_reason, source, target});
    }
  }
  if (prepared_only) {return preparedAudit(output, fixed, config, metadata);}
  const char * reverse_env = std::getenv("A2W_CACHE_BENCHMARK_REVERSE");
  const bool reverse = reverse_env && std::string(reverse_env) == "1";
  metadata << "capacity_execution_order=" << (reverse ? "16,0" : "0,16") << '\n';
  std::map<std::size_t, std::vector<BenchResult>> measured; std::size_t mismatches = 0;
  for (const std::size_t capacity : cacheExecutionOrder(reverse)) {
    const auto coarse = std::make_shared<common::QuatroRegistration>(common::QuatroRegistrationConfig{
      config.coarse.normal_radius_m, config.coarse.feature_radius_m, config.coarse.noise_bound_m,
      config.coarse.rotation_gnc_factor, config.coarse.rotation_cost_threshold, config.coarse.rotation_max_iterations,
      config.coarse.estimate_scale, config.coarse.optimized_matching, config.coarse.descriptor_distance_threshold,
      config.coarse.maximum_correspondences, config.coarse.minimum_points, capacity});
    const auto fine = std::make_shared<common::NanoGicpRegistration>(common::NanoGicpRegistrationConfig{
      config.fine.maximum_correspondence_distance_m, config.fine.thread_count, config.fine.correspondence_randomness,
      config.fine.maximum_iterations, config.fine.transformation_epsilon, config.fine.rotation_epsilon,
      config.fine.regularization_method, config.fine.fitness_score_max_range_m, config.fine.minimum_points, config.fine.source_voxel_leaf_m});
    const common::MatchValidator validator{common::MatchValidationConfig{
      config.validation.maximum_fitness, config.validation.minimum_overlap, config.validation.minimum_correspondences,
      config.validation.maximum_translation_jump_m, config.validation.maximum_rotation_jump_rad,
      config.validation.minimum_candidate_distance_separation}};
    const common::RegistrationPipeline pipeline{coarse, fine, validator, {config.evidence_distance_m}};
    for (std::size_t sequence = 0; sequence < fixed.size(); ++sequence) {
      const auto & input = fixed[sequence]; const auto stats_before = coarse->cacheStats();
      const auto start = Clock::now(); const auto result = pipeline.run(input.source.cloud, input.target.cloud, std::nullopt);
      const double wall = milliseconds(start); const auto stats = coarse->cacheStats();
      common::ValidationResult guard{false, "fine_unusable"};
      if (common::isUsableRegistration(result.fine)) {
        auto in_map = result.fine; in_map.transform = pose(matrix(result.fine.transform) * matrix(snapshots.at(input.current).at(input.current)));
        common::ValidationContext context; context.predicted_transform = snapshots.at(input.current).at(input.current);
        guard = pipeline.validateResult(in_map, context);
      }
      results << capacity << ',' << sequence << ',' << input.current << ',' << input.candidate << ',' << clean(input.reason)
        << ',' << result.coarse.success << ',' << result.coarse.converged << ',' << result.coarse.fitness << ',' << result.coarse.overlap
        << ',' << result.coarse.correspondence_count << ',' << clean(result.coarse.rejection_reason)
        << ',' << result.fine.success << ',' << result.fine.converged << ',' << result.fine.fitness << ',' << result.fine.overlap
        << ',' << result.fine.correspondence_count << ',' << clean(result.fine.rejection_reason)
        << ',' << result.success << ',' << clean(result.validation.reason) << ',' << guard.accepted << ',' << clean(guard.reason)
        << ',' << (result.success && guard.accepted) << ',' << wall << ',' << result.coarse.elapsed_ms << ',' << result.fine.elapsed_ms
        << ',' << result.coarse_evidence_ms + result.fine_evidence_ms << ',' << stats.hits - stats_before.hits << ',' << stats.misses - stats_before.misses
        << ',' << stats.hits << ',' << stats.misses << ',' << stats.entries << '\n';
      for (bool coarse_stage : {true, false}) {
        const auto & registration = coarse_stage ? result.coarse : result.fine;
        if (common::isFinitePose(registration.transform)) {
          writeTransform(transforms, input.current, input.candidate, "cache" + std::to_string(capacity), "full",
            coarse_stage ? "coarse_map_correction" : "fine_map_correction", matrix(registration.transform));
        }
      }
      measured[capacity].push_back({result, guard, wall});
      if (measured[0].size() > sequence && measured[16].size() > sequence) {
        const auto & base = measured[0].at(sequence); const auto & cached = measured[16].at(sequence);
        const auto & cached_registration = cached.registration;
        bool equal = base.registration.success == cached_registration.success &&
          base.registration.validation.accepted == cached_registration.validation.accepted && base.registration.validation.reason == cached_registration.validation.reason &&
          base.pose_guard.accepted == cached.pose_guard.accepted && base.pose_guard.reason == cached.pose_guard.reason;
        std::array<double, 4> pose_delta{};
        int stage_index = 0;
        for (bool coarse_stage : {true, false}) {
          const auto & a = coarse_stage ? base.registration.coarse : base.registration.fine;
          const auto & b = coarse_stage ? cached_registration.coarse : cached_registration.fine;
          equal = equal && a.success == b.success && a.converged == b.converged && a.rejection_reason == b.rejection_reason;
          const bool finite_a = common::isFinitePose(a.transform), finite_b = common::isFinitePose(b.transform);
          equal = equal && finite_a == finite_b;
          if (finite_a && finite_b) {
            pose_delta[stage_index] = (a.transform.translation - b.transform.translation).norm();
            pose_delta[stage_index + 1] = rotationError(matrix(a.transform), matrix(b.transform));
          } else if (finite_a != finite_b) {pose_delta[stage_index] = pose_delta[stage_index + 1] = std::numeric_limits<double>::infinity();}
          stage_index += 2;
        }
        const auto & a = base.registration;
        const bool coarse_count = a.coarse.correspondence_count == cached_registration.coarse.correspondence_count;
        const bool fine_count = a.fine.correspondence_count == cached_registration.fine.correspondence_count;
        const double coarse_fit = difference(a.coarse.fitness, cached_registration.coarse.fitness), fine_fit = difference(a.fine.fitness, cached_registration.fine.fitness),
          coarse_overlap = difference(a.coarse.overlap, cached_registration.coarse.overlap), fine_overlap = difference(a.fine.overlap, cached_registration.fine.overlap);
        const bool within = equal && *std::max_element(pose_delta.begin(), pose_delta.end()) <= 1e-6 && coarse_fit <= 1e-8 &&
          fine_fit <= 1e-8 && coarse_overlap <= 1e-8 && fine_overlap <= 1e-8 && coarse_count && fine_count && stats.entries <= capacity;
        mismatches += !within;
        comparison << sequence << ',' << input.current << ',' << input.candidate << ',' << equal;
        for (const auto delta : pose_delta) {comparison << ',' << delta;}
        comparison << ',' << coarse_fit << ',' << coarse_overlap << ',' << coarse_count << ',' << fine_fit << ',' << fine_overlap << ','
          << fine_count << ',' << within << ',' << base.wall_ms << ',' << cached.wall_ms << ',' << a.coarse.elapsed_ms << ',' << cached_registration.coarse.elapsed_ms << '\n';
        comparison.flush();
      }
      results.flush(); transforms.flush();
      std::cout << "cache=" << capacity << " seq=" << sequence << ' ' << input.current << "->" << input.candidate
        << " valid=" << (result.success && guard.accepted) << " reason=" << result.validation.reason
        << " ms=" << wall << " hits/misses=" << stats.hits << '/' << stats.misses << '\n' << std::flush;
    }
  }
  metadata << "pairs=" << fixed.size() << "\nequivalence_mismatches=" << mismatches << "\ncompleted=true\n";
  return mismatches ? 2 : 0;
}
int run(int argc, char ** argv)
{
  if (argc < 5 || argc > 7) {
    throw std::runtime_error("usage: loop_submap_probe_v4 BUNDLE ACTUAL_RUN_DIR GROUND_PROXY_JSON NEW_OUTPUT_DIR [NANO_THREADS=1] [controls_holdout|cache_benchmark|prepared_audit]");
  }
  const auto bundle = fs::canonical(argv[1]), actual = fs::canonical(argv[2]);
  const auto ground_path = fs::canonical(argv[3]), output = fs::weakly_canonical(argv[4]);
  const int threads = argc >= 6 ? std::stoi(argv[5]) : 1;
  const bool controls_holdout = argc == 7 && std::string(argv[6]) == "controls_holdout";
  const bool cache_benchmark = argc == 7 && std::string(argv[6]) == "cache_benchmark";
  const bool prepared_only = argc == 7 && std::string(argv[6]) == "prepared_audit";
  if (argc == 7 && !controls_holdout && !cache_benchmark && !prepared_only) {throw std::runtime_error("unknown experiment mode");}
  if (threads < 1 || threads > 4) {throw std::runtime_error("NANO_THREADS must be 1..4");}
  if (fs::exists(output)) {throw std::runtime_error("output must be new; refusing overwrite");}
  const auto within = [](const fs::path & child, const fs::path & parent) {
      const auto relative = child.lexically_relative(parent);
      return relative.empty() || *relative.begin() != "..";
    };
  if (within(output, bundle) || within(output, actual) || within(bundle, output) || within(actual, output)) {
    throw std::runtime_error("output overlaps read-only input");
  }
  std::vector<Frame> frames;
  for (const auto & row : csv(bundle / "keyframes/index.csv")) {
    const Id id = std::stoull(row.at("id"));
    if (id != frames.size()) {throw std::runtime_error("noncontiguous Bundle IDs");}
    frames.push_back({id, std::stoll(row.at("stamp_ns")), row.at("cloud"), readPose(row)});
  }
  std::vector<Measurement> loops;
  for (const auto & row : csv(actual / "loop_constraints.csv")) {
    Measurement loop{std::stoull(row.at("current_id")), std::stoull(row.at("candidate_id")), readPose(row, "relative_")};
    if (loop.history >= loop.current || loop.current >= frames.size() ||
      (!loops.empty() && loop.current <= loops.back().current)) {throw std::runtime_error("noncausal fixed factors");}
    loops.push_back(loop);
  }
  const std::vector<std::pair<Id, Id>> pairs{{614, 29}, {638, 15}, {1292, 61}, {1321, 582}};
  std::set<Id> selected, needed;
  for (const auto & pair : pairs) {
    selected.insert(pair.first);
    for (const auto id : window(pair.first, 5, pair.first)) {needed.insert(id);}
    for (const auto id : window(pair.second, 5, pair.first)) {needed.insert(id);}
    if (std::none_of(loops.begin(), loops.end(), [&](const auto & loop) {return loop.current == pair.first && loop.history == pair.second;})) {
      throw std::runtime_error("requested fixed pair is absent from actual run");
    }
  }
  std::map<Id, std::vector<BenchmarkCase>> cache_cases;
  if (cache_benchmark || prepared_only) {
    for (const auto & row : csv(actual / "events.csv")) {
      const Id current = std::stoull(row.at("current"));
      if (selected.count(current) && std::stod(row.at("registration_total_ms")) > 0.0 &&
        std::stoull(row.at("source_points")) > 0 && std::stoull(row.at("target_points")) > 0)
      {
        const Id candidate = std::stoull(row.at("candidate"));
        if (candidate >= current) {throw std::runtime_error("future benchmark candidate");}
        cache_cases[current].push_back({candidate, row.at("reason")});
      }
    }
    for (const auto current : selected) {
      auto & pool = cache_cases.at(current); pool = selectBenchmarkCases(pool);
      if (pool.empty() || pool.size() > 5) {throw std::runtime_error("invalid actual benchmark pool");}
      for (const auto & item : pool) {for (Id id : window(item.candidate, 5, current)) {needed.insert(id);}}
    }
  }
  const auto snapshots = preQuerySnapshots(frames, loops, selected, graphConfig(bundle / "config/mapping_effective.yaml"));
  std::map<Id, common::CloudPtr> clouds;
  for (const auto id : needed) {
    auto cloud = common::CloudPtr{new common::Cloud{}};
    if (pcl::io::loadPCDFile((bundle / frames.at(id).cloud).string(), *cloud) != 0 || cloud->empty()) {
      throw std::runtime_error("cannot load full original body cloud");
    }
    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {throw std::runtime_error("nonfinite original cloud");}
    } clouds[id] = cloud;
  }
  std::map<Id, Ground> grounds;
  for (const auto & node : YAML::LoadFile(ground_path.string())) {
    Ground g; g.reliable = node["reliable"].as<bool>(); const auto n = node["normal"];
    g.normal = Eigen::Vector3d{n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
    const double length = g.normal.norm(); g.normal /= length; g.d = node["d"].as<double>() / length;
    grounds[node["id"].as<Id>()] = g;
  }
  const auto yaml = YAML::LoadFile((bundle / "config/mapping_effective.yaml").string());
  common::LocalMapConfig local{yaml["local_map.voxel_leaf_m"].as<double>(), yaml["local_map.max_points"].as<std::size_t>()};
  const auto config = algorithmConfig(bundle / "config/registration.yaml", threads);
  if (cache_benchmark || prepared_only) {return cacheBenchmark(output, bundle, actual, frames, snapshots, cache_cases, clouds, grounds, local, config, prepared_only);}
  const auto suite = common::createDefaultAlgorithmSuite(config);
  fs::create_directories(output);
  if (!fs::create_directory(output / ".probe_claim")) {throw std::runtime_error("output reserved");}
  fs::create_directory(output / "snapshots");
  fs::copy_file(bundle / "config/registration.yaml", output / "registration_snapshot.yaml");
  fs::copy_file(bundle / "config/mapping_effective.yaml", output / "mapping_snapshot.yaml");
  fs::copy_file(actual / "loop_constraints.csv", output / "fixed_loop_constraints.csv");
  std::ofstream counts(output / "submap_counts.csv"), members(output / "submap_members.csv"),
    runs(output / "runs.csv"), transforms(output / "transforms.csv"), metrics(output / "full_point_geometry.csv"),
    pair_inputs(output / "fixed_pair_inputs.csv"), metadata(output / "metadata.txt");
  for (auto * stream : {&counts, &members, &runs, &transforms, &metrics, &pair_inputs, &metadata}) {
    stream->exceptions(std::ios::failbit | std::ios::badbit); *stream << std::setprecision(17);
  }
  counts << "current,history,variant,side,stage,points,x_min,x_max,y_min,y_max,z_min,z_max,anchor_body_relative_z_gt2_fraction,local_ground_proxy_reliable,anchor_local_ground_proxy_gt3_fraction\n";
  members << "current,history,variant,side,anchor,id,stamp_ns,original_points,optimized_vs_raw_anchor_relative_translation_m,optimized_vs_raw_anchor_relative_rotation_rad\n";
  pair_inputs << "current,history,variant,source_ids,target_ids,shared_original_scans,anchor_stamp_gap_s,nearest_cross_window_stamp_gap_s,label,independent_label_evidence\n";
  runs << "current,history,variant,mode,source_points,target_points,build_wall_ms,registration_wall_ms,quatro_ms,nano_ms,evidence_ms,geometry_valid,pose_guard_valid,operator_and_pose_valid,fitness,overlap,correspondences,correction_translation_m,correction_rotation_rad,reason\n";
  transforms << "current,history,variant,mode,name";
  for (int r = 0; r < 4; ++r) {for (int c = 0; c < 4; ++c) {transforms << ",m" << r << c;}} transforms << '\n';
  metrics << "current,history,variant,mode,direction,layer,denominator,points,nn_coverage_0_5m,nn_p50_m,nn_p95_m,fixed_coverage_0_5m,fixed_p95_m,fixed_target_normal_points,fixed_point_to_plane_p95_m,fixed_two_normal_points,fixed_normal_angle_p95_deg,nn_matches,unique_target_points,many_to_one_fraction,max_target_multiplicity,matched_2m_body_xy_cells,max_cell_match_fraction,effective_matched_cells,matched_azimuth_bins_of_12\n";
  metadata << "version=fixed_submap_probe_v4_20261021\nbundle=" << bundle << "\nactual_run=" << actual
    << "\nground_proxy=" << ground_path << "\nthreads=" << threads << "\ncontrols_holdout=" << controls_holdout
    << "\nsnapshot=all actual factors strictly earlier than query; current is raw-odom increment prediction\n"
    << "future_oracle=false\nactual_candidate_labels=unknown; no independent physical place labels\n"
    << "evaluation=full original single-anchor clouds frozen for every input variant; no deleted height layer\n"
    << "fixed_matches_and_common_support=prior-selected before registration; biased proxy not physical truth/visibility\n"
    << "normals=PCA k12 radius0.6m lambda1>1e-5 lambda0/lambda1<0.3 fixed before solver\n"
    << "height=anchor body-relative and local-ground plane proxy only; not physical gravity\n"
    << "factor=inv(inv(At)*solver*As); direction current inverse history\n"
    << "online_acceptance=false; no competitors or cross-frame confirmation evaluated by this fixed-pair probe\n"
    << "shared_original_scans=shared Bundle body-cloud IDs; laser-packet lineage unavailable\n"
    << "holdout=entire current and candidate scans excluded from solver for optimized_map_leave_centers_out only; neighboring scans remain correlated\n"
    << "build_wall_ms=diagnostic instrumentation including multiple builder calls, not online build timing\n"
    << "voxel_and_cap=" << local.voxel_leaf_m << "m," << local.max_points << " points; unchanged production builder\n";
  for (const auto & pair : pairs) {
    const Id current = pair.first, history = pair.second; const auto & snapshot = snapshots.at(current);
    savePoses(output / "snapshots" / ("before_" + std::to_string(current) + ".csv"), snapshot);
    const Matrix pc = matrix(snapshot.at(current)), ph = matrix(snapshot.at(history)), prior = ph.inverse() * pc;
    const auto fixed = fixedEvaluation(clouds.at(current), clouds.at(history), prior, grounds.at(current), grounds.at(history));
    writeTransform(transforms, current, history, "fixed_prior", "not_solver", "body_target_from_source", prior);
    evaluate(metrics, current, history, "fixed_prior", "not_solver", fixed, prior);
    const std::vector<std::string> variants = controls_holdout ?
      std::vector<std::string>{"optimized_anchor_window5", "optimized_map_leave_centers_out"} :
      std::vector<std::string>{"optimized_map_window5", "raw_anchor_window5", "raw_anchor_window2", "single_frame"};
    for (const std::string & variant : variants) {
      const std::size_t radius = variant == "single_frame" ? 0 : variant == "raw_anchor_window2" ? 2 : 5;
      auto source_ids = window(current, radius, current), target_ids = window(history, radius, current);
      const bool leaveout = variant == "optimized_map_leave_centers_out";
      if (leaveout) {source_ids = withoutCenter(source_ids, current); target_ids = withoutCenter(target_ids, history);}
      const auto started = Clock::now();
      const auto source = build(current, history, current, variant, source_ids, frames, snapshot, clouds, local,
        grounds.at(current), "source", counts, members);
      const auto target = build(current, history, history, variant, target_ids, frames, snapshot, clouds, local,
        grounds.at(history), "target", counts, members);
      const double build_ms = milliseconds(started); std::size_t shared = 0; double min_stamp_gap = std::numeric_limits<double>::infinity();
      for (const Id s : source.ids) {for (const Id t : target.ids) {
          shared += s == t; min_stamp_gap = std::min(min_stamp_gap, std::abs(static_cast<double>(frames[s].stamp - frames[t].stamp)) / 1e9);
        }}
      pair_inputs << current << ',' << history << ',' << variant << ',' << idList(source.ids) << ',' << idList(target.ids)
        << ',' << shared << ',' << static_cast<double>(frames[current].stamp - frames[history].stamp) / 1e9 << ',' << min_stamp_gap
        << ",unknown,none\n";
      const std::vector<std::string> modes = leaveout ? std::vector<std::string>{"full"} :
        std::vector<std::string>{"full", "fine_only"};
      for (const std::string & mode : modes) {
        const Matrix initial = target.anchor_to_input * prior * source.anchor_to_input.inverse();
        const auto registration_started = Clock::now();
        const auto result = mode == "full" ? suite.registration->run(source.cloud, target.cloud, std::nullopt) :
          suite.registration->runFineOnly(source.cloud, target.cloud, pose(initial));
        const double wall = milliseconds(registration_started);
        const bool usable = common::isUsableRegistration(result.fine);
        Matrix body = Matrix::Identity(), corrected = pc; bool pose_valid = false;
        std::string reason = result.validation.reason;
        if (usable) {
          body = restored(matrix(result.fine.transform), source.anchor_to_input, target.anchor_to_input);
          corrected = ph * body; auto evidence = result.fine; evidence.transform = pose(corrected);
          common::ValidationContext context; context.predicted_transform = pose(pc);
          const auto validation = suite.registration->validateResult(evidence, context);
          pose_valid = validation.accepted; if (!pose_valid) {reason = validation.reason;}
          writeTransform(transforms, current, history, variant, mode, "body_target_from_source", body);
          writeTransform(transforms, current, history, variant, mode, "graph_factor_current_inverse_history",
            factor(matrix(result.fine.transform), source.anchor_to_input, target.anchor_to_input));
          writeTransform(transforms, current, history, variant, mode, "corrected_current_map", corrected);
          evaluate(metrics, current, history, variant, mode, fixed, body);
        }
        for (char & ch : reason) {if (ch == ',' || ch == '\n' || ch == '\r') {ch = ';';}}
        runs << current << ',' << history << ',' << variant << ',' << mode << ',' << source.cloud->size() << ',' << target.cloud->size()
          << ',' << build_ms << ',' << wall << ',' << result.coarse.elapsed_ms << ',' << result.fine.elapsed_ms << ','
          << result.coarse_evidence_ms + result.fine_evidence_ms << ',' << result.success << ',' << pose_valid << ','
          << (result.success && pose_valid) << ',' << result.fine.fitness << ',' << result.fine.overlap << ',' << result.fine.correspondence_count
          << ',' << (corrected.block<3, 1>(0, 3) - pc.block<3, 1>(0, 3)).norm() << ',' << rotationError(pc, corrected) << ',' << reason << '\n';
        runs.flush(); transforms.flush(); metrics.flush(); counts.flush(); members.flush(); pair_inputs.flush();
        std::cout << current << "->" << history << ' ' << variant << ' ' << mode << " points=" << source.cloud->size()
          << '/' << target.cloud->size() << " valid=" << (result.success && pose_valid) << " ms=" << wall << '\n' << std::flush;
      }
    }
  }
  metadata << "completed=true\n"; return 0;
}
}  // namespace submap_v4

int main(int argc, char ** argv)
{
  try {return submap_v4::run(argc, argv);}
  catch (const std::exception & error) {std::cerr << "loop_submap_probe_v4: " << error.what() << '\n'; return 1;}
}
#endif
