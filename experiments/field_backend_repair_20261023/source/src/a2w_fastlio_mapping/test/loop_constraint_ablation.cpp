// Offline, post-hoc attribution of FIXED measured loop factors. Never re-registers clouds.
// Removing an earlier factor does not regenerate subsequent v2 measurements: these
// counterfactuals are not online loop-selection experiments or ground-truth labels.
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ablation
{
namespace fs = std::filesystem;
namespace mapping = a2w_fastlio_mapping;
using a2w_fastlio_common::Pose3d;
using Row = std::map<std::string, std::string>;

std::vector<std::string> cells(const std::string & line)
{
  std::vector<std::string> result;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {result.push_back(value);}
  return result;
}

std::vector<Row> readCsv(const fs::path & path)
{
  std::ifstream input(path);
  if (!input) {throw std::runtime_error("cannot read " + path.string());}
  std::string line;
  if (!std::getline(input, line)) {throw std::runtime_error("missing CSV header");}
  const auto headers = cells(line);
  std::vector<Row> result;
  while (std::getline(input, line)) {
    if (line.empty()) {continue;}
    const auto values = cells(line);
    if (values.size() != headers.size()) {throw std::runtime_error("CSV column mismatch");}
    Row row;
    for (std::size_t i = 0; i < headers.size(); ++i) {row.emplace(headers[i], values[i]);}
    result.push_back(std::move(row));
  }
  return result;
}

std::uint64_t id(const std::string & value)
{
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::runtime_error("IDs must be unsigned decimal integers");
  }
  return std::stoull(value);
}

Pose3d readPose(const Row & row, const std::string & prefix = "")
{
  const auto number = [&](const std::string & key) {
      const auto & value = row.at(prefix + key);
      std::size_t end;
      const double number = std::stod(value, &end);
      if (end != value.size() || !std::isfinite(number)) {throw std::runtime_error("invalid pose number");}
      return number;
    };
  Pose3d pose;
  pose.translation = Eigen::Vector3d{number("tx"), number("ty"), number("tz")};
  pose.rotation = Eigen::Quaterniond{number("qw"), number("qx"), number("qy"), number("qz")};
  if (pose.rotation.norm() < 1e-12) {throw std::runtime_error("invalid quaternion");}
  pose.rotation.normalize();
  return pose;
}

mapping::PoseGraphConfig readConfig(const fs::path & path)
{
  const auto yaml = YAML::LoadFile(path.string());
  mapping::PoseGraphConfig config;
#define READ_CONFIG(field) config.field = yaml["pose_graph." #field].as<decltype(config.field)>()
  READ_CONFIG(prior_rotation_sigma_rad);
  READ_CONFIG(prior_translation_sigma_m);
  READ_CONFIG(odometry_rotation_sigma_rad);
  READ_CONFIG(odometry_translation_sigma_m);
  READ_CONFIG(loop_rotation_sigma_rad);
  READ_CONFIG(loop_translation_sigma_m);
  READ_CONFIG(robust_kernel_scale);
  READ_CONFIG(relinearization_threshold);
  READ_CONFIG(relinearization_skip);
#undef READ_CONFIG
  const auto kernel = yaml["pose_graph.robust_kernel"].as<std::string>();
  if (kernel == "huber") {config.robust_kernel = mapping::RobustKernel::HUBER;}
  else if (kernel == "cauchy") {config.robust_kernel = mapping::RobustKernel::CAUCHY;}
  else {throw std::runtime_error("unsupported original robust kernel");}
  return config;
}

void save(const fs::path & path, const mapping::OptimizedPoseSnapshot & snapshot)
{
  if (fs::exists(path)) {throw std::runtime_error("refusing to overwrite " + path.string());}
  std::ofstream output(path);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output << std::setprecision(17) << "id,tx,ty,tz,qx,qy,qz,qw\n";
  for (const auto & entry : snapshot.poses) {
    const auto & p = entry.pose;
    output << entry.id << ',' << p.translation.x() << ',' << p.translation.y() << ',' << p.translation.z()
           << ',' << p.rotation.x() << ',' << p.rotation.y() << ',' << p.rotation.z() << ',' << p.rotation.w() << '\n';
  }
}

struct Measurement
{
  std::uint64_t current, history;
  Pose3d pose;
};

struct Variant
{
  std::string name;
  std::set<std::size_t> excluded;
};

void require(const mapping::GraphUpdate & state)
{
  if (!state.accepted) {throw std::runtime_error(state.reason);}
}

void replay(const std::vector<Pose3d> & raw, const std::vector<Measurement> & loops,
  const mapping::PoseGraphConfig & config, const Variant & variant, const fs::path & output)
{
  mapping::PoseGraphOptimizer graph{config};
  std::size_t retained = 0;
  for (std::uint64_t current = 0; current < raw.size(); ++current) {
    auto before = graph.optimizedPoses();
    require(graph.addKeyFrame(current, raw[current]));
    for (std::size_t k = 0; k < loops.size(); ++k) {
      if (loops[k].current != current || variant.excluded.count(k)) {continue;}
      if (variant.name == "all") {
        // This is exactly the online predictor, not an extra iSAM2 update.
        // Older poses are from the preceding committed snapshot; the new pose
        // is predicted using the original same-stamp odometry increment.
        const auto predicted = current == 0 ? mapping::toGtsamPose(raw[current]) :
          mapping::toGtsamPose(before.poses.back().pose).compose(
          mapping::toGtsamPose(raw[current - 1]).between(mapping::toGtsamPose(raw[current])));
        before.poses.push_back({current, mapping::fromGtsamPose(predicted)});
        save(output / "snapshots" / ("before_" + std::to_string(current) + ".csv"), before);
      }
      require(graph.addLoopConstraint({current, loops[k].history, loops[k].pose, true}));
      ++retained;
    }
    require(graph.update());
    if (variant.excluded.size() == 1 && variant.name.rfind("drop_", 0) == 0 &&
      variant.name.rfind("drop_group_", 0) != 0 && loops[*variant.excluded.begin()].current == current)
    {
      // Matched-node, matched-history control at the factor's actual source time.
      // No later factor has been injected here.
      save(output / "snapshots" / ("without_" + std::to_string(*variant.excluded.begin()) +
        "_at_" + std::to_string(current) + ".csv"), graph.optimizedPoses());
    }
    if (variant.name == "all") {
      for (const auto & loop : loops) {
        if (loop.current == current) {
          save(output / "snapshots" / ("after_" + std::to_string(current) + ".csv"), graph.optimizedPoses());
        }
      }
    }
  }
  require(graph.update(true));  // Same final save flush as v2.
  save(output / (variant.name + ".csv"), graph.optimizedPoses());
  std::ofstream residuals(output / (variant.name + "_residuals.csv"));
  residuals << std::setprecision(17) << "type,i,j,rotation_log_rad,translation_log_m,diagonal_whitened_norm\n";
  const auto poses = graph.optimizedPoses();
  const auto residual = [&](const char * type, std::uint64_t i, std::uint64_t j,
      const Pose3d & measurement, double rotation_sigma, double translation_sigma) {
      const auto actual = mapping::toGtsamPose(poses.poses.at(i).pose).between(
        mapping::toGtsamPose(poses.poses.at(j).pose));
      const auto error = gtsam::Pose3::Logmap(mapping::toGtsamPose(measurement).between(actual));
      const double rotation = error.head<3>().norm(), translation = error.tail<3>().norm();
      residuals << type << ',' << i << ',' << j << ',' << rotation << ',' << translation << ','
                << std::hypot(rotation / rotation_sigma, translation / translation_sigma) << '\n';
    };
  for (std::uint64_t i = 1; i < raw.size(); ++i) {
    residual("odometry", i - 1, i, mapping::fromGtsamPose(mapping::toGtsamPose(raw[i - 1]).between(
      mapping::toGtsamPose(raw[i]))), config.odometry_rotation_sigma_rad, config.odometry_translation_sigma_m);
  }
  for (std::size_t k = 0; k < loops.size(); ++k) {
    if (!variant.excluded.count(k)) {
      residual("loop", loops[k].current, loops[k].history, loops[k].pose,
        config.loop_rotation_sigma_rad, config.loop_translation_sigma_m);
    }
  }
  std::cout << variant.name << " retained_loops=" << retained << " factors="
            << raw.size() + retained << '\n' << std::flush;
}

int run(int argc, char ** argv)
{
  if (argc != 4) {throw std::runtime_error("usage: loop_constraint_ablation BUNDLE ACTUAL_LOOP_CSV NEW_OUTPUT_DIR");}
  const auto bundle = fs::canonical(argv[1]), loopfile = fs::canonical(argv[2]);
  const auto output = fs::weakly_canonical(argv[3]);
  const auto within = [](const fs::path & path, const fs::path & root) {
      auto child = path.begin();
      for (auto parent = root.begin(); parent != root.end(); ++parent, ++child) {
        if (child == path.end() || *parent != *child) {return false;}
      }
      return true;
    };
  if (within(output, bundle) || within(bundle, output) || within(loopfile, output)) {
    throw std::runtime_error("output must not overlap read-only inputs");
  }
  if (fs::exists(output)) {throw std::runtime_error("choose a new output directory");}
  std::vector<Pose3d> raw;
  for (const auto & row : readCsv(bundle / "keyframes/index.csv")) {
    if (id(row.at("id")) != raw.size()) {throw std::runtime_error("noncontiguous original odometry");}
    raw.push_back(readPose(row));
  }
  std::vector<Measurement> loops;
  for (const auto & row : readCsv(loopfile)) {
    Measurement loop{id(row.at("current_id")), id(row.at("candidate_id")), readPose(row, "relative_")};
    if (loop.current >= raw.size() || loop.history >= loop.current ||
      (!loops.empty() && loop.current <= loops.back().current)) {
      throw std::runtime_error("loop factors must be strictly time ordered, past-directed and within Bundle");
    }
    loops.push_back(loop);
  }
  const auto config = readConfig(bundle / "config/mapping_effective.yaml");
  fs::create_directories(output / "snapshots");
  std::ofstream metadata(output / "metadata.txt");
  metadata << "scope=POSTHOC_FIXED_V2_MEASUREMENT_COUNTERFACTUAL_NOT_ONLINE_SELECTION\n"
           << "original_odometry=true\noriginal_noise_and_robust_kernel=true\n"
           << "measurement_injection=only_at_original_current_id\nfuture_factors_before_source_stamp=false\n"
           << "later_measurements_not_regenerated_after_ablation=true\n"
           << "before_snapshot=previous_committed_poses_plus_same_stamp_odom_prediction\n"
           << "snapshot_delta_includes_current_odometry_update=true\n"
           << "independent_ground_truth=false\nframes=" << raw.size() << "\nloops=" << loops.size() << '\n';
  std::vector<Variant> variants{{"all", {}}, {"odometry_only", {}}};
  for (std::size_t k = 0; k < loops.size(); ++k) {
    variants[1].excluded.insert(k);
    variants.push_back({"drop_" + std::to_string(k), {k}});
  }
  const std::map<std::string, std::pair<std::uint64_t, std::uint64_t>> windows{
    {"early_corridor", {0, 1000}}, {"origin_return", {1187, 1297}},
    {"connector_late", {1315, 1359}}, {"final_origin", {1384, 1489}},
    {"focus_ground_return", {1187, 1209}}, {"focus_wall_return", {1264, 1337}}};
  for (const auto & entry : windows) {
    Variant variant{"drop_group_" + entry.first, {}};
    for (std::size_t k = 0; k < loops.size(); ++k) {
      if (loops[k].current >= entry.second.first && loops[k].current <= entry.second.second) {variant.excluded.insert(k);}
    }
    if (!variant.excluded.empty()) {variants.push_back(std::move(variant));}
  }
  Variant repeated{"drop_group_repeated_target_660", {}};
  for (std::size_t k = 0; k < loops.size(); ++k) {if (loops[k].history == 660) {repeated.excluded.insert(k);}}
  if (!repeated.excluded.empty()) {variants.push_back(std::move(repeated));}
  std::ofstream manifest(output / "variants.csv");
  manifest << "variant,excluded_loop_indices,retained_loops\n";
  for (const auto & variant : variants) {
    manifest << variant.name << ',';
    bool first = true;
    for (const auto index : variant.excluded) {manifest << (first ? "" : "|") << index; first = false;}
    manifest << ',' << loops.size() - variant.excluded.size() << '\n';
    replay(raw, loops, config, variant, output);
  }
  metadata << "status=complete\n";
  return 0;
}
}  // namespace ablation

#ifndef LOOP_CONSTRAINT_ABLATION_NO_MAIN
int main(int argc, char ** argv)
try {return ablation::run(argc, argv);}
catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
#endif
