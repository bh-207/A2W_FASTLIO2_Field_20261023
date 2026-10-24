// Standalone, consumption-complete reconstruction from fresh frontend POSTs.
// No ROS init, old keyframes, optimized poses, factors, or source-Bag re-read.
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_map/map_bundle_writer.hpp"
#include "a2w_fastlio_map/map_io.hpp"
#include "a2w_fastlio_mapping/committed_loop_audit.hpp"
#include "a2w_fastlio_mapping/keyframe_manager.hpp"
#include "a2w_fastlio_mapping/loop_pipeline.hpp"
#include "a2w_fastlio_mapping/optimized_map_builder.hpp"

#ifndef RENAME_NOREPLACE
#define RENAME_NOREPLACE (1 << 0)
#endif

namespace field_consumption_backend
{
namespace fs = std::filesystem;
namespace common = a2w_fastlio_common;
namespace mapping = a2w_fastlio_mapping;
namespace map = a2w_fastlio_map;
using Row = std::map<std::string, std::string>;

std::string readText(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw std::runtime_error("cannot read " + path.string());}
  std::ostringstream text; text << input.rdbuf();
  if (input.bad()) {throw std::runtime_error("read failure " + path.string());}
  return text.str();
}

std::string json(const std::string & text)
{
  std::ostringstream out; out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') {out << '\\' << c;}
    else if (c < 32) {out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << unsigned(c);}
    else {out << c;}
  }
  out << '"'; return out.str();
}

std::string csv(const std::string & value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) {return value;}
  std::string out = "\"";
  for (char c : value) {if (c == '"') {out += '"';} out += c;}
  return out + '"';
}

std::vector<std::string> fields(const std::string & line)
{
  std::vector<std::string> values; std::string value;
  bool quoted = false, quote_closed = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {value += '"'; ++i;}
      else if (c == '"') {quoted = false; quote_closed = true;}
      else {value += c;}
    } else if (c == ',') {values.push_back(value); value.clear(); quote_closed = false;}
    else if (c == '"' && value.empty() && !quote_closed) {quoted = true;}
    else if (quote_closed || c == '"') {throw std::runtime_error("invalid CSV quote");}
    else if (c != '\r' || i + 1 != line.size()) {value += c;}
  }
  if (quoted) {throw std::runtime_error("unterminated CSV quote");}
  values.push_back(value); return values;
}

std::vector<Row> readRows(const fs::path & path)
{
  std::ifstream input(path); std::string line;
  if (!input || !std::getline(input, line)) {throw std::runtime_error("empty or unreadable POST index");}
  const auto header = fields(line);
  if (std::set<std::string>(header.begin(), header.end()).size() != header.size()) {
    throw std::runtime_error("duplicate POST column");
  }
  const std::set<std::string> required{"source_scan_id", "source_header_stamp_ns", "scan_begin_ns",
    "scan_end_ns", "cloud", "point_ids", "tx", "ty", "tz", "qx", "qy", "qz", "qw"};
  for (const auto & name : required) {
    if (std::find(header.begin(), header.end(), name) == header.end()) {
      throw std::runtime_error("missing POST column " + name);
    }
  }
  std::vector<Row> rows;
  while (std::getline(input, line)) {
    if (line.empty()) {continue;}
    const auto values = fields(line);
    if (values.size() != header.size()) {throw std::runtime_error("POST CSV column count mismatch");}
    Row row;
    for (std::size_t i = 0; i < header.size(); ++i) {row.emplace(header[i], values[i]);}
    rows.push_back(std::move(row));
  }
  if (input.bad() || rows.empty()) {throw std::runtime_error("empty or failed POST index");}
  return rows;
}

std::uint64_t integer(const std::string & value)
{
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char c) {return c >= '0' && c <= '9';})) {
    throw std::runtime_error("nonnegative decimal integer required: " + value);
  }
  std::size_t consumed = 0; const auto result = std::stoull(value, &consumed);
  if (consumed != value.size()) {throw std::runtime_error("integer trailing data");}
  return result;
}

std::int64_t stamp(const std::string & value)
{
  const auto result = integer(value);
  if (result > std::uint64_t(std::numeric_limits<std::int64_t>::max())) {throw std::runtime_error("stamp overflows int64");}
  return static_cast<std::int64_t>(result);
}

double real(const std::string & value)
{
  std::size_t consumed = 0; const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {throw std::runtime_error("finite scalar required");}
  return result;
}

bool within(const fs::path & child, const fs::path & parent)
{
  auto c = child.begin();
  for (auto p = parent.begin(); p != parent.end(); ++p, ++c) {
    if (c == child.end() || *c != *p) {return false;}
  }
  return true;
}

fs::path inputPath(const fs::path & root, const std::string & relative)
{
  const fs::path value(relative);
  if (value.empty() || value.is_absolute()) {throw std::runtime_error("relative input path required");}
  const auto result = fs::canonical(root / value);
  if (!within(result, root)) {throw std::runtime_error("input escapes reconstruction directory");}
  return result;
}

std::string uuid()
{
  std::array<unsigned char, 16> bytes{}; std::random_device random;
  for (auto & byte : bytes) {byte = static_cast<unsigned char>(random());}
  bytes[6] = (bytes[6] & 0x0fU) | 0x40U; bytes[8] = (bytes[8] & 0x3fU) | 0x80U;
  std::ostringstream out; out << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {out << '-';}
    out << std::setw(2) << unsigned(bytes[i]);
  }
  return out.str();
}

std::string utc()
{
  const auto now = std::time(nullptr); std::tm tm{};
  if (!gmtime_r(&now, &tm)) {throw std::runtime_error("UTC conversion failed");}
  std::ostringstream out; out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ"); return out.str();
}

template<class T> T parameter(const YAML::Node & values, const std::string & key, const T & fallback)
{
  const auto value = values[key]; return value ? value.as<T>() : fallback;
}

struct Configuration
{
  mapping::KeyframeConfig keyframe;
  mapping::PoseGraphConfig graph;
  mapping::OptimizedMapConfig map;
  mapping::LoopPipelineConfig loop;
  common::DefaultAlgorithmSuiteConfig algorithms;
  common::LocalMapConfig local;
  YAML::Node effective;
  std::map<std::string, std::string> snapshots;
  std::string ingress_text;
};

Configuration configuration(const fs::path & reference, const std::optional<fs::path> & ingress)
{
  Configuration c;
  // Read only configuration bytes, never old keyframes/poses/global_map.
  for (const char * name : {"mapping_effective.yaml", "registration.yaml", "scan_context.yaml"}) {
    c.snapshots[name] = readText(reference / "config" / name);
  }
  c.effective = YAML::Load(c.snapshots.at("mapping_effective.yaml"));
  if (!c.effective.IsMap()) {throw std::runtime_error("flat mapping_effective YAML required");}
  for (const char * name : {"registration.yaml", "scan_context.yaml"}) {
    const auto subset = YAML::Load(c.snapshots.at(name));
    for (const auto & item : subset) {
      const auto key = item.first.as<std::string>();
      if (!c.effective[key] || c.effective[key].as<std::string>() != item.second.as<std::string>()) {
        throw std::runtime_error("reference configuration snapshots disagree: " + key);
      }
    }
  }
  const auto & p = c.effective;
#define LOAD(member, key) member = parameter<decltype(member)>(p, key, member)
  LOAD(c.graph.prior_rotation_sigma_rad, "pose_graph.prior_rotation_sigma_rad");
  LOAD(c.graph.prior_translation_sigma_m, "pose_graph.prior_translation_sigma_m");
  LOAD(c.graph.odometry_rotation_sigma_rad, "pose_graph.odometry_rotation_sigma_rad");
  LOAD(c.graph.odometry_translation_sigma_m, "pose_graph.odometry_translation_sigma_m");
  LOAD(c.graph.loop_rotation_sigma_rad, "pose_graph.loop_rotation_sigma_rad");
  LOAD(c.graph.loop_translation_sigma_m, "pose_graph.loop_translation_sigma_m");
  LOAD(c.graph.robust_kernel_scale, "pose_graph.robust_kernel_scale");
  LOAD(c.graph.relinearization_threshold, "pose_graph.relinearization_threshold");
  LOAD(c.graph.relinearization_skip, "pose_graph.relinearization_skip");
  const auto kernel = parameter<std::string>(p, "pose_graph.robust_kernel", "huber");
  if (kernel != "huber" && kernel != "cauchy") {throw std::runtime_error("invalid robust kernel");}
  c.graph.robust_kernel = kernel == "huber" ? mapping::RobustKernel::HUBER : mapping::RobustKernel::CAUCHY;
  LOAD(c.map.full_voxel_leaf_m, "full_map_voxel_leaf_m");
  LOAD(c.map.preview_voxel_leaf_m, "preview_voxel_leaf_m");
  LOAD(c.map.preview_max_points, "preview_max_points");
  LOAD(c.local.voxel_leaf_m, "local_map.voxel_leaf_m");
  LOAD(c.local.max_points, "local_map.max_points");
  LOAD(c.loop.local_map_before, "local_map.neighbor_keyframes_before");
  LOAD(c.loop.local_map_after, "local_map.neighbor_keyframes_after");
  LOAD(c.loop.top_k, "scan_context.top_k");
  LOAD(c.loop.exclude_recent, "scan_context.exclude_recent");
  LOAD(c.loop.maximum_descriptor_distance, "scan_context.max_distance");
  LOAD(c.loop.minimum_keyframes_between_accepted_loops, "loop_pipeline.minimum_keyframes_between_accepted_loops");
  LOAD(c.loop.queue_capacity, "loop_pipeline.queue_capacity");
  LOAD(c.algorithms.place_recognition.rings, "scan_context.rings");
  LOAD(c.algorithms.place_recognition.sectors, "scan_context.sectors");
  LOAD(c.algorithms.place_recognition.max_radius_m, "scan_context.max_radius_m");
  LOAD(c.algorithms.place_recognition.sensor_height_m, "scan_context.sensor_height_m");
  LOAD(c.algorithms.coarse.normal_radius_m, "quatro.fpfh_normal_radius_m");
  LOAD(c.algorithms.coarse.feature_radius_m, "quatro.fpfh_radius_m");
  LOAD(c.algorithms.coarse.noise_bound_m, "quatro.noise_bound_m");
  LOAD(c.algorithms.coarse.rotation_gnc_factor, "quatro.rotation_gnc_factor");
  LOAD(c.algorithms.coarse.rotation_cost_threshold, "quatro.rotation_cost_threshold");
  LOAD(c.algorithms.coarse.rotation_max_iterations, "quatro.rotation_max_iterations");
  LOAD(c.algorithms.coarse.estimate_scale, "quatro.estimate_scale");
  LOAD(c.algorithms.coarse.optimized_matching, "quatro.optimized_matching");
  LOAD(c.algorithms.coarse.descriptor_distance_threshold, "quatro.descriptor_distance_threshold");
  LOAD(c.algorithms.coarse.maximum_correspondences, "quatro.maximum_correspondences");
  LOAD(c.algorithms.coarse.minimum_points, "quatro.minimum_points");
  LOAD(c.algorithms.fine.maximum_correspondence_distance_m, "nano_gicp.maximum_correspondence_distance_m");
  LOAD(c.algorithms.fine.thread_count, "nano_gicp.thread_count");
  LOAD(c.algorithms.fine.correspondence_randomness, "nano_gicp.correspondence_randomness");
  LOAD(c.algorithms.fine.maximum_iterations, "nano_gicp.maximum_iterations");
  LOAD(c.algorithms.fine.transformation_epsilon, "nano_gicp.transformation_epsilon");
  LOAD(c.algorithms.fine.rotation_epsilon, "nano_gicp.rotation_epsilon");
  LOAD(c.algorithms.fine.regularization_method, "nano_gicp.regularization_method");
  LOAD(c.algorithms.fine.fitness_score_max_range_m, "nano_gicp.fitness_score_max_range_m");
  LOAD(c.algorithms.fine.minimum_points, "nano_gicp.minimum_points");
  LOAD(c.algorithms.validation.maximum_fitness, "loop_validation.maximum_fitness");
  LOAD(c.algorithms.validation.minimum_overlap, "loop_validation.minimum_overlap");
  LOAD(c.algorithms.validation.minimum_correspondences, "loop_validation.minimum_correspondences");
  LOAD(c.algorithms.validation.maximum_translation_jump_m, "loop_validation.maximum_translation_jump_m");
  LOAD(c.algorithms.validation.maximum_rotation_jump_rad, "loop_validation.maximum_rotation_jump_rad");
  LOAD(c.algorithms.validation.minimum_candidate_distance_separation, "loop_validation.minimum_candidate_distance_separation");
  LOAD(c.algorithms.evidence_distance_m, "pipeline.evidence_distance_m");
#undef LOAD
  // These experimental branches are outside this frontend-only comparison.
  c.loop.revisit_verification_enabled = false;
  c.loop.revocable_review_enabled = false;
  c.loop.causal_geometry_guard_enabled = false;
  c.effective["loop_pipeline.revisit_verification_enabled"] = false;
  c.effective["loop_pipeline.revocable_review_enabled"] = false;
  c.effective["loop_pipeline.causal_geometry_guard_enabled"] = false;
  // Fine source voxel has no node parameter in the audited backend: retain default zero.
  if (ingress) {
    c.ingress_text = readText(*ingress);
    const auto raw = YAML::Load(c.ingress_text);
    const auto i = raw["/mapping_ingress_node"] ? raw["/mapping_ingress_node"]["ros__parameters"] : raw;
    c.keyframe.translation_threshold_m = parameter<double>(i, "translation_threshold_m", 1.0);
    c.keyframe.rotation_threshold_rad = parameter<double>(i, "rotation_threshold_deg", 10.0) * std::acos(-1.0) / 180.;
    c.keyframe.max_interval_s = parameter<double>(i, "max_interval_s", 2.0);
  }
  return c;
}

const char * reason(mapping::KeyframeDecisionReason value)
{
  using R = mapping::KeyframeDecisionReason;
  switch (value) {
    case R::kAcceptedFirst: return "accepted_first";
    case R::kAcceptedTranslation: return "accepted_translation";
    case R::kAcceptedRotation: return "accepted_rotation";
    case R::kAcceptedMaxInterval: return "accepted_max_interval";
    case R::kRejectedBelowThreshold: return "rejected_below_threshold";
    case R::kRejectedNonMonotonicTimestamp: return "rejected_nonmonotonic_timestamp";
    case R::kRejectedInvalidPose: return "rejected_invalid_pose";
    case R::kRejectedEmptyCloud: return "rejected_empty_cloud";
  }
  throw std::runtime_error("unknown keyframe decision");
}

void writePose(std::ostream & output, const common::Pose3d & pose)
{
  output << '[' << pose.translation.x() << ',' << pose.translation.y() << ',' << pose.translation.z()
         << ',' << pose.rotation.x() << ',' << pose.rotation.y() << ',' << pose.rotation.z() << ',' << pose.rotation.w() << ']';
}

void provenance(std::ostream & output, const mapping::LoopPipelineEvent & event,
  const char * stage, std::int64_t sensor_ns)
{
  output << "{\"stage\":" << json(stage) << ",\"sensor_stamp_ns\":" << sensor_ns
         << ",\"current_id\":" << event.current_id << ",\"candidate_id\":" << event.candidate_id
         << ",\"accepted\":" << (event.accepted ? "true" : "false") << ",\"reason\":" << json(event.reason)
         << ",\"measurement_direction\":\"T_body_current_body_history\",\"source_submap_ids\":[";
  for (std::size_t i=0; i<event.source_submap_ids.size(); ++i) {if(i)output<<',';output<<event.source_submap_ids[i];}
  output << "],\"target_submap_ids\":[";
  for (std::size_t i=0; i<event.target_submap_ids.size(); ++i) {if(i)output<<',';output<<event.target_submap_ids[i];}
  output << "],\"initial_guess\":"; writePose(output, event.registration_initial_guess);
  output << ",\"coarse\":"; writePose(output, event.coarse_transform);
  output << ",\"fine_D_map\":"; writePose(output, event.registration.transform);
  if (event.evaluated_constraint) {output << ",\"evaluated_measurement\":";writePose(output,event.evaluated_constraint->relative_pose);}
  if (event.queued_constraint) {output << ",\"queued_measurement\":";writePose(output,event.queued_constraint->relative_pose);}
  if (event.optimized_pose_change) {output << ",\"optimized_pose_change\":";writePose(output,*event.optimized_pose_change);}
  output << "}\n";
}

class Provider final : public common::KeyFrameProvider
{
public:
  explicit Provider(const std::vector<common::KeyFrame> & frames) : frames_(frames) {}
  std::optional<common::KeyFrame> get(std::uint64_t id) const override
  {return id < frames_.size() ? std::optional<common::KeyFrame>{frames_[id]} : std::nullopt;}
  std::optional<std::pair<std::uint64_t,std::uint64_t>> bounds() const override
  {return frames_.empty() ? std::nullopt : std::optional<std::pair<std::uint64_t,std::uint64_t>>{{0,frames_.size()-1}};}
private:
  const std::vector<common::KeyFrame> & frames_;
};

void checkIds(const fs::path & path, std::size_t count)
{
  const std::uint16_t endian = 1;
  if (*reinterpret_cast<const unsigned char *>(&endian) != 1) {throw std::runtime_error("uint32 point IDs require little-endian host");}
  if (fs::file_size(path) != count * sizeof(std::uint32_t)) {throw std::runtime_error("point identity length != cloud point count");}
  std::vector<std::uint32_t> ids(count); std::ifstream input(path, std::ios::binary);
  input.read(reinterpret_cast<char *>(ids.data()), ids.size()*sizeof(std::uint32_t));
  if (!input) {throw std::runtime_error("point identity read failure");}
  std::sort(ids.begin(), ids.end());
  if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {throw std::runtime_error("duplicate per-scan point identity");}
}

void close(std::ofstream & output)
{
  output.flush(); if (!output) {throw std::runtime_error("output flush failed");}
  output.close(); if (!output) {throw std::runtime_error("output close failed");}
}

struct Options
{
  fs::path index, reference, output, manifest;
  std::optional<fs::path> ingress;
};
struct Result {std::size_t posts, keyframes, loops, factors, map_points;};

Result run(Options options)
{
  options.index=fs::canonical(options.index); options.reference=fs::canonical(options.reference);
  options.manifest=fs::canonical(options.manifest);
  const auto reconstruction=options.index.parent_path();
  const auto requested=fs::absolute(options.output);
  if (fs::is_symlink(fs::symlink_status(requested))) {throw std::runtime_error("output symlink forbidden");}
  options.output=fs::weakly_canonical(requested);
  if (fs::exists(options.output) || within(options.output,reconstruction) || within(reconstruction,options.output) ||
      within(options.output,options.reference) || within(options.reference,options.output) || within(options.manifest,options.output)) {
    throw std::runtime_error("fresh output must not overlap read-only inputs");
  }
  if (options.ingress) {options.ingress=fs::canonical(*options.ingress);}
  auto config=configuration(options.reference,options.ingress);
  const auto input_index_hash=map::sha256File(options.index);
  const auto rows=readRows(options.index);
  const auto source_manifest_hash=map::sha256File(options.manifest);
  const auto frontend_identity="field_consumption_manifest_sha256:"+source_manifest_hash;
  config.effective["frontend_revision"]=frontend_identity;
  YAML::Emitter effective; effective << config.effective;
  config.snapshots["mapping_effective.yaml"]=std::string(effective.c_str())+'\n';
  auto registration=YAML::Load(config.snapshots.at("registration.yaml"));
  for(const char* key:{"loop_pipeline.revisit_verification_enabled","loop_pipeline.revocable_review_enabled","loop_pipeline.causal_geometry_guard_enabled"}) {registration[key]=false;}
  YAML::Emitter registration_text;registration_text<<registration;
  config.snapshots["registration.yaml"]=std::string(registration_text.c_str())+'\n';
  fs::create_directories(options.output.parent_path());
  const auto token=uuid();
  const auto staging=options.output.parent_path()/("."+options.output.filename().string()+".consumption-"+token);
  if (!fs::create_directory(staging)) {throw std::runtime_error("staging path collision");}
  std::cerr<<"staging="<<staging<<'\n';
  fs::create_directory(staging/"reference_config"); fs::create_directory(staging/"keyframe_point_ids");
  fs::copy_file(options.manifest,staging/"source_build_manifest.json",fs::copy_options::none);
  if(map::sha256File(staging/"source_build_manifest.json")!=source_manifest_hash) {
    throw std::runtime_error("source manifest changed while copying");
  }
  for(const char*name:{"mapping_effective.yaml","registration.yaml","scan_context.yaml"}) {
    fs::copy_file(options.reference/"config"/name,staging/"reference_config"/name,fs::copy_options::none);
  }
  if(options.ingress) {fs::copy_file(*options.ingress,staging/"reference_config"/"ingress_effective.yaml",fs::copy_options::none);}
  fs::copy_file(options.index,staging/"frontend_post_index.csv",fs::copy_options::none);
  if(map::sha256File(staging/"frontend_post_index.csv")!=input_index_hash) {
    throw std::runtime_error("POST index changed while reading/copying");
  }
  std::ofstream factors(staging/"exact_factor_ledger.csv"),committed(staging/"loop_committed.log"),
    loop_provenance(staging/"loop_provenance.jsonl"),events(staging/"loop_events.csv"),
    mapping_file(staging/"scan_keyframe_map.csv"),raw_index(staging/"raw_keyframe_index.csv");
  for(auto* out:{&factors,&committed,&loop_provenance,&events,&mapping_file,&raw_index}) {
    *out<<std::setprecision(17);if(!*out)throw std::runtime_error("ledger open failed");
  }
  mapping_file<<"post_row,source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,keyframe_id,accepted,reason,body_points,cloud,point_ids\n";
  raw_index<<"keyframe_id,source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,point_ids,tx,ty,tz,qx,qy,qz,qw\n";
  events<<"keyframe_id,candidate_id,rank,accepted,reason,rejection_stage,factors,fitness,overlap,correspondences,frame_ms\n";
  mapping::writeFactorLedgerCsvHeader(factors);
  std::size_t final_ledger_factors=0,final_ledger_loops=0,committed_count=0,accepted_event_count=0;
  auto graph=std::make_shared<mapping::PoseGraphOptimizer>(config.graph);
  graph->setFactorLedgerObserver([&](const mapping::FactorLedgerRecord& record) {
    mapping::writeFactorLedgerCsv(factors,record);
    if(record.stage=="final_audit") {++final_ledger_factors;if(record.factor_type=="loop")++final_ledger_loops;}
  });
  const auto algorithms=common::createDefaultAlgorithmSuite(config.algorithms);
  mapping::LoopPipeline pipeline(algorithms.place_recognition,algorithms.descriptor_index,
    algorithms.registration,graph,common::LocalMapBuilder{config.local},config.loop);
  mapping::KeyframeManager manager(config.keyframe);
  std::vector<common::KeyFrame> frames;
  std::optional<std::uint64_t> previous_scan;
  std::optional<std::int64_t> previous_header,previous_end;
  std::uint64_t total_input_points=0;
  for(std::size_t row_id=0;row_id<rows.size();++row_id) {
    const auto& row=rows[row_id];const auto scan_id=integer(row.at("source_scan_id"));
    const auto source_header=stamp(row.at("source_header_stamp_ns")),begin=stamp(row.at("scan_begin_ns")),end=stamp(row.at("scan_end_ns"));
    if((previous_scan&&scan_id<=*previous_scan)||(previous_header&&source_header<=*previous_header)||
       (previous_end&&end<=*previous_end)||end<begin) {throw std::runtime_error("nonmonotonic POST identity or scan time");}
    previous_scan=scan_id;previous_header=source_header;previous_end=end;
    const auto cloud_path=inputPath(reconstruction,row.at("cloud")),ids_path=inputPath(reconstruction,row.at("point_ids"));
    common::CloudPtr cloud(new common::Cloud);
    if(pcl::io::loadPCDFile(cloud_path.string(),*cloud)!=0||cloud->empty()) {throw std::runtime_error("invalid/empty POST body cloud");}
    for(const auto&p:*cloud) {if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(p.intensity))throw std::runtime_error("nonfinite POST cloud point");}
    checkIds(ids_path,cloud->size());total_input_points+=cloud->size();
    common::FrontendFrame frame;frame.stamp_ns=end;frame.body_cloud=cloud;
    frame.odom_pose.translation={real(row.at("tx")),real(row.at("ty")),real(row.at("tz"))};
    frame.odom_pose.rotation=Eigen::Quaterniond(real(row.at("qw")),real(row.at("qx")),real(row.at("qy")),real(row.at("qz")));
    if(std::abs(frame.odom_pose.rotation.norm()-1.)>1e-6) {throw std::runtime_error("POST quaternion is not unit length");}
    frame.odom_pose.rotation.normalize(); // Same makeFrontendFrame/KeyframeManager boundary as the real ingress.
    auto decision=manager.consider(frame);
    mapping_file<<row_id<<','<<scan_id<<','<<source_header<<','<<begin<<','<<end<<',';
    if(decision.keyframe) {mapping_file<<decision.keyframe->id;}
    mapping_file<<','<<bool(decision.keyframe)<<','<<reason(decision.reason)<<','<<cloud->size()<<','<<csv(row.at("cloud"))<<','<<csv(row.at("point_ids"))<<'\n';
    if(!decision.keyframe) {
      if(decision.reason!=mapping::KeyframeDecisionReason::kRejectedBelowThreshold) {throw std::runtime_error("invalid frame rejected by KeyframeManager");}
      continue;
    }
    const auto& current=*decision.keyframe;
    if(current.id!=frames.size()) {throw std::runtime_error("noncontiguous fresh keyframe ID");}
    const auto started=std::chrono::steady_clock::now();
    const auto observer=[&](mapping::LoopProgressStage stage,const mapping::LoopPipelineEvent& event) {
      if(stage==mapping::LoopProgressStage::CandidateFinished&&event.evaluated_constraint) {provenance(loop_provenance,event,"candidate_finished",end);}
      if(stage==mapping::LoopProgressStage::OptimizationFinished) {
        provenance(loop_provenance,event,"optimization_finished",end);
        const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        const auto audit=mapping::committedLoopAudit(stage,event,{end,wall,elapsed});
        if(audit) {committed<<*audit<<'\n';++committed_count;std::cout<<*audit<<'\n';}
      }
    };
    const auto processed_events=pipeline.process(current,observer); // Complete this KF before consuming another POST.
    for(const auto& event:processed_events) {
      accepted_event_count+=event.accepted;
      events<<event.current_id<<','<<event.candidate_id<<','<<event.candidate_rank<<','<<event.accepted<<','<<csv(event.reason)<<','<<csv(event.rejection_stage)<<','<<event.graph_factor_count<<','<<event.registration.fitness<<','<<event.registration.overlap<<','<<event.registration.correspondence_count<<','<<event.frame_total_ms<<'\n';
    }
    const auto snapshot=graph->optimizedPoses();
    if(snapshot.poses.size()!=current.id+1||snapshot.poses.back().id!=current.id||pipeline.pending()!=0) {throw std::runtime_error("graph did not commit this keyframe; complete save disabled");}
    for(auto* out:{&factors,&committed,&loop_provenance,&events,&mapping_file}) {if(!*out)throw std::runtime_error("diagnostic ledger write failure");}
    frames.push_back(current);
    const auto identity_file="keyframe_point_ids/"+std::to_string(current.id)+".point_ids_u32";
    fs::copy_file(ids_path,staging/identity_file,fs::copy_options::none);
    raw_index<<current.id<<','<<scan_id<<','<<source_header<<','<<begin<<','<<end<<','<<identity_file;
    for(double value:{current.odom_pose.translation.x(),current.odom_pose.translation.y(),current.odom_pose.translation.z(),current.odom_pose.rotation.x(),current.odom_pose.rotation.y(),current.odom_pose.rotation.z(),current.odom_pose.rotation.w()}) {raw_index<<','<<value;}
    raw_index<<'\n';
    if(current.id%25==0) {std::cout<<"received="<<frames.size()<<" processed="<<frames.size()<<" pending=0 source_scan_id="<<scan_id<<'\n'<<std::flush;}
  }
  if(frames.empty()||frames.size()!=manager.size()||pipeline.pending()!=0) {throw std::runtime_error("keyframes not complete/drained");}
  const auto final_update=graph->update(true);
  if(!final_update.accepted) {throw std::runtime_error("final graph update failed: "+final_update.reason);}
  const auto final=graph->optimizedPoses();graph->emitFactorLedger("final_audit");
  if(final.poses.size()!=frames.size()||accepted_event_count!=committed_count||final_ledger_loops!=committed_count||
    final_ledger_factors!=final_update.factor_count||final_update.factor_count!=frames.size()+committed_count) {
    throw std::runtime_error("exact factor/LoopCommitted/node count invariant failed");
  }
  map::MapBundleData data;
  for(const auto& pose:final.poses) {
    if(pose.id>=frames.size()) {throw std::runtime_error("optimized pose ID out of range");}
    frames[pose.id].optimized_pose=pose.pose;
  }
  data.global_map=mapping::OptimizedMapBuilder{config.map}.buildFull(Provider{frames},final);
  data.keyframes=frames;
  for(const auto& frame:frames) {data.descriptors.push_back({frame.id,algorithms.place_recognition->describe(frame.body_cloud)});}
  data.metadata.bundle_uuid=token;data.metadata.created_utc=utc();
  data.metadata.map_frame=parameter<std::string>(config.effective,"map_frame","map");
  data.metadata.odom_frame=parameter<std::string>(config.effective,"odom_frame","camera_init");
  data.metadata.tracking_frame=parameter<std::string>(config.effective,"tracking_frame","body");
  data.metadata.frontend_revision=frontend_identity;
  data.metadata.global_map_leaf_m=config.map.full_voxel_leaf_m;
  data.metadata.creation_status="offline_complete_consumption_not_realtime_validated";
  data.metadata.hardware_validation_status="not_validated_user_fixed_extrinsic";
  for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"scan_context","scan_context_revision"},{"quatro","quatro_revision"},{"nano_gicp","nano_gicp_revision"},{"teaser++","teaser_revision"},{"pmc","pmc_revision"}}) {
    data.metadata.dependency_revisions[pair.first]=parameter<std::string>(config.effective,pair.second,"unspecified_in_reference");
  }
  data.metadata.dependency_revisions["gtsam"]="4.1.1";
  data.config_snapshots=config.snapshots;
  const auto saved=map::MapBundleWriter{}.write(staging/"map_bundle",data);
  if(!saved.success) {throw std::runtime_error("new MapBundleWriter failed: "+saved.message);}
  // Required read-back verifies manifest, clouds, descriptors, poses, metadata.
  const auto verified=map::MapBundleReader{}.read(staging/"map_bundle");
  if(verified.data.metadata.bundle_uuid!=token||verified.data.keyframes.size()!=frames.size()||
    verified.data.global_map->size()!=data.global_map->size()) {throw std::runtime_error("Bundle read-back count/UUID mismatch");}
  double max_translation_error=0.,max_rotation_error=0.;
  for(std::size_t i=0;i<frames.size();++i) {
    const auto& expected=frames[i];const auto& actual=verified.data.keyframes[i];
    if(actual.id!=expected.id||actual.stamp_ns!=expected.stamp_ns||actual.body_cloud->size()!=expected.body_cloud->size()) {throw std::runtime_error("Bundle raw identity/count round-trip changed");}
    for(const auto& poses:std::array<std::pair<common::Pose3d,common::Pose3d>,2>{{{expected.odom_pose,actual.odom_pose},{expected.optimized_pose,actual.optimized_pose}}}) {
      max_translation_error=std::max(max_translation_error,(poses.first.translation-poses.second.translation).norm());
      max_rotation_error=std::max(max_rotation_error,Eigen::AngleAxisd(poses.first.rotation.inverse()*poses.second.rotation).angle());
    }
    for(std::size_t j=0;j<expected.body_cloud->size();++j) {
      const auto&a=expected.body_cloud->points[j];const auto&b=actual.body_cloud->points[j];
      if(a.x!=b.x||a.y!=b.y||a.z!=b.z||a.intensity!=b.intensity) {throw std::runtime_error("Bundle body point data round-trip changed");}
    }
  }
  if(max_translation_error>1e-12||max_rotation_error>1e-12) {throw std::runtime_error("Bundle pose serialization changed");}
  if(map::sha256File(options.index)!=input_index_hash||map::sha256File(options.manifest)!=source_manifest_hash) {
    throw std::runtime_error("read-only POST index or source manifest changed during reconstruction");
  }
  for(auto* out:{&factors,&committed,&loop_provenance,&events,&mapping_file,&raw_index}) {close(*out);}
  std::ofstream summary(staging/"summary.json");summary<<std::setprecision(17);
  summary<<"{\"schema\":\"field_consumption_backend_v1\",\"status\":\"verified\",\"input_index\":"<<json(options.index.string())
    <<",\"input_index_sha256\":"<<json(input_index_hash)<<",\"reference_bundle_config_only\":"<<json(options.reference.string())
    <<",\"source_manifest\":"<<json(options.manifest.string())<<",\"source_manifest_sha256\":"<<json(source_manifest_hash)
    <<",\"frontend_revision\":"<<json(frontend_identity)<<",\"frontend_revision_is_git_commit\":false,\"post_frames_received\":"<<rows.size()
    <<",\"post_frames_processed\":"<<rows.size()<<",\"received\":"<<frames.size()<<",\"processed\":"<<frames.size()<<",\"pending\":0"
    <<",\"keyframes\":"<<frames.size()<<",\"loop_committed\":"<<committed_count<<",\"factor_count\":"<<final_update.factor_count
    <<",\"final_ledger_factor_count\":"<<final_ledger_factors<<",\"global_map_points\":"<<data.global_map->size()<<",\"input_body_points\":"<<total_input_points
    <<",\"full_map_voxel_leaf_m\":"<<config.map.full_voxel_leaf_m<<",\"keyframe_translation_threshold_m\":"<<config.keyframe.translation_threshold_m
    <<",\"keyframe_rotation_threshold_rad\":"<<config.keyframe.rotation_threshold_rad<<",\"keyframe_max_interval_s\":"<<config.keyframe.max_interval_s
    <<",\"keyframe_config_source\":"<<json(options.ingress?options.ingress->string():"audited_mapping_ingress_node_defaults")
    <<",\"revisit_verification_enabled\":false,\"revocable_review_enabled\":false,\"causal_geometry_guard_enabled\":false"
    <<",\"point_ids_sidecar_count_and_uniqueness_verified\":true,\"point_identity_cross_candidate_geometry_check\":\"separate_ground_eval\""
    <<",\"all_body_point_values_roundtrip_verified\":true,\"max_pose_translation_roundtrip_m\":"<<max_translation_error
    <<",\"max_pose_rotation_roundtrip_rad\":"<<max_rotation_error<<",\"ros_context_created\":false,\"old_pose_or_factor_imported\":false"
    <<",\"hardware_validation\":\"NOT_RUN\",\"original_rate_acceptance\":\"NOT_RUN\",\"calibration_status\":\"user_fixed_R_T_offset_not_physically_verified\""
    <<",\"reference_config_hashes\":{";
  bool first=true;for(const char*name:{"mapping_effective.yaml","registration.yaml","scan_context.yaml"}) {if(!first)summary<<',';first=false;summary<<json(name)<<':'<<json(map::sha256File(options.reference/"config"/name));}
  summary<<"}}\n";close(summary);
  if(::syscall(SYS_renameat2,AT_FDCWD,staging.c_str(),AT_FDCWD,options.output.c_str(),RENAME_NOREPLACE)!=0) {
    throw std::runtime_error("no-overwrite atomic publish failed; staging retained: "+std::string(std::strerror(errno)));
  }
  std::cout<<"status=verified output="<<options.output<<" received="<<frames.size()<<" processed="<<frames.size()<<" pending=0 loops="<<committed_count<<'\n';
  return {rows.size(),frames.size(),committed_count,final_update.factor_count,data.global_map->size()};
}

void selfTest(Options options)
{
  if(integer("9007199254740993")!=9007199254740993ULL||fields("\"a,b\",7,\"x\"\"y\"")!=std::vector<std::string>{"a,b","7","x\"y"}) {throw std::runtime_error("CSV/integer identity self-test failed");}
  bool rejected=false;try{integer("-1");}catch(const std::exception&){rejected=true;}
  if(!rejected)throw std::runtime_error("negative identity self-test failed");
  const auto fixtures=fs::absolute(options.output).parent_path()/(".field-consumption-fixture-"+uuid());
  fs::create_directories(fixtures/"body_clouds");
  const auto duplicate_ids=fixtures/"duplicate_ids_u32";
  {
    std::ofstream output(duplicate_ids,std::ios::binary);const std::array<std::uint32_t,3> values{0,0,1};
    output.write(reinterpret_cast<const char*>(values.data()),sizeof(values));close(output);
  }
  rejected=false;try{checkIds(duplicate_ids,3);}catch(const std::exception&){rejected=true;}
  if(!rejected)throw std::runtime_error("duplicate point identity self-test failed");
  rejected=false;try{checkIds(duplicate_ids,2);}catch(const std::exception&){rejected=true;}
  if(!rejected)throw std::runtime_error("point identity count self-test failed");
  options.index=fixtures/"post_index.csv";
  std::ofstream index(options.index);index<<"source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,cloud,point_ids,tx,ty,tz,qx,qy,qz,qw\n";
  for(std::size_t i=0;i<3;++i) {
    common::Cloud cloud;
    for(const auto&xyz:std::array<std::array<float,3>,3>{{{{1,0,0}},{{0,1,1}},{{-1,0,2}}}}) {common::PointT point;point.x=xyz[0];point.y=xyz[1];point.z=xyz[2];point.intensity=1;cloud.push_back(point);}
    const auto relative="body_clouds/"+std::to_string(i)+".pcd",ids=relative+".point_ids_u32";
    if(pcl::io::savePCDFileBinary((fixtures/relative).string(),cloud)!=0)throw std::runtime_error("fixture PCD write failed");
    std::ofstream identity(fixtures/ids,std::ios::binary);const std::array<std::uint32_t,3> values{2,0,1};identity.write(reinterpret_cast<const char*>(values.data()),sizeof(values));close(identity);
    const auto header=1000000000LL+static_cast<std::int64_t>(i)*100000000LL;
    index<<i<<','<<header<<','<<header<<','<<header+90000000LL<<','<<relative<<','<<ids<<','<<(i==0?0.:(i==1?.01:2.))<<",0,0,0,0,0,1\n";
  }
  close(index);
  const auto result=run(options);
  if(result.posts!=3||result.keyframes!=2||result.loops!=0||result.factors!=2||result.map_points!=6)throw std::runtime_error("real backend/Bundle round-trip self-test invariant failed");
  std::cout<<"SELF_TEST_PASS exact_large_integer=1 duplicate_identity_rejection=1 identity_length_rejection=1 keyframe_thresholds=1 graph=1 bundle_roundtrip=1\n";
}

int main(int argc,char**argv)
{
  bool test=false;int pos=1;
  if(pos<argc&&std::string(argv[pos])=="--self-test") {test=true;++pos;}
  if(argc-pos<(test?2:3)) {throw std::runtime_error("usage: field_consumption_backend [--self-test] POST_INDEX REFERENCE_BUNDLE NEW_OUTPUT_DIR [INGRESS_YAML] --source-manifest PATH (self-test omits POST_INDEX)");}
  Options options;
  if(!test)options.index=argv[pos++];options.reference=argv[pos++];options.output=argv[pos++];
  if(pos<argc&&std::string(argv[pos]).rfind("--",0)!=0)options.ingress=fs::path(argv[pos++]);
  if(pos+2!=argc||std::string(argv[pos])!="--source-manifest")throw std::runtime_error("explicit --source-manifest PATH required");
  options.manifest=argv[pos+1];
  if(test)selfTest(options);else run(options);
  return 0;
}
}  // namespace field_consumption_backend

int main(int argc,char**argv)
{
  try{return field_consumption_backend::main(argc,argv);}
  catch(const std::exception&error){std::cerr<<"field_consumption_backend: "<<error.what()<<'\n';return 1;}
}
