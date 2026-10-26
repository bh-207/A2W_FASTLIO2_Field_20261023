// Manual, read-only Map Bundle replay. No ROS graph, sensor process, or hardware.
// FIRST_MATCH_ID > 0 intentionally suppresses early matching: WINDOW DIAGNOSTIC ONLY.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/scan_context_index.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_mapping/loop_pipeline.hpp"
#include "a2w_fastlio_common/quatro_registration.hpp"
#include "a2w_fastlio_common/nano_gicp_registration.hpp"
#include "replay_arrival_queue.hpp"

namespace loop_recall_replay
{
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using a2w_fastlio_common::KeyFrame;
using a2w_fastlio_mapping::LoopPipelineEvent;

template<typename T> std::string text(const T & value)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

double elapsedMs(const Clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - started).count();
}

std::uint64_t parseId(const std::string & argument)
{
  if (argument.empty() || argument.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("keyframe bounds must be unsigned decimal integers");
  }
  std::size_t consumed = 0U;
  const auto result = std::stoull(argument, &consumed);
  if (consumed != argument.size()) {throw std::invalid_argument("invalid keyframe bound");}
  return result;
}

void validateRange(std::uint64_t first_match, std::uint64_t last, std::size_t count)
{
  if (count == 0U || first_match > last || last >= count) {
    throw std::invalid_argument("require 0 <= FIRST_MATCH_ID <= LAST_ID < bundle keyframe count");
  }
}

bool isWithin(const fs::path & path, const fs::path & root)
{
  auto child = path.begin();
  for (auto ancestor = root.begin(); ancestor != root.end(); ++ancestor, ++child) {
    if (child == path.end() || *child != *ancestor) {return false;}
  }
  return true;
}

void validateOutputPaths(const fs::path & input, const fs::path & output)
{
  const auto source = fs::canonical(input);
  const auto destination = fs::weakly_canonical(output);
  if (isWithin(destination, source) || isWithin(source, destination)) {
    throw std::invalid_argument("output and input Bundle directories must not overlap (including symlinks)");
  }
}

void reserveOutput(const fs::path & output)
{
  fs::create_directories(output);
  for (const auto * name : {"events.csv", "frames.csv", "queries.csv", "summary.csv", "loop_constraints.csv",
      "pace_frames.csv", "commit_latency.csv", "arrival_queue_events.csv",
      "optimized_poses.csv", "online_last_frame_poses.csv", "run_metadata.txt", "snapshots", ".replay_claim"})
  {
    if (fs::exists(output / name) || fs::is_symlink(output / name)) {
      throw std::runtime_error("refusing to overwrite existing replay result: " + (output / name).string());
    }
  }
  // Atomic reservation also prevents two processes that passed the checks together.
  // It is deliberately retained, including after interrupted/failed replays.
  if (!fs::create_directory(output / ".replay_claim")) {
    throw std::runtime_error("output directory was reserved by another replay");
  }
  fs::create_directory(output / "snapshots");
}

std::ofstream openOutput(const fs::path & path)
{
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path, std::ios::out | std::ios::binary);
  stream << std::setprecision(17);
  return stream;
}

void writeCsv(std::ostream & output, const std::vector<std::string> & cells)
{
  for (std::size_t i = 0U; i < cells.size(); ++i) {
    if (i != 0U) {output << ',';}
    const auto & cell = cells[i];
    if (cell.find_first_of(",\r\n\"") == std::string::npos) {output << cell; continue;}
    output << '"';
    for (const char c : cell) {if (c == '"') {output << '"';} output << c;}
    output << '"';
  }
  output << '\n';
}

KeyFrame freshFrame(const KeyFrame & saved)
{
  auto result = saved;
  result.optimized_pose = result.odom_pose;
  // Pipeline recomputes descriptors from local clouds; saved optimized descriptors
  // and global-map coordinates are not injected into the online descriptor index.
  result.scan_context_descriptor.clear();
  return result;
}

struct QueryRecord
{
  std::uint64_t current;
  std::size_t query_number;
  std::size_t requested_k;
  a2w_fastlio_common::CandidateFilter filter;
  bool suppressed;
  std::size_t result_count;
  std::optional<a2w_fastlio_common::LoopCandidate> candidate;
};

std::string sha256File(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw std::runtime_error("cannot checksum input: " + path.string());}
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> digest(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA256");
  }
  std::vector<char> buffer(1024U * 1024U);
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (input.gcount() > 0 && EVP_DigestUpdate(digest.get(), buffer.data(), static_cast<std::size_t>(input.gcount())) != 1) {
      throw std::runtime_error("cannot update SHA256");
    }
  }
  if (!input.eof()) {throw std::runtime_error("checksum read failed: " + path.string());}
  unsigned char bytes[EVP_MAX_MD_SIZE];
  unsigned int count = 0U;
  if (EVP_DigestFinal_ex(digest.get(), bytes, &count) != 1) {throw std::runtime_error("cannot finalize SHA256");}
  std::ostringstream hexadecimal;
  hexadecimal << std::hex << std::setfill('0');
  for (unsigned int i = 0U; i < count; ++i) {hexadecimal << std::setw(2) << static_cast<unsigned int>(bytes[i]);}
  return hexadecimal.str();
}

struct ScoreCache
{
  std::size_t count{0U};
  std::vector<double> distances, yaws;
  fs::path path;
  std::string sha256, descriptor_sha256;

  static std::shared_ptr<ScoreCache> read(const fs::path & filename, std::size_t expected, const fs::path & descriptors)
  {
    auto cache = std::make_shared<ScoreCache>();
    cache->path = fs::canonical(filename);
    std::ifstream input(cache->path, std::ios::binary);
    const auto read64 = [&input]() {
        unsigned char bytes[8];
        if (!input.read(reinterpret_cast<char *>(bytes), 8)) {throw std::runtime_error("truncated SC cache");}
        std::uint64_t bits = 0U;
        for (unsigned int i = 0U; i < 8U; ++i) {bits |= static_cast<std::uint64_t>(bytes[i]) << (8U * i);}
        return bits;
      };
    const auto count = read64();
    if (count != expected || count == 0U || count > 100000U) {
      throw std::runtime_error("SC cache keyframe count does not match input Bundle");
    }
    if (fs::file_size(cache->path) != 8U + 16U * count * count) {
      throw std::runtime_error("SC cache byte length mismatch");
    }
    cache->count = static_cast<std::size_t>(count);
    for (auto * matrix : {&cache->distances, &cache->yaws}) {
      matrix->reserve(cache->count * cache->count);
      for (std::size_t i = 0U; i < cache->count * cache->count; ++i) {
        const auto bits = read64();
        double value = 0.0;
        static_assert(sizeof(value) == sizeof(bits), "SC cache requires float64 doubles");
        std::memcpy(&value, &bits, sizeof(value));
        if (!std::isfinite(value) || (matrix == &cache->distances && value < 0.0) ||
          (matrix == &cache->yaws && std::abs(value) > 3.14159265358979323846 + 1e-12))
        {throw std::runtime_error("SC cache contains invalid distance/yaw");}
        matrix->push_back(value);
      }
    }
    auto sidecar_path = cache->path;
    sidecar_path.replace_extension(".meta.json");
    const auto sidecar = YAML::LoadFile(sidecar_path.string());
    cache->sha256 = sha256File(cache->path);
    cache->descriptor_sha256 = sha256File(descriptors);
    if (sidecar["cache_sha256"].as<std::string>() != cache->sha256 ||
      sidecar["descriptor_sha256"].as<std::string>() != cache->descriptor_sha256)
    {throw std::runtime_error("SC cache checksum or input descriptor checksum does not match sidecar");}
    return cache;
  }
};

class WindowIndex final : public a2w_fastlio_common::DescriptorIndex
{
public:
  WindowIndex(std::shared_ptr<a2w_fastlio_common::DescriptorIndex> inner, std::uint64_t first_match,
    std::shared_ptr<const ScoreCache> cache = {})
  : inner_(std::move(inner)), first_match_(first_match), cache_(std::move(cache)) {}

  void setCurrent(std::uint64_t current) {current_ = current; queries.clear(); query_number_ = 0U;}
  void add(std::uint64_t id, const a2w_fastlio_common::ScanDescriptor & descriptor) override
  {
    if (cache_ && id >= cache_->count) {throw std::out_of_range("SC cache add ID out of range");}
    inner_->add(id, descriptor);
    if (added_ids_.empty()) {rings_ = descriptor.rings(); sectors_ = descriptor.sectors();}
    added_ids_.push_back(id);
  }

  std::vector<a2w_fastlio_common::LoopCandidate> queryTopK(
    const a2w_fastlio_common::ScanDescriptor & descriptor, std::size_t k,
    const a2w_fastlio_common::CandidateFilter & filter) const override
  {
    if (k == 0U) {throw std::invalid_argument("Scan Context Top-K must be positive");}
    const bool suppressed = current_ < first_match_;
    std::vector<a2w_fastlio_common::LoopCandidate> candidates;
    if (!suppressed && cache_) {
      if (current_ >= cache_->count) {throw std::out_of_range("SC cache query ID out of range");}
      if (!added_ids_.empty() && (descriptor.rings() != rings_ || descriptor.sectors() != sectors_)) {
        throw std::invalid_argument("Scan Context cached query dimensions do not match index");
      }
      for (const auto id : added_ids_) {
        // Exact production eligibility, additionally forbidding non-historical IDs.
        if (id >= current_ || id > filter.max_inclusive_id ||
          (filter.exclude_recent > 0U && (filter.max_inclusive_id < filter.exclude_recent ||
          id > filter.max_inclusive_id - filter.exclude_recent))) {continue;}
        const auto offset = static_cast<std::size_t>(current_) * cache_->count + static_cast<std::size_t>(id);
        candidates.push_back({id, 0U, cache_->distances.at(offset), cache_->yaws.at(offset)});
      }
      std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
        return a.distance != b.distance ? a.distance < b.distance : a.keyframe_id < b.keyframe_id;
      });
      if (candidates.size() > k) {candidates.resize(k);}
      for (std::size_t rank = 0U; rank < candidates.size(); ++rank) {candidates[rank].rank = rank;}
    } else if (!suppressed) {candidates = inner_->queryTopK(descriptor, k, filter);}
    const auto ordinal = query_number_++;
    if (candidates.empty()) {queries.push_back({current_, ordinal, k, filter, suppressed, 0U, {}});}
    for (const auto & candidate : candidates) {
      queries.push_back({current_, ordinal, k, filter, suppressed, candidates.size(), candidate});
    }
    return candidates;
  }

  mutable std::vector<QueryRecord> queries;

private:
  std::shared_ptr<a2w_fastlio_common::DescriptorIndex> inner_;
  std::uint64_t first_match_;
  std::uint64_t current_{0U};
  mutable std::size_t query_number_{0U};
  std::shared_ptr<const ScoreCache> cache_;
  std::vector<std::uint64_t> added_ids_;
  std::size_t rings_{0U}, sectors_{0U};
};

// The tool can be compiled before the enhanced pipeline header is connected.
// An unsupported enhanced run must fail explicitly, never silently run legacy.
template<typename T, typename = void> struct HasRevisitFlag : std::false_type {};
template<typename T> struct HasRevisitFlag<T,
    std::void_t<decltype(std::declval<T &>().revisit_verification_enabled)>> : std::true_type {};

template<typename Config> void setRevisitFlag(Config & config, bool enabled)
{
  if constexpr (HasRevisitFlag<Config>::value) {
    config.revisit_verification_enabled = enabled;
  } else if (enabled) {
    throw std::invalid_argument("enhanced mode requires LoopPipelineConfig::revisit_verification_enabled");
  }
}

a2w_fastlio_mapping::LoopPipelineConfig replayLoopConfig(const std::string & mode)
{
  if (mode != "legacy" && mode != "enhanced" && mode != "scheduled" && mode != "scheduled_cached") {
    throw std::invalid_argument("MODE must be legacy, enhanced, scheduled or scheduled_cached");
  }
  a2w_fastlio_mapping::LoopPipelineConfig config{5U, 30U, 5U, 5U, 10U, 32U, .2};
  setRevisitFlag(config, mode != "legacy");
  if (mode == "scheduled" || mode == "scheduled_cached") {
    config.discovery_interval_keyframes = 3U;
    config.confirmation_min_translation_m = .5;
    config.confirmation_min_rotation_rad = .15;
  }
  return config;
}

bool parsePaceFlag(const char * value)
{
  if (!value || std::string{value} == "0") {return false;}
  if (std::string{value} == "1") {return true;}
  throw std::invalid_argument("A2W_REPLAY_PACE must be exactly 0 or 1 (or unset)");
}

class PaceSchedule
{
public:
  PaceSchedule(const std::vector<std::int64_t> & stamps, bool enabled, std::size_t capacity)
  : epoch_{Clock::now()}, enabled_{enabled}, capacity_{capacity}
  {
    if (stamps.empty() || stamps.front() < 0) {throw std::invalid_argument("pacing requires nonnegative source stamps");}
    for (std::size_t i = 0; i < stamps.size(); ++i) {
      if (i > 0U && stamps[i] <= stamps[i - 1U]) {
        throw std::invalid_argument("source stamps must strictly increase");
      }
      due_ns_.push_back(stamps[i] - stamps.front());
    }
  }
  std::int64_t dueNs(std::size_t i) const {return due_ns_.at(i);}
  Clock::time_point epoch() const {return epoch_;}
  std::size_t pendingCount(std::int64_t wall_ns, std::size_t completed) const
  {
    if (completed > due_ns_.size()) {throw std::out_of_range("completed count exceeds frozen input");}
    const auto arrived = static_cast<std::size_t>(
      std::upper_bound(due_ns_.begin(), due_ns_.end(), wall_ns) - due_ns_.begin());
    return arrived > completed ? arrived - completed : 0U;
  }
  bool capacityExceeded(std::int64_t wall_ns, std::size_t completed) const
  {return pendingCount(wall_ns, completed) > capacity_;}
  void waitUntilArrival(std::size_t i) const
  {
    if (enabled_) {std::this_thread::sleep_until(epoch_ + std::chrono::nanoseconds{dueNs(i)});}
  }
  std::int64_t elapsedNs() const
  {return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - epoch_).count();}

private:
  Clock::time_point epoch_;
  bool enabled_;
  std::size_t capacity_;
  // Arrival counts only: these timestamps never enter candidate/pose decisions.
  std::vector<std::int64_t> due_ns_;
};

// Records each actual queue mutation, including an overflow on exception paths.
// This queue holds frozen-input IDs in memory; it is not a ROS/DDS receiver.
struct QueueEvidence
{
  replay_queue::ArrivalQueue & queue;
  fs::path path;
  bool saved{false};
  QueueEvidence(replay_queue::ArrivalQueue & input, fs::path output)
  : queue(input), path(std::move(output)) {}
  QueueEvidence(const QueueEvidence &)=delete;
  QueueEvidence & operator=(const QueueEvidence &)=delete;
  void save() {
    if (saved) {return;}
    queue.cancel(); queue.waitDone();
    auto stream=openOutput(path);
    writeCsv(stream,{"action","id","actual_wall_ns","scheduled_arrival_ns","waiting"});
    for (const auto & e:queue.events()) {
      writeCsv(stream,{e.action,text(e.id),text(e.wall_ns),text(e.due_ns),text(e.waiting)});
    }
    saved=true;
  }
  ~QueueEvidence() {try {save();} catch (...) {}}
};

std::shared_ptr<a2w_fastlio_common::QuatroRegistration> cachedCoarse(
  const a2w_fastlio_common::DefaultAlgorithmSuiteConfig & c)
{
  const auto & p=c.coarse;
  return std::make_shared<a2w_fastlio_common::QuatroRegistration>(
    a2w_fastlio_common::QuatroRegistrationConfig{p.normal_radius_m,p.feature_radius_m,
      p.noise_bound_m,p.rotation_gnc_factor,p.rotation_cost_threshold,p.rotation_max_iterations,
      p.estimate_scale,p.optimized_matching,p.descriptor_distance_threshold,
      p.maximum_correspondences,p.minimum_points,16U});
}

std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> cachedPipeline(
  const a2w_fastlio_common::DefaultAlgorithmSuiteConfig & c,
  const std::shared_ptr<a2w_fastlio_common::QuatroRegistration> & coarse)
{
  namespace common=a2w_fastlio_common;
  const auto & f=c.fine; const auto & v=c.validation;
  const auto fine=std::make_shared<common::NanoGicpRegistration>(common::NanoGicpRegistrationConfig{
    f.maximum_correspondence_distance_m,f.thread_count,f.correspondence_randomness,
    f.maximum_iterations,f.transformation_epsilon,f.rotation_epsilon,f.regularization_method,
    f.fitness_score_max_range_m,f.minimum_points,f.source_voxel_leaf_m});
  return std::make_shared<common::RegistrationPipeline>(coarse,fine,
    common::MatchValidator{common::MatchValidationConfig{v.maximum_fitness,v.minimum_overlap,
      v.minimum_correspondences,v.maximum_translation_jump_m,v.maximum_rotation_jump_rad,
      v.minimum_candidate_distance_separation}},common::RegistrationPipelineConfig{c.evidence_distance_m});
}

std::vector<std::string> eventHeaders()
{
  return {"current", "stamp_ns", "candidate", "rank", "candidates", "accepted", "graph_committed",
    "reason", "graph_factor_count", "source_points", "target_points", "registration_success",
    "registration_converged", "fitness", "overlap", "correspondences", "registration_rejection_reason",
    "descriptor_ms", "scan_context_ms", "source_local_map_ms", "target_local_map_ms", "quatro_ms",
    "nano_gicp_ms", "evidence_ms", "validator_ms", "gtsam_ms", "registration_total_ms", "frame_total_ms", "sc_cached",
    "has_pose_check", "predicted_tx", "predicted_ty", "predicted_tz", "predicted_qx", "predicted_qy",
    "predicted_qz", "predicted_qw", "corrected_tx", "corrected_ty", "corrected_tz", "corrected_qx",
    "corrected_qy", "corrected_qz", "corrected_qw", "correction_dx_map_m", "correction_dy_map_m",
    "correction_dz_map_m", "translation_difference_m", "rotation_difference_rad", "relative_roll_rad",
    "relative_pitch_rad", "relative_yaw_rad", "translation_limit_m", "rotation_limit_rad",
    "position_pass", "rotation_pass", "descriptor_distance", "yaw_hint_rad", "origin_candidate",
    "confirmation_count", "rejection_stage"};
}

void appendPose(std::vector<std::string> & cells, const a2w_fastlio_common::Pose3d & pose)
{
  for (Eigen::Index i = 0; i < 3; ++i) {cells.push_back(text(pose.translation[i]));}
  for (Eigen::Index i = 0; i < 4; ++i) {cells.push_back(text(pose.rotation.coeffs()[i]));}
}

std::vector<std::string> eventCells(const LoopPipelineEvent & event, std::int64_t stamp, bool committed, bool cached = false)
{
  const auto & result = event.registration;
  std::vector<std::string> cells = {text(event.current_id), text(stamp), text(event.candidate_id),
    text(event.candidate_rank), text(event.candidate_count), text(event.accepted), text(committed),
    event.reason, text(event.graph_factor_count), text(event.source_points), text(event.target_points),
    text(result.success), text(result.converged), text(result.fitness), text(result.overlap),
    text(result.correspondence_count), result.rejection_reason, text(event.descriptor_ms),
    text(event.scan_context_ms), text(event.source_local_map_ms), text(event.target_local_map_ms),
    text(event.quatro_ms), text(event.nano_gicp_ms), text(event.evidence_ms), text(event.validator_ms),
    text(event.gtsam_ms), text(event.registration_total_ms), text(event.frame_total_ms), text(cached), text(event.pose_check.has_value())};
  if (event.pose_check) {
    const auto & check = *event.pose_check;
    appendPose(cells, check.predicted_pose);
    appendPose(cells, check.corrected_pose);
    const Eigen::Vector3d correction = check.corrected_pose.translation - check.predicted_pose.translation;
    for (Eigen::Index i = 0; i < 3; ++i) {cells.push_back(text(correction[i]));}
    cells.push_back(text(check.translation_difference_m));
    cells.push_back(text(check.rotation_difference_rad));
    for (Eigen::Index i = 0; i < 3; ++i) {cells.push_back(text(check.relative_rpy_rad[i]));}
    cells.push_back(text(check.translation_limit_m));
    cells.push_back(text(check.rotation_limit_rad));
    cells.push_back(text(check.position_pass));
    cells.push_back(text(check.rotation_pass));
  } else {
    // Pad only the original pose-check columns before appending new diagnostics.
    cells.resize(eventHeaders().size() - 5U);
  }
  cells.insert(cells.end(), {text(event.descriptor_distance), text(event.yaw_hint_rad),
    text(event.origin_candidate), text(event.confirmation_count), event.rejection_stage});
  if (cells.size() != eventHeaders().size()) {throw std::logic_error("event CSV width mismatch");}
  return cells;
}

void writePoses(const fs::path & path, const a2w_fastlio_mapping::OptimizedPoseSnapshot & snapshot)
{
  auto output = openOutput(path);
  writeCsv(output, {"id", "tx", "ty", "tz", "qx", "qy", "qz", "qw"});
  for (const auto & entry : snapshot.poses) {
    std::vector<std::string> cells{text(entry.id)};
    appendPose(cells, entry.pose);
    writeCsv(output, cells);
  }
  output.close();
}

struct SaveFinalUpdateResult
{
  std::uint64_t before_revision{0U}, after_revision{0U};
  std::size_t factor_count{0U};
  double elapsed_ms{0.0};
};

SaveFinalUpdateResult saveFinalGraphUpdate(
  a2w_fastlio_mapping::PoseGraphOptimizer & graph, const fs::path & output,
  std::ostream & metadata, std::size_t expected_factor_count)
{
  const auto online = graph.optimizedPoses();
  writePoses(output / "online_last_frame_poses.csv", online);
  metadata << "save_final_update_requested=true\nonline_last_frame_snapshot_revision=" << online.revision
           << "\nonline_last_frame_factor_count=" << expected_factor_count << '\n';
  metadata.flush();
  // Mirror Backend's single save-time empty iSAM2 update. No candidate search or
  // new loop/odometry factor is created here; the online snapshot stays available.
  const auto started = Clock::now();
  const auto update = graph.update(true);
  const double update_ms = elapsedMs(started);
  metadata << "save_final_update_ms=" << text(update_ms) << "\nsave_final_update_reason=" << update.reason << '\n';
  if (!update.accepted) {throw std::runtime_error("save-time graph update failed: " + update.reason);}
  const auto saved = graph.optimizedPoses();
  if (saved.revision != online.revision + 1U || update.factor_count != expected_factor_count ||
    saved.poses.size() != online.poses.size())
  {throw std::runtime_error("save-time graph update changed factor/node count or unexpected revision");}
  writePoses(output / "optimized_poses.csv", saved);
  metadata << "save_final_update=true\nsave_final_snapshot_revision=" << saved.revision
           << "\nsave_final_update_revision_delta=" << saved.revision - online.revision
           << "\nsave_final_update_factor_count=" << update.factor_count
           << "\nsave_final_update_node_count=" << update.node_count
           << "\nonline_last_frame_poses_sha256=" << sha256File(output / "online_last_frame_poses.csv")
           << "\noptimized_poses_sha256=" << sha256File(output / "optimized_poses.csv") << '\n';
  metadata.flush();
  return {online.revision, saved.revision, update.factor_count, update_ms};
}

struct CapturedLoopMeasurement
{
  std::uint64_t current_id{0U}, candidate_id{0U};
  a2w_fastlio_common::Pose3d relative_pose{};
};

class LoopMeasurementCapture
{
public:
  void queue(const LoopPipelineEvent & event, const a2w_fastlio_mapping::OptimizedPoseSnapshot & before)
  {
    if (!event.accepted || !event.pose_check || event.candidate_id >= event.current_id) {
      throw std::runtime_error("queued loop is missing a valid pre-update corrected/current-history pose");
    }
    const auto found = std::find_if(before.poses.begin(), before.poses.end(), [&](const auto & entry) {
      return entry.id == event.candidate_id;
    });
    if (found == before.poses.end()) {throw std::runtime_error("queued loop historical pose missing from pre-update snapshot");}
    const auto & corrected = event.pose_check->corrected_pose;
    const auto & historical = found->pose;
    if (!a2w_fastlio_common::isFinitePose(corrected) || !a2w_fastlio_common::isFinitePose(historical)) {
      throw std::runtime_error("cannot export nonfinite queued loop measurement");
    }
    a2w_fastlio_common::Pose3d relative;
    relative.rotation = (corrected.rotation.inverse() * historical.rotation).normalized();
    relative.translation = corrected.rotation.inverse() * (historical.translation - corrected.translation);
    pending_[{event.current_id, event.candidate_id}] = {event.current_id, event.candidate_id, relative};
  }

  std::optional<CapturedLoopMeasurement> finish(const LoopPipelineEvent & event)
  {
    const auto key = std::make_pair(event.current_id, event.candidate_id);
    const auto found = pending_.find(key);
    if (!event.accepted) {pending_.erase(key); return std::nullopt;}
    if (found == pending_.end()) {throw std::runtime_error("successful graph commit has no captured pre-update measurement");}
    const auto result = found->second;
    pending_.erase(found);
    return result;
  }

private:
  std::map<std::pair<std::uint64_t, std::uint64_t>, CapturedLoopMeasurement> pending_;
};

void writeConfigMetadata(
  std::ostream & output, const a2w_fastlio_mapping::LoopPipelineConfig & loop,
  const a2w_fastlio_common::DefaultAlgorithmSuiteConfig & suite,
  const a2w_fastlio_common::LocalMapConfig & local,
  const a2w_fastlio_mapping::PoseGraphConfig & graph)
{
  output << std::setprecision(17);
#define REPLAY_RECORD(section, object, field) output << section "." #field "=" << object.field << '\n'
  REPLAY_RECORD("loop_pipeline", loop, top_k);
  REPLAY_RECORD("loop_pipeline", loop, exclude_recent);
  REPLAY_RECORD("loop_pipeline", loop, local_map_before);
  REPLAY_RECORD("loop_pipeline", loop, local_map_after);
  REPLAY_RECORD("loop_pipeline", loop, minimum_keyframes_between_accepted_loops);
  REPLAY_RECORD("loop_pipeline", loop, queue_capacity);
  REPLAY_RECORD("loop_pipeline", loop, maximum_descriptor_distance);
  REPLAY_RECORD("loop_pipeline", loop, revisit_verification_enabled);
  REPLAY_RECORD("loop_pipeline", loop, origin_keyframe_count);
  REPLAY_RECORD("loop_pipeline", loop, origin_top_k);
  REPLAY_RECORD("loop_pipeline", loop, hypothesis_group_id_span);
  REPLAY_RECORD("loop_pipeline", loop, hypothesis_group_distance_m);
  REPLAY_RECORD("loop_pipeline", loop, solution_merge_distance_m);
  REPLAY_RECORD("loop_pipeline", loop, solution_merge_rotation_rad);
  REPLAY_RECORD("loop_pipeline", loop, distinct_solution_fitness_margin_m);
  REPLAY_RECORD("loop_pipeline", loop, confirmation_frames);
  REPLAY_RECORD("loop_pipeline", loop, confirmation_max_gap);
  REPLAY_RECORD("loop_pipeline", loop, retry_interval_keyframes);
  REPLAY_RECORD("loop_pipeline", loop, retry_max_attempts);
  REPLAY_RECORD("loop_pipeline", loop, retry_reset_keyframes);
  REPLAY_RECORD("loop_pipeline", loop, discovery_interval_keyframes);
  REPLAY_RECORD("loop_pipeline", loop, confirmation_min_translation_m);
  REPLAY_RECORD("loop_pipeline", loop, confirmation_min_rotation_rad);
  REPLAY_RECORD("scan_context", suite.place_recognition, rings);
  REPLAY_RECORD("scan_context", suite.place_recognition, sectors);
  REPLAY_RECORD("scan_context", suite.place_recognition, max_radius_m);
  REPLAY_RECORD("scan_context", suite.place_recognition, sensor_height_m);
  REPLAY_RECORD("quatro", suite.coarse, normal_radius_m);
  REPLAY_RECORD("quatro", suite.coarse, feature_radius_m);
  REPLAY_RECORD("quatro", suite.coarse, noise_bound_m);
  REPLAY_RECORD("quatro", suite.coarse, rotation_gnc_factor);
  REPLAY_RECORD("quatro", suite.coarse, rotation_cost_threshold);
  REPLAY_RECORD("quatro", suite.coarse, rotation_max_iterations);
  REPLAY_RECORD("quatro", suite.coarse, estimate_scale);
  REPLAY_RECORD("quatro", suite.coarse, optimized_matching);
  REPLAY_RECORD("quatro", suite.coarse, descriptor_distance_threshold);
  REPLAY_RECORD("quatro", suite.coarse, maximum_correspondences);
  REPLAY_RECORD("quatro", suite.coarse, minimum_points);
  REPLAY_RECORD("nano_gicp", suite.fine, maximum_correspondence_distance_m);
  REPLAY_RECORD("nano_gicp", suite.fine, thread_count);
  REPLAY_RECORD("nano_gicp", suite.fine, correspondence_randomness);
  REPLAY_RECORD("nano_gicp", suite.fine, maximum_iterations);
  REPLAY_RECORD("nano_gicp", suite.fine, transformation_epsilon);
  REPLAY_RECORD("nano_gicp", suite.fine, rotation_epsilon);
  REPLAY_RECORD("nano_gicp", suite.fine, regularization_method);
  REPLAY_RECORD("nano_gicp", suite.fine, fitness_score_max_range_m);
  REPLAY_RECORD("nano_gicp", suite.fine, minimum_points);
  REPLAY_RECORD("nano_gicp", suite.fine, source_voxel_leaf_m);
  REPLAY_RECORD("loop_validation", suite.validation, maximum_fitness);
  REPLAY_RECORD("loop_validation", suite.validation, minimum_overlap);
  REPLAY_RECORD("loop_validation", suite.validation, minimum_correspondences);
  REPLAY_RECORD("loop_validation", suite.validation, maximum_translation_jump_m);
  REPLAY_RECORD("loop_validation", suite.validation, maximum_rotation_jump_rad);
  REPLAY_RECORD("loop_validation", suite.validation, minimum_candidate_distance_separation);
  REPLAY_RECORD("pipeline", suite, evidence_distance_m);
  REPLAY_RECORD("local_map", local, voxel_leaf_m);
  REPLAY_RECORD("local_map", local, max_points);
  REPLAY_RECORD("pose_graph", graph, prior_rotation_sigma_rad);
  REPLAY_RECORD("pose_graph", graph, prior_translation_sigma_m);
  REPLAY_RECORD("pose_graph", graph, odometry_rotation_sigma_rad);
  REPLAY_RECORD("pose_graph", graph, odometry_translation_sigma_m);
  REPLAY_RECORD("pose_graph", graph, loop_rotation_sigma_rad);
  REPLAY_RECORD("pose_graph", graph, loop_translation_sigma_m);
  REPLAY_RECORD("pose_graph", graph, robust_kernel_scale);
  REPLAY_RECORD("pose_graph", graph, relinearization_threshold);
  REPLAY_RECORD("pose_graph", graph, relinearization_skip);
#undef REPLAY_RECORD
  output << "pose_graph.robust_kernel=" << (graph.robust_kernel == a2w_fastlio_mapping::RobustKernel::HUBER ? "huber" : "cauchy")
         << "\nloop_validation.minimum_candidate_distance_separation.units=dimensionless_scan_context_score_gap"
         << "\nloop_pipeline.distinct_solution_fitness_margin_m.units=metres_geometric_fitness_gap"
         << "\nconfig_scope=legacy_production_defaults_plus_opt_in_revisit_experiment_fields\n";
  const bool scheduled = loop.revisit_verification_enabled &&
    (loop.discovery_interval_keyframes != 1U || loop.confirmation_min_translation_m > 0.0 ||
    loop.confirmation_min_rotation_rad > 0.0);
  output << "policy_version=" << (scheduled ? "v3_scheduled_motion_v1" :
    loop.revisit_verification_enabled ? "v2_mandatory_competitors_v1" : "legacy_v1")
         << "\npolicy_acceptance=NOT_ACCEPTED_EXPERIMENT"
         << "\nconfirmation_motion_independence_proven=false"
         << "\nconfirmation_motion_source=last_counted_raw_odometry_observation"
         << "\nconfirmation_motion_rule=translation_OR_rotation_enabled_nonzero_axes"
         << "\ndiscovery_interval_scope=whole_retrieval_and_geometry_chain_pending_bypasses"
         << "\nconfirmation_motion_wait=skip_whole_query_geometry_chain_no_constraint_no_anchor_refresh"
         << "\nselected_frame_competitor_budget=all_retrieved_candidates_mandatory\n";
}

double quantile(std::vector<double> values, double probability)
{
  if (values.empty()) {return 0.0;}
  std::sort(values.begin(), values.end());
  const double location = probability * static_cast<double>(values.size() - 1U);
  const auto low = static_cast<std::size_t>(location);
  const auto high = std::min(low + 1U, values.size() - 1U);
  return values[low] + (values[high] - values[low]) * (location - static_cast<double>(low));
}

std::string graphFrameFailure(std::uint64_t before_revision,
  const a2w_fastlio_mapping::OptimizedPoseSnapshot & after, std::uint64_t current,
  const std::vector<LoopPipelineEvent> & events)
{
  for (const auto & event : events) {
    if (event.reason.find("gtsam_update_failed") != std::string::npos) {return event.reason;}
  }
  if (after.revision <= before_revision || after.poses.size() != current + 1U) {
    return "graph_frame_not_committed:revision_or_node_count";
  }
  for (std::size_t i = 0U; i < after.poses.size(); ++i) {
    if (after.poses[i].id != i || !a2w_fastlio_common::isFinitePose(after.poses[i].pose)) {
      return "graph_frame_not_committed:invalid_snapshot";
    }
  }
  return {};
}

int run(int argc, char ** argv)
{
  const auto setup_started = Clock::now();
  if (argc < 4 || argc > 6) {
    std::cerr << "usage: loop_recall_replay MAP_BUNDLE OUTPUT_DIR legacy|enhanced|scheduled [FIRST_MATCH_ID=0] [LAST_ID=1489]\n";
    return 2;
  }
  const std::string mode = argv[3];
  const auto loop_config = replayLoopConfig(mode);
  const bool paced = parsePaceFlag(std::getenv("A2W_REPLAY_PACE"));
  const auto first_match = argc >= 5 ? parseId(argv[4]) : 0U;
  const auto last = argc >= 6 ? parseId(argv[5]) : 1489U;
  auto suite_config = a2w_fastlio_common::DefaultAlgorithmSuiteConfig{};
  suite_config.fine.thread_count = 0;
  const a2w_fastlio_common::LocalMapConfig local_config{0.2, 200000U};
  const a2w_fastlio_mapping::PoseGraphConfig graph_config{};
  const auto input = fs::canonical(argv[1]);
  const auto output = fs::weakly_canonical(argv[2]);
  validateOutputPaths(input, output);
  // Reader verifies manifest and loads all Bundle data read-only. Saved optimized
  // poses/global map/descriptors remain unused as replay initialization.
  const auto bundle = a2w_fastlio_map::MapBundleReader{}.read(input, a2w_fastlio_map::ReadMode::kReadOnly);
  validateRange(first_match, last, bundle.data.keyframes.size());
  std::shared_ptr<ScoreCache> cache;
  const char * cache_env = std::getenv("A2W_REPLAY_SC_CACHE");
  if (cache_env && *cache_env != '\0') {
    cache = ScoreCache::read(cache_env, bundle.data.keyframes.size(), input / "descriptors/scan_context.bin");
    validateOutputPaths(cache->path, output);
    auto sidecar_path = cache->path;
    sidecar_path.replace_extension(".meta.json");
    validateOutputPaths(sidecar_path, output);
  }
  reserveOutput(output);
  auto metadata = openOutput(output / "run_metadata.txt");
  const std::string scope = first_match > 0U ? "WINDOW_DIAGNOSTIC_NOT_FULL_SEQUENCE_ACCEPTANCE" :
    (last + 1U == bundle.data.keyframes.size() ? "FULL_SEQUENTIAL_RAW_ODOMETRY_REPLAY" : "SEQUENTIAL_PREFIX_RAW_ODOMETRY_REPLAY");
  metadata << "scope=" << scope << "\nmode=" << mode << "\ninput=" << input.string()
           << "\nbundle_uuid=" << bundle.data.metadata.bundle_uuid << "\nfirst_match_id=" << first_match
           << "\nlast_id=" << last << "\nprocess_starts_at=0\nsaved_optimized_initialization=false"
           << "\nindependent_ground_truth=false"
           << "\nqueries_csv=raw_index_query_results_before_pipeline_distance_filter"
           << "\nframe_process_wall_ms=process_only_excludes_csv_and_snapshot_io"
           << "\nshared_frame_stage_times_repeat_across_candidate_events=true"
           << "\npose_check_missing_fields=empty_never_PASS\nstatus=started\n";
  metadata << "source_file=src/a2w_fastlio_mapping/test/loop_recall_replay.cpp"
           << "\nsource_sha256=" << (fs::is_regular_file(__FILE__) ? sha256File(__FILE__) : "<unavailable>")
           << "\nbuild.compiler=" << __VERSION__ << "\nbuild.cplusplus=" << __cplusplus
           << "\nbuild.compiled_at=" << __DATE__ << ' ' << __TIME__
           << "\nbuild.external_identity_source=optional_A2W_REPLAY_environment_variables"
           << "\nloop_constraints.measurement=inverse(corrected_current_preupdate)*candidate_previous_committed_snapshot"
           << "\nloop_constraints.capture_stage=CandidateFinished_accepted_queued_before_graph_update"
           << "\nloop_constraints.export_gate=OptimizationFinished_accepted_only\n";
  writeConfigMetadata(metadata, loop_config, suite_config, local_config, graph_config);
  metadata << "pace_enabled=" << paced
           << "\npace_transport=PRELOADED_MEMORY_SOURCE_TIMESTAMP_PACING_NO_ROS_TRANSPORT"
           << "\npace_clock=steady_clock_epoch_immediately_before_first_input"
           << "\npace_arrival_index=frozen_source_stamps_only_no_future_poses_or_descriptors"
           << "\npace_pending=arrived_input_indices_minus_completed_frames_excludes_active_at_end"
           << "\npace_end_capture=after_primary_csv_snapshot_flush_before_timing_row_write"
           << "\npace_latency_off_fields=empty_not_simulated_from_recorded_frame_costs"
           << "\npace_capacity_exceeded_action=record_FAIL_continue_all_frames_exit_3_after_save"
           << "\npace_bounded_queue_claim=false_no_real_transport_queue_is_created\n";
  metadata << "preload_setup_scope=run_entry_through_bundle_cache_suite_and_output_setup_before_pace_epoch"
           << "\npreload_setup_os_cold_cache_guaranteed=false"
           << "\nreplay_graph_failure_policy=every_frame_snapshot_revision_node_count_ids_finiteness_fail_closed_before_next_frame\n";
  metadata << "sc_cached=" << static_cast<bool>(cache)
           << "\nsc_timing_interpretation=" << (cache ? "CACHE_LOOKUP_NOT_ONLINE_SC_SPEED" : "LIVE_PRODUCTION_SC_QUERY") << '\n';
  if (cache) {
    metadata << "sc_cache_path=" << cache->path.string() << "\nsc_cache_sha256=" << cache->sha256
             << "\nsc_cache_descriptor_sha256=" << cache->descriptor_sha256
             << "\nsc_cache_only_added_historical_ids=true\nsc_cache_future_entries_never_eligible=true\n";
  }
  for (const auto * variable : {"OMP_NUM_THREADS", "OMP_DYNAMIC", "OMP_PROC_BIND", "OMP_PLACES", "OMP_THREAD_LIMIT",
      "A2W_REPLAY_BUILD_ID", "A2W_REPLAY_LIBRARY_BUILD_ID", "A2W_REPLAY_NANO_GICP_BUILD_ID",
      "A2W_REPLAY_QUATRO_BUILD_ID", "A2W_REPLAY_BUILD_MANIFEST", "A2W_REPLAY_SOURCE_SHA256"}) {
    const char * value = std::getenv(variable);
    metadata << variable << '=' << (value ? value : "<unset>") << '\n';
  }
  metadata.flush();

  auto suite = a2w_fastlio_common::createDefaultAlgorithmSuite(suite_config);
  std::shared_ptr<a2w_fastlio_common::QuatroRegistration> cached_coarse;
  const bool cache_experiment=mode == "scheduled_cached";
  if (cache_experiment) {
    cached_coarse=cachedCoarse(suite_config);
    suite.registration=cachedPipeline(suite_config,cached_coarse);
    metadata << "policy_version=v4_exact_feature_cache_v1\nquatro.feature_cache_capacity=16"
      << "\nquatro.cache_identity=ordered_exact_XYZI_content_plus_immutable_adapter_parameters"
      << "\nquatro.cache_collision_check=exact_snapshot_bytes\ncache_is_fine_only=false\n";
  }
  auto index = std::make_shared<WindowIndex>(suite.descriptor_index, first_match, cache);
  auto graph = std::make_shared<a2w_fastlio_mapping::PoseGraphOptimizer>(graph_config);
  a2w_fastlio_mapping::LoopPipeline pipeline{suite.place_recognition, index, suite.registration, graph,
    a2w_fastlio_common::LocalMapBuilder{local_config}, loop_config};
  auto events_csv = openOutput(output / "events.csv");
  auto frames_csv = openOutput(output / "frames.csv");
  auto queries_csv = openOutput(output / "queries.csv");
  auto constraints_csv = openOutput(output / "loop_constraints.csv");
  auto pace_csv = openOutput(output / "pace_frames.csv");
  auto commit_csv = openOutput(output / "commit_latency.csv");
  writeCsv(pace_csv, {"current", "stamp_ns", "pace_enabled", "scheduled_arrival_ms", "actual_start_wall_ms",
    "actual_graph_commit_wall_ms", "actual_end_after_io_wall_ms", "start_wall_latency_ms", "end_wall_latency_ms",
    "arrived_unprocessed_at_start", "arrived_waiting_after_end", "queue_capacity", "capacity_exceeded"});
  writeCsv(commit_csv, {"current", "candidate", "stamp_ns", "pace_enabled", "scheduled_arrival_ms",
    "actual_graph_commit_wall_ms", "commit_wall_latency_ms"});
  writeCsv(constraints_csv, {"current_id", "candidate_id", "relative_tx", "relative_ty", "relative_tz",
    "relative_qx", "relative_qy", "relative_qz", "relative_qw"});
  writeCsv(events_csv, eventHeaders());
  writeCsv(frames_csv, {"current", "stamp_ns", "sensor_elapsed_s", "sensor_interval_ms", "matching_suppressed", "sc_cached",
    "events", "accepted_events", "committed_loops", "process_wall_ms", "pipeline_frame_total_ms", "descriptor_ms",
    "scan_context_ms", "gtsam_ms", "reasons"});
  writeCsv(queries_csv, {"current", "query_number", "requested_k", "max_inclusive_id", "exclude_recent",
    "matching_suppressed", "raw_result_count", "candidate", "rank", "descriptor_distance", "yaw_hint_rad", "sc_cached"});
  std::size_t event_count = 0U, accepted_events = 0U, committed_loops = 0U;
  std::size_t last_factor_count = 0U;
  std::map<std::string, std::size_t> event_reasons, frame_reasons;
  std::vector<double> process_times, pipeline_times, scan_context_times;
  std::vector<double> start_latencies, end_latencies, commit_latencies;
  std::vector<std::int64_t> source_stamps;
  for (std::uint64_t current = 0U; current <= last; ++current) {
    source_stamps.push_back(bundle.data.keyframes.at(current).stamp_ns);
  }
  bool capacity_exceeded = false;
  std::size_t maximum_pending = 0U;
  const auto preload_setup_ms = elapsedMs(setup_started);
  metadata << "preload_setup_wall_ms=" << text(preload_setup_ms) << '\n';
  metadata.flush();
  const auto replay_started = Clock::now();
  const PaceSchedule pace{source_stamps, paced, loop_config.queue_capacity};
  std::unique_ptr<replay_queue::ArrivalQueue> arrival_queue;
  std::unique_ptr<QueueEvidence> queue_evidence;
  if (cache_experiment && paced) {
    arrival_queue=std::make_unique<replay_queue::ArrivalQueue>(
      source_stamps,loop_config.queue_capacity,pace.epoch());
    queue_evidence=std::make_unique<QueueEvidence>(
      *arrival_queue,output/"arrival_queue_events.csv");
    metadata << "pace_transport=BOUNDED_PRELOADED_INPUT_ID_QUEUE_NO_ROS"
      << "\ncontinuous_queue_observation=every_actual_arrival_and_dequeue_mutation"
      << "\ncontinuous_queue_overflow=stop_with_failure_preserve_original_input_archive\n";
  }
  for (std::uint64_t current = 0U; current <= last; ++current) {
    if (arrival_queue) {
      if (arrival_queue->pop() != current) {throw std::runtime_error("input queue order mismatch");}
    } else {pace.waitUntilArrival(current);}
    const auto start_wall_ns = pace.elapsedNs();
    const auto due_ns = pace.dueNs(current);
    const auto frame = freshFrame(bundle.data.keyframes.at(current));
    if (frame.id != current) {throw std::runtime_error("Bundle keyframes must have contiguous sequential IDs");}
    index->setCurrent(current);
    std::vector<LoopPipelineEvent> commits;
    std::map<std::uint64_t, std::int64_t> commit_wall_ns;
    std::vector<CapturedLoopMeasurement> committed_measurements;
    LoopMeasurementCapture measurement_capture;
    std::string observer_error;
    bool graph_update_failed = false;
    const auto before_revision = graph->optimizedPoses().revision;
    const auto frame_started = Clock::now();
    const auto events = pipeline.process(frame,
      [&](a2w_fastlio_mapping::LoopProgressStage stage, const LoopPipelineEvent & event) {
        // Pipeline deliberately swallows observer exceptions. Retain capture errors
        // and fail the replay after writing the frame's diagnostics instead.
        try {
          if (stage == a2w_fastlio_mapping::LoopProgressStage::CandidateFinished && event.accepted) {
            measurement_capture.queue(event, graph->optimizedPoses());
          }
          if (stage == a2w_fastlio_mapping::LoopProgressStage::OptimizationFinished) {
            if (event.accepted) {
              commit_wall_ns.emplace(event.candidate_id, pace.elapsedNs());
              commits.push_back(event);
            } else {graph_update_failed = true;}
            const auto measurement = measurement_capture.finish(event);
            if (measurement) {committed_measurements.push_back(*measurement);}
          }
        } catch (const std::exception & error) {
          observer_error = error.what();
        }
      });
    const auto process_ms = elapsedMs(frame_started);
    if (events.empty()) {throw std::runtime_error("pipeline returned no frame outcome");}
    const auto graph_failure = graphFrameFailure(before_revision, graph->optimizedPoses(), current, events);
    std::size_t frame_accepted = 0U;
    std::map<std::string, std::size_t> reasons;
    for (const auto & event : events) {
      const bool committed = std::any_of(commits.begin(), commits.end(), [&](const auto & commit) {
        return commit.current_id == event.current_id && commit.candidate_id == event.candidate_id;
      });
      writeCsv(events_csv, eventCells(event, frame.stamp_ns, committed, static_cast<bool>(cache)));
      ++event_reasons[event.reason];
      ++reasons[event.reason];
      frame_accepted += event.accepted ? 1U : 0U;
    }
    for (const auto & commit : commits) {
      std::ostringstream filename;
      filename << "loop_" << std::setfill('0') << std::setw(7) << current << "_to_"
               << std::setw(7) << commit.candidate_id << ".csv";
      writePoses(output / "snapshots" / filename.str(), graph->optimizedPoses());
      const auto observed_ns = commit_wall_ns.at(commit.candidate_id);
      const double latency_ms = static_cast<double>(observed_ns - due_ns) / 1e6;
      writeCsv(commit_csv, {text(current), text(commit.candidate_id), text(frame.stamp_ns), text(paced),
        text(static_cast<double>(due_ns) / 1e6), text(static_cast<double>(observed_ns) / 1e6),
        paced ? text(latency_ms) : ""});
      if (paced) {commit_latencies.push_back(latency_ms);}
    }
    for (const auto & measurement : committed_measurements) {
      std::vector<std::string> cells{text(measurement.current_id), text(measurement.candidate_id)};
      appendPose(cells, measurement.relative_pose);
      writeCsv(constraints_csv, cells);
    }
    for (const auto & query : index->queries) {
      std::vector<std::string> cells{text(query.current), text(query.query_number), text(query.requested_k),
        text(query.filter.max_inclusive_id), text(query.filter.exclude_recent), text(query.suppressed), text(query.result_count)};
      if (query.candidate) {
        cells.insert(cells.end(), {text(query.candidate->keyframe_id), text(query.candidate->rank),
          text(query.candidate->distance), text(query.candidate->yaw_hint_rad)});
      } else {cells.resize(11U);}
      cells.push_back(text(static_cast<bool>(cache)));
      writeCsv(queries_csv, cells);
    }
    std::string frame_reason;
    for (const auto & reason : reasons) {
      ++frame_reasons[reason.first];
      if (!frame_reason.empty()) {frame_reason += ';';}
      frame_reason += reason.first;
    }
    const auto & first = events.front();
    last_factor_count = first.graph_factor_count;
    writeCsv(frames_csv, {text(current), text(frame.stamp_ns),
      text(static_cast<double>(frame.stamp_ns - bundle.data.keyframes.front().stamp_ns) / 1e9),
      current == 0U ? "" : text(static_cast<double>(frame.stamp_ns - bundle.data.keyframes.at(current - 1U).stamp_ns) / 1e6),
      text(current < first_match), text(static_cast<bool>(cache)), text(events.size()), text(frame_accepted), text(commits.size()), text(process_ms),
      text(first.frame_total_ms), text(first.descriptor_ms), text(first.scan_context_ms), text(first.gtsam_ms), frame_reason});
    event_count += events.size();
    accepted_events += frame_accepted;
    committed_loops += commits.size();
    process_times.push_back(process_ms);
    pipeline_times.push_back(first.frame_total_ms);
    scan_context_times.push_back(first.scan_context_ms);
    events_csv.flush(); frames_csv.flush(); queries_csv.flush(); constraints_csv.flush(); commit_csv.flush();
    const auto end_wall_ns = pace.elapsedNs();
    const auto pending_end = pace.pendingCount(end_wall_ns, current + 1U);
    const bool exceeded = paced && pace.capacityExceeded(end_wall_ns, current + 1U);
    capacity_exceeded = capacity_exceeded || exceeded;
    if (paced) {maximum_pending = std::max(maximum_pending, pending_end);}
    const double start_latency_ms = static_cast<double>(start_wall_ns - due_ns) / 1e6;
    const double end_latency_ms = static_cast<double>(end_wall_ns - due_ns) / 1e6;
    writeCsv(pace_csv, {text(current), text(frame.stamp_ns), text(paced), text(static_cast<double>(due_ns) / 1e6),
      text(static_cast<double>(start_wall_ns) / 1e6),
      commits.empty() ? "" : text(static_cast<double>(commit_wall_ns.at(commits.back().candidate_id)) / 1e6),
      text(static_cast<double>(end_wall_ns) / 1e6), paced ? text(start_latency_ms) : "",
      paced ? text(end_latency_ms) : "", paced ? text(pace.pendingCount(start_wall_ns, current)) : "",
      paced ? text(pending_end) : "", text(loop_config.queue_capacity), text(exceeded)});
    pace_csv.flush();
    if (paced) {start_latencies.push_back(start_latency_ms); end_latencies.push_back(end_latency_ms);}
    std::cout << "replay frame=" << current << " events=" << events.size() << " committed=" << commits.size()
              << " process_ms=" << process_ms << " reason=" << frame_reason << '\n';
    if ((current + 1U) % 20U == 0U || current == last) {
      std::cerr << "replay progress frames=" << current + 1U << '/' << last + 1U
                << " current=" << current << " committed=" << committed_loops
                << " elapsed_s=" << elapsedMs(replay_started) / 1000.0 << " sc_cached=" << static_cast<bool>(cache) << '\n';
    }
    if (graph_update_failed || !graph_failure.empty()) {
      metadata << "status=failed\nfailed_frame=" << current << "\ngraph_frame_failure=" << graph_failure << '\n';
      metadata.flush();
      throw std::runtime_error("graph update failed; inspect partial CSV evidence: " + graph_failure);
    }
    if (!observer_error.empty()) {throw std::runtime_error("loop measurement capture failed: " + observer_error);}
  }
  const auto save_update = saveFinalGraphUpdate(*graph, output, metadata, last_factor_count);
  if (arrival_queue) {
    arrival_queue->waitDone();
    const auto q=arrival_queue->stats();
    queue_evidence->save();
    metadata << "continuous_queue_produced=" << q.produced << "\ncontinuous_queue_consumed=" << q.consumed
      << "\ncontinuous_queue_maximum_waiting=" << q.max_waiting << "\ncontinuous_queue_overflow=" << q.overflow
      << "\ncontinuous_offline_queue_verified=" << (!q.overflow && q.produced==last+1 && q.consumed==last+1)
      << "\nreal_ROS_queue_verified=false\n";
    if (q.overflow || q.produced != last+1 || q.consumed != last+1) {throw std::runtime_error("incomplete input queue");}
  }
  if (cached_coarse) {
    const auto stats=cached_coarse->cacheStats();
    metadata << "quatro.cache_hits=" << stats.hits << "\nquatro.cache_misses=" << stats.misses
      << "\nquatro.cache_final_entries=" << stats.entries << '\n';
  }
  metadata << "pace_final_save_complete_wall_ms=" << static_cast<double>(pace.elapsedNs()) / 1e6
           << "\npace_capacity_exceeded=" << capacity_exceeded
           << "\npace_maximum_pending=" << maximum_pending
           << "\npace_capacity_acceptance=" << (!paced ? "NOT_PACED_NOT_ASSESSED" : capacity_exceeded ? "FAIL" : "PASS") << '\n';
  auto summary = openOutput(output / "summary.csv");
  writeCsv(summary, {"metric", "value"});
  for (const auto & entry : std::vector<std::pair<std::string, std::string>>{
      {"scope", scope}, {"mode", mode}, {"frames", text(last + 1U)}, {"events", text(event_count)},
      {"preload_setup_wall_ms", text(preload_setup_ms)},
      {"accepted_events", text(accepted_events)}, {"committed_loops", text(committed_loops)}, {"sc_cached", text(static_cast<bool>(cache))},
      {"save_final_update", "true"}, {"save_final_update_ms", text(save_update.elapsed_ms)},
      {"online_last_frame_snapshot_revision", text(save_update.before_revision)},
      {"save_final_snapshot_revision", text(save_update.after_revision)},
      {"pace_enabled", text(paced)}, {"pace_capacity_exceeded", text(capacity_exceeded)},
      {"pace_maximum_pending", text(maximum_pending)},
      {"pace_capacity_acceptance", !paced ? "NOT_PACED_NOT_ASSESSED" : capacity_exceeded ? "FAIL" : "PASS"},
      {"replay_wall_ms_including_csv_snapshot_io", text(elapsedMs(replay_started))}})
  {writeCsv(summary, {entry.first, entry.second});}
  const std::vector<std::pair<std::string, std::vector<double>>> distributions{
    {"process_wall_ms", process_times}, {"pipeline_frame_total_ms", pipeline_times}, {"scan_context_ms", scan_context_times},
    {"actual_start_wall_latency_ms", start_latencies}, {"actual_end_wall_latency_ms", end_latencies},
    {"actual_commit_wall_latency_ms", commit_latencies}};
  for (const auto & distribution : distributions) {
    if (distribution.second.empty()) {continue;}
    for (const auto & percentile : std::vector<std::pair<std::string, double>>{{"p50", 0.5}, {"p95", 0.95}, {"max", 1.0}}) {
      writeCsv(summary, {distribution.first + "_" + percentile.first, text(quantile(distribution.second, percentile.second))});
    }
  }
  for (const auto & reason : event_reasons) {writeCsv(summary, {"event_reason:" + reason.first, text(reason.second)});}
  for (const auto & reason : frame_reasons) {writeCsv(summary, {"frames_with_reason:" + reason.first, text(reason.second)});}
  summary.close(); events_csv.close(); frames_csv.close(); queries_csv.close(); constraints_csv.close();
  pace_csv.close(); commit_csv.close();
  metadata << "status=complete\n";
  metadata.close();
  std::cout << "replay complete scope=" << scope << " frames=" << last + 1U << " committed_loops=" << committed_loops << '\n';
  return capacity_exceeded ? 3 : 0;
}
}  // namespace loop_recall_replay

#ifndef LOOP_RECALL_REPLAY_NO_MAIN
int main(int argc, char ** argv)
try {return loop_recall_replay::run(argc, argv);}
catch (const std::exception & error) {
  std::cerr << "loop recall replay failed: " << error.what() << '\n';
  return 1;
}
#endif
