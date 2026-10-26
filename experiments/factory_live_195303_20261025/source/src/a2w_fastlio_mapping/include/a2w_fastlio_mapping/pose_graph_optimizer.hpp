#pragma once

#include <cstddef>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_mapping
{

enum class RobustKernel
{
  HUBER,
  CAUCHY,
};

struct PoseGraphConfig
{
  double prior_rotation_sigma_rad{1e-3};
  double prior_translation_sigma_m{1e-3};
  double odometry_rotation_sigma_rad{0.05};
  double odometry_translation_sigma_m{0.1};
  double loop_rotation_sigma_rad{0.05};
  double loop_translation_sigma_m{0.1};
  RobustKernel robust_kernel{RobustKernel::HUBER};
  double robust_kernel_scale{1.0};
  double relinearization_threshold{0.1};
  int relinearization_skip{1};
};

struct LoopConstraint
{
  std::uint64_t from_id{0U};
  std::uint64_t to_id{0U};
  a2w_fastlio_common::Pose3d relative_pose{};
  bool accepted{false};
};

struct GraphUpdate
{
  bool accepted{false};
  std::string reason{};
  std::size_t node_count{0U};
  std::size_t factor_count{0U};
};

struct OptimizedPose
{
  std::uint64_t id{0U};
  a2w_fastlio_common::Pose3d pose{};
};

struct OptimizedPoseSnapshot
{
  std::uint64_t revision{0U};
  std::vector<OptimizedPose> poses{};
};

struct LoopConstraintPreview
{
  GraphUpdate update{};
  OptimizedPoseSnapshot snapshot{};
};

// Exact GTSAM right-local Pose3 tangent order: rx,ry,rz [rad], tx,ty,tz [m].
// Diagnostic only: an empty observer (the default) retains no ledger factors.
struct FactorLedgerRecord
{
  std::string stage, factor_type;
  std::uint64_t factor_index{0U}, from_id{0U}, to_id{0U}, commit_sequence{0U};
  std::uint64_t factor_commit_sequence{0U};
  a2w_fastlio_common::Pose3d measurement{};
  std::array<double, 6> sigmas{}, raw_residual{}, gaussian_whitened{}, effective_whitened{};
  std::string robust_kernel{"none"};
  double robust_scale{0.0}, effective_robust_weight{1.0}, actual_cost{0.0};
};
using FactorLedgerObserver = std::function<void(const FactorLedgerRecord &)>;
void writeFactorLedgerCsvHeader(std::ostream & output);
void writeFactorLedgerCsv(std::ostream & output, const FactorLedgerRecord & record);

gtsam::Pose3 toGtsamPose(const a2w_fastlio_common::Pose3d & pose);
a2w_fastlio_common::Pose3d fromGtsamPose(const gtsam::Pose3 & pose);

class PoseGraphOptimizer
{
public:
  explicit PoseGraphOptimizer(PoseGraphConfig config);
  ~PoseGraphOptimizer();

  PoseGraphOptimizer(const PoseGraphOptimizer &) = delete;
  PoseGraphOptimizer & operator=(const PoseGraphOptimizer &) = delete;
  PoseGraphOptimizer(PoseGraphOptimizer &&) noexcept;
  PoseGraphOptimizer & operator=(PoseGraphOptimizer &&) noexcept;

  GraphUpdate addKeyFrame(
    std::uint64_t id, const a2w_fastlio_common::Pose3d & odom_pose);
  GraphUpdate addLoopConstraint(const LoopConstraint & constraint);
  // Opt-in transactional trial. Copies committed iSAM2 and pending state;
  // neither a rejected/failed nor a successful preview changes the live graph.
  LoopConstraintPreview previewLoopConstraint(const LoopConstraint & constraint) const;
  LoopConstraintPreview previewLoopSet(const std::vector<LoopConstraint> & constraints) const;
  // Opt-in replacement sinks/barrier are mandatory: failure preserves live state.
  GraphUpdate replaceLoopSet(const std::vector<LoopConstraint> & constraints,
    const std::function<void(const OptimizedPoseSnapshot &)> & commit_barrier = {});
  GraphUpdate update(bool allow_empty_update = false);
  OptimizedPoseSnapshot optimizedPoses() const;
  // Set before adding the first keyframe. Observer exceptions are isolated.
  void setFactorLedgerObserver(FactorLedgerObserver observer);
  void emitFactorLedger(const std::string & stage = "final_audit") const;

private:
  std::unique_ptr<PoseGraphOptimizer> rebuiltLoopSet(
    const std::vector<LoopConstraint> & constraints, FactorLedgerObserver observer) const;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace a2w_fastlio_mapping
