#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

gtsam::Key key(const std::uint64_t id)
{
  return gtsam::Symbol{'x', id};
}

gtsam::SharedNoiseModel diagonalNoise(double rotation_sigma, double translation_sigma)
{
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
    translation_sigma, translation_sigma, translation_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

bool validConfig(const PoseGraphConfig & config)
{
  const bool valid_kernel = config.robust_kernel == RobustKernel::HUBER ||
    config.robust_kernel == RobustKernel::CAUCHY;
  return valid_kernel &&
         config.prior_rotation_sigma_rad > 0.0 && config.prior_translation_sigma_m > 0.0 &&
         config.odometry_rotation_sigma_rad > 0.0 &&
         config.odometry_translation_sigma_m > 0.0 && config.loop_rotation_sigma_rad > 0.0 &&
         config.loop_translation_sigma_m > 0.0 && config.robust_kernel_scale > 0.0 &&
         config.relinearization_threshold > 0.0 && config.relinearization_skip > 0;
}

gtsam::SharedNoiseModel robustLoopNoise(const PoseGraphConfig & config)
{
  gtsam::noiseModel::mEstimator::Base::shared_ptr loss;
  switch (config.robust_kernel) {
    case RobustKernel::HUBER:
      loss = gtsam::noiseModel::mEstimator::Huber::Create(config.robust_kernel_scale);
      break;
    case RobustKernel::CAUCHY:
      loss = gtsam::noiseModel::mEstimator::Cauchy::Create(config.robust_kernel_scale);
      break;
    default:
      throw std::invalid_argument{"unsupported robust kernel"};
  }
  return gtsam::noiseModel::Robust::Create(
    loss, diagonalNoise(config.loop_rotation_sigma_rad, config.loop_translation_sigma_m));
}

}  // namespace

gtsam::Pose3 toGtsamPose(const a2w_fastlio_common::Pose3d & pose)
{
  if (!a2w_fastlio_common::isFinitePose(pose)) {
    throw std::invalid_argument{"cannot convert a non-finite pose"};
  }
  const auto quaternion = pose.rotation.normalized();
  return gtsam::Pose3{
    gtsam::Rot3::Quaternion(
      quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()),
    gtsam::Point3{pose.translation.x(), pose.translation.y(), pose.translation.z()}};
}

a2w_fastlio_common::Pose3d fromGtsamPose(const gtsam::Pose3 & pose)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation().toQuaternion().normalized();
  result.translation = pose.translation();
  if (!a2w_fastlio_common::isFinitePose(result)) {
    throw std::runtime_error{"GTSAM produced a non-finite pose"};
  }
  return result;
}

struct PoseGraphOptimizer::Impl
{
  explicit Impl(const PoseGraphConfig & input_config)
  : config{input_config}, isam{isamParameters(input_config)} {}

  static gtsam::ISAM2Params isamParameters(const PoseGraphConfig & config)
  {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = config.relinearization_threshold;
    parameters.relinearizeSkip = config.relinearization_skip;
    return parameters;
  }

  GraphUpdate state(bool accepted, std::string reason) const
  {
    return {accepted, std::move(reason), odometry_poses.size(), factor_count};
  }

  struct LedgerEntry
  {
    FactorLedgerRecord record;
    gtsam::NoiseModelFactor::shared_ptr factor;
  };

  void recordFactor(const char * type, std::uint64_t from, std::uint64_t to,
    const gtsam::Pose3 & measurement, double rotation_sigma, double translation_sigma)
  {
    if (!ledger_observer) {return;}
    FactorLedgerRecord record;
    record.factor_type = type; record.factor_index = factor_count;
    record.from_id = from; record.to_id = to;
    record.factor_commit_sequence = snapshot.revision + 1U;
    record.measurement = fromGtsamPose(measurement);
    record.sigmas = {rotation_sigma, rotation_sigma, rotation_sigma,
      translation_sigma, translation_sigma, translation_sigma};
    if (record.factor_type == "loop") {
      record.robust_kernel = config.robust_kernel == RobustKernel::HUBER ? "huber" : "cauchy";
      record.robust_scale = config.robust_kernel_scale;
    }
    ledger.push_back({record, boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(pending_factors.back())});
  }

  void emit(const LedgerEntry & entry, const gtsam::Values & values,
    const std::string & stage, std::uint64_t sequence) const
  {
    if (!ledger_observer) {return;}
    try {
      auto row = entry.record; row.stage = stage; row.commit_sequence = sequence;
      const auto raw = entry.factor->unwhitenedError(values);
      const auto gaussian = entry.factor->unweightedWhitenedError(values);
      const auto effective = entry.factor->whitenedError(values);
      for (std::size_t i = 0U; i < 6U; ++i) {
        row.raw_residual[i] = raw[i]; row.gaussian_whitened[i] = gaussian[i];
        row.effective_whitened[i] = effective[i];
      }
      // Measure the weight actually used by GTSAM WhitenSystem. Some installed
      // GTSAM versions' factor.weight(values) use a different residual norm.
      row.effective_robust_weight = gaussian.squaredNorm() > 0.0 ?
        effective.squaredNorm() / gaussian.squaredNorm() : 1.0;
      row.actual_cost = entry.factor->error(values);
      ledger_observer(row);
    } catch (...) {}  // A diagnostic sink cannot change graph decisions.
  }

  PoseGraphConfig config;
  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::Values pending_values;
  std::vector<a2w_fastlio_common::Pose3d> odometry_poses;
  std::size_t factor_count{0U};
  OptimizedPoseSnapshot snapshot;
  FactorLedgerObserver ledger_observer;
  std::vector<LedgerEntry> ledger;
  std::size_t committed_factor_count{0U};
};

PoseGraphOptimizer::PoseGraphOptimizer(PoseGraphConfig config)
{
  if (!validConfig(config)) {
    throw std::invalid_argument{"invalid pose-graph configuration"};
  }
  impl_ = std::make_unique<Impl>(config);
}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;
PoseGraphOptimizer::PoseGraphOptimizer(PoseGraphOptimizer &&) noexcept = default;
PoseGraphOptimizer & PoseGraphOptimizer::operator=(PoseGraphOptimizer &&) noexcept = default;

GraphUpdate PoseGraphOptimizer::addKeyFrame(
  const std::uint64_t id, const a2w_fastlio_common::Pose3d & odom_pose)
{
  if (!a2w_fastlio_common::isFinitePose(odom_pose)) {
    return impl_->state(false, "keyframe_pose_not_finite");
  }
  if (id != impl_->odometry_poses.size()) {
    return impl_->state(false, "keyframe_id_not_contiguous");
  }

  const auto current = toGtsamPose(odom_pose);
  if (id == 0U) {
    impl_->pending_factors.add(gtsam::PriorFactor<gtsam::Pose3>{
      key(id), current,
      diagonalNoise(
        impl_->config.prior_rotation_sigma_rad, impl_->config.prior_translation_sigma_m)});
    impl_->recordFactor("prior", id, id, current,
      impl_->config.prior_rotation_sigma_rad, impl_->config.prior_translation_sigma_m);
  } else {
    const auto previous = toGtsamPose(impl_->odometry_poses.back());
    impl_->pending_factors.add(gtsam::BetweenFactor<gtsam::Pose3>{
      key(id - 1U), key(id), previous.between(current),
      diagonalNoise(
        impl_->config.odometry_rotation_sigma_rad,
        impl_->config.odometry_translation_sigma_m)});
    impl_->recordFactor("odometry", id - 1U, id, previous.between(current),
      impl_->config.odometry_rotation_sigma_rad, impl_->config.odometry_translation_sigma_m);
  }
  impl_->pending_values.insert(key(id), current);
  impl_->odometry_poses.push_back(odom_pose);
  ++impl_->factor_count;
  return impl_->state(true, "queued");
}

GraphUpdate PoseGraphOptimizer::addLoopConstraint(const LoopConstraint & constraint)
{
  if (!constraint.accepted) {
    return impl_->state(false, "loop_not_validated");
  }
  if (constraint.from_id >= impl_->odometry_poses.size() ||
    constraint.to_id >= impl_->odometry_poses.size())
  {
    return impl_->state(false, "loop_keyframe_out_of_range");
  }
  if (constraint.from_id == constraint.to_id) {
    return impl_->state(false, "loop_keyframes_not_distinct");
  }
  if (!a2w_fastlio_common::isFinitePose(constraint.relative_pose)) {
    return impl_->state(false, "loop_pose_not_finite");
  }

  impl_->pending_factors.add(gtsam::BetweenFactor<gtsam::Pose3>{
    key(constraint.from_id), key(constraint.to_id), toGtsamPose(constraint.relative_pose),
    robustLoopNoise(impl_->config)});
  impl_->recordFactor("loop", constraint.from_id, constraint.to_id,
    toGtsamPose(constraint.relative_pose), impl_->config.loop_rotation_sigma_rad,
    impl_->config.loop_translation_sigma_m);
  ++impl_->factor_count;
  return impl_->state(true, "queued");
}

GraphUpdate PoseGraphOptimizer::update(const bool allow_empty_update)
{
  if (impl_->pending_factors.empty() && !allow_empty_update) {
    return impl_->state(false, "no_pending_factors");
  }

  try {
    if (impl_->ledger_observer) {
      auto before = impl_->isam.calculateEstimate();
      before.insert(impl_->pending_values);
      for (const auto & entry : impl_->ledger) {
        if (entry.record.factor_index >= impl_->committed_factor_count || entry.record.factor_type == "loop") {
          impl_->emit(entry, before, "pre_update", impl_->snapshot.revision + 1U);
        }
      }
    }
    impl_->isam.update(impl_->pending_factors, impl_->pending_values);
    const auto estimate = impl_->isam.calculateEstimate();
    OptimizedPoseSnapshot next;
    next.revision = impl_->snapshot.revision + 1U;
    next.poses.reserve(impl_->odometry_poses.size());
    for (std::uint64_t id = 0U; id < impl_->odometry_poses.size(); ++id) {
      next.poses.push_back({id, fromGtsamPose(estimate.at<gtsam::Pose3>(key(id)))});
    }
    impl_->pending_factors.resize(0U);
    impl_->pending_values.clear();
    impl_->snapshot = std::move(next);
    for (const auto & entry : impl_->ledger) {
      if (entry.record.factor_index >= impl_->committed_factor_count || entry.record.factor_type == "loop") {
        impl_->emit(entry, estimate, "committed", impl_->snapshot.revision);
      }
    }
    impl_->committed_factor_count = impl_->factor_count;
    return impl_->state(true, "updated");
  } catch (const std::exception & error) {
    return impl_->state(false, std::string{"gtsam_update_failed:"} + error.what());
  }
}

LoopConstraintPreview PoseGraphOptimizer::previewLoopConstraint(
  const LoopConstraint & constraint) const
{
  try {
    PoseGraphOptimizer trial{impl_->config};
    // ISAM2's BayesTree copy constructor clones its mutable clique tree.
    // Nonlinear factors/noise models are shared immutable measurements; Values,
    // delta/relinearization counters, pending graph, raw poses and snapshot copy.
    trial.impl_ = std::make_unique<Impl>(*impl_);
    trial.impl_->ledger_observer = {};
    trial.impl_->ledger.clear();
    const auto queued = trial.addLoopConstraint(constraint);
    if (!queued.accepted) {return {queued, impl_->snapshot};}
    const auto update = trial.update();
    return {update, update.accepted ? trial.optimizedPoses() : impl_->snapshot};
  } catch (const std::exception & error) {
    return {impl_->state(false, std::string{"loop_preview_failed:"} + error.what()), impl_->snapshot};
  }
}

OptimizedPoseSnapshot PoseGraphOptimizer::optimizedPoses() const
{
  return impl_->snapshot;
}

void PoseGraphOptimizer::setFactorLedgerObserver(FactorLedgerObserver observer)
{
  if (impl_->factor_count != 0U) {throw std::logic_error{"set ledger observer before adding keyframes"};}
  impl_->ledger_observer = std::move(observer);
}

void PoseGraphOptimizer::emitFactorLedger(const std::string & stage) const
{
  if (!impl_->ledger_observer) {return;}
  const auto estimate = impl_->isam.calculateEstimate();
  for (const auto & entry : impl_->ledger) {impl_->emit(entry, estimate, stage, impl_->snapshot.revision);}
}

void writeFactorLedgerCsvHeader(std::ostream & output)
{
  output << "stage,factor_type,factor_index,from_id,to_id,commit_sequence,factor_commit_sequence,tangent_convention,measurement_tx,measurement_ty,measurement_tz,measurement_qx,measurement_qy,measurement_qz,measurement_qw,robust_kernel,robust_scale,effective_robust_weight,actual_gtsam_cost";
  for (const auto * prefix : {"sigma", "information_diagonal", "raw", "gaussian_whitened", "effective_whitened"}) {
    for (const auto * axis : {"rx", "ry", "rz", "tx", "ty", "tz"}) {output << ',' << prefix << '_' << axis;}
  }
  output << '\n';
}

void writeFactorLedgerCsv(std::ostream & output, const FactorLedgerRecord & row)
{
  const auto & p = row.measurement;
  output << std::setprecision(17) << row.stage << ',' << row.factor_type << ',' << row.factor_index
    << ',' << row.from_id << ',' << row.to_id << ',' << row.commit_sequence
    << ',' << row.factor_commit_sequence
    << ",right_local_rotation_rad_translation_m," << p.translation.x() << ',' << p.translation.y()
    << ',' << p.translation.z() << ',' << p.rotation.x() << ',' << p.rotation.y() << ',' << p.rotation.z()
    << ',' << p.rotation.w() << ',' << row.robust_kernel << ',' << row.robust_scale << ','
    << row.effective_robust_weight << ',' << row.actual_cost;
  for (const auto value : row.sigmas) {output << ',' << value;}
  for (const auto value : row.sigmas) {output << ',' << 1.0 / (value * value);}
  for (const auto & values : {row.raw_residual, row.gaussian_whitened, row.effective_whitened}) {
    for (const auto value : values) {output << ',' << value;}
  }
  output << '\n';
}

}  // namespace a2w_fastlio_mapping
