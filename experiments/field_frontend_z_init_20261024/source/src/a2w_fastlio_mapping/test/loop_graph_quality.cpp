// Standalone offline same-factor graph audit. Does not alter production optimizer.
#define LOOP_CONSTRAINT_ABLATION_NO_MAIN
#include "loop_constraint_ablation.cpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <chrono>
#include <limits>
#include <numeric>

namespace graph_quality
{
namespace fs = std::filesystem;
namespace mapping = a2w_fastlio_mapping;
using ablation::Pose3d;
using ablation::Measurement;

struct FactorAudit
{
  gtsam::Vector residual, gaussian_whitened, effective_whitened;
  double weight{0}, factor_weight_api{0}, actual_cost{0}, gaussian_cost{0}, canonical_robust_cost{0};
};
FactorAudit auditFactor(const gtsam::NoiseModelFactor & factor, const gtsam::Values & values)
{
  FactorAudit row;
  row.residual = factor.unwhitenedError(values);
  row.gaussian_whitened = factor.unweightedWhitenedError(values);
  row.effective_whitened = factor.whitenedError(values);
  row.factor_weight_api = factor.weight(values);
  // This installed GTSAM's weight(Values) sees native residuals, not Gaussian
  // whitened ones. Measure the weight actually used by WhitenSystem instead.
  row.weight = row.gaussian_whitened.squaredNorm() > 0 ?
    row.effective_whitened.squaredNorm() / row.gaussian_whitened.squaredNorm() : 1.;
  row.actual_cost = factor.error(values);
  row.gaussian_cost = .5 * row.gaussian_whitened.squaredNorm();
  const auto robust = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(factor.noiseModel());
  row.canonical_robust_cost = robust ? robust->robust()->loss(row.gaussian_whitened.norm()) : row.gaussian_cost;
  return row;
}
gtsam::SharedNoiseModel diagonal(double rotation_sigma, double translation_sigma)
{
  if (!std::isfinite(rotation_sigma) || !std::isfinite(translation_sigma) ||
    rotation_sigma <= 0 || translation_sigma <= 0) {throw std::runtime_error("invalid sigma");}
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
    translation_sigma, translation_sigma, translation_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}
gtsam::SharedNoiseModel loopNoise(const mapping::PoseGraphConfig & config)
{
  gtsam::noiseModel::mEstimator::Base::shared_ptr loss;
  if (config.robust_kernel == mapping::RobustKernel::HUBER) {
    loss = gtsam::noiseModel::mEstimator::Huber::Create(config.robust_kernel_scale);
  } else if (config.robust_kernel == mapping::RobustKernel::CAUCHY) {
    loss = gtsam::noiseModel::mEstimator::Cauchy::Create(config.robust_kernel_scale);
  } else {throw std::runtime_error("unsupported kernel");}
  return gtsam::noiseModel::Robust::Create(loss,
    diagonal(config.loop_rotation_sigma_rad, config.loop_translation_sigma_m));
}
gtsam::Key key(std::uint64_t id) {return gtsam::Symbol{'x', id};}

class ExactBetweenFactor : public gtsam::BetweenFactor<gtsam::Pose3>
{
public:
  using gtsam::BetweenFactor<gtsam::Pose3>::BetweenFactor;
  gtsam::Vector evaluateError(const gtsam::Pose3 & a, const gtsam::Pose3 & b,
    boost::optional<gtsam::Matrix &> ha = boost::none,
    boost::optional<gtsam::Matrix &> hb = boost::none) const override
  {
    // Same measurement, residual and noise as BetweenFactor. Only restore the
    // Local chart chain rule that its default fast Jacobian omits. This is the
    // diagnostic equivalent of GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR, confined to
    // this class (no global macro/ABI or production optimizer change).
    gtsam::Matrix66 ba,bb,local;
    const auto predicted = a.between(b,ha ? &ba : nullptr,hb ? &bb : nullptr);
    const auto error = measured().localCoordinates(predicted,boost::none,(ha||hb) ? &local : nullptr);
    if(ha){*ha=local*ba;} if(hb){*hb=local*bb;}
    return error;
  }
};

struct GraphData
{
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values raw;
};
GraphData buildGraph(const std::vector<Pose3d> & raw, const std::vector<Measurement> & loops,
  const mapping::PoseGraphConfig & config, const std::vector<double> & loop_sigma_multipliers = {})
{
  if (raw.empty()) {throw std::runtime_error("empty raw odometry");}
  if(!loop_sigma_multipliers.empty() && loop_sigma_multipliers.size()!=loops.size()) {
    throw std::runtime_error("per-loop noise multiplier count mismatch");
  }
  GraphData data;
  for (std::uint64_t i = 0U; i < raw.size(); ++i) {
    const auto pose = mapping::toGtsamPose(raw[i]);
    data.raw.insert(key(i), pose);
    if (i == 0U) {
      data.factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key(i), pose,
        diagonal(config.prior_rotation_sigma_rad, config.prior_translation_sigma_m));
    } else {
      data.factors.emplace_shared<ExactBetweenFactor>(key(i - 1), key(i),
        mapping::toGtsamPose(raw[i - 1]).between(pose),
        diagonal(config.odometry_rotation_sigma_rad, config.odometry_translation_sigma_m));
    }
  }
  std::size_t loop_index=0U;
  for (const auto & loop : loops) {
    if (loop.current >= raw.size() || loop.history >= loop.current) {
      throw std::runtime_error("invalid measured loop endpoints");
    }
    auto edge_config=config;
    if(!loop_sigma_multipliers.empty()){
      const double multiplier=loop_sigma_multipliers[loop_index];
      if(!std::isfinite(multiplier)||multiplier<1.){throw std::runtime_error("invalid correlation noise multiplier");}
      edge_config.loop_rotation_sigma_rad*=multiplier;
      edge_config.loop_translation_sigma_m*=multiplier;
    }
    data.factors.emplace_shared<ExactBetweenFactor>(key(loop.current), key(loop.history),
      mapping::toGtsamPose(loop.pose), loopNoise(edge_config));
    ++loop_index;
  }
  return data;
}

gtsam::Vector gradient(const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values)
{
  return graph.linearize(values)->gradientAtZero().vector();
}

struct SolveResult
{
  gtsam::Values values;
  double initial_error{0}, final_error{0}, gradient_inf{1};
  std::size_t iterations{0};
  std::string stop{"not_implemented"};
  bool converged{false};
};
SolveResult solve(const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & initial,
  std::ostream * trace = nullptr)
{
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = 1000;
  params.relativeErrorTol = 0.; params.absoluteErrorTol = 0.;
  params.lambdaUpperBound = 1e16;
  gtsam::LevenbergMarquardtOptimizer optimizer{graph, initial, params};
  SolveResult result;
  result.initial_error = graph.error(initial);
  if (trace) {*trace << "iteration,error,gradient_l2,gradient_inf,step_l2,lambda,relative_error_drop\n";}
  for (std::size_t iteration = 0; iteration <= params.maxIterations; ++iteration) {
    const auto g = gradient(graph, optimizer.values());
    const double error = graph.error(optimizer.values());
    const double inf = g.lpNorm<Eigen::Infinity>();
    if (!std::isfinite(error) || !g.allFinite()) {throw std::runtime_error("nonfinite batch objective/gradient");}
    if (iteration == 0 && trace) {*trace << "0," << error << ',' << g.norm() << ',' << inf << ",0," << optimizer.lambda() << ",0\n";}
    if (inf < 1e-6) {result.stop = "gradient_tolerance"; result.converged = true; break;}
    if (iteration == params.maxIterations) {result.stop = "iteration_limit_NOT_CONVERGED"; break;}
    const auto before = optimizer.values();
    optimizer.iterate();
    const auto after_gradient = gradient(graph, optimizer.values());
    const double after_error = graph.error(optimizer.values());
    const double step = before.localCoordinates(optimizer.values()).norm();
    const double relative_drop = (error - after_error) / std::max(1., std::abs(error));
    ++result.iterations;
    if (trace) {*trace << result.iterations << ',' << after_error << ',' << after_gradient.norm() << ','
                     << after_gradient.lpNorm<Eigen::Infinity>() << ',' << step << ',' << optimizer.lambda()
                     << ',' << relative_drop << '\n';}
    if (after_error > error + 1e-10 * std::max(1., error)) {throw std::runtime_error("LM objective increased");}
    if (step < 1e-10 || optimizer.lambda() >= params.lambdaUpperBound) {
      result.converged = after_gradient.lpNorm<Eigen::Infinity>() < 1e-4;
      result.stop = result.converged ? "stalled_step_small_gradient" : "stalled_NOT_CONVERGED";
      break;
    }
  }
  result.values = optimizer.values();
  result.final_error = graph.error(result.values);
  result.gradient_inf = gradient(graph, result.values).lpNorm<Eigen::Infinity>();
  return result;
}

std::set<std::uint64_t> support(std::uint64_t center, std::uint64_t current,
  std::size_t before = 5U, std::size_t after = 5U)
{
  if (center > current) {throw std::runtime_error("submap cannot include future center");}
  std::set<std::uint64_t> result;
  const auto low = center > before ? center - before : 0U;
  const auto high = center + std::min<std::uint64_t>(after, current - center);
  for (auto i = low; i <= high; ++i) {result.insert(i);}
  return result;
}
double jaccard(const std::set<std::uint64_t> & a, const std::set<std::uint64_t> & b)
{
  std::vector<std::uint64_t> overlap, combined;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(overlap));
  std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(combined));
  return combined.empty() ? 0. : static_cast<double>(overlap.size()) / combined.size();
}

struct CorrelationBudgetEdge
{
  std::size_t index{0U}, past_loops{0U}, overlapping_past_targets{0U};
  double jaccard_sum{0.}, sigma_multiplier{1.};
};
std::vector<CorrelationBudgetEdge> correlationBudget(const std::vector<Measurement> & loops)
{
  std::vector<CorrelationBudgetEdge> rules;
  std::vector<std::set<std::uint64_t>> committed_targets;
  for(std::size_t index=0;index<loops.size();++index){
    const auto &loop=loops[index];
    if(loop.history>=loop.current || (index && loop.current<=loops[index-1].current)) {
      throw std::runtime_error("correlation rule requires source-time ordered committed past-directed loops");
    }
    const auto target=support(loop.history,loop.current);
    CorrelationBudgetEdge rule;rule.index=index;rule.past_loops=committed_targets.size();
    for(const auto &past:committed_targets){
      const auto overlap=jaccard(target,past);rule.jaccard_sum+=overlap;
      if(overlap>0){++rule.overlapping_past_targets;}
    }
    rule.sigma_multiplier=std::sqrt(1.+rule.jaccard_sum);
    rules.push_back(rule);committed_targets.push_back(target);
  }
  return rules;
}
void protectOutput(const fs::path & output, const std::vector<fs::path> & inputs)
{
  const auto destination = fs::weakly_canonical(output);
  const auto within = [](const fs::path & child, const fs::path & parent) {
      auto i = child.begin();
      for (auto j = parent.begin(); j != parent.end(); ++j, ++i) {
        if (i == child.end() || *i != *j) {return false;}
      }
      return true;
    };
  if (fs::exists(destination) || fs::is_symlink(destination)) {
    throw std::runtime_error("refusing existing graph audit output");
  }
  for (const auto & input : inputs) {
    const auto source = fs::canonical(input);
    if (within(destination, source) || within(source, destination)) {
      throw std::runtime_error("graph audit output overlaps read-only input");
    }
  }
}

std::ofstream outputFile(const fs::path & path)
{
  if (fs::exists(path)) {throw std::runtime_error("refusing to overwrite " + path.string());}
  std::ofstream output(path);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output << std::setprecision(17);
  return output;
}

gtsam::Values readValues(const fs::path & path, std::size_t count)
{
  gtsam::Values values;
  for (const auto & row : ablation::readCsv(path)) {
    if (ablation::id(row.at("id")) != values.size()) {throw std::runtime_error("noncontiguous saved poses");}
    values.insert(key(values.size()), mapping::toGtsamPose(ablation::readPose(row)));
  }
  if (values.size() != count) {throw std::runtime_error("saved pose count does not match raw");}
  return values;
}

void saveValues(const fs::path & path, const gtsam::Values & values)
{
  mapping::OptimizedPoseSnapshot snapshot;
  for (std::size_t i = 0; i < values.size(); ++i) {
    snapshot.poses.push_back({i, mapping::fromGtsamPose(values.at<gtsam::Pose3>(key(i)))});
  }
  ablation::save(path, snapshot);
}

std::pair<double,double> maximumPoseChange(const gtsam::Values & a, const gtsam::Values & b)
{
  double translation = 0., rotation = 0.;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const auto delta = a.at<gtsam::Pose3>(key(i)).between(b.at<gtsam::Pose3>(key(i)));
    translation = std::max(translation, delta.translation().norm());
    rotation = std::max(rotation, gtsam::Rot3::Logmap(delta.rotation()).norm());
  }
  return {translation,rotation};
}

double numericalObjectiveGradientInf(const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values)
{
  // Independent central difference of the ACTUAL nonlinear objective, not IRLS
  // surrogate. Evaluate only incident factors; no approximation to graph cost.
  std::map<gtsam::Key,std::vector<gtsam::NonlinearFactor::shared_ptr>> incident;
  for (const auto & factor : graph) {for (auto k : factor->keys()) {incident[k].push_back(factor);}}
  auto probe = values;
  double inf = 0.;
  constexpr double h = 1e-6;
  for (const auto & entry : incident) {
    const auto pose = values.at<gtsam::Pose3>(entry.first);
    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero(); delta[d] = h;
      probe.update(entry.first, pose.retract(delta));
      double plus = 0.; for (const auto & factor : entry.second) {plus += factor->error(probe);}
      probe.update(entry.first, pose.retract(-delta));
      double minus = 0.; for (const auto & factor : entry.second) {minus += factor->error(probe);}
      inf = std::max(inf, std::abs(plus - minus) / (2. * h));
    }
    probe.update(entry.first,pose);
  }
  return inf;
}

void factorAuditCsv(const fs::path & path, const GraphData & data, const gtsam::Values & values,
  const std::vector<Pose3d> & raw, const std::vector<Measurement> & loops)
{
  auto output = outputFile(path);
  output << "factor_index,type,i,j,log_rx_rad,log_ry_rad,log_rz_rad,log_tx_m,log_ty_m,log_tz_m";
  for (const auto & prefix : {"factor_native_", "gaussian_whitened_", "noise_whitened_"}) {
    for (const char * axis : {"rx","ry","rz","tx","ty","tz"}) {output << ',' << prefix << axis;}
  }
  output << ",log_native_difference_norm,gaussian_whitened_norm,actual_irls_block_weight,factor_weight_api,actual_factor_cost,canonical_rho_gaussian_norm,gaussian_reference_cost,irls_quadratic_surrogate\n";
  double sum = 0.;
  for (std::size_t k = 0; k < data.factors.size(); ++k) {
    const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(data.factors[k]);
    const auto row = auditFactor(*factor,values);
    std::uint64_t i = 0, j = 0;
    gtsam::Pose3 error;
    std::string type;
    if (k == 0U) {
      type = "prior"; error = mapping::toGtsamPose(raw[0]).between(values.at<gtsam::Pose3>(key(0)));
    } else {
      const bool odom = k < raw.size(); type = odom ? "odometry" : "loop";
      i = odom ? k - 1U : loops.at(k - raw.size()).current;
      j = odom ? k : loops.at(k - raw.size()).history;
      const auto measurement = odom ? mapping::toGtsamPose(raw[i]).between(mapping::toGtsamPose(raw[j])) :
        mapping::toGtsamPose(loops.at(k - raw.size()).pose);
      error = measurement.between(values.at<gtsam::Pose3>(key(i)).between(values.at<gtsam::Pose3>(key(j))));
    }
    const auto log = gtsam::Pose3::Logmap(error);
    output << k << ',' << type << ',' << i << ',' << j;
    for (const auto & vector : {gtsam::Vector{log},row.residual,row.gaussian_whitened,row.effective_whitened}) {
      for (int axis = 0; axis < 6; ++axis) {output << ',' << vector[axis];}
    }
    output << ',' << (log-row.residual).norm() << ',' << row.gaussian_whitened.norm() << ',' << row.weight
           << ',' << row.factor_weight_api << ',' << row.actual_cost << ',' << row.canonical_robust_cost
           << ',' << row.gaussian_cost << ',' << .5*row.effective_whitened.squaredNorm() << '\n';
    sum += row.actual_cost;
  }
  if (std::abs(sum - data.factors.error(values)) > 1e-9 * std::max(1.,sum)) {
    throw std::runtime_error("factor costs do not sum to graph.error");
  }
}

void correlationCsv(const fs::path & path, const std::vector<Measurement> & loops,
  const std::vector<Pose3d> & raw, const std::vector<std::int64_t> & stamps)
{
  auto output = outputFile(path);
  output << "loop_a,loop_b,current_a,current_b,history_a,history_b,source_jaccard,target_jaccard,all_support_jaccard,any_shared_scan,shared_endpoint,raw_source_translation_m,raw_source_rotation_rad,source_time_gap_s\n";
  for (std::size_t a = 0; a < loops.size(); ++a) {
    for (std::size_t b = a+1; b < loops.size(); ++b) {
      const auto & x = loops[a]; const auto & y = loops[b];
      const auto sx = support(x.current,x.current), sy = support(y.current,y.current);
      const auto tx = support(x.history,x.current), ty = support(y.history,y.current);
      auto ux = sx; ux.insert(tx.begin(),tx.end());
      auto uy = sy; uy.insert(ty.begin(),ty.end());
      const auto delta = mapping::toGtsamPose(raw[x.current]).between(mapping::toGtsamPose(raw[y.current]));
      output << a << ',' << b << ',' << x.current << ',' << y.current << ',' << x.history << ',' << y.history
             << ',' << jaccard(sx,sy) << ',' << jaccard(tx,ty) << ',' << jaccard(ux,uy)
             << ',' << (jaccard(ux,uy)>0) << ',' << (x.current==y.current || x.history==y.history || x.current==y.history || x.history==y.current)
             << ',' << delta.translation().norm() << ',' << gtsam::Rot3::Logmap(delta.rotation()).norm()
             << ',' << static_cast<double>(stamps[y.current]-stamps[x.current])/1e9 << '\n';
    }
  }
}

std::string readBytes(const fs::path & path)
{
  std::ifstream input(path,std::ios::binary);
  if(!input){throw std::runtime_error("cannot read immutable input");}
  return {std::istreambuf_iterator<char>{input},std::istreambuf_iterator<char>{}};
}

int run(int argc, char ** argv)
{
  if (argc != 5 && argc != 6) {throw std::runtime_error("usage: loop_graph_quality BUNDLE ACTUAL_LOOP_CSV SAVED_ISAM_POSES NEW_OUTPUT_DIR [correlation-budget-v1]");}
  const bool correlation_budget=argc==6;
  if(correlation_budget && std::string{argv[5]}!="correlation-budget-v1") {
    throw std::runtime_error("unknown graph experiment; only predeclared correlation-budget-v1 is supported");
  }
  const auto bundle=fs::canonical(argv[1]), loop_path=fs::canonical(argv[2]), saved_path=fs::canonical(argv[3]);
  const auto output=fs::weakly_canonical(argv[4]);
  protectOutput(output,{bundle,loop_path,saved_path});
  const auto raw_path=bundle/"keyframes/index.csv", config_path=bundle/"config/mapping_effective.yaml";
  std::map<fs::path,std::string> immutable;
  for(const auto &path:{raw_path,config_path,loop_path,saved_path}){immutable.emplace(path,readBytes(path));}
  std::vector<Pose3d> raw;std::vector<std::int64_t> stamps;
  for(const auto &row:ablation::readCsv(raw_path)){
    if(ablation::id(row.at("id"))!=raw.size()){throw std::runtime_error("noncontiguous raw input");}
    raw.push_back(ablation::readPose(row));stamps.push_back(std::stoll(row.at("stamp_ns")));
  }
  std::vector<Measurement> loops;
  for(const auto &row:ablation::readCsv(loop_path)){
    Measurement measured{ablation::id(row.at("current_id")),ablation::id(row.at("candidate_id")),ablation::readPose(row,"relative_")};
    if(measured.current>=raw.size() || measured.history>=measured.current ||
      (!loops.empty() && measured.current<=loops.back().current)){throw std::runtime_error("invalid chronological measured loops");}
    loops.push_back(measured);
  }
  const auto config=ablation::readConfig(config_path);
  const auto rules=correlation_budget?correlationBudget(loops):std::vector<CorrelationBudgetEdge>{};
  std::vector<double> loop_multipliers;
  for(const auto &rule:rules){loop_multipliers.push_back(rule.sigma_multiplier);}
  const auto data=buildGraph(raw,loops,config,loop_multipliers);
  const auto saved=readValues(saved_path,raw.size());
  if(!fs::create_directories(output)){throw std::runtime_error("output reservation failed");}
  auto metadata=outputFile(output/"run_metadata.txt");
  metadata << "scope=OFFLINE_SAME_FIXED_MEASUREMENT_GRAPH_NOT_ONLINE_ORACLE\nindependent_ground_truth=false\n"
           << "raw_odometry_and_original_prior_odometry_noise_preserved=true\nloop_measurements_changed=false\n"
           << "original_loop_noise_preserved=" << (!correlation_budget) << "\nloop_noise_correlation_scaled=" << correlation_budget << '\n'
           << "noise_policy=" << (correlation_budget?"correlation_budget_v1_sqrt_1_plus_sum_past_target_jaccard":"original_unscaled") << '\n'
           << "isam_reference=frozen_original_unscaled_solution_not_reoptimized_for_scaled_noise\n"
           << "residual_order=rotation_rad_xyz_then_translation_m_xyz\n"
           << "actual_cost=installed_GTSAM_factor.error_and_graph.error\n"
           << "canonical_robust_cost=reference_rho_of_Gaussian_whitened_norm_not_substituted_into_optimizer\n"
           << "actual_irls_weight=norm_noise.whiten_squared_div_norm_gaussian.whiten_squared\n"
           << "factor_weight_api_recorded_separately_may_use_native_unwhitened_norm=true\n"
           << "linearization_gradient=GTSAM_linearize_gradientAtZero_not_assumed_actual_objective_gradient\n"
           << "batch_between_jacobian=exact_Local_chain_rule_same_residual_measurement_noise_factor_cost\n"
           << "production_isam_jacobian_unchanged=true\n"
           << "numerical_actual_objective_gradient=central_difference_retract_step_1e-6_all_pose_dimensions\n"
           << "convergence=linearization_gradient_inf_lt_1e-6_or_stall_with_lt_1e-4_AND_numerical_actual_gradient_inf_lt_1e-4\n"
           << "lm_max_iterations=1000\nlm_lambda_upper_bound=1e16\n"
           << "correlation_support=local_map_before5_after5_clipped_at_actual_source_frame\n"
           << "correlation_limit=shared_frame_ids_raw_motion_are_proxies_NOT_covariance_or_independence_truth\n"
           << "bundle=" << bundle.string() << "\nactual_loops=" << loop_path.string() << "\nsaved_isam=" << saved_path.string()
           << "\nframes=" << raw.size() << "\nloops=" << loops.size() << "\nfactors=" << data.factors.size() << '\n';
#ifdef GTSAM_POSE3_EXPMAP
  metadata << "GTSAM_POSE3_EXPMAP=true\n";
#else
  metadata << "GTSAM_POSE3_EXPMAP=false_LOG_AND_NATIVE_MAY_DIFFER\n";
#endif
  metadata << "noise.prior_rotation_sigma_rad=" << config.prior_rotation_sigma_rad << "\nnoise.prior_translation_sigma_m=" << config.prior_translation_sigma_m
           << "\nnoise.odom_rotation_sigma_rad=" << config.odometry_rotation_sigma_rad << "\nnoise.odom_translation_sigma_m=" << config.odometry_translation_sigma_m
           << "\nnoise.loop_rotation_sigma_rad=" << config.loop_rotation_sigma_rad << "\nnoise.loop_translation_sigma_m=" << config.loop_translation_sigma_m
           << "\nnoise.robust_kernel=" << (config.robust_kernel==mapping::RobustKernel::HUBER?"huber":"cauchy") << "\nnoise.robust_scale=" << config.robust_kernel_scale << '\n';
  if(correlation_budget){
    auto policy=outputFile(output/"correlation_budget_rule.json");
    policy << "{\"rule_id\":\"correlation_budget_v1\",\"acceptance\":\"NOT_ACCEPTED_EXPERIMENT\","
           << "\"predeclared_formula\":\"sigma_multiplier=sqrt(1+sum_jaccard(new_target,past_committed_target))\","
           << "\"support\":\"target_center_before5_after5_clipped_at_actual_source_id\","
           << "\"past_only_weight_assignment\":true,\"covariance_truth_claim\":false,\"future_geometry_oracle\":false,"
           << "\"uniform_loop_scaling\":false,\"id_blacklist\":false,\"rotation_and_translation_sigmas_scaled_equally\":true,"
           << "\"prior_odometry_unchanged\":true,\"measurements_unchanged\":true,"
           << "\"batch_future_derived_offline_not_online_replay\":true,\"subsequent_rule_tuning\":false}\n";
    auto edges=outputFile(output/"correlation_budget_edges.csv");
    edges << "loop_index,current,history,past_committed_loops,target_first,target_last,target_count,overlapping_past_targets,jaccard_sum,sigma_multiplier,base_rotation_sigma_rad,base_translation_sigma_m,scaled_rotation_sigma_rad,scaled_translation_sigma_m,nominal_diagonal_information_multiplier\n";
    auto terms=outputFile(output/"correlation_budget_terms.csv");
    terms << "new_loop_index,past_loop_index,new_current,past_current,new_target_center,past_target_center,jaccard\n";
    std::size_t scaled=0U;double nominal_information=0.;
    for(const auto &rule:rules){
      const auto &loop=loops.at(rule.index);const auto target=support(loop.history,loop.current);
      const double information=1./(rule.sigma_multiplier*rule.sigma_multiplier);
      edges << rule.index << ',' << loop.current << ',' << loop.history << ',' << rule.past_loops << ','
            << *target.begin() << ',' << *target.rbegin() << ',' << target.size() << ',' << rule.overlapping_past_targets
            << ',' << rule.jaccard_sum << ',' << rule.sigma_multiplier << ',' << config.loop_rotation_sigma_rad << ','
            << config.loop_translation_sigma_m << ',' << config.loop_rotation_sigma_rad*rule.sigma_multiplier << ','
            << config.loop_translation_sigma_m*rule.sigma_multiplier << ',' << information << '\n';
      for(std::size_t past=0;past<rule.index;++past){
        const auto &old=loops[past];
        terms << rule.index << ',' << past << ',' << loop.current << ',' << old.current << ',' << loop.history << ',' << old.history
              << ',' << jaccard(target,support(old.history,old.current)) << '\n';
      }
      if(rule.sigma_multiplier>1.){++scaled;}nominal_information+=information;
    }
    metadata << "correlation_budget.scaled_edges=" << scaled << "\ncorrelation_budget.unchanged_edges=" << loops.size()-scaled
             << "\ncorrelation_budget.nominal_diagonal_information_multiplier_sum=" << nominal_information
             << "\ncorrelation_budget.information_sum_is_NOT_effective_independent_observation_count=true\n";
  }
  // Independent reconstruction uses the unchanged production backend and source-time
  // injection. Batch has the same graph but is explicitly future-derived/offline.
  mapping::PoseGraphOptimizer replay{config};
  for(std::uint64_t i=0;i<raw.size();++i){
    ablation::require(replay.addKeyFrame(i,raw[i]));
    for(const auto &loop:loops){if(loop.current==i){ablation::require(replay.addLoopConstraint({i,loop.history,loop.pose,true}));}}
    ablation::require(replay.update());
  }
  ablation::require(replay.update(true));
  ablation::save(output/"isam_replayed.csv",replay.optimizedPoses());
  const auto reproduced=readValues(output/"isam_replayed.csv",raw.size());
  const auto match=maximumPoseChange(saved,reproduced);
  metadata << "isam_reproduction_max_translation_m=" << match.first << "\nisam_reproduction_max_rotation_rad=" << match.second << '\n';
  if(match.first>1e-8 || match.second>1e-8){throw std::runtime_error("saved iSAM solution not reproduced by same graph/noise");}
  auto solutions=outputFile(output/"solutions.csv");
  solutions << "solution,initial_error,final_error,linearization_gradient_inf,numerical_objective_gradient_inf,iterations,converged,stop_reason,max_translation_change_vs_isam_m,max_rotation_change_vs_isam_rad\n";
  const auto writeSolution=[&](const std::string &name,const gtsam::Values &poses,double initial_error,std::size_t iterations,bool converged,const std::string &stop){
      const double numerical=numericalObjectiveGradientInf(data.factors,poses);
      converged=converged && numerical<1e-4;
      const auto change=maximumPoseChange(saved,poses);
      solutions << name << ',' << initial_error << ',' << data.factors.error(poses) << ','
                << gradient(data.factors,poses).lpNorm<Eigen::Infinity>() << ',' << numerical << ',' << iterations
                << ',' << converged << ',' << (numerical>=1e-4?stop+"_actual_gradient_NOT_CONVERGED":stop)
                << ',' << change.first << ',' << change.second << '\n';
      saveValues(output/(name+".csv"),poses);factorAuditCsv(output/(name+"_factors.csv"),data,poses,raw,loops);
    };
  writeSolution("isam",saved,data.factors.error(saved),0,false,"frozen_production_snapshot_not_convergence_claim");
  for(const auto &name:{std::string{"batch_warm"},std::string{"batch_raw"}}){
    auto trace=outputFile(output/(name+"_iterations.csv"));
    const auto result=solve(data.factors,name=="batch_warm"?saved:data.raw,&trace);
    writeSolution(name,result.values,result.initial_error,result.iterations,result.converged,result.stop);
    std::cout << name << " initial=" << result.initial_error << " final=" << result.final_error
              << " linearization_gradient_inf=" << result.gradient_inf << " iterations=" << result.iterations << " stop=" << result.stop << '\n' << std::flush;
  }
  correlationCsv(output/"loop_correlation_proxy.csv",loops,raw,stamps);
  auto inputs=outputFile(output/"input_immutability.csv");inputs<<"path,size_bytes,exact_contents_unchanged\n";
  for(const auto &entry:immutable){
    if(readBytes(entry.first)!=entry.second){throw std::runtime_error("audit input changed while running");}
    inputs<<entry.first.string()<<','<<entry.second.size()<<",true\n";
  }
  metadata << "inputs_exact_contents_unchanged=true\nstatus=complete\n";
  return 0;
}
}  // namespace graph_quality

#ifndef LOOP_GRAPH_QUALITY_NO_MAIN
int main(int argc, char ** argv)
try {return graph_quality::run(argc, argv);}
catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
#endif
