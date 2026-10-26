#define LOOP_GRAPH_QUALITY_NO_MAIN
#include "loop_graph_quality.cpp"
#include <chrono>
#include <gtest/gtest.h>

namespace graph_quality
{
TEST(LoopGraphQuality, WhitenRotationFirstThenTranslationBeforeBlockHuberWeightAndCost)
{
  mapping::PoseGraphConfig config;
  const auto model = loopNoise(config);
  ASSERT_TRUE(model);
  gtsam::Vector6 residual;
  residual << .1, 0, 0, 0, .4, 0;
  gtsam::Values values;
  values.insert(0U, gtsam::Pose3{});
  values.insert(1U, gtsam::Pose3::Expmap(residual));
  gtsam::BetweenFactor<gtsam::Pose3> factor(0U, 1U, gtsam::Pose3{}, model);
  const auto actual = auditFactor(factor, values);
  ASSERT_EQ(actual.residual.size(), 6);
  EXPECT_NEAR(actual.residual[0], .1, 1e-12);
  EXPECT_NEAR(actual.residual[4], .4, 1e-12);
  EXPECT_NEAR(actual.gaussian_whitened[0], 2., 1e-12);
  EXPECT_NEAR(actual.gaussian_whitened[4], 4., 1e-12);
  EXPECT_NEAR(actual.weight, 1. / std::sqrt(20.), 1e-12);
  EXPECT_NEAR(actual.gaussian_cost, 10., 1e-12);
  EXPECT_NEAR(actual.canonical_robust_cost, std::sqrt(20.) - .5, 1e-12);
  // Characterize the installed 4.1.1 API separately: factor.error applies rho
  // after robust whiten's reweight, and weight(Values) uses native residuals.
  EXPECT_DOUBLE_EQ(actual.factor_weight_api, 1.);
  EXPECT_NEAR(actual.actual_cost, std::sqrt(std::sqrt(20.)) - .5, 1e-12);
  EXPECT_NEAR(actual.effective_whitened.norm(), std::sqrt(std::sqrt(20.)), 1e-12);
}

TEST(LoopGraphQuality, CauchyWeightAndTrueCostAreNotTheIrlsQuadraticSurrogate)
{
  mapping::PoseGraphConfig config;
  config.robust_kernel = mapping::RobustKernel::CAUCHY;
  const auto model = loopNoise(config);
  ASSERT_TRUE(model);
  gtsam::Values values;
  values.insert(0U, gtsam::Pose3{});
  values.insert(1U, gtsam::Pose3{gtsam::Rot3{}, gtsam::Point3{.2, 0, 0}});
  const auto row = auditFactor(gtsam::BetweenFactor<gtsam::Pose3>{0U, 1U, {}, model}, values);
  EXPECT_NEAR(row.weight, .2, 1e-12);
  EXPECT_NEAR(row.canonical_robust_cost, .5 * std::log(5.), 1e-12);
  EXPECT_NEAR(row.actual_cost, .5 * std::log(1.8), 1e-12);
  EXPECT_NEAR(row.factor_weight_api, 1. / 1.04, 1e-12);
  EXPECT_NEAR(row.gaussian_cost, 2., 1e-12);
  EXPECT_LT(row.actual_cost, .5 * row.effective_whitened.squaredNorm());
}

TEST(LoopGraphQuality, BatchPreservesRawFactorsAndConvergesOnInconsistentTinyGraph)
{
  std::vector<Pose3d> raw(3);
  raw[1].translation.x() = 1.; raw[2].translation.x() = 2.;
  Pose3d measured; measured.translation.x() = -1.8;
  const auto graph = buildGraph(raw, {{2U, 0U, measured}}, {});
  ASSERT_EQ(graph.factors.size(), 4U);
  const auto odom = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(graph.factors[1]);
  ASSERT_TRUE(odom);
  EXPECT_DOUBLE_EQ(odom->measured().translation().x(), 1.);
  const auto result = solve(graph.factors, graph.raw);
  EXPECT_LT(result.final_error, result.initial_error - .2);
  EXPECT_LT(result.gradient_inf, 1e-5);
  EXPECT_GT(result.iterations, 0U);
  EXPECT_EQ(result.stop, "gradient_tolerance");
  ASSERT_EQ(result.values.size(), 3U);
  // Three equal translation sigmas distribute the 0.2m inconsistency equally.
  EXPECT_NEAR(result.values.at<gtsam::Pose3>(gtsam::Symbol{'x',2}).translation().x(), 1.8666666667, 1e-6);
}

TEST(LoopGraphQuality, BatchBetweenJacobianMatchesTheSameNonzeroResidualObjective)
{
  ExactBetweenFactor factor{0U,1U,gtsam::Pose3{},diagonal(.05,.1)};
  gtsam::Vector6 tangent;tangent<<.15,-.2,.1,.4,.2,-.3;
  const gtsam::Pose3 a{},b=gtsam::Pose3::Expmap(tangent);
  gtsam::Matrix ha,hb;factor.evaluateError(a,b,ha,hb);
  gtsam::BetweenFactor<gtsam::Pose3> original{0U,1U,{},diagonal(.05,.1)};
  EXPECT_LT((factor.evaluateError(a,b)-original.evaluateError(a,b)).norm(),1e-12);
  for(int d=0;d<6;++d){
    gtsam::Vector6 delta=gtsam::Vector6::Zero();delta[d]=1e-6;
    const gtsam::Vector numerical=(factor.evaluateError(a,b.retract(delta))-factor.evaluateError(a,b.retract(-delta)))/2e-6;
    EXPECT_LT((hb.col(d)-numerical).norm(),1e-7);
  }
}

TEST(LoopGraphQuality, SharedScanSupportClipsFutureAndCountsActualIntersection)
{
  const auto source = support(10U, 10U);
  EXPECT_EQ(source, (std::set<std::uint64_t>{5,6,7,8,9,10}));
  const auto next = support(12U, 12U);
  EXPECT_DOUBLE_EQ(jaccard(source, next), .5);
  EXPECT_EQ(support(0U, 10U), (std::set<std::uint64_t>{0,1,2,3,4,5}));
  EXPECT_DOUBLE_EQ(jaccard(source, support(30U,30U)), 0.);
}

TEST(LoopGraphQuality, CorrelationBudgetUsesOnlyPriorTargetWindowsAndLeavesDisjointEdgesUnchanged)
{
  std::vector<Measurement> loops{{100,20,{}},{110,22,{}},{120,70,{}},{130,20,{}}};
  const auto rules=correlationBudget(loops);
  ASSERT_EQ(rules.size(),4U);
  EXPECT_DOUBLE_EQ(rules[0].sigma_multiplier,1.);
  // [15,25] and [17,27]:9 common /13 union. [65,75] is disjoint.
  EXPECT_NEAR(rules[1].jaccard_sum,9./13.,1e-15);
  EXPECT_NEAR(rules[1].sigma_multiplier,std::sqrt(22./13.),1e-15);
  EXPECT_DOUBLE_EQ(rules[2].sigma_multiplier,1.);
  EXPECT_EQ(rules[2].overlapping_past_targets,0U);
  EXPECT_NEAR(rules[3].jaccard_sum,1.+9./13.,1e-15);
  EXPECT_EQ(rules[3].past_loops,3U);
  EXPECT_EQ(rules[3].overlapping_past_targets,2U);
  const auto prefix=correlationBudget({loops.begin(),loops.begin()+2});
  ASSERT_EQ(prefix.size(),2U);
  EXPECT_DOUBLE_EQ(prefix[1].sigma_multiplier,rules[1].sigma_multiplier);
}

TEST(LoopGraphQuality, CorrelationScalingTouchesOnlySpecifiedLoopNoiseNotPriorOdometryOrMeasurement)
{
  std::vector<Pose3d> raw(3);raw[1].translation.x()=1.;raw[2].translation.x()=2.;
  Pose3d measured;measured.translation.x()=-1.8;
  const auto graph=buildGraph(raw,{{2U,0U,measured}},{},{2.});
  const auto prior=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph.factors[0]);
  const auto odom=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph.factors[1]);
  const auto loop=boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(graph.factors[3]);
  ASSERT_TRUE(prior);ASSERT_TRUE(odom);ASSERT_TRUE(loop);
  EXPECT_DOUBLE_EQ(prior->noiseModel()->sigmas()[3],.001);
  EXPECT_DOUBLE_EQ(odom->noiseModel()->sigmas()[3],.1);
  EXPECT_DOUBLE_EQ(loop->measured().translation().x(),-1.8);
  const auto audit=auditFactor(*loop,graph.raw);
  EXPECT_NEAR(audit.gaussian_whitened.norm(),1.,1e-12);
  const auto noise=boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(loop->noiseModel());
  ASSERT_TRUE(noise);
  EXPECT_DOUBLE_EQ(noise->noise()->sigmas()[0],.1);
  EXPECT_DOUBLE_EQ(noise->noise()->sigmas()[3],.2);
  EXPECT_THROW(buildGraph(raw,{{2U,0U,measured}},{},{1.,2.}),std::runtime_error);
}

TEST(LoopGraphQuality, RefusesExistingOutputAndInputSymlinkOverlap)
{
  const auto temp = fs::temp_directory_path() / ("loop_graph_quality_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(temp / "bundle");
  fs::create_directory_symlink(temp / "bundle", temp / "alias");
  EXPECT_THROW(protectOutput(temp / "bundle" / "new", {temp / "bundle"}), std::runtime_error);
  EXPECT_THROW(protectOutput(temp / "alias" / "new", {temp / "bundle"}), std::runtime_error);
  EXPECT_THROW(protectOutput(temp, {temp / "bundle"}), std::runtime_error);
  EXPECT_NO_THROW(protectOutput(temp / "fresh", {temp / "bundle"}));
  fs::remove_all(temp);
}

TEST(LoopGraphQuality, StandaloneToolExportsSameFactorCostsAndImmutableInputs)
{
  const auto temp = fs::temp_directory_path() / ("loop_graph_cli_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(temp / "bundle/keyframes"); fs::create_directories(temp / "bundle/config");
  {std::ofstream file(temp / "bundle/keyframes/index.csv");
    file << "id,stamp_ns,tx,ty,tz,qx,qy,qz,qw\n0,0,0,0,0,0,0,0,1\n1,1,1,0,0,0,0,0,1\n2,2,2,0,0,0,0,0,1\n";}
  {std::ofstream file(temp / "loops.csv");
    file << "current_id,candidate_id,relative_tx,relative_ty,relative_tz,relative_qx,relative_qy,relative_qz,relative_qw\n2,0,-1.8,0,0,0,0,0,1\n";}
  {std::ofstream file(temp / "bundle/config/mapping_effective.yaml");
    file << "pose_graph.prior_rotation_sigma_rad: 0.001\npose_graph.prior_translation_sigma_m: 0.001\n"
         << "pose_graph.odometry_rotation_sigma_rad: 0.05\npose_graph.odometry_translation_sigma_m: 0.1\n"
         << "pose_graph.loop_rotation_sigma_rad: 0.05\npose_graph.loop_translation_sigma_m: 0.1\n"
         << "pose_graph.robust_kernel_scale: 1\npose_graph.relinearization_threshold: 0.1\n"
         << "pose_graph.relinearization_skip: 1\npose_graph.robust_kernel: huber\n";}
  mapping::PoseGraphOptimizer online{{}};
  for (std::uint64_t i=0;i<3;++i) {
    Pose3d pose;pose.translation.x()=i;
    ASSERT_TRUE(online.addKeyFrame(i,pose).accepted);
    if(i==2){Pose3d measured;measured.translation.x()=-1.8;
      ASSERT_TRUE(online.addLoopConstraint({2,0,measured,true}).accepted);}
    ASSERT_TRUE(online.update().accepted);
  }
  ASSERT_TRUE(online.update(true).accepted);
  ablation::save(temp / "isam.csv",online.optimizedPoses());
  std::vector<std::string> args{"loop_graph_quality",(temp/"bundle").string(),(temp/"loops.csv").string(),
    (temp/"isam.csv").string(),(temp/"out").string()};
  std::vector<char*> argv;for(auto &arg:args){argv.push_back(arg.data());}
  EXPECT_EQ(run(argv.size(),argv.data()),0);
  EXPECT_TRUE(fs::exists(temp / "out/batch_warm.csv"));
  EXPECT_EQ(ablation::readCsv(temp / "out/isam_factors.csv").size(),4U);
  EXPECT_EQ(ablation::readCsv(temp / "out/solutions.csv").size(),3U);
  EXPECT_THROW(run(argv.size(),argv.data()),std::runtime_error);
  args.back()=(temp/"budget").string();args.push_back("correlation-budget-v1");
  argv.clear();for(auto &arg:args){argv.push_back(arg.data());}
  EXPECT_EQ(run(argv.size(),argv.data()),0);
  const auto edge_rules=ablation::readCsv(temp/"budget/correlation_budget_edges.csv");
  ASSERT_EQ(edge_rules.size(),1U);
  EXPECT_EQ(edge_rules[0].at("sigma_multiplier"),"1");
  EXPECT_EQ(edge_rules[0].at("past_committed_loops"),"0");
  fs::remove_all(temp);
}
}  // namespace graph_quality
