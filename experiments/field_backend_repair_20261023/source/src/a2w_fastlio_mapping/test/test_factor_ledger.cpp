#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <gtest/gtest.h>
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"
namespace a2w_fastlio_mapping {
TEST(FactorLedger, ExactMeasurementUnitsResidualsAndCommitAreRecorded) {
  PoseGraphOptimizer graph{PoseGraphConfig{}};
  std::vector<FactorLedgerRecord> records;
  graph.setFactorLedgerObserver([&](const auto & r) { records.push_back(r); });
  a2w_fastlio_common::Pose3d p;
  ASSERT_TRUE(graph.addKeyFrame(0,p).accepted); ASSERT_TRUE(graph.update().accepted);
  p.translation.x()=1.; ASSERT_TRUE(graph.addKeyFrame(1,p).accepted);
  a2w_fastlio_common::Pose3d measured;
  measured.translation={-2.,.13,-.07};
  measured.rotation=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX())*
    Eigen::AngleAxisd(-.2,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(graph.addLoopConstraint({1,0,measured,true}).accepted);
  ASSERT_TRUE(graph.update().accepted); graph.emitFactorLedger("final_audit");
  bool loop_seen=false, prior_seen=false;
  for (const auto & r:records) {
    if(r.factor_type=="prior") prior_seen=true;
    if(r.factor_type!="loop"||r.stage!="pre_update") continue;
    loop_seen=true; EXPECT_EQ(r.from_id,1U); EXPECT_EQ(r.to_id,0U);
    EXPECT_EQ(r.commit_sequence,2U); EXPECT_DOUBLE_EQ(r.measurement.translation.x(),-2.);
    EXPECT_NEAR((r.measurement.rotation.coeffs()-measured.rotation.coeffs()).norm(),0.,1e-15);
    EXPECT_DOUBLE_EQ(r.measurement.translation.y(),.13); EXPECT_DOUBLE_EQ(r.measurement.translation.z(),-.07);
    EXPECT_DOUBLE_EQ(r.sigmas[0],.05); EXPECT_DOUBLE_EQ(r.sigmas[3],.1);
    double gaussian2=0,effective2=0;
    for(std::size_t i=0;i<6;++i) { EXPECT_NEAR(r.gaussian_whitened[i],r.raw_residual[i]/r.sigmas[i],1e-12);
      gaussian2+=r.gaussian_whitened[i]*r.gaussian_whitened[i]; effective2+=r.effective_whitened[i]*r.effective_whitened[i]; }
    EXPECT_NEAR(r.effective_robust_weight,effective2/gaussian2,1e-12);
    EXPECT_LT(r.effective_robust_weight,1.); EXPECT_EQ(r.robust_kernel,"huber");
  }
  EXPECT_TRUE(loop_seen); EXPECT_TRUE(prior_seen);
  std::ostringstream out; writeFactorLedgerCsvHeader(out); writeFactorLedgerCsv(out,records.back());
  EXPECT_NE(out.str().find("right_local_rotation_rad_translation_m"),std::string::npos);
}
TEST(FactorLedger, ThrowingObserverCannotChangeGraphCommit) {
  PoseGraphOptimizer graph{PoseGraphConfig{}};
  graph.setFactorLedgerObserver([](const auto &){throw std::runtime_error("sink failed");});
  ASSERT_TRUE(graph.addKeyFrame(0,{}).accepted); EXPECT_TRUE(graph.update().accepted);
  EXPECT_EQ(graph.optimizedPoses().revision,1U);
}
TEST(FactorLedger, EnablingDiagnosticsPreservesUnobservedGraphExactly) {
  PoseGraphConfig config;config.robust_kernel=RobustKernel::CAUCHY;
  PoseGraphOptimizer plain{config},observed{config};std::vector<FactorLedgerRecord>rows;
  observed.setFactorLedgerObserver([&](const auto&r){rows.push_back(r);});
  for(std::uint64_t i=0;i<5;++i) {
    a2w_fastlio_common::Pose3d p;p.translation={double(i),.2*double(i),.01*double(i)};
    p.rotation=Eigen::AngleAxisd(.1*double(i),Eigen::Vector3d::UnitZ());
    ASSERT_TRUE(plain.addKeyFrame(i,p).accepted);ASSERT_TRUE(observed.addKeyFrame(i,p).accepted);
    if(i==4) {a2w_fastlio_common::Pose3d m;m.translation={-3.5,.2,-.1};
      m.rotation=Eigen::AngleAxisd(-.3,Eigen::Vector3d::UnitZ());
      ASSERT_TRUE(plain.addLoopConstraint({4,0,m,true}).accepted);
      ASSERT_TRUE(observed.addLoopConstraint({4,0,m,true}).accepted);}
    ASSERT_TRUE(plain.update().accepted);ASSERT_TRUE(observed.update().accepted);
  }
  ASSERT_TRUE(plain.update(true).accepted);ASSERT_TRUE(observed.update(true).accepted);
  observed.emitFactorLedger();
  for(std::size_t i=0;i<5;++i) {
    EXPECT_DOUBLE_EQ((plain.optimizedPoses().poses[i].pose.translation-observed.optimizedPoses().poses[i].pose.translation).norm(),0.);
    EXPECT_DOUBLE_EQ((plain.optimizedPoses().poses[i].pose.rotation.coeffs()-observed.optimizedPoses().poses[i].pose.rotation.coeffs()).norm(),0.);
  }
  ASSERT_FALSE(rows.empty());EXPECT_EQ(rows.back().robust_kernel,"cauchy");
  EXPECT_EQ(rows.back().commit_sequence,6U);EXPECT_EQ(rows.back().factor_commit_sequence,5U);
}
}
namespace a2w_fastlio_mapping {
TEST(TransactionalLoopPreview, RejectedPreviewLeavesPendingStateAndObserversUntouched) {
 PoseGraphOptimizer live{PoseGraphConfig{}},control{PoseGraphConfig{}};
 std::size_t observations=0;live.setFactorLedgerObserver([&](const auto&){++observations;});
 for(std::uint64_t i=0;i<5;++i){a2w_fastlio_common::Pose3d p;p.translation.x()=double(i);
  ASSERT_TRUE(live.addKeyFrame(i,p).accepted);ASSERT_TRUE(control.addKeyFrame(i,p).accepted);
  if(i<4){ASSERT_TRUE(live.update().accepted);ASSERT_TRUE(control.update().accepted);}}
 const auto before=live.optimizedPoses();const auto count=observations;
 const auto invalid=live.previewLoopConstraint({4,99,{},true});
 EXPECT_FALSE(invalid.update.accepted);EXPECT_EQ(observations,count);
 a2w_fastlio_common::Pose3d measurement;measurement.translation.x()=-3.5;
 for(int k=0;k<3;++k){const auto preview=live.previewLoopConstraint({4,0,measurement,true});
  ASSERT_TRUE(preview.update.accepted);EXPECT_EQ(preview.snapshot.revision,before.revision+1);
  EXPECT_EQ(live.optimizedPoses().revision,before.revision);EXPECT_EQ(observations,count);}
 ASSERT_TRUE(live.update().accepted);ASSERT_TRUE(control.update().accepted);
 for(std::size_t i=0;i<5;++i)EXPECT_DOUBLE_EQ((live.optimizedPoses().poses[i].pose.translation-control.optimizedPoses().poses[i].pose.translation).norm(),0.);
}
TEST(TransactionalLoopPreview, ApprovedLiveCommitEqualsIsolatedTrial) {
 PoseGraphOptimizer live{PoseGraphConfig{}};
 for(std::uint64_t i=0;i<30;++i){a2w_fastlio_common::Pose3d p;p.translation={double(i),.2*double(i),.01*double(i)};
  p.rotation=Eigen::AngleAxisd(.03*double(i),Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(live.addKeyFrame(i,p).accepted);if(i<29)ASSERT_TRUE(live.update().accepted);}
 a2w_fastlio_common::Pose3d measurement;measurement.translation={-29.,2.,0.};
 measurement.rotation=Eigen::AngleAxisd(-.8,Eigen::Vector3d::UnitZ());
 const LoopConstraint constraint{29,0,measurement,true};const auto trial=live.previewLoopConstraint(constraint);
 ASSERT_TRUE(trial.update.accepted);ASSERT_TRUE(live.addLoopConstraint(constraint).accepted);ASSERT_TRUE(live.update().accepted);
 const auto actual=live.optimizedPoses();ASSERT_EQ(actual.revision,trial.snapshot.revision);
 for(std::size_t i=0;i<actual.poses.size();++i){
  EXPECT_DOUBLE_EQ((actual.poses[i].pose.translation-trial.snapshot.poses[i].pose.translation).norm(),0.);
  EXPECT_DOUBLE_EQ((actual.poses[i].pose.rotation.coeffs()-trial.snapshot.poses[i].pose.rotation.coeffs()).norm(),0.);}
}
}
namespace a2w_fastlio_mapping {
TEST(TransactionalLoopPreview, ExistingLoopCliquesAndFutureRelinearizationAreIsolated) {
 PoseGraphConfig config;config.relinearization_threshold=.001;
 PoseGraphOptimizer live{config},control{config};
 std::vector<a2w_fastlio_common::Pose3d>raw;
 for(std::uint64_t i=0;i<100;++i){a2w_fastlio_common::Pose3d p;
  p.translation={4.*std::sin(.04*double(i)),4.*std::cos(.04*double(i)),.01*double(i)};
  p.rotation=Eigen::AngleAxisd(.04*double(i),Eigen::Vector3d::UnitZ());raw.push_back(p);
  ASSERT_TRUE(live.addKeyFrame(i,p).accepted);ASSERT_TRUE(control.addKeyFrame(i,p).accepted);
  if(i==20||i==60){auto m=fromGtsamPose(toGtsamPose(p).between(toGtsamPose(raw[i-15])));m.translation.x()+=.04;
   ASSERT_TRUE(live.addLoopConstraint({i,i-15,m,true}).accepted);ASSERT_TRUE(control.addLoopConstraint({i,i-15,m,true}).accepted);}
  if(i<99){ASSERT_TRUE(live.update().accepted);ASSERT_TRUE(control.update().accepted);}
 }
 auto m=fromGtsamPose(toGtsamPose(raw.back()).between(toGtsamPose(raw[10])));m.translation.z()+=.2;
 ASSERT_TRUE(live.previewLoopConstraint({99,10,m,true}).update.accepted);
 ASSERT_TRUE(live.update().accepted);ASSERT_TRUE(control.update().accepted);
 ASSERT_TRUE(live.update(true).accepted);ASSERT_TRUE(control.update(true).accepted);
 for(std::size_t i=0;i<100;++i){
  EXPECT_DOUBLE_EQ((live.optimizedPoses().poses[i].pose.translation-control.optimizedPoses().poses[i].pose.translation).norm(),0.);
  EXPECT_DOUBLE_EQ((live.optimizedPoses().poses[i].pose.rotation.coeffs()-control.optimizedPoses().poses[i].pose.rotation.coeffs()).norm(),0.);}
}
}
