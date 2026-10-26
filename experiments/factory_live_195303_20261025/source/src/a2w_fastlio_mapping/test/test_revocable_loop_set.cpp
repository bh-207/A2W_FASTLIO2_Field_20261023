#include <gtest/gtest.h>
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"
namespace m=a2w_fastlio_mapping;
using a2w_fastlio_common::Pose3d;
TEST(RevocableLoopSet, PreviewReplacementDoesNotKeepFirstArrivalOrMutateLive) {
 m::PoseGraphOptimizer graph{{}}; Pose3d p;
 for(unsigned i=0;i<8;++i){p.translation.x()=i; ASSERT_TRUE(graph.addKeyFrame(i,p).accepted);ASSERT_TRUE(graph.update().accepted);}
 Pose3d a;a.translation.x()=-2.;m::LoopConstraint early{7,0,a,true};
 ASSERT_TRUE(graph.addLoopConstraint(early).accepted);ASSERT_TRUE(graph.update().accepted);
 const auto committed=graph.optimizedPoses();
 const auto trial=graph.previewLoopSet({});ASSERT_TRUE(trial.update.accepted);
 EXPECT_NEAR(trial.snapshot.poses.back().pose.translation.x(),7.,1e-7);
 EXPECT_EQ(graph.optimizedPoses().revision,committed.revision);
 EXPECT_DOUBLE_EQ(graph.optimizedPoses().poses.back().pose.translation.x(),committed.poses.back().pose.translation.x());
 ASSERT_TRUE(graph.replaceLoopSet({}).accepted);
 EXPECT_NEAR(graph.optimizedPoses().poses.back().pose.translation.x(),7.,1e-7);
}
TEST(RevocableLoopSet, FutureOrInvalidReplacementLeavesLiveAndObserverUnchanged) {
 m::PoseGraphOptimizer graph{{}};unsigned emitted=0;graph.setFactorLedgerObserver([&](const auto&){++emitted;});
 Pose3d p;ASSERT_TRUE(graph.addKeyFrame(0,p).accepted);ASSERT_TRUE(graph.update().accepted);
 const auto before=graph.optimizedPoses();const auto old_count=emitted;
 EXPECT_FALSE(graph.replaceLoopSet({{4,0,p,true}}).accepted);
 EXPECT_EQ(emitted,old_count);EXPECT_EQ(graph.optimizedPoses().revision,before.revision);
 EXPECT_DOUBLE_EQ(graph.optimizedPoses().poses.front().pose.translation.x(),before.poses.front().pose.translation.x());
}
TEST(RevocableLoopSet, ReplacementUsesExactNoiseAndOriginalCausalInsertion) {
 m::PoseGraphOptimizer actual{{}},reference{{}};Pose3d p,a;a.translation.x()=-1.7;
 std::vector<m::LoopConstraint> loops{{3,0,a,true}};
 for(unsigned i=0;i<8;++i){p.translation.x()=i;ASSERT_TRUE(actual.addKeyFrame(i,p).accepted);ASSERT_TRUE(actual.update().accepted);
 ASSERT_TRUE(reference.addKeyFrame(i,p).accepted);if(i==3)ASSERT_TRUE(reference.addLoopConstraint(loops[0]).accepted);ASSERT_TRUE(reference.update().accepted);}
 const auto trial=actual.previewLoopSet(loops);ASSERT_TRUE(trial.update.accepted);
 ASSERT_TRUE(actual.replaceLoopSet(loops).accepted);
 for(unsigned i=0;i<8;++i){EXPECT_DOUBLE_EQ(actual.optimizedPoses().poses[i].pose.translation.x(),trial.snapshot.poses[i].pose.translation.x());EXPECT_DOUBLE_EQ(actual.optimizedPoses().poses[i].pose.translation.x(),reference.optimizedPoses().poses[i].pose.translation.x());}
}

TEST(RevocableLoopSet, RequiredCommitBarrierFailureLeavesLiveRevisionAndPosesUnchanged) {
 m::PoseGraphOptimizer graph{{}};Pose3d p;
 for(unsigned i=0;i<8;++i){p.translation.x()=i;ASSERT_TRUE(graph.addKeyFrame(i,p).accepted);ASSERT_TRUE(graph.update().accepted);}
 const auto before=graph.optimizedPoses();Pose3d a;a.translation.x()=-2.;
 EXPECT_FALSE(graph.replaceLoopSet({{7,0,a,true}},[](const auto&){throw std::runtime_error("injected_review_commit_flush_failure");}).accepted);
 const auto after=graph.optimizedPoses();EXPECT_EQ(after.revision,before.revision);
 for(unsigned i=0;i<8;++i)EXPECT_DOUBLE_EQ(after.poses[i].pose.translation.x(),before.poses[i].pose.translation.x());
}
TEST(RevocableLoopSet, RequiredReplacementFactorObserverFailureDoesNotSwapGraph) {
 m::PoseGraphOptimizer graph{{}};bool fail=false;
 graph.setFactorLedgerObserver([&](const auto&){if(fail)throw std::runtime_error("injected_factor_ledger_failure");});
 Pose3d p;for(unsigned i=0;i<8;++i){p.translation.x()=i;ASSERT_TRUE(graph.addKeyFrame(i,p).accepted);ASSERT_TRUE(graph.update().accepted);}
 const auto before=graph.optimizedPoses();fail=true;Pose3d a;a.translation.x()=-2.;
 EXPECT_FALSE(graph.replaceLoopSet({{7,0,a,true}}).accepted);
 const auto after=graph.optimizedPoses();EXPECT_EQ(after.revision,before.revision);
 for(unsigned i=0;i<8;++i)EXPECT_DOUBLE_EQ(after.poses[i].pose.translation.x(),before.poses[i].pose.translation.x());
}
