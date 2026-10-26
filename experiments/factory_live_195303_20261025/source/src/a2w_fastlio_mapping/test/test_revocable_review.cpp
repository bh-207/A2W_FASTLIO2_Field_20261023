#include <gtest/gtest.h>
#include <sstream>
#include "a2w_fastlio_mapping/revocable_loop_review.hpp"
namespace m=a2w_fastlio_mapping;
m::ReviewHypothesis h(unsigned mask,unsigned count,double p95,bool observable=true){
 m::ReviewHypothesis v;v.mask=mask;v.loop_count=count;v.independently_observable=observable;
 v.geometry.verdict=m::CausalGeometryGuardVerdict::SUPPORTED_PASS;
 m::CausalGeometryPairRecord p;p.matched_patch_count=8;p.trial.height_absolute_p95_m=p95;v.geometry.pairs.push_back(p);return v;
}
TEST(RevocableReview, WithdrawsEarlierSupportedSetWhenIndependentGeometryPrefersReplacement) {
 std::vector<m::ReviewHypothesis> hs{h(3,2,.12),h(5,2,.02),h(1,1,.01)};
 auto selected=m::selectReviewHypothesis(hs);ASSERT_TRUE(selected);EXPECT_EQ(hs[*selected].mask,5U);
 std::reverse(hs.begin(),hs.end());selected=m::selectReviewHypothesis(hs);ASSERT_TRUE(selected);EXPECT_EQ(hs[*selected].mask,5U);
}
TEST(RevocableReview, UnknownAndStaticUnresponsiveSupportCannotCommit) {
 auto unknown=h(7,3,.001);unknown.geometry.verdict=m::CausalGeometryGuardVerdict::UNKNOWN;
 EXPECT_FALSE(m::selectReviewHypothesis({unknown,h(3,2,.001,false)}));
}
TEST(RevocableReview, EqualIndependentScoresDeferInsteadOfKeepingFirst) {
 EXPECT_FALSE(m::selectReviewHypothesis({h(3,2,.02),h(5,2,.02)}));
}

TEST(RevocableReview, DeferredProposalCannotPermanentlyOccupyCapacityAndBlockLaterEvidence) {
 std::vector<m::ReviewEdge> pool;
 for(unsigned i=1;i<=4;++i)pool.push_back({{i,0,{},true},{i,0}});
 const std::vector<m::ReviewEdge> active{pool[0]};
 std::ostringstream ledger;
 ASSERT_TRUE(m::appendReviewEdgeBounded(pool,{{80,0,{},true},{80,0}},active,80,ledger));
 ASSERT_EQ(pool.size(),4U);
 EXPECT_EQ(pool.front().constraint.from_id,1U); // active graph edge stays intact
 EXPECT_EQ(pool[1].constraint.from_id,3U); // oldest deferred measurement 2 was evicted
 EXPECT_EQ(pool.back().constraint.from_id,80U); // late new support is not blocked
 EXPECT_NE(ledger.str().find("review_pending_evicted"),std::string::npos);
}
TEST(RevocableReview, FailedCommitLedgerPreservesActiveSetAndGraphButGoodLedgerAllowsWithdrawal) {
 m::PoseGraphOptimizer graph{{}};a2w_fastlio_common::Pose3d p;
 for(unsigned i=0;i<8;++i){p.translation.x()=i;ASSERT_TRUE(graph.addKeyFrame(i,p).accepted);ASSERT_TRUE(graph.update().accepted);}
 a2w_fastlio_common::Pose3d measured;measured.translation.x()=-2.;
 const m::ReviewEdge old{{7,0,measured,true},{0,7}};
 ASSERT_TRUE(graph.addLoopConstraint(old.constraint).accepted);ASSERT_TRUE(graph.update().accepted);
 std::vector<m::ReviewEdge> active{old};const auto before=graph.optimizedPoses();
 std::ostringstream failed;failed.setstate(std::ios::badbit);
 EXPECT_THROW(m::commitReviewLoopSet(graph,active,{},7,failed),m::LoopReviewPersistenceError);
 ASSERT_EQ(active.size(),1U);EXPECT_EQ(active.front().constraint.from_id,7U);
 EXPECT_EQ(graph.optimizedPoses().revision,before.revision);
 EXPECT_DOUBLE_EQ(graph.optimizedPoses().poses.back().pose.translation.x(),before.poses.back().pose.translation.x());
 std::ostringstream good;ASSERT_TRUE(m::commitReviewLoopSet(graph,active,{},7,good).accepted);
 EXPECT_TRUE(active.empty());EXPECT_NEAR(graph.optimizedPoses().poses.back().pose.translation.x(),7.,1e-7);
 EXPECT_NE(good.str().find("withdrawn_edges"),std::string::npos);
 EXPECT_NE(good.str().find("review_commit"),std::string::npos);
}
