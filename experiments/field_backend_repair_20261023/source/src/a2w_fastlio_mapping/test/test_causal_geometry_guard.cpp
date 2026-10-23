#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <gtest/gtest.h>
#include "a2w_fastlio_mapping/causal_geometry_guard.hpp"

namespace a2w_fastlio_mapping
{
namespace
{
using a2w_fastlio_common::KeyFrame;
using a2w_fastlio_common::Pose3d;

struct Fixture
{
  std::map<std::uint64_t, KeyFrame> frames;
  OptimizedPoseSnapshot before, trial;
  Fixture()
  {
    for (std::uint64_t id : {0U,10U,20U,30U}) {
      KeyFrame k;k.id=id;k.stamp_ns=static_cast<std::int64_t>(id)*4000000000LL;
      for (int x=-30;x<=30;++x) {for (int y=-30;y<=30;++y) {
        a2w_fastlio_common::PointT p;p.x=.1F*x;p.y=.1F*y;p.z=.2F*p.x+.05F*p.y;k.body_cloud->push_back(p);
      }}
      frames[id]=k;before.poses.push_back({id,k.odom_pose});
    }
    trial=before;
  }
};

TEST(CausalGeometryGuard, FreeRampPassesWithoutFlattening)
{
  Fixture f;auto r=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{});
  EXPECT_TRUE(r.accepted()) << r.reason;
  EXPECT_GE(r.raw_supported_frame_pairs,2U);EXPECT_GE(r.records.size(),8U);
  EXPECT_LT(r.trial.height_absolute_p95_m,1e-5);
  ASSERT_FALSE(r.records.empty());EXPECT_GT(std::abs(r.records[0].raw_axis.x()),.1);
}

TEST(CausalGeometryGuard, RejectsNewCrossTimeNormalShift)
{
  Fixture f;f.trial.poses[2].pose.translation.z()=.12;
  auto r=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{});
  EXPECT_EQ(r.verdict,CausalGeometryGuardVerdict::REJECT) << r.reason;
  EXPECT_FALSE(r.accepted());
}

TEST(CausalGeometryGuard, RejectsNewNormalDisagreement)
{
  Fixture f;f.trial.poses[2].pose.rotation=Eigen::AngleAxisd(.08,Eigen::Vector3d::UnitY());
  auto r=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{});
  EXPECT_EQ(r.verdict,CausalGeometryGuardVerdict::REJECT) << r.reason;
}

TEST(CausalGeometryGuard, GlobalSE3DoesNotChangeDecisionOrSeamValues)
{
  Fixture f;f.trial.poses[2].pose.translation.z()=.12;
  auto original=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{});
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.4,Eigen::Vector3d{1.,2.,3.}.normalized()));
  const Eigen::Vector3d t{13.,-47.,6.};
  auto transform=[&](Pose3d & p) {p.translation=q*p.translation+t;p.rotation=q*p.rotation;};
  for (auto & [id,k]:f.frames) {transform(k.odom_pose);}
  for (auto & p:f.before.poses) {transform(p.pose);}
  for (auto & p:f.trial.poses) {transform(p.pose);}
  auto changed=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{});
  EXPECT_EQ(original.verdict,changed.verdict);EXPECT_EQ(original.records.size(),changed.records.size());
  EXPECT_NEAR(original.trial.height_absolute_p95_m,changed.trial.height_absolute_p95_m,1e-8);
}

TEST(CausalGeometryGuard, ExcludesAllRegistrationIdentitiesAndReturnsUnknown)
{
  Fixture f;auto r=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{0U,10U,20U});
  EXPECT_EQ(r.verdict,CausalGeometryGuardVerdict::UNKNOWN);EXPECT_FALSE(r.accepted());EXPECT_TRUE(r.records.empty());
}

TEST(CausalGeometryGuard, UnrelatedRawSurfacesNeverCountAsIndependentAnchors)
{
  Fixture f;f.frames[10U].odom_pose.translation.z()=.4;f.frames[20U].odom_pose.translation.z()=.8;
  auto r=CausalGeometryGuard{}.assess(f.frames,f.before,f.trial,30U,25U,{});
  EXPECT_EQ(r.verdict,CausalGeometryGuardVerdict::UNKNOWN);EXPECT_FALSE(r.accepted());
}

TEST(CausalGeometryGuard, WorkCapsAndPointIdentityAreAuditable)
{
  Fixture f;CausalGeometryGuardConfig c;c.maximum_frames=3U;c.maximum_patch_records=12U;
  auto r=CausalGeometryGuard{c}.assess(f.frames,f.before,f.trial,30U,25U,{});
  EXPECT_LE(r.records.size(),12U);EXPECT_LE(r.sampled_frames,3U);
  std::ostringstream ledger;writeCausalGeometryGuardLedger(ledger,r);
  EXPECT_NE(ledger.str().find("first_point_indices"),std::string::npos);
  for (const auto & p:r.records) {for (auto i:p.first_point_indices) {EXPECT_LT(i,f.frames.at(p.first_id).body_cloud->size());}}
}

TEST(CausalGeometryGuard, UniformCoverageCannotHideLatePairAfterManyEarlyStaticAnchors)
{
  Fixture base;std::map<std::uint64_t,KeyFrame> frames;OptimizedPoseSnapshot before,trial;
  for(std::uint64_t id=0U;id<82U;++id){
    auto k=base.frames.at(0U);k.id=id;k.stamp_ns=static_cast<std::int64_t>(id)*40000000000LL;
    if(id>=80U){k.odom_pose.translation.x()=12.;}
    frames[id]=k;before.poses.push_back({id,k.odom_pose});
  }
  trial=before;trial.poses.back().pose.translation.z()=.12;
  const auto r=CausalGeometryGuard{}.assess(frames,before,trial,90U,85U,{});
  EXPECT_EQ(r.verdict,CausalGeometryGuardVerdict::REJECT) << r.reason;
  bool late=false;for(const auto & p:r.pairs){if(p.first_id==80U&&p.second_id==81U){late=true;EXPECT_EQ(p.reason,"cross_time_plane_median_retreat");}}
  EXPECT_TRUE(late);EXPECT_LE(r.records.size(),512U);EXPECT_LE(r.sampled_frames,128U);
}
}  // namespace
}  // namespace a2w_fastlio_mapping
