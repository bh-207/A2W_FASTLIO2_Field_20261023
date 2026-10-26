#include <gtest/gtest.h>
#include <cstring>
#include <quatro/quatro_module.h>
#include "a2w_fastlio_common/quatro_registration.hpp"

namespace a2w_fastlio_common {
namespace {
CloudPtr fixture(float shift = 0.0F) {
  auto c = std::make_shared<Cloud>();
  for (int x=0;x<8;++x) for (int y=0;y<6;++y) {
    PointT p; p.x=.35F*x+shift; p.y=.4F*y;
    p.z=.12F*((x*x+3*y)%7)+.05F*x; p.intensity=x*10+y;
    c->push_back(p);
  }
  return c;
}
TEST(QuatroFeatureCache, ReusesExactContentAcrossDifferentPointers) {
  QuatroRegistrationConfig cfg; cfg.feature_cache_capacity=4;
  QuatroRegistration solver(cfg);
  auto a=fixture(), b=fixture(.2F);
  solver.align(a,b,std::nullopt);
  auto s=solver.cacheStats(); EXPECT_EQ(s.misses,2U); EXPECT_EQ(s.hits,0U);
  solver.align(std::make_shared<Cloud>(*a),std::make_shared<Cloud>(*b),std::nullopt);
  s=solver.cacheStats(); EXPECT_EQ(s.misses,2U); EXPECT_EQ(s.hits,2U); EXPECT_EQ(s.entries,2U);
}
TEST(QuatroFeatureCache, MutationAndPointOrderInvalidateAndLruIsBounded) {
  QuatroRegistrationConfig cfg; cfg.feature_cache_capacity=2;
  QuatroRegistration solver(cfg);
  auto a=fixture(), b=fixture(.2F);
  solver.align(a,b,std::nullopt);
  a->front().z += .03F;
  solver.align(a,b,std::nullopt);
  EXPECT_EQ(solver.cacheStats().misses,3U);
  EXPECT_EQ(solver.cacheStats().hits,1U);
  std::swap(a->front(),a->back());
  solver.align(a,b,std::nullopt);
  EXPECT_EQ(solver.cacheStats().misses,4U);
  EXPECT_EQ(solver.cacheStats().entries,2U);
}
TEST(QuatroFeatureCache, DisabledByDefaultAndPreservesKnownTransform) {
  QuatroRegistrationConfig cfg; cfg.fpfh_normal_radius_m=.45;
  cfg.fpfh_radius_m=.8; cfg.noise_bound_m=.15;
  auto a=fixture(); auto b=std::make_shared<Cloud>();
  const Eigen::Matrix3d R=Eigen::AngleAxisd(.35,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d t{1.2,-.7,.25};
  for (auto p:a->points) { auto v=(R*p.getVector3fMap().cast<double>()+t).eval();
    p.x=v.x();p.y=v.y();p.z=v.z();b->push_back(p); }
  QuatroRegistration plain(cfg);
  cfg.feature_cache_capacity=4; QuatroRegistration cached(cfg);
  auto base=plain.align(a,b,std::nullopt), first=cached.align(a,b,std::nullopt);
  auto repeat=cached.align(a,b,std::nullopt);
  ASSERT_TRUE(base.success); ASSERT_TRUE(first.success); ASSERT_TRUE(repeat.success);
  EXPECT_LT((first.transform.translation-t).norm(),.1);
  EXPECT_LT(Eigen::AngleAxisd(Eigen::Quaterniond(R).inverse()*first.transform.rotation).angle(),.1);
  EXPECT_LT((base.transform.translation-first.transform.translation).norm(),1e-6);
  EXPECT_LT(Eigen::AngleAxisd(base.transform.rotation.inverse()*first.transform.rotation).angle(),1e-6);
  EXPECT_LT((first.transform.translation-repeat.transform.translation).norm(),1e-6);
  EXPECT_LT(Eigen::AngleAxisd(first.transform.rotation.inverse()*repeat.transform.rotation).angle(),1e-6);
  EXPECT_EQ(plain.cacheStats().entries,0U); EXPECT_EQ(cached.cacheStats().hits,2U);
}
TEST(QuatroFeatureCache, PreparedFeaturesEqualOriginalSingleEstimatorProcedure) {
  quatro<PointT> solver(.5,.8,.25,1.4,1e-4,100,false,true,30.,500);
  auto source=fixture(),target=fixture(.2F);
  auto s=solver.prepare(*source),t=solver.prepare(*target);
  teaser::FPFHEstimation original;
  auto sf=original.computeFPFHFeatures(s.points,.5,.8);
  auto tf=original.computeFPFHFeatures(t.points,.5,.8);
  ASSERT_EQ(sf->size(),s.features->size()); ASSERT_EQ(tf->size(),t.features->size());
  for (std::size_t i=0;i<sf->size();++i) {
    EXPECT_EQ(std::memcmp(sf->at(i).histogram,s.features->at(i).histogram,33*sizeof(float)),0);
  }
  for (std::size_t i=0;i<tf->size();++i) {
    EXPECT_EQ(std::memcmp(tf->at(i).histogram,t.features->at(i).histogram,33*sizeof(float)),0);
  }
  quatro<PointT>::PreparedCloud invalid;
  bool valid=true;
  EXPECT_TRUE(solver.alignPrepared(s,invalid,valid).isIdentity()); EXPECT_FALSE(valid);
}
} }
