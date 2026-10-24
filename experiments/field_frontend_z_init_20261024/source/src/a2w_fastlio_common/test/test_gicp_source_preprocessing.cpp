#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/registration.h>

#include "a2w_fastlio_common/nano_gicp_registration.hpp"
#include "a2w_fastlio_common/registration_pipeline.hpp"

namespace a2w_fastlio_common
{
namespace
{

CloudPtr loadCloud(const std::string & name)
{
  auto cloud = std::make_shared<Cloud>();
  if (pcl::io::loadPCDFile(std::string{REAL_FINE_FAILURE_DATA_DIR} + "/" + name, *cloud) != 0) {
    throw std::runtime_error{"real fine failure fixture unavailable"};
  }
  return cloud;
}

Pose3d prediction()
{
  Pose3d pose;
  pose.translation = {0.65127376286826166, 0.17738902905194054, 0.022960542287028284};
  pose.rotation = Eigen::Quaterniond{
    0.96499904474504727, 0.014139162013309697, -0.023444691371128628,
    0.26082038682821596};
  return pose;
}

NanoGicpRegistrationConfig config()
{
  return {2.0, 4, 15, 32, 0.001, 0.001, 3, 1.0, 20U};
}

class UnusedCoarse final : public CoarseRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &, const std::optional<Pose3d> &) const override
  {
    throw std::logic_error{"fine-only regression unexpectedly ran coarse registration"};
  }
};

// Independent upstream PCL scoring reference: no optimization or sampling.
class FixedPoseFitness final : public pcl::Registration<PointT, PointT>
{
public:
  FixedPoseFitness(const CloudConstPtr & source, const CloudConstPtr & target, const Pose3d & pose)
  {
    setInputSource(source);
    setInputTarget(target);
    tree_->setInputCloud(target);
    final_transformation_.setIdentity();
    final_transformation_.block<3, 3>(0, 0) = pose.rotation.toRotationMatrix().cast<float>();
    final_transformation_.block<3, 1>(0, 3) = pose.translation.cast<float>();
  }
private:
  void computeTransformation(Cloud &, const Eigen::Matrix4f &) override {}
};

// Removing the optimizer sampling brings back the observed density-driven
// nonconvergence. Scoring a sampled source instead of the original source
// loses thousands of independently checked correspondences.
TEST(GicpSourcePreprocessing, RealFailureRecoversWithoutChangingFullSourceEvidence)
{
  const auto source = loadCloud("source_body.pcd");
  const auto original_first = source->front();
  ASSERT_EQ(source->size(), 9460U);
  auto sampled_config = config();
  sampled_config.source_voxel_leaf_m = 0.10;
  const auto fine = std::make_shared<NanoGicpRegistration>(sampled_config);
  const RegistrationPipeline pipeline{
    std::make_shared<UnusedCoarse>(), fine,
    MatchValidator{MatchValidationConfig{0.25, 0.35, 30U, 5.0, 1.0, 0.05}},
    RegistrationPipelineConfig{0.5}};

  for (const auto & target_name : {
      "target_current_34_35_26.pcd", "target_previous_34_35_33.pcd"})
  {
    const auto target = loadCloud(target_name);
    EXPECT_FALSE(NanoGicpRegistration{config()}.align(source, target, prediction()).converged);
    const auto native = fine->align(source, target, prediction());
    ASSERT_TRUE(native.success);
    FixedPoseFitness full_source_score{source, target, native.transform};
    EXPECT_NEAR(native.fitness, full_source_score.getFitnessScore(1.0), 1e-6);
    const auto result = pipeline.runFineOnly(source, target, prediction());
    ASSERT_TRUE(result.success) << target_name << ": " << result.validation.reason;
    EXPECT_LT((result.final_transform.translation - prediction().translation).norm(), 0.5);
    EXPECT_LT(Eigen::AngleAxisd(
        prediction().rotation.inverse() * result.final_transform.rotation).angle(),
      10.0 * std::acos(-1.0) / 180.0);
    EXPECT_GT(result.fine.correspondence_count, 4500U);  // optimizer has only 3020 points.
    EXPECT_NEAR(result.fine.overlap,
      static_cast<double>(result.fine.correspondence_count) / 9460.0, 1e-12);
    EXPECT_LT(result.fine.fitness, 0.25);
  }
  EXPECT_EQ(source->size(), 9460U);
  EXPECT_EQ(source->front().x, original_first.x);
  EXPECT_EQ(source->front().intensity, original_first.intensity);
}

TEST(GicpSourcePreprocessing, RejectsInvalidOptimizerLeaf)
{
  for (const auto leaf : {-0.1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), 1e300, 1e-300})
  {
    auto value = config();
    value.source_voxel_leaf_m = leaf;
    EXPECT_THROW((NanoGicpRegistration{value}), std::invalid_argument);
  }
}

TEST(GicpSourcePreprocessing, DisabledSamplingPreservesFrozenRawOutputAndInvalidGuessRejection)
{
  const auto source = loadCloud("source_body.pcd");
  const auto target = loadCloud("target_current_34_35_26.pcd");
  const auto raw = NanoGicpRegistration{config()}.align(source, target, prediction());
  EXPECT_FALSE(raw.converged);
  EXPECT_DOUBLE_EQ(raw.transform.translation.x(), 1.6971123218536377);
  EXPECT_DOUBLE_EQ(raw.transform.translation.y(), 0.09230688214302063);
  EXPECT_DOUBLE_EQ(raw.transform.translation.z(), 0.03656971454620361);
  auto invalid = prediction();
  invalid.translation.x() = std::numeric_limits<double>::quiet_NaN();
  auto sampled = config();
  sampled.source_voxel_leaf_m = 0.1;
  EXPECT_FALSE(NanoGicpRegistration{sampled}.align(source, target, invalid).success);
}

TEST(GicpSourcePreprocessing, RejectsSamplingThatLeavesTooFewNeighbors)
{
  auto value = config();
  value.source_voxel_leaf_m = 1000.0;
  const auto source = loadCloud("source_body.pcd");
  const auto target = loadCloud("target_current_34_35_26.pcd");
  const auto rejected = NanoGicpRegistration{value}.align(source, target, prediction());
  EXPECT_FALSE(rejected.success);
  EXPECT_FALSE(rejected.converged);
  EXPECT_EQ(rejected.rejection_reason, "optimizer source too small after voxel sampling");
}

}  // namespace
}  // namespace a2w_fastlio_common
