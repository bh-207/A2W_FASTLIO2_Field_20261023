#include <memory>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/map_matcher.hpp"

namespace a2w_fastlio_localization
{
namespace
{
using namespace a2w_fastlio_common;

class FakeRegistration final : public CoarseRegistration, public FineRegistration
{
public:
  RegistrationResult align(const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    RegistrationResult result;
    result.success = true;
    result.converged = true;
    result.fitness = fitness;
    result.overlap = 1.0;
    result.correspondence_count = 40U;
    return result;
  }
  mutable int calls{0};
  double fitness{0.01};
};

a2w_fastlio_map::MapSnapshot mapSnapshot()
{
  a2w_fastlio_map::MapSnapshot map;
  KeyFrame frame;
  for (int index = 0; index < 40; ++index) {
    PointT point;
    point.x = static_cast<float>(index) * 0.1F;
    frame.body_cloud->push_back(point);
  }
  map.keyframes.push_back(frame);
  return map;
}

TEST(MapMatcher, UsesCoarseFineAndValidatorInterfaces)
{
  auto algorithm = std::make_shared<FakeRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  MapMatcher matcher{mapSnapshot(), pipeline, {0.0, 1000U}};
  FrontendFrame frame;
  frame.body_cloud = mapSnapshot().keyframes[0].body_cloud;
  SelectionResult selection;
  selection.success = true;
  selection.keyframe_ids = {0U};
  const auto result = matcher.match(frame, selection, MapMatchingMode::kFull);
  EXPECT_TRUE(result.success) << result.reason;
  EXPECT_EQ(algorithm->calls, 2);
  EXPECT_DOUBLE_EQ(result.map_body.translation.x(), 0.0);
  EXPECT_EQ(result.source_points, 40U);
  EXPECT_EQ(result.target_points, 40U);
}

TEST(MapMatcher, FineOnlySkipsCoarseRegistration)
{
  auto algorithm = std::make_shared<FakeRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  MapMatcher matcher{mapSnapshot(), pipeline, {0.0, 1000U}};
  FrontendFrame frame;
  frame.body_cloud = mapSnapshot().keyframes[0].body_cloud;
  SelectionResult selection;
  selection.success = true;
  selection.keyframe_ids = {0U};

  const auto result = matcher.match(frame, selection, MapMatchingMode::kFineOnly);

  EXPECT_TRUE(result.success) << result.reason;
  EXPECT_EQ(algorithm->calls, 1);
}

TEST(MapMatcher, RejectsInvalidSelectionWithoutCallingAlgorithms)
{
  auto algorithm = std::make_shared<FakeRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  MapMatcher matcher{mapSnapshot(), pipeline, {0.0, 1000U}};
  EXPECT_FALSE(matcher.match({}, {}, MapMatchingMode::kFull).success);
  EXPECT_EQ(algorithm->calls, 0);
}

// Repeated localization in the same immutable neighborhood should reuse its
// exact target, while changing the neighborhood must invalidate that target.
TEST(MapMatcher, ReusesTargetForUnchangedNeighborhoodAndInvalidatesOnChange)
{
  auto algorithm = std::make_shared<FakeRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  auto snapshot = mapSnapshot();
  auto second = snapshot.keyframes[0];
  second.id = 1;
  second.optimized_pose.translation.y() = 0.2;
  snapshot.keyframes.push_back(second);
  MapMatcher matcher{snapshot, pipeline, {0.0, 1000U}};
  FrontendFrame input;
  input.body_cloud = snapshot.keyframes[0].body_cloud;
  SelectionResult selection;
  selection.success = true;
  selection.keyframe_ids = {0U};
  const auto first = matcher.match(input, selection, MapMatchingMode::kFineOnly);
  const auto repeated = matcher.match(input, selection, MapMatchingMode::kFineOnly);
  ASSERT_TRUE(first.success);
  EXPECT_EQ(first.target_cloud, repeated.target_cloud);
  selection.keyframe_ids = {0U, 1U};
  const auto changed = matcher.match(input, selection, MapMatchingMode::kFineOnly);
  EXPECT_NE(first.target_cloud, changed.target_cloud);
  EXPECT_EQ(changed.target_points, 80U);
}

// Catch rebuilding/changing B's input, dropping A's nonidentity guess, or
// silently replacing a failed A with a successful B in the diagnostic result.
TEST(MapMatcher, DiagnosticReusesExactInputsAndKeepsFineFailureSeparate)
{
  class RecordingRegistration final : public CoarseRegistration, public FineRegistration
  {
  public:
    RegistrationResult align(const CloudConstPtr & source, const CloudConstPtr & target,
      const std::optional<Pose3d> & guess) const override
    {
      sources.push_back(source);
      targets.push_back(target);
      guesses.push_back(guess);
      RegistrationResult result;
      result.success = sources.size() != 1U;
      result.converged = result.success;
      result.transform = *guess;
      result.fitness = 0.0;
      result.rejection_reason = result.success ? "" : "diagnostic_test_failure";
      return result;
    }
    mutable std::vector<CloudConstPtr> sources, targets;
    mutable std::vector<std::optional<Pose3d>> guesses;
  };
  auto algorithm = std::make_shared<RecordingRegistration>();
  auto pipeline = std::make_shared<RegistrationPipeline>(
    algorithm, algorithm, MatchValidator{{0.1, 0.5, 10U, 5.0, 1.0, 0.0}},
    RegistrationPipelineConfig{0.5});
  auto snapshot = mapSnapshot();
  snapshot.keyframes[0].optimized_pose.translation = {5.0, 2.0, 0.0};
  snapshot.keyframes[0].optimized_pose.rotation = Eigen::AngleAxisd(
    std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ());
  MapMatcher matcher{snapshot, pipeline, {0.0, 1000U}};
  FrontendFrame frame;
  frame.body_cloud = snapshot.keyframes[0].body_cloud;
  SelectionResult selection;
  selection.success = true;
  selection.keyframe_ids = {0U};
  selection.initial_map_body = snapshot.keyframes[0].optimized_pose;

  const auto result = matcher.compareModes(frame, selection);

  EXPECT_FALSE(result.fine_only.success);
  EXPECT_TRUE(result.full.success) << result.full.reason;
  ASSERT_EQ(algorithm->sources.size(), 3U);  // A fine; B coarse, fine.
  for (std::size_t index = 0; index < 3U; ++index) {
    EXPECT_EQ(algorithm->sources[index], frame.body_cloud);
    EXPECT_EQ(algorithm->targets[index], algorithm->targets.front());
    ASSERT_TRUE(algorithm->guesses[index]);
    EXPECT_TRUE(algorithm->guesses[index]->translation.isApprox(Eigen::Vector3d{5, 2, 0}));
    EXPECT_NEAR(algorithm->guesses[index]->rotation.angularDistance(
        selection.initial_map_body.rotation), 0.0, 1e-8);
  }
  EXPECT_EQ(result.fine_only.target_points, result.full.target_points);
  EXPECT_GE(result.fine_only_latency_ms, 0.0);
  EXPECT_GE(result.full_latency_ms, 0.0);
}

}  // namespace
}  // namespace a2w_fastlio_localization
