#include <memory>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/global_relocalizer.hpp"

namespace a2w_fastlio_localization
{
namespace
{

using namespace a2w_fastlio_common;

CloudPtr cloud()
{
  auto result = CloudPtr{new Cloud{}};
  for (int index = 0; index < 40; ++index) {
    PointT point;
    point.x = 0.1F * static_cast<float>(index);
    point.y = 0.2F * static_cast<float>(index % 5);
    point.z = 0.1F * static_cast<float>(index % 3);
    result->push_back(point);
  }
  return result;
}

class ScriptedIndex final : public DescriptorIndex
{
public:
  void add(std::uint64_t, const ScanDescriptor &) override {}
  std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor &, const std::size_t k, const CandidateFilter & filter) const override
  {
    last_k = k;
    last_filter = filter;
    return candidates;
  }
  std::vector<LoopCandidate> candidates;
  mutable std::size_t last_k{0U};
  mutable CandidateFilter last_filter{};
};

class Provider final : public KeyFrameProvider
{
public:
  std::optional<KeyFrame> get(const std::uint64_t id) const override
  {
    for (const auto & item : frames) {
      if (item.id == id) {
        return item;
      }
    }
    return std::nullopt;
  }
  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    return frames.empty() ? std::nullopt :
      std::optional<std::pair<std::uint64_t, std::uint64_t>>{{0U, frames.back().id}};
  }
  std::vector<KeyFrame> frames;
};

RegistrationResult usable(const double x)
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.transform.translation.x() = x;
  result.fitness = 0.0;
  result.elapsed_ms = 0.1;
  return result;
}

class Coarse final : public CoarseRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr & target,
    const std::optional<Pose3d> & hint) const override
  {
    ++calls;
    EXPECT_TRUE(hint.has_value());
    return usable(target->front().x);
  }
  mutable std::size_t calls{0U};
};

class Fine final : public FineRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr & target,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    const double target_x = target->front().x;
    // The rank-zero repeated structure deliberately converges to a false transform.
    return usable(target_x > 50.0 ? 0.0 : target_x);
  }
  mutable std::size_t calls{0U};
};

KeyFrame frame(const std::uint64_t id, const double x)
{
  KeyFrame result;
  result.id = id;
  result.optimized_pose.translation.x() = x;
  result.body_cloud = cloud();
  return result;
}

RelocalizationConfig config()
{
  RelocalizationConfig result;
  result.top_k = 5U;
  result.neighbor_keyframes_before = 0U;
  result.neighbor_keyframes_after = 0U;
  result.maximum_descriptor_distance = 1.0;
  result.minimum_score_margin = 0.05;
  result.descriptor_score_weight = 0.0;
  result.fitness_score_weight = 1.0;
  result.overlap_score_weight = 1.0;
  result.strong_maximum_fitness = 0.01;
  result.strong_minimum_overlap = 0.9;
  result.strong_minimum_correspondences = 30U;
  return result;
}

GlobalRelocalizer make(
  const std::shared_ptr<ScriptedIndex> & index, const std::shared_ptr<Provider> & provider,
  const std::shared_ptr<Coarse> & coarse, const std::shared_ptr<Fine> & fine)
{
  return GlobalRelocalizer{
    index, provider, coarse, fine,
    MatchValidator{MatchValidationConfig{0.05, 0.8, 30U, 1000.0, 3.0, 0.0}},
    LocalMapConfig{0.0, 10000U}, RegistrationPipelineConfig{0.25}};
}

TEST(GlobalRelocalizer, AuditsTopKAndGeometryBeatsScanContextRank)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {
    {0U, 0U, 0.01, 0.0}, {99U, 1U, 0.02, 0.0}, {1U, 2U, 0.20, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {frame(0U, 100.0), frame(1U, 5.0)};
  const auto coarse = std::make_shared<Coarse>();
  const auto fine = std::make_shared<Fine>();

  const auto result = make(index, provider, coarse, fine).evaluate(
    cloud(), ScanDescriptor{1U, 1U, {1.0F}}, config());

  ASSERT_EQ(result.audits.size(), 3U);
  EXPECT_EQ(result.audits[0].reason, "fitness_above_limit");
  ASSERT_TRUE(result.audits[0].candidate_map_pose.has_value());
  EXPECT_DOUBLE_EQ(result.audits[0].candidate_map_pose->translation.x(), 100.0);
  EXPECT_EQ(result.audits[0].target_points, 40U);
  EXPECT_TRUE(result.audits[0].coarse.success);
  EXPECT_TRUE(result.audits[0].registration.converged);
  EXPECT_EQ(result.audits[1].reason, "candidate_local_map_invalid");
  EXPECT_TRUE(result.audits[2].accepted);
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.strong);
  EXPECT_EQ(result.candidate_id, 1U);
  EXPECT_DOUBLE_EQ(result.map_body.translation.x(), 5.0);
  EXPECT_EQ(coarse->calls, 2U);
  EXPECT_EQ(fine->calls, 2U);
  EXPECT_EQ(index->last_k, 5U);
  EXPECT_EQ(index->last_filter.exclude_recent, 0U);
}

TEST(GlobalRelocalizer, RejectsTwoGeometricallyValidRepeatedStructuresAsAmbiguous)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {{2U, 0U, 0.01, 0.0}, {3U, 1U, 0.02, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {frame(2U, 0.0), frame(3U, 5.0)};
  const auto coarse = std::make_shared<Coarse>();
  const auto fine = std::make_shared<Fine>();

  const auto result = make(index, provider, coarse, fine).evaluate(
    cloud(), ScanDescriptor{1U, 1U, {1.0F}}, config());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "candidate_score_ambiguous");
  ASSERT_TRUE(result.best_candidate_id.has_value());
  EXPECT_EQ(*result.best_candidate_id, 2U);
  ASSERT_EQ(result.audits.size(), 2U);
  EXPECT_TRUE(result.audits[0].accepted);
  EXPECT_TRUE(result.audits[1].accepted);
}

TEST(GlobalRelocalizer, NearbyFinalPosesAreOneHypothesisDespiteSimilarScores)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {{7U, 0U, 0.007, 0.0}, {8U, 1U, 0.010, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {frame(7U, 5.0), frame(8U, 5.02)};
  const auto result = make(index, provider, std::make_shared<Coarse>(),
      std::make_shared<Fine>()).evaluate(cloud(), ScanDescriptor{1U, 1U, {1.0F}}, config());

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(std::isinf(result.second_best_score));
  EXPECT_EQ(result.audits.size(), 2U);
}

TEST(GlobalRelocalizer, NearbyDuplicatesDoNotHideAnotherValidLocation)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {{7U, 0U, 0.007, 0.0}, {8U, 1U, 0.010, 0.0},
    {9U, 2U, 0.012, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {frame(7U, 5.0), frame(8U, 5.02), frame(9U, 10.0)};
  const auto result = make(index, provider, std::make_shared<Coarse>(),
      std::make_shared<Fine>()).evaluate(cloud(), ScanDescriptor{1U, 1U, {1.0F}}, config());

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "candidate_score_ambiguous");
  EXPECT_TRUE(std::isfinite(result.second_best_score));
}

TEST(GlobalRelocalizer, FixedRegionFiltersBeforeTopKAndExcludesReturnVisit)
{
  const auto index = std::make_shared<ScriptedIndex>();
  index->candidates = {
    {300U, 0U, 0.001, 0.0},  // Same physical start after a drifted return visit.
    {40U, 1U, 0.002, 0.0},   // Outside the configured 3 x 3 metre region.
    {10U, 2U, 0.010, 0.0},
    {0U, 3U, 0.020, 0.0}};
  const auto provider = std::make_shared<Provider>();
  provider->frames = {
    frame(0U, 0.0), frame(10U, 1.0), frame(40U, 5.0), frame(300U, 0.5)};
  auto bounded = config();
  bounded.top_k = 2U;
  bounded.candidate_region.enabled = true;
  bounded.candidate_region.center_x = 0.0;
  bounded.candidate_region.center_y = 0.0;
  bounded.candidate_region.size_x = 3.0;
  bounded.candidate_region.size_y = 3.0;
  bounded.candidate_region.anchor_keyframe_id = 0U;
  bounded.candidate_region.keyframe_window = 30U;

  const auto result = make(
    index, provider, std::make_shared<Coarse>(), std::make_shared<Fine>()).evaluate(
    cloud(), ScanDescriptor{1U, 1U, {1.0F}}, bounded);

  ASSERT_EQ(result.audits.size(), 2U);
  EXPECT_EQ(result.audits[0].candidate.keyframe_id, 10U);
  EXPECT_EQ(result.audits[0].candidate.rank, 0U);
  EXPECT_EQ(result.audits[1].candidate.keyframe_id, 0U);
  EXPECT_EQ(result.audits[1].candidate.rank, 1U);
  EXPECT_EQ(index->last_k, std::numeric_limits<std::size_t>::max());
}

TEST(GlobalRelocalizer, FormatsOneChineseInitializationDiagnostic)
{
  RelocalizationResult result;
  result.source_points = 10000U;
  result.elapsed_ms = 8123.0;
  result.time_budget_ms = 5000.0;
  result.reason = "candidate_score_ambiguous";
  result.best_candidate_id = 24U;
  CandidateAudit audit;
  audit.candidate = {24U, 0U, 0.12, 0.25};
  audit.candidate_map_pose = Pose3d{};
  audit.candidate_map_pose->translation.x() = 1.5;
  audit.candidate_map_pose->translation.y() = -2.0;
  audit.target_points = 1659U;
  audit.coarse = usable(1.5);
  audit.registration = usable(1.5);
  audit.registration.fitness = 0.14;
  audit.registration.overlap = 0.97;
  audit.registration.correspondence_count = 9500U;
  audit.reason = "accepted";
  result.audits.push_back(audit);
  RelocalizationSessionSnapshot session;
  session.active = true;
  session.reason = "confirmation_pending";

  const auto summary = formatInitializationDiagnostic(result, session);

  EXPECT_NE(summary.find("【Top-K候选】"), std::string::npos);
  EXPECT_NE(summary.find("id=24"), std::string::npos);
  EXPECT_NE(summary.find("SC=0.120000"), std::string::npos);
  EXPECT_NE(summary.find("target=1659"), std::string::npos);
  EXPECT_NE(summary.find("Quatro=成功"), std::string::npos);
  EXPECT_NE(summary.find("Nano-GICP=收敛"), std::string::npos);
  EXPECT_NE(summary.find("【最终拒绝层级】ambiguity / best-second margin"), std::string::npos);
  EXPECT_NE(summary.find("耗时告警=超出预算"), std::string::npos);
}

TEST(GlobalRelocalizer, RejectsInvalidConfigurationAndEmptyInputs)
{
  const auto index = std::make_shared<ScriptedIndex>();
  const auto provider = std::make_shared<Provider>();
  const auto coarse = std::make_shared<Coarse>();
  const auto fine = std::make_shared<Fine>();
  auto invalid = config();
  invalid.top_k = 1U;
  EXPECT_THROW(
    make(index, provider, coarse, fine).evaluate(
      cloud(), ScanDescriptor{1U, 1U, {1.0F}}, invalid), std::invalid_argument);
  invalid = config();
  invalid.candidate_region.enabled = true;
  invalid.candidate_region.size_x = 0.0;
  invalid.candidate_region.size_y = 3.0;
  invalid.candidate_region.keyframe_window = 30U;
  EXPECT_THROW(
    make(index, provider, coarse, fine).evaluate(
      cloud(), ScanDescriptor{1U, 1U, {1.0F}}, invalid), std::invalid_argument);
  EXPECT_THROW(
    make(index, provider, coarse, fine).evaluate(
      CloudPtr{new Cloud{}}, ScanDescriptor{1U, 1U, {1.0F}}, config()),
    std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_localization
