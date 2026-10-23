#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/committed_loop_audit.hpp"
#include "a2w_fastlio_mapping/loop_pipeline.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

using namespace a2w_fastlio_common;

KeyFrame frame(std::uint64_t id)
{
  KeyFrame result;
  result.id = id;
  result.stamp_ns = static_cast<std::int64_t>(id + 1U) * 1000000000LL;
  for (int point_index = 0; point_index < 40; ++point_index) {
    PointT point;
    point.x = 0.1F * static_cast<float>(point_index);
    point.y = 0.2F * static_cast<float>(point_index % 5);
    point.z = 0.1F * static_cast<float>(point_index % 3);
    result.body_cloud->push_back(point);
  }
  return result;
}

class FakePlaceRecognition final : public PlaceRecognition
{
public:
  ScanDescriptor describe(const CloudConstPtr &) const override
  {
    return ScanDescriptor{1U, 1U, {1.0F}};
  }
};

class ScriptedIndex final : public DescriptorIndex
{
public:
  void add(std::uint64_t id, const ScanDescriptor &) override {added_ids.push_back(id);}

  std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor &, std::size_t k, const CandidateFilter & filter) const override
  {
    ++query_calls;
    last_k = k;
    last_filter = filter;
    if (query_calls <= empty_query_count) {
      return {};
    }
    return candidates;
  }

  mutable std::size_t query_calls{0U};
  mutable std::size_t last_k{0U};
  mutable CandidateFilter last_filter{};
  std::size_t empty_query_count{1U};
  std::vector<LoopCandidate> candidates{};
  std::vector<std::uint64_t> added_ids{};
};

RegistrationResult successfulRegistration()
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.fitness = 0.0;
  result.elapsed_ms = 1.0;
  return result;
}

class ScriptedCoarse final : public CoarseRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    const auto call = calls++;
    if (call < results.size()) {
      return results[call];
    }
    return successfulRegistration();
  }
  mutable std::size_t calls{0U};
  std::vector<RegistrationResult> results{};
};

class RecordingFine final : public FineRegistration
{
public:
  RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    last_source = source;
    if (calls <= results.size()) {return results[calls - 1U];}
    return result;
  }
  mutable std::size_t calls{0U};
  mutable CloudConstPtr last_source{};
  RegistrationResult result{successfulRegistration()};
  std::vector<RegistrationResult> results{};
};

struct Fixture
{
  std::shared_ptr<FakePlaceRecognition> place{std::make_shared<FakePlaceRecognition>()};
  std::shared_ptr<ScriptedIndex> index{std::make_shared<ScriptedIndex>()};
  std::shared_ptr<ScriptedCoarse> coarse{std::make_shared<ScriptedCoarse>()};
  std::shared_ptr<RecordingFine> fine{std::make_shared<RecordingFine>()};
  std::shared_ptr<RegistrationPipeline> registration{
    std::make_shared<RegistrationPipeline>(
      coarse, fine,
      MatchValidator{MatchValidationConfig{0.1, 0.8, 30U, 5.0, 1.0, 0.05}},
      RegistrationPipelineConfig{0.25})};
  std::shared_ptr<PoseGraphOptimizer> graph{
    std::make_shared<PoseGraphOptimizer>(PoseGraphConfig{})};

  LoopPipeline make(LoopPipelineConfig config = {})
  {
    return LoopPipeline{
      place, index, registration, graph, LocalMapBuilder{LocalMapConfig{0.0, 10000U}}, config};
  }
};

void seed(LoopPipeline & pipeline)
{
  EXPECT_FALSE(pipeline.process(frame(0U)).empty());
  EXPECT_FALSE(pipeline.process(frame(1U)).empty());
}

KeyFrame observation(std::uint64_t id, double x, double yaw = 0.0)
{
  auto result = frame(id);
  result.odom_pose.translation.x() = x;
  result.odom_pose.rotation = Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()};
  // The same static scene observed from a moving body; identity map correction
  // remains geometrically valid, while raw odometry motion is independently known.
  for (auto & point : result.body_cloud->points) {
    point.getVector3fMap() = (result.odom_pose.rotation.inverse() *
      (point.getVector3fMap().cast<double>() - result.odom_pose.translation)).cast<float>();
  }
  return result;
}

TEST(LoopPipeline, ScheduledDiscoveryStillAddsEveryFrameAndDescriptor)
{
  Fixture fixture;
  fixture.index->empty_query_count = 0U;
  fixture.index->candidates = {{0U, 0U, .1, 0.0}};
  RegistrationResult failure;failure.rejection_reason = "no_geometry";
  fixture.coarse->results = std::vector<RegistrationResult>(20U, failure);
  LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, .2};
  config.revisit_verification_enabled = true;
  config.discovery_interval_keyframes = 3U;
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));
  EXPECT_NE(pipeline.process(frame(1U)).front().target_points, 0U);
  EXPECT_EQ(pipeline.process(frame(2U)).front().reason, "loop_discovery_interval_pending");
  EXPECT_EQ(pipeline.process(frame(3U)).front().reason, "loop_discovery_interval_pending");
  EXPECT_NE(pipeline.process(frame(4U)).front().target_points, 0U);
  EXPECT_EQ(fixture.graph->optimizedPoses().poses.size(), 5U);
  EXPECT_EQ(fixture.index->added_ids, (std::vector<std::uint64_t>{0, 1, 2, 3, 4}));
  EXPECT_EQ(fixture.coarse->calls, 2U);
}

TEST(LoopPipeline, PendingConfirmationBypassesDiscoveryIntervalOnNextFrame)
{
  Fixture fixture;fixture.index->empty_query_count = 0U;
  fixture.index->candidates = {{0U, 0U, .1, 0.0}};
  LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, .2};
  config.revisit_verification_enabled = true;
  config.discovery_interval_keyframes = 3U;
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));
  EXPECT_EQ(pipeline.process(frame(1U)).back().reason, "loop_confirmation_pending");
  const auto confirmed = pipeline.process(frame(2U));
  EXPECT_TRUE(confirmed.back().accepted);
  EXPECT_EQ(confirmed.back().confirmation_count, 2U);
  EXPECT_EQ(fixture.fine->calls, 2U);
}

TEST(LoopPipeline, ScheduledSelectedAndConfirmationFramesVerifyEveryCandidate)
{
  Fixture fixture;fixture.index->empty_query_count = 0U;
  LoopPipelineConfig config{10U, 0U, 0U, 0U, 1U, 20U, .2};
  config.revisit_verification_enabled = true;
  config.discovery_interval_keyframes = 3U;
  config.confirmation_min_translation_m = .5;
  config.origin_keyframe_count = 100U;
  auto pipeline = fixture.make(config);
  for (std::uint64_t id = 0; id < 10U; ++id) {pipeline.process(frame(id));}
  for (std::uint64_t id = 0; id < 10U; ++id) {
    fixture.index->candidates.push_back({id, static_cast<std::size_t>(id), .1, 0.0});
  }
  const auto selected = pipeline.process(frame(10U));
  EXPECT_EQ(selected.size(), 10U);
  EXPECT_EQ(std::count_if(selected.begin(), selected.end(), [](const auto & event) {
    return event.target_points > 0U;
  }), 10);
  EXPECT_EQ(selected.back().confirmation_count, 1U);
  const auto waiting = pipeline.process(frame(11U));
  EXPECT_EQ(waiting.back().reason, "loop_confirmation_motion_waiting");
  EXPECT_EQ(fixture.coarse->calls, 10U);
  const auto confirmed = pipeline.process(observation(12U, .6));
  EXPECT_EQ(confirmed.size(), 10U);
  EXPECT_EQ(std::count_if(confirmed.begin(), confirmed.end(), [](const auto & event) {
    return event.target_points > 0U;
  }), 10);
  EXPECT_TRUE(confirmed.back().accepted);
  EXPECT_EQ(fixture.coarse->calls, 20U);
}

TEST(LoopPipeline, MotionConfirmationAccumulatesFromLastCountedRawObservation)
{
  Fixture fixture;fixture.index->empty_query_count = 0U;
  fixture.index->candidates = {{0U, 0U, .1, 0.0}};
  LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, .2};
  config.revisit_verification_enabled = true;
  config.discovery_interval_keyframes = 3U;
  config.confirmation_min_translation_m = .5;
  config.confirmation_min_rotation_rad = .15;
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));
  EXPECT_EQ(pipeline.process(observation(1U, 0)).back().confirmation_count, 1U);
  const auto insufficient = pipeline.process(observation(2U, .3));
  EXPECT_FALSE(insufficient.back().accepted);
  EXPECT_EQ(insufficient.back().confirmation_count, 1U);
  EXPECT_EQ(insufficient.back().reason, "loop_confirmation_motion_waiting");
  EXPECT_EQ(fixture.fine->calls, 1U);
  EXPECT_EQ(fixture.index->added_ids, (std::vector<std::uint64_t>{0, 1, 2}));
  EXPECT_EQ(fixture.graph->optimizedPoses().poses.size(), 3U);
  const auto sufficient = pipeline.process(observation(3U, .6));
  EXPECT_TRUE(sufficient.back().accepted);
  EXPECT_EQ(sufficient.back().confirmation_count, 2U);
  EXPECT_EQ(fixture.fine->calls, 2U);  // Waiting frames cannot submit any partial geometric decision.
}

TEST(LoopPipeline, EitherEnabledMotionAxisCanConfirmButZeroAxisCannotBypassGate)
{
  for (const bool rotation_only : {false, true}) {
    Fixture fixture;fixture.index->empty_query_count = 0U;
    fixture.index->candidates = {{0U, 0U, .1, 0.0}};
    LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, .2};
    config.revisit_verification_enabled = true;
    config.confirmation_min_translation_m = rotation_only ? 0.0 : .5;
    config.confirmation_min_rotation_rad = rotation_only ? .15 : 0.0;
    auto pipeline = fixture.make(config);
    pipeline.process(frame(0U));pipeline.process(frame(1U));
    EXPECT_FALSE(pipeline.process(frame(2U)).back().accepted);
    EXPECT_TRUE(pipeline.process(observation(3U, rotation_only ? 0 : .6, rotation_only ? .16 : 0)).back().accepted);
  }
}

TEST(LoopPipeline, MotionInsufficientObservationsCannotRefreshConfirmationMaximumGap)
{
  Fixture fixture;fixture.index->empty_query_count = 0U;
  fixture.index->candidates = {{0U, 0U, .1, 0.0}};
  LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, .2};
  config.revisit_verification_enabled = true;
  config.confirmation_max_gap = 2U;
  config.discovery_interval_keyframes = 10U;
  config.confirmation_min_translation_m = .5;
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));pipeline.process(frame(1U));
  EXPECT_FALSE(pipeline.process(observation(2U, .1)).back().accepted);
  EXPECT_FALSE(pipeline.process(observation(3U, .2)).back().accepted);
  EXPECT_EQ(fixture.fine->calls, 1U);
  const auto expired = pipeline.process(observation(4U, .6));
  EXPECT_FALSE(expired.back().accepted);
  EXPECT_EQ(expired.back().confirmation_count, 1U);
  EXPECT_EQ(fixture.fine->calls, 2U);
}

TEST(LoopPipeline, LegacyIgnoresExperimentalDiscoveryAndMotionOptions)
{
  Fixture fixture;fixture.index->empty_query_count = 0U;
  fixture.index->candidates = {{0U, 0U, .1, 0.0}};
  RegistrationResult failure;failure.rejection_reason = "no_geometry";
  fixture.coarse->results = std::vector<RegistrationResult>(20U, failure);
  LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, .2};
  config.discovery_interval_keyframes = 3U;
  config.confirmation_min_translation_m = .5;
  auto pipeline = fixture.make(config);
  for (std::uint64_t id = 0; id < 4U; ++id) {pipeline.process(frame(id));}
  EXPECT_EQ(fixture.coarse->calls, 3U);
}

TEST(LoopPipeline, IteratesTopKUntilOneCandidateIsAccepted)
{
  Fixture fixture;
  auto failure = RegistrationResult{};
  failure.rejection_reason = "coarse_rejected";
  fixture.coarse->results = {failure, successfulRegistration()};
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{5U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].candidate_id, 0U);
  EXPECT_EQ(events[0].reason, "coarse_registration_failed:coarse_rejected");
  EXPECT_EQ(events[1].candidate_id, 1U);
  EXPECT_TRUE(events[1].accepted);
  EXPECT_EQ(fixture.coarse->calls, 2U);
  EXPECT_EQ(fixture.fine->calls, 1U);
  EXPECT_EQ(events[1].graph_factor_count, 4U);
  EXPECT_EQ(fixture.index->last_k, 5U);
}

TEST(LoopPipeline, RejectedCandidatesDoNotAddLoopFactors)
{
  Fixture fixture;
  auto failure = RegistrationResult{};
  failure.rejection_reason = "no_match";
  fixture.coarse->results = {failure, failure};
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 2U);
  EXPECT_FALSE(events[0].accepted);
  EXPECT_FALSE(events[1].accepted);
  EXPECT_EQ(events.back().graph_factor_count, 3U);
}

TEST(LoopPipeline, AmbiguousDescriptorCandidatesSkipAllRegistration)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.12, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].reason, "candidate_separation_below_limit");
  EXPECT_EQ(events[1].reason, "candidate_separation_below_limit");
  EXPECT_EQ(events[0].candidate_count, 2U);
  EXPECT_EQ(events[1].candidate_count, 2U);
  EXPECT_GE(events[0].descriptor_ms, 0.0);
  EXPECT_GE(events[0].scan_context_ms, 0.0);
  EXPECT_DOUBLE_EQ(events[0].registration_total_ms, 0.0);
  EXPECT_EQ(fixture.coarse->calls, 0U);
  EXPECT_EQ(fixture.fine->calls, 0U);
}

TEST(LoopPipeline, RevisitModeVerifiesSamePlaceDespiteSimilarDescriptorsAndConfirmsNewFrames)
{
  // Removing geometric verification or admitting the first unconfirmed result
  // must fail this test; the real registration evidence and iSAM2 remain active.
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.12, 0.0}};
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  auto pipeline = fixture.make(config);
  seed(pipeline);
  const auto first = pipeline.process(frame(2U));
  ASSERT_FALSE(first.empty());
  EXPECT_FALSE(std::any_of(first.begin(), first.end(), [](const auto & e) {return e.accepted;}));
  EXPECT_EQ(first.back().reason, "loop_confirmation_pending");
  EXPECT_EQ(first.back().graph_factor_count, 3U);
  const auto second = pipeline.process(frame(3U));
  EXPECT_EQ(std::count_if(second.begin(), second.end(), [](const auto & e) {return e.accepted;}), 1);
  EXPECT_EQ(second.back().graph_factor_count, 5U);
}

TEST(LoopPipeline, RevisitModeBoundsFailedHypothesisRetriesButAllowsLaterNewObservations)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  RegistrationResult failure;
  failure.rejection_reason = "no_geometry";
  fixture.coarse->results = std::vector<RegistrationResult>(100U, failure);
  LoopPipelineConfig config{1U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  config.origin_top_k = 1U;
  config.retry_interval_keyframes = 2U;
  config.retry_max_attempts = 3U;
  config.retry_reset_keyframes = 12U;
  auto pipeline = fixture.make(config);
  seed(pipeline);
  for (std::uint64_t id = 2U; id < 12U; ++id) {pipeline.process(frame(id));}
  EXPECT_EQ(fixture.coarse->calls, 3U);
  for (std::uint64_t id = 12U; id < 21U; ++id) {pipeline.process(frame(id));}
  EXPECT_GT(fixture.coarse->calls, 3U);
  EXPECT_EQ(fixture.graph->optimizedPoses().poses.size(), 21U);
}

TEST(LoopPipeline, RevisitModeStillRejectsTwoDistinctGeometricallyValidPlaces)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.11, 0.0}};
  auto shifted = successfulRegistration();
  shifted.transform.translation.x() = 2.0;
  fixture.fine->results = {successfulRegistration(), shifted};
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  config.hypothesis_group_distance_m = 0.25;
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));
  auto historical = frame(1U); historical.odom_pose.translation.x() = 2.0;
  pipeline.process(historical);
  const auto events = pipeline.process(frame(2U));
  ASSERT_EQ(events.size(), 2U);
  EXPECT_TRUE(std::all_of(events.begin(), events.end(), [](const auto & e) {
    return !e.accepted && e.reason == "loop_distinct_solutions_ambiguous";
  }));
  EXPECT_EQ(events.back().graph_factor_count, 3U);
}

TEST(LoopPipeline, NearbyHistoryGroupDoesNotHideASecondDistinctGeometrySolution)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.11, 0.0}};
  auto shifted = successfulRegistration();shifted.transform.translation.x() = 2.0;
  fixture.fine->results = {successfulRegistration(), shifted};
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  config.confirmation_frames = 1U;
  // Both historical centres are within the 3m pre-group radius, but their
  // corrected robot poses differ by 2m. They must remain competing solutions.
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));
  auto historical = frame(1U);historical.odom_pose.translation.x() = 2.0;
  pipeline.process(historical);
  const auto events = pipeline.process(frame(2U));
  EXPECT_TRUE(std::none_of(events.begin(), events.end(), [](const auto & e) {return e.accepted;}));
  EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const auto & e) {
    return e.reason == "loop_distinct_solutions_ambiguous";
  }));
}

class OriginAwareIndex final : public DescriptorIndex
{
public:
  void add(std::uint64_t, const ScanDescriptor &) override {}
  std::vector<LoopCandidate> queryTopK(
    const ScanDescriptor &, std::size_t, const CandidateFilter & filter) const override
  {
    if (!active) {return {};}
    return filter.exclude_recent == 0U ?
           std::vector<LoopCandidate>{{3U, 0U, 0.12, 0.0}} :
           std::vector<LoopCandidate>{{28U, 0U, 0.10, 0.0}};
  }
  bool active{false};
};

TEST(LoopPipeline, OriginDescriptorSupplementCanVerifyCandidateMissingFromGlobalTopK)
{
  Fixture fixture;
  auto index = std::make_shared<OriginAwareIndex>();
  LoopPipelineConfig config{1U, 30U, 0U, 0U, 1U, 10U, 0.2};
  config.revisit_verification_enabled = true;
  config.confirmation_frames = 1U;
  LoopPipeline pipeline{fixture.place, index, fixture.registration, fixture.graph,
    LocalMapBuilder{{0.0, 10000U}}, config};
  for (std::uint64_t id = 0U; id < 61U; ++id) {pipeline.process(frame(id));}
  index->active = true;
  RegistrationResult failure;failure.rejection_reason = "global_candidate_wrong";
  fixture.coarse->results = {failure, successfulRegistration()};
  const auto events = pipeline.process(frame(61U));
  EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const auto & e) {
    return e.accepted && e.candidate_id == 3U;
  }));
}

TEST(LoopPipeline, PositiveEvidenceRechecksDeferredCompetitorsBeforeConfirmation)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.11, 0.0}};
  RegistrationResult failure;
  failure.rejection_reason = "no_geometry";
  fixture.coarse->results = {failure, successfulRegistration(), successfulRegistration(),
    failure, successfulRegistration()};
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  config.hypothesis_group_id_span = 0U;
  auto pipeline = fixture.make(config);
  seed(pipeline);
  const auto first = pipeline.process(frame(2U));
  EXPECT_EQ(first.back().reason, "loop_confirmation_pending");
  const auto confirmed = pipeline.process(frame(3U));
  EXPECT_TRUE(std::any_of(confirmed.begin(), confirmed.end(), [](const auto & e) {
    return e.candidate_id == 0U && e.target_points > 0U &&
           e.reason == "coarse_registration_failed:no_geometry";
  }));
  EXPECT_EQ(std::count_if(confirmed.begin(), confirmed.end(), [](const auto & e) {return e.accepted;}), 1);
}

TEST(LoopPipeline, WorseDescriptorDeferredCandidateMustStillBeGeometricallyRuledOut)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{1U, 0U, 0.10, 0.0}, LoopCandidate{0U, 1U, 0.19, 0.0}};
  RegistrationResult failure;failure.rejection_reason = "initial_insufficient_observation";
  fixture.coarse->results = {successfulRegistration(), failure, successfulRegistration(), successfulRegistration()};
  auto shifted = successfulRegistration();shifted.transform.translation.x() = 2.0;
  fixture.fine->results = {successfulRegistration(), successfulRegistration(), shifted};
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  config.hypothesis_group_distance_m = 0.25;
  auto pipeline = fixture.make(config);
  auto historical = frame(0U);historical.odom_pose.translation.x() = 2.0;
  pipeline.process(historical);
  pipeline.process(frame(1U));
  const auto first = pipeline.process(frame(2U));
  EXPECT_EQ(first.back().reason, "loop_confirmation_pending");
  const auto second = pipeline.process(frame(3U));
  EXPECT_TRUE(std::none_of(second.begin(), second.end(), [](const auto & e) {return e.accepted;}));
  EXPECT_TRUE(std::any_of(second.begin(), second.end(), [](const auto & e) {
    return e.candidate_id == 0U && e.target_points > 0U &&
           e.reason == "loop_distinct_solutions_ambiguous";
  }));
}

TEST(LoopPipeline, FailedHistoryGroupKeepsBudgetWhenDescriptorRepresentativeAlternates)
{
  Fixture fixture;
  RegistrationResult failure;failure.rejection_reason = "no_geometry";
  fixture.coarse->results = std::vector<RegistrationResult>(100U, failure);
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  auto pipeline = fixture.make(config);
  seed(pipeline);
  std::vector<std::uint64_t> attempted_frames;
  for (std::uint64_t id = 2U; id < 12U; ++id) {
    fixture.index->candidates = id % 2U == 0U ?
      std::vector<LoopCandidate>{{0U, 0U, 0.10, 0.0}, {1U, 1U, 0.11, 0.0}} :
      std::vector<LoopCandidate>{{1U, 0U, 0.10, 0.0}, {0U, 1U, 0.11, 0.0}};
    for (const auto & event : pipeline.process(frame(id))) {
      if (event.target_points > 0U) {attempted_frames.push_back(id);}
    }
  }
  EXPECT_EQ(attempted_frames, (std::vector<std::uint64_t>{2U, 2U, 4U, 4U, 6U, 6U}));
}

TEST(LoopPipeline, DistinctHistoricalPlacesDoNotShareRetryBudgetJustBecauseIdsAreAdjacent)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.11, 0.0}};
  RegistrationResult failure;
  failure.rejection_reason = "no_geometry";
  fixture.coarse->results = {failure, successfulRegistration()};
  auto shifted = successfulRegistration();shifted.transform.translation.x() = 2.0;
  fixture.fine->result = shifted;
  LoopPipelineConfig config{2U, 0U, 0U, 0U, 1U, 10U, 1.0};
  config.revisit_verification_enabled = true;
  config.hypothesis_group_distance_m = 0.25;
  config.confirmation_frames = 1U;
  auto pipeline = fixture.make(config);
  pipeline.process(frame(0U));
  auto historical = frame(1U);historical.odom_pose.translation.x() = 2.0;
  pipeline.process(historical);
  const auto events = pipeline.process(frame(2U));
  // Unassessed first place has just been geometrically ruled out in this same frame.
  EXPECT_TRUE(std::any_of(events.begin(), events.end(), [](const auto & e) {
    return e.accepted && e.candidate_id == 1U;
  }));
}

TEST(LoopPipeline, RejectsCandidatesBeyondConfiguredDescriptorDistance)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 0.20});
  seed(pipeline);

  const auto events = pipeline.process(frame(2U));

  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().reason, "no_loop_candidates");
  EXPECT_EQ(fixture.coarse->calls, 0U);
}

TEST(LoopPipeline, AcceptsAtMostOneLoopAndHonorsPolicyWindow)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.30, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 2U, 10U, 1.0});
  seed(pipeline);
  const auto accepted = pipeline.process(frame(2U));
  ASSERT_EQ(accepted.size(), 1U);
  ASSERT_TRUE(accepted.front().accepted);

  const auto cooldown = pipeline.process(frame(3U));

  ASSERT_EQ(cooldown.size(), 1U);
  EXPECT_FALSE(cooldown.front().accepted);
  EXPECT_EQ(cooldown.front().reason, "loop_policy_cooldown");
  EXPECT_EQ(fixture.coarse->calls, 1U);
}

TEST(LoopPipeline, QueueIsBoundedUnderConcurrentProducers)
{
  Fixture fixture;
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 8U, 1.0});
  std::atomic<std::size_t> accepted{0U};
  std::vector<std::thread> producers;
  for (std::uint64_t id = 0U; id < 32U; ++id) {
    producers.emplace_back([&pipeline, &accepted, id]() {
        if (pipeline.enqueue(frame(id))) {
          ++accepted;
        }
      });
  }
  for (auto & producer : producers) {
    producer.join();
  }

  EXPECT_EQ(accepted.load(), 8U);
  EXPECT_EQ(pipeline.pending(), 8U);
}

TEST(LoopPipeline, ProcessNextConsumesOneQueuedFrameOutsideProducerPath)
{
  Fixture fixture;
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 2U, 1.0});
  ASSERT_TRUE(pipeline.enqueue(frame(0U)));
  ASSERT_TRUE(pipeline.enqueue(frame(1U)));

  const auto first = pipeline.processNext();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(pipeline.pending(), 1U);
  const auto second = pipeline.processNext();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(pipeline.pending(), 0U);
  EXPECT_FALSE(pipeline.processNext().has_value());
}

TEST(LoopPipeline, RejectsOutOfOrderFramesWithoutGraphMutation)
{
  Fixture fixture;
  auto pipeline = fixture.make();
  seed(pipeline);
  const auto events = pipeline.process(frame(1U));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().reason, "keyframe_id_not_contiguous");
  EXPECT_EQ(events.front().graph_factor_count, 2U);
}

TEST(LoopPipeline, ProgressIsEmittedBeforeRegistrationAndAfterGraphCommit)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);
  std::vector<LoopProgressStage> stages;
  const auto events = pipeline.process(frame(2U), [&](auto stage, const auto & event) {
      stages.push_back(stage);
      const auto audit = committedLoopAudit(stage, event, {3000000000LL, 4000000000LL, 1.0});
      EXPECT_EQ(audit.has_value(), stage == LoopProgressStage::OptimizationFinished);
      EXPECT_EQ(event.current_id, 2U);
      EXPECT_EQ(event.candidate_id, 0U);
      if (stage == LoopProgressStage::RegistrationStarted) {
        EXPECT_EQ(fixture.coarse->calls, 0U);
        EXPECT_EQ(fixture.fine->calls, 0U);
      }
      if (stage == LoopProgressStage::OptimizationStarted) {
        EXPECT_EQ(fixture.graph->optimizedPoses().poses.size(), 2U);
      }
      if (stage == LoopProgressStage::OptimizationFinished) {
        EXPECT_TRUE(event.accepted);
        EXPECT_EQ(fixture.graph->optimizedPoses().poses.size(), 3U);
        ASSERT_TRUE(audit);
        EXPECT_NE(audit->find("source_id=2 target_id=0 sensor_stamp_ns=3000000000"), std::string::npos);
        EXPECT_NE(audit->find("factors=4"), std::string::npos);
        ASSERT_TRUE(event.optimized_pose_change.has_value());
        EXPECT_NEAR(event.optimized_pose_change->translation.norm(), 0.0, 1e-8);
      }
    });
  EXPECT_EQ(stages, (std::vector<LoopProgressStage>{
    LoopProgressStage::CandidateDetected, LoopProgressStage::RegistrationStarted,
    LoopProgressStage::CandidateFinished, LoopProgressStage::OptimizationStarted,
    LoopProgressStage::OptimizationFinished}));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_TRUE(events.front().accepted);
}

TEST(LoopPipeline, AmbiguityReportsRejectionWithoutClaimingRegistrationOrLoopOptimization)
{
  Fixture fixture;
  fixture.index->candidates = {
    LoopCandidate{0U, 0U, 0.10, 0.0}, LoopCandidate{1U, 1U, 0.12, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);
  std::size_t detected = 0U, rejected = 0U;
  pipeline.process(frame(2U), [&](auto stage, const auto & event) {
      if (stage == LoopProgressStage::CandidateDetected) {++detected; return;}
      EXPECT_EQ(stage, LoopProgressStage::CandidateFinished);
      EXPECT_FALSE(event.accepted);
      EXPECT_EQ(event.reason, "candidate_separation_below_limit");
      ++rejected;
    });
  EXPECT_EQ(detected, 2U);
  EXPECT_EQ(rejected, 2U);
}

TEST(LoopPipeline, RegistrationFailureIsReportedAndDoesNotClaimLoopOptimization)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  RegistrationResult failure;
  failure.rejection_reason = "no_match";
  fixture.coarse->results = {failure};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);
  std::vector<LoopProgressStage> stages;
  pipeline.process(frame(2U), [&](auto stage, const auto & event) {
      stages.push_back(stage);
      if (stage == LoopProgressStage::CandidateFinished) {
        EXPECT_FALSE(event.accepted);
        EXPECT_EQ(event.reason, "coarse_registration_failed:no_match");
      }
    });
  EXPECT_EQ(stages, (std::vector<LoopProgressStage>{
    LoopProgressStage::CandidateDetected, LoopProgressStage::RegistrationStarted,
    LoopProgressStage::CandidateFinished}));
}

TEST(LoopPipeline, ObserverFailureCannotChangeAcceptedLoopAndNoCandidatesAreQuiet)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  pipeline.process(frame(0U), [](auto, const auto &) {FAIL() << "No candidate must be quiet";});
  pipeline.process(frame(1U), [](auto, const auto &) {FAIL() << "No candidate must be quiet";});
  const auto events = pipeline.process(frame(2U), [](auto, const auto &) {
      throw std::runtime_error{"diagnostic sink unavailable"};
    });
  ASSERT_EQ(events.size(), 1U);
  EXPECT_TRUE(events.front().accepted);
  EXPECT_EQ(fixture.graph->optimizedPoses().poses.size(), 3U);
}

TEST(LoopPipeline, ChineseFailureSummaryPreservesStageAndOriginalReason)
{
  EXPECT_EQ(loopRejectionChinese("fine_registration_failed:no convergence"),
    "Nano-GICP 精配准失败（未收敛或结果无效）：fine_registration_failed:no convergence");
  EXPECT_EQ(loopRejectionChinese("fitness_above_limit"), "配准误差超过门限：fitness_above_limit");
  EXPECT_EQ(loopRejectionChinese("rotation_jump_above_limit"),
    "旋转跳变超过门限：rotation_jump_above_limit");
  EXPECT_EQ(loopRejectionChinese("candidate_separation_below_limit"),
    "候选歧义，最佳与次佳区分不足：candidate_separation_below_limit");
  EXPECT_EQ(loopRejectionChinese("unexpected_reason"), "候选未通过：unexpected_reason");
}

TEST(LoopPipeline, LargeRobotPoseCorrectionCannotCreateLoopEvenWithPerfectCloudFit)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  seed(pipeline);
  auto current = frame(2U);
  current.odom_pose.translation.x() = 30.0;
  current.optimized_pose = current.odom_pose;
  fixture.fine->result.transform.translation.x() = -30.0;
  const auto events = pipeline.process(current);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_FALSE(events.front().accepted);
  EXPECT_EQ(events.front().reason, "translation_jump_above_limit");
  EXPECT_EQ(events.front().graph_factor_count, 3U);
  EXPECT_GT(events.front().registration.overlap, 0.99);
  ASSERT_TRUE(events.front().pose_check);
  EXPECT_NEAR(events.front().pose_check->translation_difference_m, 30.0, 1e-6);
  EXPECT_FALSE(events.front().pose_check->position_pass);
  EXPECT_TRUE(events.front().pose_check->rotation_pass);
  EXPECT_NEAR(fixture.graph->optimizedPoses().poses.back().pose.translation.x(), 30.0, 1e-6);
}

TEST(LoopPipeline, CurrentAndHistoricalCloudsUseOptimizedMapFrameAfterAcceptedLoop)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 1U, 0U, 1U, 10U, 1.0});
  seed(pipeline);
  auto previous = frame(2U);
  previous.odom_pose.translation.x() = 0.15;
  previous.optimized_pose = previous.odom_pose;
  fixture.fine->result.transform.translation.x() = -0.10;
  ASSERT_TRUE(pipeline.process(previous).front().accepted);
  const auto optimized_previous = fixture.graph->optimizedPoses().poses.back().pose;
  ASSERT_GT((optimized_previous.translation - previous.odom_pose.translation).norm(), 0.001);
  auto current = frame(3U);
  current.odom_pose.translation.x() = 0.30;
  current.optimized_pose = current.odom_pose;
  for (auto & point : current.body_cloud->points) {point.intensity = 1234.0F;}
  const auto expected = toGtsamPose(optimized_previous).compose(
    toGtsamPose(previous.odom_pose).between(toGtsamPose(current.odom_pose)));
  fixture.fine->result = successfulRegistration();
  pipeline.process(current);
  ASSERT_TRUE(fixture.fine->last_source);
  bool found = false;
  // The first body point is (0,0,0), so its map position equals predicted translation.
  for (const auto & point : fixture.fine->last_source->points) {
    if (point.intensity == 1234.0F &&
      (point.getVector3fMap().cast<double>() - expected.translation()).norm() < 1e-5)
    {found = true;}
  }
  EXPECT_TRUE(found) << "Current cloud must use last optimized pose plus raw odometry increment";
}

TEST(LoopPipeline, RobotPoseGuardDoesNotConfuseMapOriginLeverArmWithRobotDisplacement)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  const Eigen::Quaterniond rotation{Eigen::AngleAxisd{0.1, Eigen::Vector3d::UnitZ()}};
  auto history = frame(0U);
  history.odom_pose.translation.x() = 100.0;
  for (auto & point : history.body_cloud->points) {
    point.getVector3fMap() = (rotation * point.getVector3fMap().cast<double>()).cast<float>();
  }
  pipeline.process(history);
  auto previous = frame(1U);
  previous.odom_pose.translation.x() = 100.0;
  pipeline.process(previous);
  auto current = frame(2U);
  current.odom_pose.translation.x() = 100.0;
  fixture.fine->result.transform.rotation = rotation;
  fixture.fine->result.transform.translation =
    current.odom_pose.translation - rotation * current.odom_pose.translation;
  ASSERT_GT(fixture.fine->result.transform.translation.norm(), 5.0);
  const auto event = pipeline.process(current).front();
  ASSERT_TRUE(event.pose_check);
  EXPECT_TRUE(event.accepted);
  EXPECT_NEAR(event.pose_check->translation_difference_m, 0.0, 1e-6);
  EXPECT_NEAR(event.pose_check->rotation_difference_rad, 0.1, 1e-6);
}

TEST(LoopPipeline, RotationGuardRejectsPerfectFitAboveExistingOneRadianLimit)
{
  Fixture fixture;
  fixture.index->candidates = {LoopCandidate{0U, 0U, 0.10, 0.0}};
  auto pipeline = fixture.make(LoopPipelineConfig{2U, 0U, 0U, 0U, 1U, 10U, 1.0});
  const Eigen::Quaterniond rotation{Eigen::AngleAxisd{1.01, Eigen::Vector3d::UnitX()}};
  auto history = frame(0U);
  for (auto & point : history.body_cloud->points) {
    point.getVector3fMap() = (rotation * point.getVector3fMap().cast<double>()).cast<float>();
  }
  pipeline.process(history);
  pipeline.process(frame(1U));
  fixture.fine->result.transform.rotation = rotation;
  const auto event = pipeline.process(frame(2U)).front();
  ASSERT_TRUE(event.pose_check);
  EXPECT_FALSE(event.accepted);
  EXPECT_EQ(event.reason, "rotation_jump_above_limit");
  EXPECT_TRUE(event.pose_check->position_pass);
  EXPECT_FALSE(event.pose_check->rotation_pass);
  EXPECT_NEAR(event.pose_check->relative_rpy_rad.x(), 1.01, 1e-6);
  EXPECT_EQ(event.graph_factor_count, 3U);
}

TEST(LoopPipeline, PoseCheckLogUsesConfiguredFramesAndShowsActualDecision)
{
  LoopPipelineEvent event;
  event.current_id = 47U;
  event.candidate_id = 16U;
  event.reason = "translation_jump_above_limit";
  LoopPoseCheck check;
  check.translation_difference_m = 29.47;
  check.rotation_difference_rad = 0.066;
  check.translation_limit_m = 5.0;
  check.rotation_limit_rad = 1.0;
  check.rotation_pass = true;
  event.pose_check = check;
  const auto message = loopPoseCheckChinese(event, "factory", "sensor_body");
  EXPECT_NE(message.find("factory <- sensor_body"), std::string::npos);
  EXPECT_NE(message.find("47 -> 16"), std::string::npos);
  EXPECT_NE(message.find("29.470000"), std::string::npos);
  EXPECT_NE(message.find("position_jump_check=REJECT"), std::string::npos);
  EXPECT_NE(message.find("rotation_jump_check=PASS"), std::string::npos);
  EXPECT_NE(message.find("最终结果=REJECT"), std::string::npos);
  event.pose_check.reset();
  EXPECT_NE(loopPoseCheckChinese(event, "map", "body").find("N/A"), std::string::npos);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
