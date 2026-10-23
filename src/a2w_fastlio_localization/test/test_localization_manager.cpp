#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/localization_manager.hpp"
#include "a2w_fastlio_localization/localization_monitor.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_common::FrontendFrame frame(const std::int64_t stamp, const double x)
{
  a2w_fastlio_common::FrontendFrame result;
  result.stamp_ns = stamp;
  result.odom_pose.translation.x() = x;
  auto cloud = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  cloud->push_back(a2w_fastlio_common::PointT{});
  result.body_cloud = cloud;
  return result;
}

MapMatchResult accepted(const double map_x)
{
  MapMatchResult result;
  result.success = true;
  result.reason = "accepted";
  result.map_body.translation.x() = map_x;
  result.registration.success = true;
  return result;
}

LocalizationManagerConfig managerConfig(
  const double maximum_translation_jump_m = 5.0,
  const double maximum_rotation_jump_rad = 1.0)
{
  LocalizationManagerConfig result;
  result.maximum_correction_translation_jump_m = maximum_translation_jump_m;
  result.maximum_correction_rotation_jump_rad = maximum_rotation_jump_rad;
  return result;
}

struct DelayedCorrectionCase
{
  std::int64_t delay_ns;
  double odom_x;
  double odom_y;
  double odom_yaw_deg;
  double expected_map_x;
  double expected_map_y;
  double expected_map_yaw_deg;
};

class HistoricalCorrection : public ::testing::TestWithParam<DelayedCorrectionCase>
{};

a2w_fastlio_common::FrontendFrame historicalFrame(
  const std::int64_t stamp, const double x, const double y, const double yaw_deg)
{
  auto result = frame(stamp, x);
  result.odom_pose.translation.y() = y;
  result.odom_pose.rotation = Eigen::AngleAxisd{
    yaw_deg * std::acos(-1.0) / 180.0, Eigen::Vector3d::UnitZ()};
  return result;
}

TEST_P(HistoricalCorrection, DelayedMatchUsesScanOdomAndPreservesCurrentPropagation)
{
  const auto scenario = GetParam();
  constexpr auto scan_stamp = 100'000'000'000LL;
  std::int64_t steady_now = 1'000;
  LocalizationManager * instance = nullptr;
  auto configured = managerConfig();
  // Geometry and source provenance remain correct even for a slow match when
  // the configured result budget explicitly permits that computation.
  configured.maximum_match_result_age_ns = 15'000'000'000LL;
  LocalizationManager manager{configured, [&](const auto & source, const auto &, const auto) {
      EXPECT_EQ(source.stamp_ns, scan_stamp);
      EXPECT_TRUE(source.odom_pose.translation.isApprox(Eigen::Vector3d{2.0, 1.0, 0.0}));
      // Simulate high rate odometry advancing while an asynchronous match runs.
      instance->propagate(historicalFrame(scan_stamp + scenario.delay_ns,
        scenario.odom_x, scenario.odom_y, scenario.odom_yaw_deg));
      steady_now += scenario.delay_ns;
      auto match = accepted(8.0);
      match.map_body.translation.y() = 6.0;
      match.map_body.rotation = Eigen::AngleAxisd{
        120.0 * std::acos(-1.0) / 180.0, Eigen::Vector3d::UnitZ()};
      return match;
    }, [&]() {return steady_now;}};
  instance = &manager;

  const auto result = manager.matchAndUpdate(
    historicalFrame(scan_stamp, 2.0, 1.0, 30.0), MapMatchingMode::kFull);

  ASSERT_TRUE(result.match_accepted);
  EXPECT_TRUE(result.map_camera_init.translation.isApprox(Eigen::Vector3d{9.0, 4.0, 0.0}));
  EXPECT_NEAR(result.map_camera_init.rotation.angularDistance(Eigen::Quaterniond{
      Eigen::AngleAxisd{std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ()}}), 0.0, 1e-8);
  EXPECT_EQ(result.correction_stamp_ns, scan_stamp);
  EXPECT_EQ(result.correction_age_ns, 0);
  const auto latest = manager.latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->stamp_ns, scan_stamp + scenario.delay_ns);
  EXPECT_TRUE(latest->valid);
  EXPECT_TRUE(latest->map_body.translation.isApprox(Eigen::Vector3d{
      scenario.expected_map_x, scenario.expected_map_y, 0.0}));
  EXPECT_NEAR(latest->map_body.rotation.angularDistance(Eigen::Quaterniond{
      Eigen::AngleAxisd{scenario.expected_map_yaw_deg * std::acos(-1.0) / 180.0,
        Eigen::Vector3d::UnitZ()}}), 0.0, 1e-8);
  EXPECT_EQ(latest->correction_source_age_ns, scenario.delay_ns);
  EXPECT_EQ(latest->correction_age_ns, 0);
}

TEST_P(HistoricalCorrection, HistoricalRestoreSurvivesCurrentPropagationAndRejectsOlderCorrection)
{
  const auto scenario = GetParam();
  constexpr auto scan_stamp = 100'000'000'000LL;
  std::int64_t steady_now = 1'000;
  LocalizationManager manager{managerConfig(), [](const auto &, const auto &, const auto) {
      return MapMatchResult{};
    }, [&]() {return steady_now;}};
  manager.propagate(historicalFrame(scan_stamp + scenario.delay_ns,
    scenario.odom_x, scenario.odom_y, scenario.odom_yaw_deg));
  a2w_fastlio_common::Pose3d correction;
  correction.translation = {9.0, 4.0, 0.0};
  correction.rotation = Eigen::AngleAxisd{std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ()};

  ASSERT_NO_THROW(manager.restoreCorrection(scan_stamp, correction));

  const auto latest = manager.latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->stamp_ns, scan_stamp + scenario.delay_ns);
  EXPECT_TRUE(latest->valid);
  EXPECT_TRUE(latest->map_body.translation.isApprox(Eigen::Vector3d{
      scenario.expected_map_x, scenario.expected_map_y, 0.0}));
  EXPECT_EQ(latest->correction_stamp_ns, scan_stamp);
  EXPECT_EQ(latest->correction_source_age_ns, scenario.delay_ns);
  EXPECT_EQ(latest->correction_age_ns, 0);
  steady_now += 470;
  EXPECT_THROW(manager.restoreCorrection(scan_stamp, {}), std::invalid_argument);
  EXPECT_THROW(manager.restoreCorrection(scan_stamp - 1, {}), std::invalid_argument);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 470);
  EXPECT_TRUE(manager.latest()->map_camera_init.translation.isApprox(correction.translation));
}

TEST_P(HistoricalCorrection, DefaultBudgetAcceptsOneAndThreeSecondsRejectsFiveAndTen)
{
  const auto scenario = GetParam();
  std::int64_t clock = 1'000;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      clock += scenario.delay_ns;
      return accepted(1.0);
    }, [&]() {return clock;}};
  manager.restoreCorrection(10, {});

  const auto result = manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);

  const bool expected_accepted = scenario.delay_ns == 1'000'000'000LL ||
    scenario.delay_ns == 3'000'000'000LL;
  EXPECT_EQ(result.match_accepted, expected_accepted);
  EXPECT_EQ(result.result_stale, !expected_accepted);
  EXPECT_EQ(result.correction_stamp_ns, expected_accepted ? 100 : 10);
  EXPECT_EQ(result.correction_age_ns, expected_accepted ? 0 : scenario.delay_ns);
}

INSTANTIATE_TEST_SUITE_P(OneThreeFiveTenSeconds, HistoricalCorrection, ::testing::Values(
    DelayedCorrectionCase{1'000'000'000LL, 3.0, 1.0, 40.0, 8.0, 7.0, 130.0},
    DelayedCorrectionCase{3'000'000'000LL, 5.0, 2.0, 60.0, 7.0, 9.0, 150.0},
    DelayedCorrectionCase{5'000'000'000LL, 7.0, 4.0, 90.0, 5.0, 11.0, 180.0},
    DelayedCorrectionCase{10'000'000'000LL, 12.0, 7.0, 120.0, 2.0, 16.0, 210.0}));

TEST(LocalizationManager, InitialMatchCreatesCorrectionAndHighRatePoseContinues)
{
  int calls = 0;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      ++calls;
      return accepted(10.0);
    }};
  const auto before_match = manager.propagate(frame(900, 1.0));
  EXPECT_FALSE(before_match.valid);
  EXPECT_EQ(calls, 0);

  const auto first = manager.matchAndUpdate(frame(1000, 2.0), MapMatchingMode::kFull);
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.match_accepted);
  EXPECT_DOUBLE_EQ(first.map_camera_init.translation.x(), 8.0);
  EXPECT_DOUBLE_EQ(first.map_body.translation.x(), 10.0);
  EXPECT_EQ(first.correction_source_age_ns, 0);

  const auto high_rate = manager.propagate(frame(1050, 3.0));
  EXPECT_TRUE(high_rate.valid);
  EXPECT_FALSE(high_rate.match_attempted);
  EXPECT_DOUBLE_EQ(high_rate.map_body.translation.x(), 11.0);
  EXPECT_EQ(high_rate.correction_source_age_ns, 50);
  EXPECT_EQ(calls, 1);
}

TEST(LocalizationManager, FailedMatchDoesNotRefreshCorrectionAge)
{
  int calls = 0;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      return calls++ == 0 ? accepted(0.0) : MapMatchResult{};
    }};
  ASSERT_TRUE(manager.matchAndUpdate(frame(1000, 0.0), MapMatchingMode::kFull).valid);
  const auto failed = manager.matchAndUpdate(frame(1100, 1.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(failed.valid);
  EXPECT_TRUE(failed.match_attempted);
  EXPECT_FALSE(failed.match_accepted);
  EXPECT_EQ(failed.correction_stamp_ns, 1000);
  EXPECT_EQ(failed.correction_source_age_ns, 100);
}

TEST(LocalizationManager, FullSe3MapAlignmentSurvivesCorrectionAndFinePrediction)
{
  // Deliberately non-planar, non-commuting rotations. This is an interface
  // convention test, not historical gravity or real-sensor validation.
  const double half_pi = std::acos(-1.0) / 2.0;
  const Eigen::Quaterniond rx{Eigen::AngleAxisd{half_pi, Eigen::Vector3d::UnitX()}};
  const Eigen::Quaterniond ry{Eigen::AngleAxisd{half_pi, Eigen::Vector3d::UnitY()}};
  const Eigen::Quaterniond rz{Eigen::AngleAxisd{half_pi, Eigen::Vector3d::UnitZ()}};
  int calls = 0;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto & prediction, const auto mode) {
      auto match = accepted(0.0);
      if (++calls == 1) {
        match.map_body.translation = {10.0, 20.0, 30.0};
        match.map_body.rotation = ry;
      } else {
        EXPECT_EQ(mode, MapMatchingMode::kFineOnly);
        EXPECT_TRUE(prediction.translation.isApprox(Eigen::Vector3d{8.0, 23.0, 29.0}, 1e-10));
        EXPECT_NEAR(prediction.rotation.angularDistance(ry * rz), 0.0, 1e-10);
        match.map_body = prediction;
      }
      return match;
    }};
  auto first = frame(100, 1.0);
  first.odom_pose.translation = {1.0, 2.0, 3.0};
  first.odom_pose.rotation = rx;
  const auto matched = manager.matchAndUpdate(first, MapMatchingMode::kFull);
  ASSERT_TRUE(matched.match_accepted);
  EXPECT_TRUE(matched.map_camera_init.translation.isApprox(Eigen::Vector3d{12.0, 17.0, 31.0}, 1e-10));
  EXPECT_NEAR(matched.map_camera_init.rotation.angularDistance(ry * rx.inverse()), 0.0, 1e-10);
  auto next = frame(200, 2.0);
  next.odom_pose.translation = {2.0, 4.0, 6.0};
  next.odom_pose.rotation = rx * rz;
  const auto propagated = manager.propagate(next);
  ASSERT_TRUE(propagated.valid);
  EXPECT_TRUE(propagated.map_body.translation.isApprox(Eigen::Vector3d{8.0, 23.0, 29.0}, 1e-10));
  EXPECT_NEAR(propagated.map_body.rotation.angularDistance(ry * rz), 0.0, 1e-10);
  ASSERT_TRUE(manager.matchAndUpdate(next, MapMatchingMode::kFineOnly).match_accepted);
  EXPECT_EQ(calls, 2);
}

TEST(LocalizationManager, RejectsJumpAndNonMonotonicFrames)
{
  int calls = 0;
  LocalizationManager manager{managerConfig(2.0, 0.5), [&](const auto &, const auto &, const auto) {
      return calls++ == 0 ? accepted(0.0) : accepted(100.0);
    }};
  ASSERT_TRUE(manager.matchAndUpdate(frame(10, 0.0), MapMatchingMode::kFull).valid);
  const auto jumped = manager.matchAndUpdate(frame(11, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_FALSE(jumped.match_accepted);
  EXPECT_EQ(jumped.reason, "correction_jump_rejected");
  EXPECT_DOUBLE_EQ(jumped.map_body.translation.x(), 0.0);
  EXPECT_NO_THROW(manager.propagate(frame(11, 0.0)));
  EXPECT_THROW(manager.propagate(frame(11, 0.0)), std::invalid_argument);
}

TEST(LocalizationManager, GlobalRelocalizationCanAtomicallyRestoreCorrection)
{
  LocalizationManager manager{LocalizationManagerConfig{}, [](const auto &, const auto &, const auto) {
      return MapMatchResult{};
    }};
  a2w_fastlio_common::Pose3d correction;
  correction.translation.x() = 42.0;
  manager.restoreCorrection(5, correction);
  auto input = frame(10, 0.0);

  const auto output = manager.propagate(input);

  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.map_camera_init.translation.x(), 42.0);
  EXPECT_DOUBLE_EQ(output.map_body.translation.x(), 42.0);
  EXPECT_EQ(output.correction_stamp_ns, 5);
  EXPECT_THROW(manager.restoreCorrection(5, correction), std::invalid_argument);
}

// Characterize the existing handoff: source age is not a transform error, and
// the prediction must apply correction BEFORE the current local odometry.
TEST(LocalizationManager, OldGlobalCorrectionPropagatesRotatedCurrentOdomIntoFineGuess)
{
  int calls = 0;
  LocalizationManager manager{managerConfig(), [&](const auto & input, const auto & predicted,
      const auto mode) {
      ++calls;
      EXPECT_EQ(mode, MapMatchingMode::kFineOnly);
      EXPECT_EQ(input.stamp_ns, 40'000'000'000LL);
      EXPECT_TRUE(predicted.translation.isApprox(Eigen::Vector3d{4.0, 5.0, 0.0}));
      EXPECT_NEAR(predicted.rotation.angularDistance(Eigen::Quaterniond{
          Eigen::AngleAxisd{std::acos(-1.0) / 2.0 + 0.5, Eigen::Vector3d::UnitZ()}}),
        0.0, 1e-8);
      return MapMatchResult{};
    }};
  a2w_fastlio_common::Pose3d correction;
  correction.translation = {5.0, 2.0, 0.0};
  correction.rotation = Eigen::AngleAxisd{std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ()};
  manager.restoreCorrection(1'000'000'000LL, correction);
  auto current = frame(40'000'000'000LL, 3.0);
  current.odom_pose.translation.y() = 1.0;
  current.odom_pose.rotation = Eigen::AngleAxisd{0.5, Eigen::Vector3d::UnitZ()};
  const auto propagated = manager.propagate(current);
  EXPECT_TRUE(propagated.valid);
  EXPECT_TRUE(propagated.map_body.translation.isApprox(Eigen::Vector3d{4.0, 5.0, 0.0}));
  EXPECT_EQ(propagated.correction_source_age_ns, 39'000'000'000LL);
  const auto output = manager.matchAndUpdate(current, MapMatchingMode::kFineOnly);
  EXPECT_FALSE(output.match_accepted);
  EXPECT_TRUE(output.map_camera_init.translation.isApprox(correction.translation));
  EXPECT_EQ(calls, 1);
}

TEST(LocalizationManager, PropagationContinuesWhileMatchingIsBlocked)
{
  std::promise<void> matcher_started;
  std::promise<void> release_matcher;
  auto release = release_matcher.get_future().share();
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      matcher_started.set_value();
      release.wait();
      return accepted(10.0);
    }};
  a2w_fastlio_common::Pose3d correction;
  correction.translation.x() = 5.0;
  manager.restoreCorrection(10, correction);

  auto matching = std::async(std::launch::async, [&]() {
      return manager.matchAndUpdate(frame(100, 2.0), MapMatchingMode::kFineOnly);
    });
  ASSERT_EQ(matcher_started.get_future().wait_for(std::chrono::seconds{1}),
    std::future_status::ready);

  auto propagation = std::async(std::launch::async, [&]() {
      return manager.propagate(frame(200, 3.0));
    });
  ASSERT_EQ(propagation.wait_for(std::chrono::milliseconds{100}), std::future_status::ready);
  const auto output = propagation.get();
  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.map_body.translation.x(), 8.0);

  release_matcher.set_value();
  EXPECT_TRUE(matching.get().match_accepted);
}

TEST(LocalizationManager, OlderWorkerResultCannotOverwriteNewerCorrection)
{
  std::promise<void> matcher_started;
  std::promise<void> release_matcher;
  auto release = release_matcher.get_future().share();
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      matcher_started.set_value();
      release.wait();
      return accepted(10.0);
    }};

  auto matching = std::async(std::launch::async, [&]() {
      return manager.matchAndUpdate(frame(100, 2.0), MapMatchingMode::kFineOnly);
    });
  ASSERT_EQ(matcher_started.get_future().wait_for(std::chrono::seconds{1}),
    std::future_status::ready);

  a2w_fastlio_common::Pose3d newer_correction;
  newer_correction.translation.x() = 42.0;
  manager.restoreCorrection(200, newer_correction);
  release_matcher.set_value();

  const auto stale = matching.get();
  EXPECT_TRUE(stale.result_stale);
  EXPECT_FALSE(stale.match_accepted);
  EXPECT_EQ(stale.reason, "stale_match_result");
  const auto propagated = manager.propagate(frame(300, 1.0));
  EXPECT_DOUBLE_EQ(propagated.map_camera_init.translation.x(), 42.0);
  EXPECT_DOUBLE_EQ(propagated.map_body.translation.x(), 43.0);
}

TEST(LocalizationManager, ForwardsRequestedMatchingMode)
{
  auto observed_mode = MapMatchingMode::kFull;
  LocalizationManager manager{managerConfig(),
    [&](const auto &, const auto &, const MapMatchingMode mode) {
      observed_mode = mode;
      return accepted(0.0);
    }};

  const auto output = manager.matchAndUpdate(
    frame(100, 0.0), MapMatchingMode::kFineOnly);

  EXPECT_TRUE(output.match_accepted);
  EXPECT_EQ(observed_mode, MapMatchingMode::kFineOnly);
  EXPECT_EQ(output.matching_mode, MapMatchingMode::kFineOnly);
}

TEST(LocalizationManager, NewlyAcceptedOldSourceIsNotReportedAsStale)
{
  LocalizationManager manager{managerConfig(), [](const auto &, const auto &, const auto) {
      return MapMatchResult{};
    }};
  manager.restoreCorrection(1'000'000'000LL, {});
  const auto output = manager.propagate(frame(40'000'000'000LL, 0.0));
  // This is age since acceptance, not the 39-second-old source timestamp.
  EXPECT_LT(output.correction_age_ns, 100'000'000LL);

  LocalizationMonitor monitor{LocalizationMonitorConfig{}};
  MatchEvidence initial;
  initial.stamp_ns = 1'000'000'000LL;
  initial.match_attempted = true;
  initial.accepted = true;
  initial.correction_available = true;
  initial.source = EvidenceSource::kRelocalization;
  initial.confirmation_complete = true;
  ASSERT_EQ(monitor.update(initial).state, LocalizationState::kLocalized);
  for (const auto stamp : {40'000'000'000LL, 40'100'000'000LL}) {
    const auto failed = manager.matchAndUpdate(frame(stamp, 0.0), MapMatchingMode::kFineOnly);
    MatchEvidence evidence;
    evidence.stamp_ns = stamp;
    evidence.match_attempted = true;
    evidence.correction_available = true;
    evidence.correction_age_ns = failed.correction_age_ns;
    monitor.update(evidence);
  }
  EXPECT_EQ(monitor.latest().state, LocalizationState::kLocalized);
  EXPECT_TRUE(monitor.latest().full_recovery_required);
  EXPECT_EQ(monitor.latest().reason, "full_recovery_pending");
}

TEST(LocalizationManager, AcceptedClockIgnoresSourceAgeAndOnlyRefreshesForValidCorrection)
{
  std::int64_t clock = 1'000;
  int outcome = 0;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      clock += 9'000;  // A slow matcher must not backdate a newly accepted correction.
      return outcome == 0 ? MapMatchResult{} : accepted(outcome == 1 ? 0.0 : 100.0);
    }, [&]() {return clock;}};
  manager.restoreCorrection(1, {});
  clock = 1'470;
  auto propagated = manager.propagate(frame(37'500, 0.0));
  EXPECT_EQ(propagated.correction_age_ns, 470);
  EXPECT_EQ(propagated.correction_source_age_ns, 37'499);
  const auto failed = manager.matchAndUpdate(frame(38'000, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_EQ(failed.correction_age_ns, 9'470);
  EXPECT_EQ(failed.prediction_correction_stamp_ns, 1);
  EXPECT_EQ(failed.prediction_correction_accepted_steady_ns, 1'000);

  outcome = 1;
  const auto success = manager.matchAndUpdate(frame(39'000, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(success.match_accepted);
  EXPECT_EQ(success.correction_age_ns, 0);
  clock += 200;
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 200);
  outcome = 2;
  const auto jumped = manager.matchAndUpdate(frame(40'000, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_EQ(jumped.reason, "correction_jump_rejected");
  EXPECT_EQ(jumped.correction_age_ns, 9'200);
  EXPECT_THROW(manager.restoreCorrection(39'000, {}), std::invalid_argument);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 9'200);
}

TEST(LocalizationManager, StaleWorkerCannotRefreshAcceptedClockOrRewritePredictionProvenance)
{
  std::int64_t clock = 1'000;
  LocalizationManager * instance = nullptr;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      clock = 2'000;
      a2w_fastlio_common::Pose3d newer;
      newer.translation.x() = 42;
      instance->restoreCorrection(200, newer);
      clock = 2'470;
      return accepted(10.0);
    }, [&]() {return clock;}};
  instance = &manager;
  a2w_fastlio_common::Pose3d old;
  old.translation.x() = 5;
  manager.restoreCorrection(10, old);
  const auto output = manager.matchAndUpdate(frame(100, 2.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(output.result_stale);
  EXPECT_EQ(output.correction_stamp_ns, 200);
  EXPECT_EQ(output.correction_age_ns, 470);
  EXPECT_EQ(output.prediction_correction_stamp_ns, 10);
  EXPECT_EQ(output.prediction_correction_accepted_steady_ns, 1'000);
  EXPECT_DOUBLE_EQ(output.predicted_map_body.translation.x(), 7.0);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 470);
}

TEST(LocalizationManager, HistoricalRestoreInvalidatesMatchPredictedBeforeRestoration)
{
  std::int64_t clock = 1'000;
  LocalizationManager * instance = nullptr;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      a2w_fastlio_common::Pose3d recovered;
      recovered.translation.x() = 42.0;
      clock = 2'000;
      // The recovery source is older than the in-flight match source. Source
      // ordering alone cannot protect this newly restored correction.
      instance->restoreCorrection(50, recovered);
      clock = 2'470;
      return accepted(40.0);
    }, [&]() {return clock;}};
  instance = &manager;
  manager.restoreCorrection(10, {});

  const auto output = manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);

  EXPECT_TRUE(output.result_stale);
  EXPECT_FALSE(output.match_accepted);
  EXPECT_EQ(output.correction_stamp_ns, 50);
  EXPECT_DOUBLE_EQ(output.map_camera_init.translation.x(), 42.0);
  EXPECT_EQ(output.correction_age_ns, 470);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 470);
}

TEST(LocalizationManager, ExpiredMatchDoesNotRefreshAcceptedClock)
{
  std::int64_t clock = 1'000;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      clock += 4'000'000'000LL;
      return accepted(1.0);
    }, [&]() {return clock;}};
  manager.restoreCorrection(10, {});

  const auto output = manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);

  EXPECT_TRUE(output.result_stale);
  EXPECT_FALSE(output.match_accepted);
  EXPECT_EQ(output.reason, "expired_match_result");
  EXPECT_EQ(output.correction_stamp_ns, 10);
  EXPECT_DOUBLE_EQ(output.map_camera_init.translation.x(), 0.0);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 4'000'000'000LL);
}

TEST(LocalizationManager, InvalidatedPendingMatchCannotRestoreTrustAfterLoss)
{
  std::int64_t clock = 1'000;
  LocalizationManager * instance = nullptr;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      instance->invalidatePendingMatches();
      clock += 470;
      return accepted(1.0);
    }, [&]() {return clock;}};
  instance = &manager;
  manager.restoreCorrection(10, {});

  const auto output = manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);

  EXPECT_TRUE(output.result_stale);
  EXPECT_FALSE(output.match_accepted);
  EXPECT_EQ(output.reason, "invalidated_match_result");
  EXPECT_EQ(output.correction_stamp_ns, 10);
  EXPECT_DOUBLE_EQ(output.map_camera_init.translation.x(), 0.0);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 470);
}

TEST(LocalizationManager, InvalidationBlocksLateDispatchUntilConfirmedRestore)
{
  std::int64_t clock = 1'000;
  double map_x = 1.0;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      return accepted(map_x);
    }, [&]() {return clock;}};
  manager.restoreCorrection(10, {});
  // A worker may already have removed its job from the queue but only enter
  // the manager after LOST invalidation. Sampling the new generation alone
  // would let that obsolete dispatch refresh the correction.
  manager.invalidatePendingMatches();
  clock += 470;
  const auto late = manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(late.result_stale);
  EXPECT_FALSE(late.match_accepted);
  EXPECT_EQ(late.reason, "invalidated_match_result");
  EXPECT_EQ(late.correction_stamp_ns, 10);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 470);

  a2w_fastlio_common::Pose3d confirmed;
  confirmed.translation.x() = 42.0;
  manager.restoreCorrection(150, confirmed);
  map_x = 43.0;
  const auto fresh = manager.matchAndUpdate(frame(200, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(fresh.match_accepted);
  EXPECT_EQ(fresh.correction_stamp_ns, 200);
}

TEST(LocalizationManager, CompletedInboxResultStaysInvalidAfterLossAndGlobalRestore)
{
  double map_x = 1.0;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      return accepted(map_x);
    }};
  manager.restoreCorrection(10, {});
  // Waiting for this future is the completion barrier: the solve and manager
  // update have finished, while the result consumer has not processed it.
  auto producer = std::async(std::launch::async, [&]() {
      return manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);
    });
  const auto inbox = producer.get();
  ASSERT_TRUE(inbox.match_accepted);
  EXPECT_TRUE(manager.isCurrentResult(inbox));

  manager.invalidatePendingMatches();
  EXPECT_FALSE(manager.isCurrentResult(inbox));
  a2w_fastlio_common::Pose3d restored;
  restored.translation.x() = 42.0;
  manager.restoreCorrection(150, restored);
  EXPECT_FALSE(manager.isCurrentResult(inbox));
  EXPECT_DOUBLE_EQ(manager.latest()->map_camera_init.translation.x(), 42.0);

  map_x = 43.0;
  const auto fresh = manager.matchAndUpdate(frame(200, 0.0), MapMatchingMode::kFineOnly);
  ASSERT_TRUE(fresh.match_accepted);
  EXPECT_TRUE(manager.isCurrentResult(fresh));
  EXPECT_FALSE(manager.isCurrentResult(inbox));
}

TEST(LocalizationManager, ResultBudgetAcceptsBoundaryAndRejectsBeyondIt)
{
  std::int64_t clock = 1'000;
  std::int64_t latency = 3'000'000'000LL;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      clock += latency;
      return accepted(1.0);
    }, [&]() {return clock;}};
  EXPECT_TRUE(manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFull).match_accepted);
  ++latency;
  const auto expired = manager.matchAndUpdate(frame(200, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(expired.result_stale);
  EXPECT_EQ(expired.reason, "expired_match_result");
  EXPECT_EQ(expired.correction_stamp_ns, 100);
  EXPECT_EQ(manager.correctionAcceptedAgeNs(), 3'000'000'001LL);
}

TEST(LocalizationManager, SmallBodyCorrectionFarFromOdomOriginIsAccepted)
{
  LocalizationManager manager{managerConfig(), [](const auto &, const auto &, const auto) {
      auto match = accepted(1000.0);
      match.map_body.rotation = Eigen::AngleAxisd{0.1, Eigen::Vector3d::UnitZ()};
      return match;
    }};
  manager.restoreCorrection(10, {});

  const auto result = manager.matchAndUpdate(frame(100, 1000.0), MapMatchingMode::kFineOnly);

  EXPECT_TRUE(result.match_accepted);
  EXPECT_TRUE(result.map_body.translation.isApprox(Eigen::Vector3d{1000.0, 0.0, 0.0}));
}

TEST(LocalizationManager, LargeBodyCorrectionCannotHideAsSmallRotationAtOdomOrigin)
{
  LocalizationManager manager{managerConfig(), [](const auto &, const auto &, const auto) {
      // A correction with zero translation and yaw 0.1 rad rotates a distant
      // body by almost 100 m despite its harmless-looking origin translation.
      auto match = accepted(1000.0 * std::cos(0.1));
      match.map_body.translation.y() = 1000.0 * std::sin(0.1);
      match.map_body.rotation = Eigen::AngleAxisd{0.1, Eigen::Vector3d::UnitZ()};
      return match;
    }};
  manager.restoreCorrection(10, {});

  const auto result = manager.matchAndUpdate(frame(100, 1000.0), MapMatchingMode::kFineOnly);

  EXPECT_FALSE(result.match_accepted);
  EXPECT_EQ(result.reason, "correction_jump_rejected");
  EXPECT_TRUE(result.map_body.translation.isApprox(Eigen::Vector3d{1000.0, 0.0, 0.0}));
}

TEST(LocalizationManager, HistoricalRotationCannotJumpTheCurrentBodyPose)
{
  LocalizationManager * instance = nullptr;
  LocalizationManager manager{managerConfig(), [&](const auto &, const auto &, const auto) {
      instance->propagate(frame(200, 1000.0));
      auto match = accepted(0.0);
      match.map_body.rotation = Eigen::AngleAxisd{0.1, Eigen::Vector3d::UnitZ()};
      return match;
    }};
  instance = &manager;
  manager.restoreCorrection(10, {});

  const auto result = manager.matchAndUpdate(frame(100, 0.0), MapMatchingMode::kFineOnly);

  EXPECT_FALSE(result.match_accepted);
  EXPECT_EQ(result.reason, "correction_jump_rejected");
  ASSERT_TRUE(manager.latest().has_value());
  EXPECT_EQ(manager.latest()->stamp_ns, 200);
  EXPECT_TRUE(manager.latest()->map_body.translation.isApprox(Eigen::Vector3d{1000.0, 0.0, 0.0}));
}

}  // namespace
}  // namespace a2w_fastlio_localization
