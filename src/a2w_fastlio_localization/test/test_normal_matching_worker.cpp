#include <chrono>
#include <cmath>
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/normal_matching_worker.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_common::FrontendFrame frame(
  const std::int64_t stamp_ns, const double x, const double yaw_rad = 0.0)
{
  a2w_fastlio_common::FrontendFrame result;
  result.stamp_ns = stamp_ns;
  result.odom_pose.translation.x() = x;
  result.odom_pose.rotation = Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()};
  auto cloud = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  cloud->push_back(a2w_fastlio_common::PointT{});
  result.body_cloud = cloud;
  return result;
}

TEST(NormalMatchMotionGate, SubmitsFirstFrameAndEachConfiguredOrCondition)
{
  constexpr double five_degrees = 5.0 * M_PI / 180.0;
  NormalMatchMotionGate gate{{0.5, five_degrees, 3'000'000'000LL}};
  const auto origin = frame(1'000'000'000LL, 0.0);
  ASSERT_TRUE(gate.shouldSubmit(origin));
  gate.recordSubmission(origin);

  EXPECT_FALSE(gate.shouldSubmit(frame(3'999'999'999LL, 0.49, five_degrees - 0.001)));
  EXPECT_TRUE(gate.shouldSubmit(frame(2'000'000'000LL, 0.5)));
  EXPECT_TRUE(gate.shouldSubmit(frame(2'000'000'000LL, 0.0, five_degrees)));
  EXPECT_TRUE(gate.shouldSubmit(frame(4'000'000'000LL, 0.0)));
}

TEST(NormalMatchingWorker, AllowsOnlyOneActiveMatch)
{
  std::promise<void> evaluator_started;
  std::promise<void> release_evaluator;
  std::promise<NormalMatchingWorkResult> completed;
  auto release = release_evaluator.get_future().share();
  NormalMatchingWorker worker{
    [&](const auto & input) {
      evaluator_started.set_value();
      release.wait();
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      LocalizationOutput output;
      output.stamp_ns = input.stamp_ns;
      output.match_attempted = true;
      return output;
    },
    [&](const auto & result) {completed.set_value(result);}};

  ASSERT_TRUE(worker.trySubmit(frame(10, 0.0)));
  ASSERT_EQ(evaluator_started.get_future().wait_for(std::chrono::seconds{1}),
    std::future_status::ready);
  EXPECT_TRUE(worker.busy());
  EXPECT_FALSE(worker.trySubmit(frame(20, 1.0)));

  release_evaluator.set_value();
  auto result = completed.get_future();
  ASSERT_EQ(result.wait_for(std::chrono::seconds{1}), std::future_status::ready);
  const auto work = result.get();
  EXPECT_EQ(work.frame.stamp_ns, 10);
  EXPECT_GE(work.elapsed_ms, 15.0);
  for (int attempt = 0; attempt < 100 && worker.busy(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  EXPECT_FALSE(worker.busy());
}

TEST(NormalResultPublication, CompletedResultCannotReportAcceptanceAfterLossAndRestore)
{
  LocalizationManager manager{{}, [](const auto &, const auto & predicted, auto) {
      MapMatchResult matched;
      matched.success = true;
      matched.map_body = predicted;
      return matched;
    }};
  manager.restoreCorrection(1, {});
  const auto completed = manager.matchAndUpdate(frame(10, 0.0), MapMatchingMode::kFineOnly);
  ASSERT_TRUE(completed.match_accepted);
  EXPECT_TRUE(normalResultPublication(completed, manager.isCurrentResult(completed),
    LocalizationState::kLocalized).accepted);
  manager.invalidatePendingMatches();
  EXPECT_FALSE(normalResultPublication(completed, manager.isCurrentResult(completed),
    LocalizationState::kLost).accepted);
  manager.restoreCorrection(15, {});
  const auto stale = normalResultPublication(completed, manager.isCurrentResult(completed),
    LocalizationState::kLocalized);
  EXPECT_FALSE(stale.accepted);
  EXPECT_FALSE(stale.usable);
  EXPECT_EQ(stale.reason, "discarded_normal_result:match_accepted");
  const auto fresh = manager.matchAndUpdate(frame(20, 0.0), MapMatchingMode::kFineOnly);
  EXPECT_TRUE(normalResultPublication(fresh, manager.isCurrentResult(fresh),
    LocalizationState::kLocalized).accepted);
}

TEST(NormalResultPublication, RejectedAndExpiredResultsKeepTheirActualReasons)
{
  LocalizationOutput output;
  output.match_attempted = true;
  output.reason = "fitness_above_limit";
  EXPECT_FALSE(normalResultPublication(output, true, LocalizationState::kDegraded).accepted);
  EXPECT_EQ(normalResultPublication(output, true, LocalizationState::kDegraded).reason, output.reason);
  output.match_accepted = true;
  output.result_stale = true;
  output.reason = "expired_match_result";
  EXPECT_FALSE(normalResultPublication(output, false, LocalizationState::kLocalized).accepted);
  EXPECT_FALSE(normalResultPublication(output, false, LocalizationState::kLocalized).usable);
  EXPECT_EQ(normalResultPublication(output, false, LocalizationState::kLocalized).reason, output.reason);
}

TEST(NormalResultPublication, RelocalizingCannotPublishNormalAcceptanceEvenBeforeInvalidation)
{
  LocalizationOutput output;
  output.match_attempted = true;
  output.match_accepted = true;
  output.reason = "match_accepted";
  const auto report = normalResultPublication(output, true, LocalizationState::kRelocalizing);
  EXPECT_FALSE(report.accepted);
  EXPECT_FALSE(report.usable);
  EXPECT_EQ(report.reason, "untrusted_normal_result:match_accepted");
}

}  // namespace
}  // namespace a2w_fastlio_localization
