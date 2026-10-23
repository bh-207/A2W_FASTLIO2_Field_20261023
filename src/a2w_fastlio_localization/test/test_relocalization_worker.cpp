#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/relocalization_worker.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_common::FrontendFrame frame(const std::int64_t stamp)
{
  a2w_fastlio_common::FrontendFrame result;
  result.stamp_ns = stamp;
  auto points = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  points->push_back(a2w_fastlio_common::PointT{});
  result.body_cloud = points;
  return result;
}

TEST(RelocalizationWorker, ReplacesOldPendingFrameWithLatestWithoutGrowingQueue)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false;
  bool release = false;
  std::vector<std::int64_t> completed;
  RelocalizationWorker worker{
    [&](const auto & input) {
      std::unique_lock<std::mutex> lock{mutex};
      entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release;});
      RelocalizationResult result;
      result.reason = std::to_string(input.stamp_ns);
      return result;
    },
    [&](const auto & output) {
      std::lock_guard<std::mutex> lock{mutex};
      completed.push_back(output.frame.stamp_ns);
      condition.notify_all();
    }, 1U};
  EXPECT_TRUE(worker.enqueue(frame(10)));
  {
    std::unique_lock<std::mutex> lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds{1}, [&]() {return entered;}));
  }
  EXPECT_TRUE(worker.enqueue(frame(20)));
  EXPECT_TRUE(worker.enqueue(frame(30)));
  EXPECT_EQ(worker.queued(), 1U);
  {
    std::lock_guard<std::mutex> lock{mutex};
    release = true;
    condition.notify_all();
  }
  {
    std::unique_lock<std::mutex> lock{mutex};
    ASSERT_TRUE(condition.wait_for(
      lock, std::chrono::seconds{1}, [&]() {return completed.size() == 2U;}));
  }
  EXPECT_EQ(completed, (std::vector<std::int64_t>{10, 30}));
}

TEST(RelocalizationWorker, CompletedResultsCarryGenerationForConsumerRevalidation)
{
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<RelocalizationWorkResult> completed;
  RelocalizationWorker worker{
    [](const auto &) {return RelocalizationResult{};},
    [&](const auto & output) {
      std::lock_guard<std::mutex> lock{mutex};
      completed.push_back(output);
      condition.notify_all();
    }, 1U};
  ASSERT_TRUE(worker.enqueue(frame(10)));
  {
    std::unique_lock<std::mutex> lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds{1}, [&] {return completed.size() == 1U;}));
  }
  EXPECT_EQ(completed[0].generation, worker.generation());
  worker.cancelPending();
  EXPECT_NE(completed[0].generation, worker.generation());
  ASSERT_TRUE(worker.enqueue(frame(20)));
  {
    std::unique_lock<std::mutex> lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds{1}, [&] {return completed.size() == 2U;}));
  }
  EXPECT_EQ(completed[1].generation, worker.generation());
}

TEST(RelocalizationWorker, CancellationDropsQueuedAndInFlightSessionResults)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false;
  bool release = false;
  std::vector<std::int64_t> completed;
  RelocalizationWorker worker{
    [&](const auto &) {
      std::unique_lock<std::mutex> lock{mutex};
      entered = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release;});
      return RelocalizationResult{};
    },
    [&](const auto & output) {
      std::lock_guard<std::mutex> lock{mutex};
      completed.push_back(output.frame.stamp_ns);
      condition.notify_all();
    }, 2U};
  worker.enqueue(frame(10));
  {
    std::unique_lock<std::mutex> lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds{1}, [&]() {return entered;}));
  }
  worker.enqueue(frame(20));
  worker.cancelPending();
  {
    std::lock_guard<std::mutex> lock{mutex};
    release = true;
    condition.notify_all();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  EXPECT_TRUE(completed.empty());
}

TEST(RelocalizationWorker, RejectsInvalidConstructionAndFrames)
{
  const RelocalizationWorker::EvaluateFunction evaluator = [](const auto &) {
      return RelocalizationResult{};
    };
  const RelocalizationWorker::ResultCallback callback = [](const auto &) {};
  EXPECT_THROW(RelocalizationWorker(evaluator, callback, 0U), std::invalid_argument);
  EXPECT_THROW(RelocalizationWorker({}, callback, 1U), std::invalid_argument);
  RelocalizationWorker worker{evaluator, callback, 1U};
  auto invalid = frame(10);
  invalid.body_cloud.reset();
  EXPECT_FALSE(worker.enqueue(invalid));
}

}  // namespace
}  // namespace a2w_fastlio_localization
