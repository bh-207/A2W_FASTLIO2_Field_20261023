#pragma once

#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "a2w_fastlio_common/types.hpp"
#include "a2w_fastlio_localization/localization_manager.hpp"
#include "a2w_fastlio_localization/localization_monitor.hpp"

namespace a2w_fastlio_localization
{

struct NormalMatchMotionGateConfig
{
  double translation_threshold_m{0.5};
  double rotation_threshold_rad{0.08726646259971647};
  std::int64_t maximum_elapsed_ns{3'000'000'000LL};
};

class NormalMatchMotionGate
{
public:
  explicit NormalMatchMotionGate(NormalMatchMotionGateConfig config);
  bool shouldSubmit(const a2w_fastlio_common::FrontendFrame & frame) const;
  void recordSubmission(const a2w_fastlio_common::FrontendFrame & frame);

private:
  NormalMatchMotionGateConfig config_;
  std::optional<a2w_fastlio_common::Pose3d> last_pose_;
  std::optional<std::int64_t> last_stamp_ns_;
};

struct NormalMatchingWorkResult
{
  a2w_fastlio_common::FrontendFrame frame{};
  LocalizationOutput output{};
  double elapsed_ms{0.0};
  double queue_wait_ms{0.0};
};

struct NormalResultPublication
{
  bool accepted{false};
  bool usable{false};
  std::string reason;
};

NormalResultPublication normalResultPublication(
  const LocalizationOutput & output, bool current_generation, LocalizationState state);

class NormalMatchingWorker
{
public:
  using EvaluateFunction = std::function<LocalizationOutput(
      const a2w_fastlio_common::FrontendFrame &)>;
  using ResultCallback = std::function<void(const NormalMatchingWorkResult &)>;

  NormalMatchingWorker(EvaluateFunction evaluator, ResultCallback callback);
  ~NormalMatchingWorker();

  NormalMatchingWorker(const NormalMatchingWorker &) = delete;
  NormalMatchingWorker & operator=(const NormalMatchingWorker &) = delete;

  bool trySubmit(const a2w_fastlio_common::FrontendFrame & frame);
  bool busy() const;

private:
  void run();

  EvaluateFunction evaluator_;
  ResultCallback callback_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<a2w_fastlio_common::FrontendFrame> job_;
  std::chrono::steady_clock::time_point submitted_;
  bool busy_{false};
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace a2w_fastlio_localization
