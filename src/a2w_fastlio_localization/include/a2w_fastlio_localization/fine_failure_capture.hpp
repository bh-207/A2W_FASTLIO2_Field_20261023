#pragma once

#include <filesystem>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_localization/fine_match_diagnostics.hpp"

namespace a2w_fastlio_localization
{

struct FineFailureCaptureConfig
{
  bool enabled{false};
  std::filesystem::path output_root;
};

struct FineFailureCaptureResult
{
  bool captured{false};
  std::filesystem::path directory;
  std::string reason;
};

enum class OfflineAbcCause
{
  kFineConvergenceBasin,
  kLocalMapSwitch,
  kCorrectionPrediction,
  kOnlineTimingOrLifetime,
  kInconclusive,
};

OfflineAbcCause classifyOfflineAbc(bool a_success, bool b_success, bool c_success) noexcept;

struct OfflineAbcInput
{
  a2w_fastlio_common::CloudConstPtr source;
  a2w_fastlio_common::CloudConstPtr current_target;
  a2w_fastlio_common::CloudConstPtr previous_target;
  a2w_fastlio_common::Pose3d initial_guess{};
};

struct OfflineAbcResult
{
  a2w_fastlio_common::PipelineResult a;
  a2w_fastlio_common::PipelineResult b;
  a2w_fastlio_common::PipelineResult c;
  double a_latency_ms{0.0};
  double b_latency_ms{0.0};
  double c_latency_ms{0.0};
};

struct FineFailureReplayData
{
  OfflineAbcInput abc;
  std::vector<std::uint64_t> current_keyframes;
  std::vector<std::uint64_t> previous_keyframes;
  a2w_fastlio_common::DefaultAlgorithmSuiteConfig algorithms;
  std::int64_t failure_source_stamp_ns{0};
  a2w_fastlio_common::Pose3d failure_odom{};
  a2w_fastlio_common::Pose3d failure_correction{};
  a2w_fastlio_common::Pose3d previous_prediction_before_update{};
  a2w_fastlio_common::Pose3d previous_pose_after_update{};
  a2w_fastlio_common::Pose3d previous_odom{};
  a2w_fastlio_common::Pose3d previous_correction_after_update{};
};

FineFailureReplayData loadFineFailureCapture(const std::filesystem::path & directory);

OfflineAbcResult runOfflineAbc(
  const OfflineAbcInput & input,
  const std::shared_ptr<a2w_fastlio_common::RegistrationPipeline> & pipeline);

class FineFailureCapture
{
public:
  explicit FineFailureCapture(FineFailureCaptureConfig config);
  std::optional<FineFailureCaptureResult> observe(
    const FineMatchRecord & record,
    const a2w_fastlio_common::DefaultAlgorithmSuiteConfig & algorithms);

private:
  FineFailureCaptureConfig config_;
  bool captured_{false};
  std::optional<FineMatchRecord> previous_success_;
};

}  // namespace a2w_fastlio_localization
