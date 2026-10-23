#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_localization/fine_failure_capture.hpp"

namespace
{

using a2w_fastlio_common::PipelineResult;
using a2w_fastlio_common::Pose3d;

Pose3d compose(const Pose3d & lhs, const Pose3d & rhs)
{
  Pose3d result;
  result.rotation = (lhs.rotation.normalized() * rhs.rotation.normalized()).normalized();
  result.translation = lhs.rotation.normalized() * rhs.translation + lhs.translation;
  return result;
}

Pose3d inverse(const Pose3d & pose)
{
  Pose3d result;
  result.rotation = pose.rotation.normalized().conjugate();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

void printPose(const Pose3d & pose)
{
  std::cout << "xyz=[" << pose.translation.transpose() << "] q_xyzw=["
            << pose.rotation.coeffs().transpose() << "]";
}

void printResult(
  const char * heading, const PipelineResult & result, const double latency_ms,
  const std::size_t target_points, const Pose3d & initial_guess)
{
  std::cout << heading << '\n' << "initial_guess=";
  printPose(initial_guess);
  std::cout << "\ntarget_points=" << target_points
            << " latency_ms=" << latency_ms
            << " convergence=" << (result.fine.converged ? "true" : "false")
            << " accepted=" << (result.success ? "true" : "false")
            << " fitness=" << result.fine.fitness
            << " overlap=" << result.fine.overlap
            << " correspondences=" << result.fine.correspondence_count
            << " reason=" << result.validation.reason << "\nfinal_pose=";
  printPose(result.fine.transform);
  std::cout << '\n';
}

const char * causeText(const a2w_fastlio_localization::OfflineAbcCause cause)
{
  using a2w_fastlio_localization::OfflineAbcCause;
  switch (cause) {
    case OfflineAbcCause::kFineConvergenceBasin:
      return "A失败、B成功：fine-only 收敛域或当前 prediction 问题。";
    case OfflineAbcCause::kLocalMapSwitch:
      return "A、B失败而C成功：Local Map切换或当前关键帧地图问题。";
    case OfflineAbcCause::kCorrectionPrediction:
      return "A、B、C均失败：优先检查前一次 correction 更新造成的 prediction 偏移。";
    case OfflineAbcCause::kOnlineTimingOrLifetime:
      return "在线失败但离线A成功：优先检查异步任务、缓存生命周期或任务时序。";
    default:
      return "结果组合不属于预设四类，需要结合三组指标继续判断。";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "用法: offline_failure_replay CAPTURE_DIRECTORY\n";
    return 2;
  }
  try {
    const auto capture = a2w_fastlio_localization::loadFineFailureCapture(
      std::filesystem::path{argv[1]});
    const auto algorithms = a2w_fastlio_common::createDefaultAlgorithmSuite(capture.algorithms);
    const auto result = a2w_fastlio_localization::runOfflineAbc(capture.abc,
      algorithms.registration);
    std::cout << std::fixed << std::setprecision(6);
    printResult(
      "【A结果】", result.a, result.a_latency_ms,
      capture.abc.current_target->size(), capture.abc.initial_guess);
    printResult(
      "【B结果】", result.b, result.b_latency_ms,
      capture.abc.current_target->size(), capture.abc.initial_guess);
    printResult(
      "【C结果】", result.c, result.c_latency_ms,
      capture.abc.previous_target->size(), capture.abc.initial_guess);

    const auto correction_before = compose(
      capture.previous_prediction_before_update, inverse(capture.previous_odom));
    const auto & correction_after = capture.previous_correction_after_update;
    const auto expected_failure_prediction = compose(correction_after, capture.failure_odom);
    std::cout << "【0.27m correction检查】\n"
              << "correction_translation_update_m="
              << (correction_after.translation - correction_before.translation).norm()
              << " correction_rotation_update_rad="
              << correction_after.rotation.angularDistance(correction_before.rotation)
              << " failure_prediction_translation_error_m="
              << (expected_failure_prediction.translation -
      capture.abc.initial_guess.translation).norm()
              << " failure_prediction_rotation_error_rad="
              << expected_failure_prediction.rotation.angularDistance(
      capture.abc.initial_guess.rotation) << '\n';
    const auto cause = a2w_fastlio_localization::classifyOfflineAbc(
      result.a.success, result.b.success, result.c.success);
    std::cout << "【根因判断】\n" << causeText(cause) << '\n';
  } catch (const std::exception & error) {
    std::cerr << "离线A/B/C失败: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
