#include <gtest/gtest.h>

#include "a2w_fastlio_localization/fine_match_diagnostics.hpp"

namespace a2w_fastlio_localization
{
namespace
{
FineMatchRecord record(std::int64_t stamp, bool accepted)
{
  FineMatchRecord value;
  value.output.stamp_ns = stamp;
  value.output.match_attempted = true;
  value.output.matching_mode = MapMatchingMode::kFineOnly;
  value.output.match_accepted = accepted;
  value.output.reason = accepted ? "match_accepted" : "Nano-GICP did not converge";
  value.output.predicted_map_body.translation.x() = accepted ? 1.0 : 2.0;
  value.output.prediction_correction_stamp_ns = 10;
  value.output.prediction_correction_accepted_steady_ns = 100;
  value.output.correction_stamp_ns = 10;
  value.output.registration.elapsed_ms = 5500;
  value.selection.stamp_ns = stamp;
  value.selection.initial_guess = value.output.predicted_map_body;
  value.selection.keyframes = {{accepted ? 3U : 8U, accepted ? 0.25 : 1.5}};
  value.odom_pose.translation.x() = accepted ? 0.0 : 1.0;
  value.source_age_ns = 5'600'000'000LL;
  value.correction_accepted_age_ns = 5'500'000'000LL;
  value.prediction_correction_accepted_age_ns = value.correction_accepted_age_ns;
  value.match_latency_ms = 5550;
  return value;
}

TEST(FineMatchDiagnostics, OnlyFailuresEmitAndKeepLastAcceptedFineFrame)
{
  FineMatchDiagnostics diagnostics;
  EXPECT_FALSE(diagnostics.observe(record(100, true)));
  const auto failed = diagnostics.observe(record(200, false));
  ASSERT_TRUE(failed);
  EXPECT_NE(failed->find("[fine_only_failure]"), std::string::npos);
  EXPECT_NE(failed->find("source_stamp_ns=200"), std::string::npos);
  EXPECT_NE(failed->find("source_stamp_ns=100"), std::string::npos);
  EXPECT_NE(failed->find("8:1.500000"), std::string::npos);
  EXPECT_NE(failed->find("3:0.250000"), std::string::npos);
  EXPECT_NE(failed->find("Nano-GICP did not converge"), std::string::npos);
  EXPECT_EQ(failed->find('\n'), std::string::npos);  // Exactly one log line.
  EXPECT_FALSE(diagnostics.observe(record(200, false)));  // One per failed task.
  const auto next_failure = diagnostics.observe(record(300, false));
  ASSERT_TRUE(next_failure);
  EXPECT_NE(next_failure->find("source_stamp_ns=100"), std::string::npos);
  EXPECT_EQ(next_failure->find("source_stamp_ns=200"), std::string::npos);
}

TEST(FineMatchDiagnostics, FullAndStaleResultsNeverReplaceLastSuccessfulFineFrame)
{
  FineMatchDiagnostics diagnostics;
  EXPECT_FALSE(diagnostics.observe(record(100, true)));
  auto full = record(200, true);
  full.output.matching_mode = MapMatchingMode::kFull;
  EXPECT_FALSE(diagnostics.observe(full));
  auto stale = record(300, false);
  stale.output.result_stale = true;
  stale.output.reason = "stale_match_result";
  ASSERT_TRUE(diagnostics.observe(stale));
  const auto failure = diagnostics.observe(record(400, false));
  ASSERT_TRUE(failure);
  EXPECT_NE(failure->find("source_stamp_ns=100"), std::string::npos);
  EXPECT_EQ(failure->find("source_stamp_ns=300"), std::string::npos);
}

TEST(FineMatchDiagnostics, ReportsMissingPreviousAndMismatchedSelectionWithoutInventingData)
{
  FineMatchDiagnostics diagnostics;
  auto failed = record(100, false);
  failed.selection.stamp_ns = 99;
  const auto summary = diagnostics.observe(failed);
  ASSERT_TRUE(summary);
  EXPECT_NE(summary->find("上一成功=无"), std::string::npos);
  EXPECT_NE(summary->find("selection_stamp_ns=99"), std::string::npos);
  EXPECT_NE(summary->find("selection_stamp_matches=0"), std::string::npos);
}

TEST(FineMatchDiagnostics, SelectionDistancesAreFromOptimizedPoseToActualPrediction)
{
  a2w_fastlio_common::KeyFrame keyframe;
  keyframe.id = 42;
  keyframe.optimized_pose.translation = {4.0, 6.0, 0.0};
  SelectionResult selection;
  selection.success = true;
  selection.keyframe_ids = {42};
  selection.initial_map_body.translation = {1.0, 2.0, 0.0};
  const auto captured = captureFineSelection(100, 82, selection, {keyframe});
  EXPECT_EQ(captured.stamp_ns, 100);
  EXPECT_EQ(captured.source_age_at_start_ns, 82);
  ASSERT_EQ(captured.keyframes.size(), 1U);
  EXPECT_EQ(captured.keyframes[0].first, 42U);
  EXPECT_DOUBLE_EQ(captured.keyframes[0].second, 5.0);
  EXPECT_TRUE(captured.initial_guess.translation.isApprox(Eigen::Vector3d{1, 2, 0}));
}
}  // namespace
}  // namespace a2w_fastlio_localization
