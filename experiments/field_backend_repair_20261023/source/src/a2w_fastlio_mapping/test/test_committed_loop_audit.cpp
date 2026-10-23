#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "a2w_fastlio_mapping/committed_loop_audit.hpp"

namespace a2w_fastlio_mapping
{
namespace
{
LoopPipelineEvent committedEvent()
{
  LoopPipelineEvent event;
  event.current_id = 42U;
  event.candidate_id = 7U;
  event.accepted = true;
  event.optimized_pose_change = a2w_fastlio_common::Pose3d{};
  event.optimized_pose_change->translation.x() = 0.25;
  event.registration.fitness = 0.125;
  event.registration.overlap = 0.75;
  event.registration.correspondence_count = 123U;
  event.graph_factor_count = 44U;
  event.gtsam_ms = 2.5;
  return event;
}

TEST(CommittedLoopAudit, ReportsSensorWallAndSteadyTimesOnlyAfterActualGraphCommit)
{
  const auto record = committedLoopAudit(
    LoopProgressStage::OptimizationFinished, committedEvent(),
    {123456789000LL, 987654321000LL, 12.5});
  ASSERT_TRUE(record);
  RecordProperty("committed_log", *record);
  EXPECT_EQ(record->find("[LoopCommitted] "), 0U);
  for (const auto * field : {
      "source_id=42", "target_id=7", "sensor_stamp_ns=123456789000",
      "commit_wall_ns=987654321000", "process_to_commit_ms=12.5",
      "sensor_wall_latency_ms=unknown", "clock_relation=unverified",
      "fitness=0.125", "overlap=0.75", "correspondences=123", "factors=44",
      "optimized_translation_m=0.25", "optimized_rotation_rad=0", "gtsam_ms=2.5"})
  {
    EXPECT_NE(record->find(field), std::string::npos) << field;
  }
}

TEST(CommittedLoopAudit, NeverReportsCandidateAcceptanceOrOptimizationStartAsCommit)
{
  for (const auto stage : {
      LoopProgressStage::CandidateDetected, LoopProgressStage::RegistrationStarted,
      LoopProgressStage::CandidateFinished, LoopProgressStage::OptimizationStarted})
  {
    EXPECT_FALSE(committedLoopAudit(stage, committedEvent(), {1, 2, 3.0}));
  }
}

TEST(CommittedLoopAudit, NeverReportsFailedGraphUpdateOrMissingCommittedPose)
{
  auto event = committedEvent();
  event.accepted = false;
  event.reason = "gtsam_update_failed";
  EXPECT_FALSE(committedLoopAudit(LoopProgressStage::OptimizationFinished, event, {1, 2, 3.0}));
  event = committedEvent();
  event.optimized_pose_change.reset();
  EXPECT_FALSE(committedLoopAudit(LoopProgressStage::OptimizationFinished, event, {1, 2, 3.0}));
}

TEST(CommittedLoopAudit, DoesNotManufactureCrossClockOrNegativeLatency)
{
  auto record = committedLoopAudit(
    LoopProgressStage::OptimizationFinished, committedEvent(), {10000000000LL, 1, 4.0});
  ASSERT_TRUE(record);
  EXPECT_NE(record->find("sensor_wall_latency_ms=unknown"), std::string::npos);
  EXPECT_NE(record->find("clock_relation=unverified"), std::string::npos);
  EXPECT_EQ(record->find("latency_ms=-"), std::string::npos);
  for (const auto invalid : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
    record = committedLoopAudit(
      LoopProgressStage::OptimizationFinished, committedEvent(), {1, 2, invalid});
    ASSERT_TRUE(record);
    EXPECT_NE(record->find("process_to_commit_ms=unknown"), std::string::npos);
    EXPECT_NE(record->find("process_clock_status=anomaly"), std::string::npos);
  }
}
}  // namespace
}  // namespace a2w_fastlio_mapping
