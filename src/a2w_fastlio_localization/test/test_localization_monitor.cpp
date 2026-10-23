#include <limits>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/localization_monitor.hpp"

namespace a2w_fastlio_localization
{
namespace
{

LocalizationMonitorConfig config();

MatchEvidence evidence(
  const std::int64_t stamp, const bool accepted,
  const EvidenceSource source = EvidenceSource::kNormal,
  const NormalMatchKind normal_match_kind = NormalMatchKind::kFineOnly)
{
  MatchEvidence result;
  result.stamp_ns = stamp;
  result.match_attempted = true;
  result.accepted = accepted;
  result.correction_available = accepted;
  result.source = source;
  result.normal_match_kind = normal_match_kind;
  result.candidate_id = 7U;
  result.registration.fitness = accepted ? 0.05 : 1.0;
  result.registration.overlap = accepted ? 0.8 : 0.0;
  result.registration.correspondence_count = accepted ? 100U : 0U;
  return result;
}

TEST(LocalizationMonitor, FineFailuresRequestFullRecoveryWithoutLeavingLocalized)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  ASSERT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);

  const auto first_failure = monitor.update(evidence(20, false));
  EXPECT_EQ(first_failure.state, LocalizationState::kLocalized);
  EXPECT_FALSE(first_failure.full_recovery_required);
  EXPECT_EQ(first_failure.consecutive_failures, 1U);

  const auto recovery_pending = monitor.update(evidence(30, false));
  EXPECT_EQ(recovery_pending.state, LocalizationState::kLocalized);
  EXPECT_TRUE(recovery_pending.full_recovery_required);
  EXPECT_EQ(recovery_pending.reason, "full_recovery_pending");
  EXPECT_EQ(recovery_pending.consecutive_failures, 2U);
}

TEST(LocalizationMonitor, FullRecoverySuccessKeepsLocalizedAndClearsFailureBudget)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  ASSERT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);
  monitor.update(evidence(20, false));
  ASSERT_TRUE(monitor.update(evidence(30, false)).full_recovery_required);

  const auto recovered = monitor.update(
    evidence(40, true, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  EXPECT_EQ(recovered.state, LocalizationState::kLocalized);
  EXPECT_FALSE(recovered.full_recovery_required);
  EXPECT_EQ(recovered.consecutive_failures, 0U);
  EXPECT_EQ(recovered.reason, "full_recovery_accepted");
}

TEST(LocalizationMonitor, FullRecoveryFailureEntersDegradedThenFullFailuresReachLost)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  ASSERT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);
  monitor.update(evidence(20, false));
  ASSERT_TRUE(monitor.update(evidence(30, false)).full_recovery_required);

  const auto degraded = monitor.update(
    evidence(40, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  EXPECT_EQ(degraded.state, LocalizationState::kDegraded);
  EXPECT_FALSE(degraded.full_recovery_required);
  EXPECT_EQ(degraded.reason, "full_recovery_failed");

  const auto lost = monitor.update(
    evidence(50, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  EXPECT_EQ(lost.state, LocalizationState::kLost);
  EXPECT_EQ(lost.reason, "consecutive_match_failures");
}

LocalizationMonitorConfig config()
{
  LocalizationMonitorConfig result;
  result.initialization_successes_required = 2U;
  result.degraded_failures_required = 2U;
  result.lost_failures_required = 4U;
  result.normal_recovery_successes_required = 2U;
  result.relocalization_successes_required = 3U;
  result.correction_stale_after_ns = 100;
  result.relocalization_timeout_ns = 500;
  result.strong_maximum_fitness = 0.02;
  result.strong_minimum_overlap = 0.9;
  result.strong_minimum_correspondences = 150U;
  return result;
}

TEST(LocalizationMonitor, CoversInitializationDegradationLossAndReset)
{
  LocalizationMonitor monitor{config()};
  EXPECT_EQ(monitor.latest().state, LocalizationState::kInitializing);
  EXPECT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kInitializing);
  EXPECT_EQ(monitor.update(evidence(20, true)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(30, false)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(40, false)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(
      50, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(
      60, true, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(
      70, true, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(80, false)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(90, false)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(
      100, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(
      110, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kLost);
  const auto reset = monitor.reset(120);
  EXPECT_EQ(reset.state, LocalizationState::kInitializing);
  EXPECT_EQ(reset.reason, "reset");
}

TEST(LocalizationMonitor, StaleCorrectionStillRespectsFailureBudget)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  EXPECT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);

  auto propagation = evidence(110, false);
  propagation.match_attempted = false;
  propagation.correction_available = true;
  propagation.correction_age_ns = 100;
  EXPECT_EQ(monitor.update(propagation).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.latest().reason, "no_match_attempt");

  auto failed_match = evidence(120, false);
  failed_match.correction_available = true;
  failed_match.correction_age_ns = 110;
  const auto first_failure = monitor.update(failed_match);
  EXPECT_EQ(first_failure.state, LocalizationState::kLocalized);
  EXPECT_EQ(first_failure.consecutive_failures, 1U);
  EXPECT_EQ(monitor.latest().reason, "match_rejected");

  for (std::int64_t stamp = 130; stamp < 160; stamp += 10) {
    failed_match.stamp_ns = stamp;
    failed_match.correction_age_ns += 10;
    if (stamp >= 140) {
      failed_match.normal_match_kind = NormalMatchKind::kFullRecovery;
    }
    const auto status = monitor.update(failed_match);
    if (stamp == 130) {
      EXPECT_EQ(status.state, LocalizationState::kLocalized);
      EXPECT_TRUE(status.full_recovery_required);
    } else if (stamp < 150) {
      EXPECT_EQ(status.state, LocalizationState::kDegraded);
    } else {
      EXPECT_EQ(status.state, LocalizationState::kLost);
      EXPECT_EQ(status.consecutive_failures, configured.lost_failures_required);
      EXPECT_EQ(status.reason, "correction_stale");
    }
  }
}

TEST(LocalizationMonitor, InitialGlobalRelocalizationRequiresExternalConfirmation)
{
  LocalizationMonitor monitor{config()};
  auto pending = evidence(10, true, EvidenceSource::kRelocalization);
  pending.defer_confirmation = true;

  const auto first = monitor.update(pending);

  EXPECT_EQ(first.state, LocalizationState::kInitializing);
  EXPECT_EQ(first.reason, "initial_global_confirmation_pending");
  pending.stamp_ns = 20;
  pending.confirmation_complete = true;
  const auto confirmed = monitor.update(pending);
  EXPECT_EQ(confirmed.state, LocalizationState::kLocalized);
  EXPECT_EQ(confirmed.reason, "initial_global_localization_confirmed");
}

TEST(LocalizationMonitor, LostRequiresExplicitRelocalizationAndConsistentRecovery)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  ASSERT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);
  ASSERT_EQ(monitor.update(evidence(20, false)).state, LocalizationState::kLocalized);
  ASSERT_EQ(monitor.update(evidence(
      30, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kDegraded);
  ASSERT_EQ(monitor.update(evidence(
      40, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery)).state,
    LocalizationState::kLost);
  EXPECT_EQ(monitor.update(evidence(50, true)).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.latest().reason, "consecutive_match_failures");
  EXPECT_EQ(monitor.beginRelocalization(60).state, LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(70, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(80, false, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(90, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(100, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(110, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, StrongRelocalizationCanRecoverOnceAndTimeoutReturnsLost)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  monitor.update(evidence(20, false));
  monitor.update(evidence(30, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.update(evidence(40, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.beginRelocalization(50);
  auto strong = evidence(60, true, EvidenceSource::kRelocalization);
  strong.registration.fitness = 0.01;
  strong.registration.overlap = 0.95;
  strong.registration.correspondence_count = 200U;
  EXPECT_EQ(monitor.update(strong).state, LocalizationState::kLocalized);

  monitor.update(evidence(70, false));
  monitor.update(evidence(80, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.update(evidence(90, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.beginRelocalization(100);
  auto idle = evidence(600, false, EvidenceSource::kRelocalization);
  idle.match_attempted = false;
  EXPECT_EQ(monitor.update(idle).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.latest().reason, "relocalization_timeout");
}

TEST(LocalizationMonitor, ExternalConsistencyPolicyGatesRecovery)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  monitor.update(evidence(20, false));
  monitor.update(evidence(30, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.update(evidence(40, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.beginRelocalization(50);
  auto pending = evidence(60, true, EvidenceSource::kRelocalization);
  pending.defer_confirmation = true;
  EXPECT_EQ(monitor.update(pending).state, LocalizationState::kRelocalizing);
  pending.stamp_ns = 570;
  EXPECT_EQ(monitor.update(pending).state, LocalizationState::kRelocalizing);
  pending.stamp_ns = 1080;
  pending.confirmation_complete = true;
  EXPECT_EQ(monitor.update(pending).state, LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, ExternalSessionTimeoutStillReturnsLost)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  monitor.update(evidence(20, false));
  monitor.update(evidence(30, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.update(evidence(40, false, EvidenceSource::kNormal, NormalMatchKind::kFullRecovery));
  monitor.beginRelocalization(50);
  auto failed = evidence(1000, false, EvidenceSource::kRelocalization);
  failed.defer_confirmation = true;
  failed.confirmation_timed_out = true;
  EXPECT_EQ(monitor.update(failed).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.latest().reason, "relocalization_timeout");
}

TEST(LocalizationMonitor, RejectsInvalidConfigurationAndNonMonotonicEvidence)
{
  auto invalid = config();
  invalid.lost_failures_required = invalid.degraded_failures_required - 1U;
  EXPECT_THROW(LocalizationMonitor{invalid}, std::invalid_argument);
  invalid = config();
  invalid.strong_maximum_fitness = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(LocalizationMonitor{invalid}, std::invalid_argument);

  LocalizationMonitor monitor{config()};
  monitor.update(evidence(10, true));
  EXPECT_THROW(monitor.update(evidence(10, true)), std::invalid_argument);
  EXPECT_THROW(monitor.beginRelocalization(9), std::invalid_argument);
}

TEST(LocalizationMonitor, RejectedRelocalizationCommandDoesNotAdvanceStateTimestamp)
{
  LocalizationMonitor monitor{config()};
  ASSERT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kInitializing);

  EXPECT_THROW(monitor.beginRelocalization(100), std::logic_error);

  EXPECT_EQ(monitor.latest().stamp_ns, 10);
  EXPECT_EQ(monitor.latest().consecutive_successes, 1U);
  EXPECT_NO_THROW(monitor.update(evidence(20, true)));
  EXPECT_EQ(monitor.latest().state, LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, FailureInterruptsRecoverySuccessDebounce)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  monitor.update(evidence(20, false));
  monitor.update(evidence(30, false));
  ASSERT_EQ(monitor.update(evidence(40, false, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(50, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(60, false, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(70, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(80, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery)).state, LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, HealthTimeoutRequiresDebouncedFullRecoveryAndPreservesMatchQuality)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  const auto initialized = monitor.update(evidence(10, true));
  EXPECT_EQ(monitor.checkHealth(20, true, 99, true).state, LocalizationState::kLocalized);

  const auto stale = monitor.checkHealth(21, true, 100, true);

  EXPECT_EQ(stale.state, LocalizationState::kDegraded);
  EXPECT_EQ(stale.reason, "correction_stale");
  EXPECT_TRUE(stale.full_recovery_required);
  EXPECT_EQ(stale.consecutive_successes, 0U);
  EXPECT_EQ(stale.consecutive_failures, 0U);
  EXPECT_EQ(stale.candidate_id, initialized.candidate_id);
  EXPECT_DOUBLE_EQ(stale.registration.fitness, initialized.registration.fitness);
  EXPECT_DOUBLE_EQ(stale.registration.overlap, initialized.registration.overlap);

  const auto fine = monitor.update(evidence(22, true));
  EXPECT_EQ(fine.state, LocalizationState::kDegraded);
  EXPECT_TRUE(fine.full_recovery_required);
  EXPECT_EQ(fine.consecutive_successes, 0U);
  const auto first_full = monitor.update(evidence(23, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery));
  EXPECT_EQ(first_full.state, LocalizationState::kDegraded);
  EXPECT_EQ(first_full.consecutive_successes, 1U);
  EXPECT_TRUE(first_full.full_recovery_required);
  EXPECT_EQ(monitor.checkHealth(24, true, 0, true).consecutive_successes, 1U);
  const auto recovered = monitor.update(evidence(25, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery));
  EXPECT_EQ(recovered.state, LocalizationState::kLocalized);
  EXPECT_FALSE(recovered.full_recovery_required);
}

TEST(LocalizationMonitor, SustainedAcceptedAgeTimeoutReachesLostWithoutInventingFailures)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  EXPECT_EQ(monitor.checkHealth(20, true, 100, true).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.checkHealth(30, true, 199, true).state, LocalizationState::kDegraded);
  const auto lost = monitor.checkHealth(40, true, 200, true);
  EXPECT_EQ(lost.state, LocalizationState::kLost);
  EXPECT_EQ(lost.reason, "correction_stale");
  EXPECT_EQ(lost.consecutive_failures, 0U);
  EXPECT_FALSE(lost.full_recovery_required);
  EXPECT_EQ(monitor.checkHealth(50, true, 0, true).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.update(evidence(60, true)).state, LocalizationState::kLost);
}

TEST(LocalizationMonitor, MissingFrontendLosesTrustAndRequiresExplicitRelocalization)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));

  const auto lost = monitor.checkHealth(20, true, 0, false);

  EXPECT_EQ(lost.state, LocalizationState::kLost);
  EXPECT_EQ(lost.reason, "frontend_timeout");
  EXPECT_EQ(lost.consecutive_failures, 0U);
  EXPECT_EQ(monitor.checkHealth(30, true, 0, true).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.beginRelocalization(40).state, LocalizationState::kRelocalizing);
}

TEST(LocalizationMonitor, HealthTimerKeepsInitializingAndExternalRelocalizationPolicy)
{
  LocalizationMonitor monitor{config()};
  EXPECT_EQ(monitor.checkHealth(10, false, 0, false).state, LocalizationState::kInitializing);
  auto initial = evidence(20, true, EvidenceSource::kRelocalization);
  initial.confirmation_complete = true;
  monitor.update(initial);
  monitor.checkHealth(30, true, 0, false);
  ASSERT_EQ(monitor.beginRelocalization(40).state, LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.checkHealth(1000, true, 500, false).state, LocalizationState::kRelocalizing);
  auto confirmed = evidence(1100, true, EvidenceSource::kRelocalization);
  confirmed.defer_confirmation = true;
  confirmed.confirmation_complete = true;
  EXPECT_EQ(monitor.update(confirmed).state, LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, HealthTimerRejectsInvalidAgeWithoutAdvancingTimestamp)
{
  LocalizationMonitor monitor{config()};
  monitor.update(evidence(10, true));
  EXPECT_THROW(monitor.checkHealth(100, true, -1, true), std::invalid_argument);
  EXPECT_EQ(monitor.latest().stamp_ns, 10);
  EXPECT_NO_THROW(monitor.checkHealth(20, true, 0, true));
}

TEST(LocalizationMonitor, HealthRecoveryRemainsRequiredAfterFullMatchFailures)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  ASSERT_EQ(monitor.checkHealth(20, true, 100, true).state, LocalizationState::kDegraded);
  for (const auto stamp : {30, 40}) {
    const auto failed = monitor.update(evidence(stamp, false, EvidenceSource::kNormal,
        NormalMatchKind::kFullRecovery));
    EXPECT_EQ(failed.state, LocalizationState::kDegraded);
    EXPECT_TRUE(failed.full_recovery_required);
  }
  EXPECT_EQ(monitor.update(evidence(50, true)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.latest().consecutive_successes, 0U);
}

TEST(LocalizationMonitor, RepeatedHealthChecksPreserveFullRecoveryProgress)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  ASSERT_EQ(monitor.checkHealth(20, true, 100, true).state, LocalizationState::kDegraded);
  const auto first = monitor.update(evidence(30, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery));
  ASSERT_EQ(first.state, LocalizationState::kDegraded);
  ASSERT_EQ(first.consecutive_successes, 1U);

  EXPECT_EQ(monitor.checkHealth(40, true, 0, true).consecutive_successes, 1U);
  EXPECT_EQ(monitor.checkHealth(50, true, 100, true).consecutive_successes, 1U);
  EXPECT_EQ(monitor.checkHealth(60, true, 110, true).consecutive_successes, 1U);
  const auto second = monitor.update(evidence(70, true, EvidenceSource::kNormal,
      NormalMatchKind::kFullRecovery));
  EXPECT_EQ(second.state, LocalizationState::kLocalized);
  EXPECT_FALSE(second.full_recovery_required);
}

TEST(LocalizationMonitor, GeneratedEventSequencesPreserveRecoveryInvariants)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.degraded_failures_required = 1U;
  configured.lost_failures_required = 2U;
  configured.relocalization_successes_required = 2U;
  for (std::uint32_t pattern = 0U; pattern < 256U; ++pattern) {
    LocalizationMonitor monitor{configured};
    std::int64_t stamp = 10;
    auto previous = monitor.update(evidence(stamp, true)).state;
    ASSERT_EQ(previous, LocalizationState::kLocalized);
    for (std::uint32_t index = 0U; index < 8U; ++index) {
      stamp += 10;
      const bool accepted = (pattern & (1U << index)) != 0U;
      const auto before = monitor.latest();
      const auto kind = before.state == LocalizationState::kDegraded ||
        before.full_recovery_required ?
        NormalMatchKind::kFullRecovery : NormalMatchKind::kFineOnly;
      const auto current = monitor.update(
        evidence(stamp, accepted, EvidenceSource::kNormal, kind)).state;
      if (previous == LocalizationState::kLost) {
        EXPECT_EQ(current, LocalizationState::kLost) << "pattern=" << pattern;
      }
      EXPECT_GE(static_cast<int>(current), static_cast<int>(LocalizationState::kInitializing));
      EXPECT_LE(static_cast<int>(current), static_cast<int>(LocalizationState::kRelocalizing));
      previous = current;
    }
  }
}

}  // namespace
}  // namespace a2w_fastlio_localization
