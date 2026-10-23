#include <gtest/gtest.h>

#include "a2w_fastlio_localization/global_relocalizer.hpp"

namespace a2w_fastlio_localization
{
namespace
{

RelocalizationResult match(const std::uint64_t id, const double x, const bool strong = false)
{
  RelocalizationResult result;
  result.success = true;
  result.strong = strong;
  result.candidate_id = id;
  result.map_body.translation.x() = x;
  return result;
}

RelocalizationSessionConfig config()
{
  return RelocalizationSessionConfig{3U, 0.5, 0.2, 500};
}

TEST(RelocalizationSession, RequiresConsistentMultiFrameEvidence)
{
  RelocalizationSession session{config()};
  session.start(10);
  EXPECT_FALSE(session.observe(20, match(7U, 10.0)).confirmed_correction.has_value());
  EXPECT_FALSE(session.observe(30, match(8U, 10.0)).confirmed_correction.has_value());
  EXPECT_EQ(session.latest().consecutive_matches, 2U);
  const auto confirmed = session.observe(40, match(8U, 10.1));
  ASSERT_TRUE(confirmed.confirmed_correction.has_value());
  EXPECT_EQ(confirmed.candidate_id, 8U);
  EXPECT_DOUBLE_EQ(confirmed.confirmed_correction->translation.x(), 10.1);
}

TEST(RelocalizationSession, SlowValidComputationsDoNotConsumeEvidenceWaitTimeout)
{
  RelocalizationSessionConfig settings;
  settings.timeout_ns = 10'000'000'000LL;
  RelocalizationSession session{settings};
  session.start(0);
  auto result = match(7U, 10.0);
  result.elapsed_ms = 16000.0;
  EXPECT_EQ(session.observe(16'100'000'000LL, result).consecutive_matches, 1U);
  result.candidate_id = 8U;
  EXPECT_EQ(session.observe(32'200'000'000LL, result).consecutive_matches, 2U);
  const auto confirmed = session.observe(48'300'000'000LL, result);
  ASSERT_TRUE(confirmed.confirmed_correction);
  EXPECT_EQ(confirmed.reason, "multiframe_confirmed");
}

TEST(RelocalizationSession, SlowFailureStillTimesOutAndIdleCannotBeHiddenByOneEvaluation)
{
  RelocalizationSessionConfig settings;
  settings.timeout_ns = 10'000'000'000LL;
  RelocalizationSession session{settings};
  session.start(0);
  auto failed = RelocalizationResult{};
  failed.elapsed_ms = 16000.0;
  EXPECT_EQ(session.observe(16'000'000'000LL, failed).reason, "session_timeout");
  session.start(17'000'000'000LL);
  auto valid = match(7U, 10.0);
  valid.elapsed_ms = 16000.0;
  EXPECT_EQ(session.observe(44'000'000'000LL, valid).reason, "session_timeout");
}

TEST(RelocalizationSession, RotationChangeAndNonFinitePoseCannotConfirm)
{
  RelocalizationSession session{config()};
  session.start(10);
  session.observe(20, match(7U, 10.0));
  auto rotated = match(8U, 10.0);
  rotated.map_body.rotation = Eigen::AngleAxisd{0.5, Eigen::Vector3d::UnitZ()};
  EXPECT_EQ(session.observe(30, rotated).consecutive_matches, 1U);
  rotated.map_body.translation.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(session.observe(40, rotated).consecutive_matches, 0U);
}

TEST(RelocalizationSession, AmbiguityAndTransformJumpResetConfirmation)
{
  RelocalizationSession session{config()};
  session.start(10);
  session.observe(20, match(7U, 10.0));
  auto ambiguous = RelocalizationResult{};
  ambiguous.reason = "candidate_score_ambiguous";
  EXPECT_EQ(session.observe(30, ambiguous).consecutive_matches, 0U);
  session.observe(40, match(7U, 10.0));
  EXPECT_EQ(session.observe(50, match(7U, 12.0)).consecutive_matches, 1U);
}

TEST(RelocalizationSession, SmallPairwiseDriftCannotEscapeTheFirstCorrectionAnchor)
{
  RelocalizationSession session{config()};
  session.start(10);
  session.observe(20, match(7U, 10.0));
  EXPECT_EQ(session.observe(30, match(8U, 10.4)).consecutive_matches, 2U);
  const auto drifting = session.observe(40, match(9U, 10.8));
  EXPECT_FALSE(drifting.confirmed_correction);
  EXPECT_EQ(drifting.consecutive_matches, 1U);
}

TEST(RelocalizationSession, StrongResultConfirmsOnceAndTimeoutOrCancelStopsSession)
{
  RelocalizationSession session{config()};
  session.start(10);
  EXPECT_TRUE(session.observe(20, match(7U, 3.0, true)).confirmed_correction.has_value());
  EXPECT_FALSE(session.latest().active);
  EXPECT_THROW(session.observe(30, match(7U, 3.0)), std::logic_error);

  session.start(40);
  const auto timeout = session.observe(540, match(7U, 3.0));
  EXPECT_FALSE(timeout.active);
  EXPECT_EQ(timeout.reason, "session_timeout");

  session.start(600);
  EXPECT_EQ(session.cancel(610).reason, "cancelled");
  EXPECT_FALSE(session.latest().active);
}

TEST(RelocalizationSession, RejectsInvalidConfigAndNonMonotonicTime)
{
  auto invalid = config();
  invalid.confirmation_count = 1U;
  EXPECT_THROW(RelocalizationSession{invalid}, std::invalid_argument);
  RelocalizationSession session{config()};
  session.start(10);
  EXPECT_THROW(session.observe(10, match(1U, 0.0)), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_localization
