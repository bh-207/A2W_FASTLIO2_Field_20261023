#define LOOP_SUBMAP_PROBE_V4_NO_MAIN
#include "loop_submap_probe_v4.cpp"
#include <gtest/gtest.h>

namespace
{
using namespace submap_v4;
Matrix translation(double x, double y, double z)
{
  Matrix m = Matrix::Identity(); m(0, 3) = x; m(1, 3) = y; m(2, 3) = z; return m;
}

TEST(LoopSubmapV4, SourceWindowNeverIncludesFutureAvailableOnlyInBundle)
{
  EXPECT_EQ(window(9, 2, 9), (std::vector<std::uint64_t>{7, 8, 9}));
  EXPECT_EQ(window(1, 4, 9), (std::vector<std::uint64_t>{0, 1, 2, 3, 4, 5}));
}

TEST(LoopSubmapV4, HoldoutExcludesTheWholeCenterScanOnBothSides)
{
  EXPECT_EQ(withoutCenter(window(9, 2, 9), 9), (std::vector<std::uint64_t>{7, 8}));
  EXPECT_EQ(withoutCenter(window(3, 2, 9), 3), (std::vector<std::uint64_t>{1, 2, 4, 5}));
}

TEST(LoopSubmapV4, CacheCasesRetainActualFailureAndAcceptedWithoutDuplicatingEvents)
{
  const std::vector<BenchmarkCase> actual{{10, "loop_same_solution"}, {11, "fine_registration_failed"},
    {10, "loop_same_solution"}, {12, "loop_same_solution"}, {13, "fitness_above_limit"},
    {14, "loop_accepted"}, {15, "translation_jump_above_limit"}};
  const auto selected = selectBenchmarkCases(actual);
  std::vector<std::uint64_t> ids;
  for (const auto & item : selected) {ids.push_back(item.candidate);}
  EXPECT_EQ(ids, (std::vector<std::uint64_t>{10, 11, 12, 13, 14}));
}

TEST(LoopSubmapV4, PreparedAuditChecksAll33BinsBitwiseNotOnlyGeometricOutcome)
{
  std::array<float, 33> a{}, b{};
  EXPECT_EQ(differingFeatureBins(a.data(), b.data()), 0U);
  b[32] = 1.0F;
  EXPECT_EQ(differingFeatureBins(a.data(), b.data()), 1U);
  b[0] = -0.0F;
  EXPECT_EQ(differingFeatureBins(a.data(), b.data()), 2U);
}

TEST(LoopSubmapV4, CacheBenchmarkReverseControlsDoNotAlwaysRunCacheLast)
{
  EXPECT_EQ(cacheExecutionOrder(false), (std::array<std::size_t, 2>{0U, 16U}));
  EXPECT_EQ(cacheExecutionOrder(true), (std::array<std::size_t, 2>{16U, 0U}));
}

TEST(LoopSubmapV4, AnchorCoordinateRestorationCannotCommuteSourceAndTarget)
{
  Matrix source = Matrix::Identity();
  source(0, 0) = 0; source(0, 1) = -1; source(1, 0) = 1; source(1, 1) = 0;
  source(0, 3) = 10;
  const Matrix result = restored(translation(1, 2, 0), source, translation(0, 5, 0));
  EXPECT_NEAR(result(0, 3), 11, 1e-12);
  EXPECT_NEAR(result(1, 3), -3, 1e-12);
  EXPECT_NEAR(result(1, 0), 1, 1e-12);
}

TEST(LoopSubmapV4, GraphFactorIsCurrentInverseHistoryNotRegistrationDirection)
{
  Matrix source = Matrix::Identity();
  source(0, 0) = 0; source(0, 1) = -1; source(1, 0) = 1; source(1, 1) = 0;
  source(0, 3) = 10;
  const Matrix result = factor(translation(1, 2, 0), source, translation(0, 5, 0));
  EXPECT_NEAR(result(0, 3), 3, 1e-12);
  EXPECT_NEAR(result(1, 3), 11, 1e-12);
  EXPECT_NEAR(result(1, 0), -1, 1e-12);
}

TEST(LoopSubmapV4, LocalRigidInconsistencyDetectsInternalDistortionNotGlobalGauge)
{
  const Matrix raw_neighbor = translation(2, 0, 0);
  EXPECT_NEAR(rigidTranslationError(Matrix::Identity(), raw_neighbor,
    translation(10, 20, 0), translation(12, 20, 0)), 0, 1e-12);
  EXPECT_NEAR(rigidTranslationError(Matrix::Identity(), raw_neighbor,
    translation(10, 20, 0), translation(12.5, 20, 0)), .5, 1e-12);
}
}  // namespace
