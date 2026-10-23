#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "a2w_fastlio_localization/fine_failure_capture.hpp"

namespace a2w_fastlio_localization
{
namespace
{

using a2w_fastlio_common::Cloud;
using a2w_fastlio_common::PointT;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  : path_{std::filesystem::temp_directory_path() /
      ("a2w_fine_failure_capture_" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()))}
  {
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() {std::filesystem::remove_all(path_);}
  const std::filesystem::path & path() const {return path_;}

private:
  std::filesystem::path path_;
};

std::shared_ptr<Cloud> cloud(const std::size_t count, const float offset)
{
  auto result = std::make_shared<Cloud>();
  for (std::size_t index = 0; index < count; ++index) {
    PointT point;
    point.x = offset + static_cast<float>(index);
    point.y = offset;
    result->push_back(point);
  }
  return result;
}

FineMatchRecord record(
  const std::int64_t stamp, const bool accepted,
  const std::vector<std::uint64_t> & ids,
  const std::shared_ptr<Cloud> & source,
  const std::shared_ptr<Cloud> & target)
{
  FineMatchRecord value;
  value.output.stamp_ns = stamp;
  value.output.match_attempted = true;
  value.output.matching_mode = MapMatchingMode::kFineOnly;
  value.output.match_accepted = accepted;
  value.output.predicted_map_body.translation = {1.0, 2.0, 3.0};
  value.output.map_camera_init.translation = {0.25, -0.5, 0.0};
  value.output.match_target = target;
  value.odom_pose.translation = {0.75, 2.5, 3.0};
  value.source_cloud = source;
  value.selection.initial_guess = value.output.predicted_map_body;
  for (const auto id : ids) {
    value.selection.keyframes.emplace_back(id, 0.1);
  }
  return value;
}

// Catches saving a failure without the exact previous-success target, saving
// more than once, or writing metadata that cannot reproduce A/B/C.
TEST(FineFailureCapture, SavesFirstFailureWithCurrentAndPreviousExactInputs)
{
  TemporaryDirectory temporary;
  FineFailureCapture capture{{true, temporary.path()}};
  const auto previous_source = cloud(4U, 10.0F);
  const auto previous_target = cloud(5U, 20.0F);
  const auto failure_source = cloud(6U, 30.0F);
  const auto current_target = cloud(7U, 40.0F);
  auto algorithms = a2w_fastlio_common::DefaultAlgorithmSuiteConfig{};
  algorithms.fine.source_voxel_leaf_m = 0.1;

  EXPECT_FALSE(capture.observe(record(
      100, true, {26U, 34U, 33U}, previous_source, previous_target), algorithms));
  const auto saved = capture.observe(record(
      200, false, {27U, 28U, 29U}, failure_source, current_target), algorithms);
  ASSERT_TRUE(saved);
  EXPECT_TRUE(saved->captured) << saved->reason;
  EXPECT_FALSE(capture.observe(record(
      300, false, {1U}, cloud(2U, 0.0F), cloud(3U, 0.0F)), algorithms));

  Cloud loaded;
  EXPECT_EQ(pcl::io::loadPCDFile((saved->directory / "source_body.pcd").string(), loaded), 0);
  EXPECT_EQ(loaded.size(), 6U);
  EXPECT_EQ(pcl::io::loadPCDFile(
      (saved->directory / "target_current_27_28_29.pcd").string(), loaded), 0);
  EXPECT_EQ(loaded.size(), 7U);
  EXPECT_EQ(pcl::io::loadPCDFile(
      (saved->directory / "target_previous_26_34_33.pcd").string(), loaded), 0);
  EXPECT_EQ(loaded.size(), 5U);

  const auto metadata = YAML::LoadFile((saved->directory / "metadata.yaml").string());
  EXPECT_EQ(metadata["failure_source_stamp_ns"].as<std::int64_t>(), 200);
  EXPECT_EQ(metadata["current_keyframes"].as<std::vector<std::uint64_t>>(),
    (std::vector<std::uint64_t>{27U, 28U, 29U}));
  EXPECT_EQ(metadata["previous_keyframes"].as<std::vector<std::uint64_t>>(),
    (std::vector<std::uint64_t>{26U, 34U, 33U}));
  EXPECT_DOUBLE_EQ(metadata["initial_guess"]["translation"][0].as<double>(), 1.0);
  EXPECT_EQ(metadata["registration"]["fine"]["maximum_iterations"].as<int>(),
    algorithms.fine.maximum_iterations);

  const auto replay = loadFineFailureCapture(saved->directory);
  EXPECT_EQ(replay.abc.source->size(), 6U);
  EXPECT_EQ(replay.abc.current_target->size(), 7U);
  EXPECT_EQ(replay.abc.previous_target->size(), 5U);
  EXPECT_TRUE(replay.abc.initial_guess.translation.isApprox(Eigen::Vector3d{1.0, 2.0, 3.0}));
  EXPECT_EQ(replay.current_keyframes, (std::vector<std::uint64_t>{27U, 28U, 29U}));
  EXPECT_EQ(replay.previous_keyframes, (std::vector<std::uint64_t>{26U, 34U, 33U}));
  EXPECT_EQ(replay.algorithms.fine.maximum_iterations, algorithms.fine.maximum_iterations);
  EXPECT_DOUBLE_EQ(replay.algorithms.fine.source_voxel_leaf_m, 0.1);

  // Old captures retain their original full-source optimizer behavior.
  auto legacy_metadata = YAML::LoadFile((saved->directory / "metadata.yaml").string());
  legacy_metadata["registration"]["fine"].remove("source_voxel_leaf_m");
  {
    std::ofstream legacy{saved->directory / "metadata.yaml"};
    legacy << legacy_metadata;
  }
  EXPECT_DOUBLE_EQ(loadFineFailureCapture(saved->directory).algorithms.fine.source_voxel_leaf_m, 0.0);
}

TEST(FineFailureCapture, IgnoresStaleFailure)
{
  TemporaryDirectory temporary;
  FineFailureCapture capture{{true, temporary.path()}};
  const auto algorithms = a2w_fastlio_common::DefaultAlgorithmSuiteConfig{};
  capture.observe(record(100, true, {26U}, cloud(4U, 0.0F), cloud(5U, 0.0F)), algorithms);
  auto stale = record(200, false, {27U}, cloud(4U, 0.0F), cloud(5U, 0.0F));
  stale.output.result_stale = true;
  EXPECT_FALSE(capture.observe(stale, algorithms));
  EXPECT_TRUE(std::filesystem::is_empty(temporary.path()));
}

TEST(OfflineAbcDiagnosis, ClassifiesTheFourRequestedOutcomes)
{
  EXPECT_EQ(classifyOfflineAbc(false, true, false), OfflineAbcCause::kFineConvergenceBasin);
  EXPECT_EQ(classifyOfflineAbc(false, false, true), OfflineAbcCause::kLocalMapSwitch);
  EXPECT_EQ(classifyOfflineAbc(false, false, false), OfflineAbcCause::kCorrectionPrediction);
  EXPECT_EQ(classifyOfflineAbc(true, false, false), OfflineAbcCause::kOnlineTimingOrLifetime);
}

// Catches accidentally rebuilding the source, using a different initial
// guess for C, or feeding the previous target into A/B.
TEST(OfflineAbcReplay, ReusesFailureSourceAndGuessAcrossAllThreeRuns)
{
  class RecordingRegistration final :
    public a2w_fastlio_common::CoarseRegistration,
    public a2w_fastlio_common::FineRegistration
  {
  public:
    a2w_fastlio_common::RegistrationResult align(
      const a2w_fastlio_common::CloudConstPtr & source,
      const a2w_fastlio_common::CloudConstPtr & target,
      const std::optional<a2w_fastlio_common::Pose3d> & guess) const override
    {
      sources.push_back(source);
      targets.push_back(target);
      guesses.push_back(guess);
      a2w_fastlio_common::RegistrationResult result;
      result.success = true;
      result.converged = true;
      result.transform = *guess;
      result.fitness = 0.01;
      result.overlap = 1.0;
      result.correspondence_count = source->size();
      return result;
    }
    mutable std::vector<a2w_fastlio_common::CloudConstPtr> sources;
    mutable std::vector<a2w_fastlio_common::CloudConstPtr> targets;
    mutable std::vector<std::optional<a2w_fastlio_common::Pose3d>> guesses;
  };

  const auto algorithm = std::make_shared<RecordingRegistration>();
  const auto pipeline = std::make_shared<a2w_fastlio_common::RegistrationPipeline>(
    algorithm, algorithm,
    a2w_fastlio_common::MatchValidator{{0.1, 0.5, 1U, 5.0, 1.0, 0.0}},
    a2w_fastlio_common::RegistrationPipelineConfig{0.5});
  OfflineAbcInput input;
  input.source = cloud(6U, 1.0F);
  input.current_target = cloud(6U, 1.0F);
  input.previous_target = cloud(6U, 1.0F);

  const auto result = runOfflineAbc(input, pipeline);

  EXPECT_TRUE(result.a.success);
  EXPECT_TRUE(result.b.success);
  EXPECT_TRUE(result.c.success);
  ASSERT_EQ(algorithm->sources.size(), 4U);  // A fine, B coarse+fine, C fine.
  for (std::size_t index = 0; index < algorithm->sources.size(); ++index) {
    EXPECT_EQ(algorithm->sources[index], input.source);
    ASSERT_TRUE(algorithm->guesses[index]);
    EXPECT_TRUE(algorithm->guesses[index]->translation.isApprox(
        input.initial_guess.translation));
  }
  EXPECT_EQ(algorithm->targets[0], input.current_target);
  EXPECT_EQ(algorithm->targets[1], input.current_target);
  EXPECT_EQ(algorithm->targets[2], input.current_target);
  EXPECT_EQ(algorithm->targets[3], input.previous_target);
  EXPECT_GE(result.a_latency_ms, 0.0);
  EXPECT_GE(result.b_latency_ms, 0.0);
  EXPECT_GE(result.c_latency_ms, 0.0);
}

}  // namespace
}  // namespace a2w_fastlio_localization
