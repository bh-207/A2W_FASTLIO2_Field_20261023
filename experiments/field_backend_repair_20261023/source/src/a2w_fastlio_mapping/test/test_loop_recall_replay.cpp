// Standalone behavior checks: no ROS and no registration/replay of the real dataset.
#define LOOP_RECALL_REPLAY_NO_MAIN
#include "loop_recall_replay.cpp"

#include <gtest/gtest.h>
#include <unistd.h>

namespace loop_recall_replay
{
namespace
{

class Scratch
{
public:
  Scratch()
  {
    std::string pattern = (fs::temp_directory_path() / "loop_recall_replay_test_XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const char * created = ::mkdtemp(buffer.data());
    if (!created) {throw std::runtime_error("mkdtemp failed");}
    root = created;
  }
  ~Scratch() {std::error_code error; fs::remove_all(root, error);}
  fs::path root;
};

TEST(LoopRecallReplay, RejectsNegativeTrailingAndReversedBoundsBeforeAnyReplay)
{
  EXPECT_THROW(parseId("-1"), std::invalid_argument);
  EXPECT_THROW(parseId("12junk"), std::invalid_argument);
  EXPECT_THROW(parseId(""), std::invalid_argument);
  EXPECT_EQ(parseId("1490"), 1490U);
  EXPECT_THROW(validateRange(20U, 10U, 30U), std::invalid_argument);
  EXPECT_THROW(validateRange(0U, 30U, 30U), std::invalid_argument);
  EXPECT_NO_THROW(validateRange(0U, 29U, 30U));
}

TEST(LoopRecallReplay, RejectsInputOutputOverlapIncludingSymlinkAliases)
{
  Scratch scratch;
  const auto input = scratch.root / "bundle";
  fs::create_directories(input);
  fs::create_directory_symlink(input, scratch.root / "alias");
  EXPECT_THROW(validateOutputPaths(input, input / "results"), std::invalid_argument);
  EXPECT_THROW(validateOutputPaths(input, scratch.root), std::invalid_argument);
  EXPECT_THROW(validateOutputPaths(input, scratch.root / "alias" / "results"), std::invalid_argument);
  EXPECT_NO_THROW(validateOutputPaths(input, scratch.root / "results"));
}

TEST(LoopRecallReplay, RefusesExistingResultsAndReservesOutputAgainstSecondWriter)
{
  Scratch scratch;
  const auto output = scratch.root / "results";
  fs::create_directory(output);
  {std::ofstream file(output / "events.csv"); file << "preserve me\n";}
  EXPECT_THROW(reserveOutput(output), std::runtime_error);
  std::ifstream original(output / "events.csv");
  std::string line;
  std::getline(original, line);
  EXPECT_EQ(line, "preserve me");
  fs::create_directory(scratch.root / "constraints_only");
  {std::ofstream file(scratch.root / "constraints_only/loop_constraints.csv"); file << "existing factors\n";}
  EXPECT_THROW(reserveOutput(scratch.root / "constraints_only"), std::runtime_error);
  const auto fresh = scratch.root / "fresh";
  EXPECT_NO_THROW(reserveOutput(fresh));
  EXPECT_THROW(reserveOutput(fresh), std::runtime_error);
}

TEST(LoopRecallReplay, GatedIndexRetainsHistoryAndReleasesQueriesAtFirstMatch)
{
  auto real = std::make_shared<a2w_fastlio_common::ScanContextIndex>();
  WindowIndex index(real, 2U);
  a2w_fastlio_common::ScanDescriptor descriptor{1U, 2U, {1.0F, 2.0F}};
  index.setCurrent(0U);
  index.add(0U, descriptor);
  index.setCurrent(1U);
  EXPECT_TRUE(index.queryTopK(descriptor, 5U, {0U, 0U}).empty());
  index.add(1U, descriptor);
  index.setCurrent(2U);
  const auto candidates = index.queryTopK(descriptor, 5U, {1U, 0U});
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(real->size(), 2U);
  EXPECT_EQ(candidates[0].keyframe_id, 0U);
  EXPECT_EQ(candidates[1].keyframe_id, 1U);
}

TEST(LoopRecallReplay, NeverInjectsSavedOptimizedPoseIntoFreshFrame)
{
  a2w_fastlio_common::KeyFrame saved;
  saved.id = 17U;
  saved.odom_pose.translation = Eigen::Vector3d{1.0, 2.0, 3.0};
  saved.optimized_pose.translation = Eigen::Vector3d{91.0, 92.0, 93.0};
  auto online = freshFrame(saved);
  EXPECT_DOUBLE_EQ(online.optimized_pose.translation.x(), 1.0);
  EXPECT_DOUBLE_EQ(saved.optimized_pose.translation.x(), 91.0);
  EXPECT_EQ(online.id, 17U);
}

TEST(LoopRecallReplay, CachedQueriesRespectAddedHistoryRecentExclusionAndAnchorFilter)
{
  auto cache = std::make_shared<ScoreCache>();
  cache->count = 5U;
  cache->distances.assign(25U, 0.0);
  cache->yaws.assign(25U, 0.0);
  cache->distances[3U * 5U + 0U] = 0.2;
  cache->distances[3U * 5U + 1U] = 0.1;
  cache->distances[3U * 5U + 2U] = 0.1;
  cache->yaws[3U * 5U + 1U] = -0.7;
  auto real = std::make_shared<a2w_fastlio_common::ScanContextIndex>();
  WindowIndex index(real, 0U, cache);
  const a2w_fastlio_common::ScanDescriptor descriptor{1U, 2U, {1.0F, 2.0F}};
  index.add(0U, descriptor); index.add(1U, descriptor); index.add(2U, descriptor);
  index.setCurrent(3U);
  auto candidates = index.queryTopK(descriptor, 5U, {2U, 1U});
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].keyframe_id, 1U);
  EXPECT_EQ(candidates[1].keyframe_id, 0U);
  EXPECT_DOUBLE_EQ(candidates[0].yaw_hint_rad, -0.7);
  // An unbounded filter still cannot expose current/future entries in the cache.
  candidates = index.queryTopK(descriptor, 5U, {99U, 0U});
  ASSERT_EQ(candidates.size(), 3U);
  EXPECT_EQ(candidates[0].keyframe_id, 1U);
  EXPECT_EQ(candidates[1].keyframe_id, 2U);  // equal scores break ties by ID
  EXPECT_EQ(candidates[2].keyframe_id, 0U);
  candidates = index.queryTopK(descriptor, 5U, {1U, 0U});  // initial-anchor query
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates[0].keyframe_id, 1U);
  EXPECT_EQ(candidates[1].keyframe_id, 0U);
  EXPECT_TRUE(index.queryTopK(descriptor, 5U, {1U, 2U}).empty());
  EXPECT_THROW(index.queryTopK(descriptor, 0U, {1U, 0U}), std::invalid_argument);
}

TEST(LoopRecallReplay, CacheLoaderBindsChecksumAndDescriptorInputAndRejectsMalformedFiles)
{
  Scratch scratch;
  const auto path = scratch.root / "scores.bin";
  const auto descriptor = scratch.root / "descriptor.bin";
  // Independently constructed LE count=2, followed by eight float64 zeros.
  std::vector<unsigned char> bytes(72U, 0U);
  bytes[0] = 2U;
  {std::ofstream file(path, std::ios::binary); file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());}
  {std::ofstream file(descriptor); file << "abc";}
  {std::ofstream file(scratch.root / "scores.meta.json"); file << R"({
    "cache_sha256":"d4039c58eb150938055242a74979b87340671a9a0d43e0c932322b159cca926e",
    "descriptor_sha256":"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
  })";}
  const auto cache = ScoreCache::read(path, 2U, descriptor);
  ASSERT_EQ(cache->count, 2U);
  ASSERT_EQ(cache->distances.size(), 4U);
  EXPECT_DOUBLE_EQ(cache->distances[3], 0.0);
  EXPECT_THROW(ScoreCache::read(path, 3U, descriptor), std::runtime_error);
  {std::ofstream file(descriptor); file << "different bundle descriptors";}
  EXPECT_THROW(ScoreCache::read(path, 2U, descriptor), std::runtime_error);
  {std::ofstream file(descriptor); file << "abc";}
  // Finite altered payload passes the shape check but fails cache checksum binding.
  bytes[14] = 0xf0U; bytes[15] = 0x3fU;  // first distance becomes +1.0
  {std::ofstream file(path, std::ios::binary); file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());}
  EXPECT_THROW(ScoreCache::read(path, 2U, descriptor), std::runtime_error);
  {std::ofstream file(path, std::ios::binary); file.write(reinterpret_cast<const char *>(bytes.data()), 9);}
  EXPECT_THROW(ScoreCache::read(path, 2U, descriptor), std::runtime_error);
}

TEST(LoopRecallReplay, MissingPoseChecksRemainBlankAndReasonsAreCsvEscaped)
{
  a2w_fastlio_mapping::LoopPipelineEvent event;
  event.current_id = 7U;
  event.reason = "failure, details\nsecond \"line\"";
  event.descriptor_distance = 0.125;
  event.yaw_hint_rad = -0.75;
  event.origin_candidate = true;
  event.confirmation_count = 2U;
  event.rejection_stage = "geometric_verification";
  const auto cells = eventCells(event, 123LL, false);
  const auto headers = eventHeaders();
  ASSERT_EQ(cells.size(), headers.size());
  std::map<std::string, std::string> row;
  for (std::size_t i = 0; i < headers.size(); ++i) {row[headers[i]] = cells[i];}
  EXPECT_EQ(row.at("has_pose_check"), "0");
  EXPECT_TRUE(row.at("translation_difference_m").empty());
  EXPECT_TRUE(row.at("relative_roll_rad").empty());
  EXPECT_EQ(row.at("graph_committed"), "0");
  EXPECT_DOUBLE_EQ(std::stod(row.at("descriptor_distance")), 0.125);
  EXPECT_DOUBLE_EQ(std::stod(row.at("yaw_hint_rad")), -0.75);
  EXPECT_EQ(row.at("origin_candidate"), "1");
  EXPECT_EQ(row.at("confirmation_count"), "2");
  EXPECT_EQ(row.at("rejection_stage"), "geometric_verification");
  std::ostringstream output;
  writeCsv(output, cells);
  EXPECT_NE(output.str().find("\"failure, details\nsecond \"\"line\"\"\""), std::string::npos);
}

TEST(LoopRecallReplay, RecordsPregraphCorrectionAndRelativeAttitudeWithoutMixingOptimizedChange)
{
  a2w_fastlio_mapping::LoopPipelineEvent event;
  a2w_fastlio_mapping::LoopPoseCheck check;
  check.predicted_pose.translation = Eigen::Vector3d{10.0, 20.0, 30.0};
  check.corrected_pose.translation = Eigen::Vector3d{11.0, 18.0, 33.0};
  check.translation_difference_m = std::sqrt(14.0);
  check.relative_rpy_rad = Eigen::Vector3d{0.1, -0.2, 0.3};
  event.pose_check = check;
  event.optimized_pose_change = a2w_fastlio_common::Pose3d{};
  const auto cells = eventCells(event, 0LL, true);
  const auto headers = eventHeaders();
  std::map<std::string, std::string> row;
  for (std::size_t i = 0; i < headers.size(); ++i) {row[headers[i]] = cells[i];}
  EXPECT_DOUBLE_EQ(std::stod(row.at("correction_dx_map_m")), 1.0);
  EXPECT_DOUBLE_EQ(std::stod(row.at("correction_dy_map_m")), -2.0);
  EXPECT_DOUBLE_EQ(std::stod(row.at("correction_dz_map_m")), 3.0);
  EXPECT_DOUBLE_EQ(std::stod(row.at("relative_pitch_rad")), -0.2);
  EXPECT_EQ(row.at("graph_committed"), "1");
}

TEST(LoopRecallReplay, CapturesFactorFromHistoricalPreupdatePoseAndWaitsForGraphCommit)
{
  a2w_fastlio_mapping::LoopPipelineEvent event;
  event.current_id = 9U; event.candidate_id = 2U; event.accepted = true;
  a2w_fastlio_mapping::LoopPoseCheck check;
  check.corrected_pose.translation = Eigen::Vector3d{10.0, 20.0, 30.0};
  check.corrected_pose.rotation = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ());
  event.pose_check = check;
  a2w_fastlio_mapping::OptimizedPoseSnapshot before;
  a2w_fastlio_common::Pose3d historical;
  historical.translation = Eigen::Vector3d{12.0, 23.0, 34.0};
  before.poses.push_back({2U, historical});
  LoopMeasurementCapture capture;
  capture.queue(event, before);
  // Simulate graph optimization changing the historical endpoint after queueing.
  before.poses[0].pose.translation = Eigen::Vector3d{900.0, 800.0, 700.0};
  const auto measurement = capture.finish(event);
  ASSERT_TRUE(measurement.has_value());
  EXPECT_EQ(measurement->current_id, 9U);
  EXPECT_EQ(measurement->candidate_id, 2U);
  EXPECT_NEAR(measurement->relative_pose.translation.x(), 3.0, 1e-12);
  EXPECT_NEAR(measurement->relative_pose.translation.y(), -2.0, 1e-12);
  EXPECT_NEAR(measurement->relative_pose.translation.z(), 4.0, 1e-12);
  EXPECT_NEAR(measurement->relative_pose.rotation.z(), -std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(measurement->relative_pose.rotation.w(), std::sqrt(0.5), 1e-12);
  LoopMeasurementCapture failed;
  failed.queue(event, before);
  event.accepted = false;
  EXPECT_FALSE(failed.finish(event).has_value());
  event.accepted = true;
  EXPECT_THROW(failed.finish(event), std::runtime_error);
}

TEST(LoopRecallReplay, MetadataRecordsActualExperimentAndSuiteValuesWithSeparateGapUnits)
{
  a2w_fastlio_mapping::LoopPipelineConfig loop;
  loop.origin_top_k = 7U;
  loop.distinct_solution_fitness_margin_m = 0.125;
  a2w_fastlio_common::DefaultAlgorithmSuiteConfig suite;
  suite.validation.minimum_candidate_distance_separation = 0.375;
  suite.coarse.noise_bound_m = 0.625;
  suite.fine.maximum_iterations = 73;
  a2w_fastlio_common::LocalMapConfig local{0.3, 123U};
  a2w_fastlio_mapping::PoseGraphConfig graph;
  graph.loop_translation_sigma_m = 0.75;
  std::ostringstream output;
  writeConfigMetadata(output, loop, suite, local, graph);
  EXPECT_NE(output.str().find("loop_pipeline.origin_top_k=7\n"), std::string::npos);
  EXPECT_NE(output.str().find("loop_pipeline.distinct_solution_fitness_margin_m=0.125\n"), std::string::npos);
  EXPECT_NE(output.str().find("loop_validation.minimum_candidate_distance_separation=0.375\n"), std::string::npos);
  EXPECT_NE(output.str().find("quatro.noise_bound_m=0.625\n"), std::string::npos);
  EXPECT_NE(output.str().find("nano_gicp.maximum_iterations=73\n"), std::string::npos);
  EXPECT_NE(output.str().find("local_map.max_points=123\n"), std::string::npos);
  EXPECT_NE(output.str().find("pose_graph.loop_translation_sigma_m=0.75\n"), std::string::npos);
  EXPECT_NE(output.str().find("dimensionless_scan_context_score_gap"), std::string::npos);
  EXPECT_NE(output.str().find("metres_geometric_fitness_gap"), std::string::npos);
}

TEST(LoopRecallReplay, ScheduledModeIsExplicitAndRecordsMotionPolicyWithoutChangingAlgorithmLimits)
{
  const auto scheduled = replayLoopConfig("scheduled");
  EXPECT_TRUE(scheduled.revisit_verification_enabled);
  EXPECT_EQ(scheduled.discovery_interval_keyframes, 3U);
  EXPECT_DOUBLE_EQ(scheduled.confirmation_min_translation_m, .5);
  EXPECT_DOUBLE_EQ(scheduled.confirmation_min_rotation_rad, .15);
  EXPECT_DOUBLE_EQ(scheduled.maximum_descriptor_distance, .2);
  const auto enhanced = replayLoopConfig("enhanced");
  EXPECT_EQ(enhanced.discovery_interval_keyframes, 1U);
  EXPECT_DOUBLE_EQ(enhanced.confirmation_min_translation_m, 0.0);
  EXPECT_FALSE(replayLoopConfig("legacy").revisit_verification_enabled);
  EXPECT_THROW(replayLoopConfig("unknown"), std::invalid_argument);
  std::ostringstream metadata;
  writeConfigMetadata(metadata, scheduled, {}, {0.2, 200000U}, {});
  EXPECT_NE(metadata.str().find("loop_pipeline.discovery_interval_keyframes=3\n"), std::string::npos);
  EXPECT_NE(metadata.str().find("policy_version=v3_scheduled_motion_v1\n"), std::string::npos);
  EXPECT_NE(metadata.str().find("confirmation_motion_independence_proven=false\n"), std::string::npos);
}

TEST(LoopRecallReplay, PacingFlagRejectsAmbiguousEnvironmentValues)
{
  EXPECT_FALSE(parsePaceFlag(nullptr));
  EXPECT_FALSE(parsePaceFlag("0"));
  EXPECT_TRUE(parsePaceFlag("1"));
  for (const char * invalid : {"", "true", "2", "-1", " 1"}) {
    EXPECT_THROW(parsePaceFlag(invalid), std::invalid_argument);
  }
}

TEST(LoopRecallReplay, CachedExperimentPreservesScheduledDiscoveryAndLimits)
{
  const auto normal=replayLoopConfig("scheduled");
  const auto cached=replayLoopConfig("scheduled_cached");
  EXPECT_EQ(cached.discovery_interval_keyframes,normal.discovery_interval_keyframes);
  EXPECT_EQ(cached.confirmation_frames,normal.confirmation_frames);
  EXPECT_DOUBLE_EQ(cached.confirmation_min_translation_m,normal.confirmation_min_translation_m);
  EXPECT_DOUBLE_EQ(cached.confirmation_min_rotation_rad,normal.confirmation_min_rotation_rad);
  EXPECT_DOUBLE_EQ(cached.maximum_descriptor_distance,normal.maximum_descriptor_distance);
  EXPECT_EQ(cached.origin_top_k,normal.origin_top_k);
}

TEST(LoopRecallReplay, QueueEvidenceDoesNotStopProducerAtConstructionAndSavesAtScopeExit)
{
  Scratch scratch;
  replay_queue::ArrivalQueue q({0,2000000},32);
  {QueueEvidence evidence(q,scratch.root/"arrivals.csv");
    EXPECT_EQ(q.pop(),0U); EXPECT_EQ(q.pop(),1U); q.waitDone();}
  EXPECT_EQ(q.stats().produced,2U); EXPECT_EQ(q.stats().consumed,2U);
  std::ifstream input(scratch.root/"arrivals.csv");
  const std::string saved((std::istreambuf_iterator<char>(input)),{});
  EXPECT_NE(saved.find("arrival,1,"),std::string::npos);
  EXPECT_NE(saved.find("dequeue,1,"),std::string::npos);
}

TEST(LoopRecallReplay, QueueEvidenceSavesActualOverflowWhenPopThrows)
{
  Scratch scratch;
  replay_queue::ArrivalQueue q({0,1,2},2);
  q.waitDone();
  EXPECT_THROW({QueueEvidence evidence(q,scratch.root/"overflow.csv");q.pop();},std::runtime_error);
  std::ifstream input(scratch.root/"overflow.csv");
  const std::string saved((std::istreambuf_iterator<char>(input)),{});
  EXPECT_NE(saved.find("overflow,2,"),std::string::npos);
  EXPECT_TRUE(q.stats().overflow); EXPECT_EQ(q.stats().max_waiting,2U);
  EXPECT_EQ(q.stats().consumed,0U);
}

TEST(LoopRecallReplay, QueueEvidenceSurvivesUnrelatedProcessingException)
{
  Scratch scratch;
  replay_queue::ArrivalQueue q({0,1000000000},32);
  EXPECT_THROW({QueueEvidence evidence(q,scratch.root/"exception.csv");
    q.pop();throw std::runtime_error("diagnostic failure");},std::runtime_error);
  std::ifstream input(scratch.root/"exception.csv");
  const std::string saved((std::istreambuf_iterator<char>(input)),{});
  EXPECT_NE(saved.find("arrival,0,"),std::string::npos);
  EXPECT_NE(saved.find("dequeue,0,"),std::string::npos);
}

TEST(LoopRecallReplay, PacingUsesActualTimestampOffsetsAndStrictQueueCapacityBoundary)
{
  PaceSchedule pace({1000000000LL, 1010000000LL, 1020000000LL, 1030000000LL}, false, 2U);
  EXPECT_EQ(pace.dueNs(0U), 0LL);
  EXPECT_EQ(pace.dueNs(2U), 20000000LL);
  EXPECT_EQ(pace.pendingCount(-1LL, 0U), 0U);
  EXPECT_EQ(pace.pendingCount(0LL, 0U), 1U);
  EXPECT_EQ(pace.pendingCount(19999999LL, 1U), 1U);
  EXPECT_EQ(pace.pendingCount(20000000LL, 1U), 2U);
  EXPECT_FALSE(pace.capacityExceeded(20000000LL, 1U));
  EXPECT_TRUE(pace.capacityExceeded(30000000LL, 1U));
  EXPECT_EQ(pace.pendingCount(100000000LL, 4U), 0U);
  EXPECT_THROW(pace.pendingCount(0LL, 5U), std::out_of_range);
}

TEST(LoopRecallReplay, EnabledPacingWaitsAgainstSteadyClockRatherThanRecordedProcessingCosts)
{
  PaceSchedule pace({0LL, 10000000LL, 20000000LL}, true, 2U);
  pace.waitUntilArrival(2U);
  EXPECT_GE(pace.elapsedNs(), 20000000LL);
  EXPECT_THROW((PaceSchedule{{}, false, 2U}), std::invalid_argument);
  EXPECT_THROW((PaceSchedule{{1LL, 1LL}, false, 2U}), std::invalid_argument);
  EXPECT_THROW((PaceSchedule{{2LL, 1LL}, false, 2U}), std::invalid_argument);
  EXPECT_THROW((PaceSchedule{{-1LL}, false, 2U}), std::invalid_argument);
}

TEST(LoopRecallReplay, EveryFrameRequiresCommittedGraphEvenWhenPolicyReasonHidesUpdateFailure)
{
  a2w_fastlio_mapping::OptimizedPoseSnapshot after;
  after.revision = 3U;
  after.poses = {{0U, {}}, {1U, {}}, {2U, {}}};
  LoopPipelineEvent event;
  event.reason = "loop_discovery_interval_pending";
  EXPECT_TRUE(graphFrameFailure(2U, after, 2U, {event}).empty());
  EXPECT_FALSE(graphFrameFailure(3U, after, 2U, {event}).empty());
  EXPECT_FALSE(graphFrameFailure(2U, after, 3U, {event}).empty());
  event.reason = "gtsam_update_failed:synthetic";
  EXPECT_FALSE(graphFrameFailure(2U, after, 2U, {event}).empty());
  after.poses[2].id = 99U;
  event.reason = "no_loop_candidates";
  EXPECT_FALSE(graphFrameFailure(2U, after, 2U, {event}).empty());
}

TEST(LoopRecallReplay, SaveFinalUpdateAfterLastFrameLoopAdvancesRevisionWithoutAddingFactors)
{
  Scratch scratch;
  a2w_fastlio_mapping::PoseGraphOptimizer graph(a2w_fastlio_mapping::PoseGraphConfig{});
  a2w_fastlio_common::Pose3d pose;
  ASSERT_TRUE(graph.addKeyFrame(0U, pose).accepted);
  ASSERT_TRUE(graph.update().accepted);
  pose.translation.x() = 1.0;
  ASSERT_TRUE(graph.addKeyFrame(1U, pose).accepted);
  ASSERT_TRUE(graph.update().accepted);
  pose.translation.x() = 0.1;
  ASSERT_TRUE(graph.addKeyFrame(2U, pose).accepted);
  a2w_fastlio_mapping::LoopConstraint loop;
  loop.from_id = 2U; loop.to_id = 0U; loop.accepted = true;
  ASSERT_TRUE(graph.addLoopConstraint(loop).accepted);
  const auto last_frame_update = graph.update();
  ASSERT_TRUE(last_frame_update.accepted);
  ASSERT_EQ(last_frame_update.factor_count, 4U);
  const auto online_revision = graph.optimizedPoses().revision;
  std::ostringstream metadata;
  const auto saved = saveFinalGraphUpdate(graph, scratch.root, metadata, last_frame_update.factor_count);
  EXPECT_EQ(saved.before_revision, online_revision);
  EXPECT_EQ(saved.after_revision, online_revision + 1U);
  EXPECT_EQ(saved.factor_count, 4U);
  EXPECT_EQ(graph.optimizedPoses().poses.size(), 3U);
  EXPECT_GE(saved.elapsed_ms, 0.0);
  EXPECT_NE(metadata.str().find("save_final_update=true\n"), std::string::npos);
  for (const auto * name : {"online_last_frame_poses.csv", "optimized_poses.csv"}) {
    EXPECT_GT(fs::file_size(scratch.root / name), 50U);
    const auto hash = sha256File(scratch.root / name);
    EXPECT_EQ(hash.size(), 64U);
    EXPECT_EQ(hash.find_first_not_of("0123456789abcdef"), std::string::npos);
  }
}

}  // namespace
}  // namespace loop_recall_replay
