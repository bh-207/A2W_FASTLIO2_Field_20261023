// Manual sequential replay of saved keyframes through the real LoopPipeline.
// The Map Bundle is read-only and no ROS graph or hardware is involved.
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_mapping/loop_pipeline.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace
{

double elapsedMs(const std::chrono::steady_clock::time_point & started)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
}

}  // namespace

int main(int argc, char ** argv)
try {
  if (argc != 2 && argc != 3) {
    std::cerr << "usage: mapping_replay_benchmark MAP_BUNDLE [LAST_KEYFRAME_ID]\n";
    return 2;
  }
  const auto bundle = a2w_fastlio_map::MapBundleReader{}.read(argv[1]);
  const auto last_id = argc == 3 ? std::stoull(argv[2]) :
    std::numeric_limits<unsigned long long>::max();
  auto suite = a2w_fastlio_common::createDefaultAlgorithmSuite({});
  auto graph = std::make_shared<a2w_fastlio_mapping::PoseGraphOptimizer>(
    a2w_fastlio_mapping::PoseGraphConfig{});
  a2w_fastlio_mapping::LoopPipeline pipeline{
    suite.place_recognition, suite.descriptor_index, suite.registration, graph,
    a2w_fastlio_common::LocalMapBuilder{{0.2, 200000U}},
    a2w_fastlio_mapping::LoopPipelineConfig{5U, 30U, 5U, 5U, 10U, 32U, 0.2}};

  const auto replay_started = std::chrono::steady_clock::now();
  std::size_t accepted_loops = 0U;
  std::size_t candidate_events = 0U;
  std::size_t processed = 0U;
  for (auto frame : bundle.data.keyframes) {
    if (frame.id > last_id) {break;}
    // This is the state used when a fresh online keyframe enters Mapping.
    frame.optimized_pose = frame.odom_pose;
    const auto frame_started = std::chrono::steady_clock::now();
    const auto events = pipeline.process(frame);
    ++processed;
    candidate_events += events.size();
    for (const auto & event : events) {
      if (event.accepted) {
        ++accepted_loops;
      }
      std::cout << "event current=" << event.current_id
                << " candidate=" << event.candidate_id
                << " rank=" << event.candidate_rank
                << " accepted=" << event.accepted
                << " reason=" << event.reason << '\n';
      if (event.target_points > 0U) {
        std::cout << a2w_fastlio_mapping::loopPoseCheckChinese(
          event, bundle.data.metadata.map_frame, bundle.data.metadata.tracking_frame) << '\n';
      }
    }
    std::cout << "frame current=" << frame.id
              << " events=" << events.size()
              << " frame_ms=" << elapsedMs(frame_started) << '\n';
  }
  std::cout << "summary frames=" << processed
            << " events=" << candidate_events
            << " accepted_loops=" << accepted_loops
            << " total_ms=" << elapsedMs(replay_started) << '\n';
  return 0;
} catch (const std::exception & error) {
  std::cerr << "mapping replay failed: " << error.what() << '\n';
  return 1;
}
