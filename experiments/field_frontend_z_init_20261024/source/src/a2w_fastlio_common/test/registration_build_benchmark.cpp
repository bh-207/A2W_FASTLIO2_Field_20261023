// Offline build-mode comparison: same PCD, parameters and known rigid transform.
// This is a timing probe, not a JT128 hardware or loop-closure acceptance test.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>

#include "a2w_fastlio_common/nano_gicp_registration.hpp"
#include "a2w_fastlio_common/quatro_registration.hpp"

int main(int argc, char ** argv)
{
  using namespace a2w_fastlio_common;
  if (argc != 2) {
    std::cerr << "usage: registration_build_benchmark INPUT.pcd\n";
    return 2;
  }
  auto source = CloudPtr{new Cloud};
  if (pcl::io::loadPCDFile<PointT>(argv[1], *source) < 0 || source->empty()) {
    std::cerr << "cannot load source PCD\n";
    return 2;
  }
  Pose3d expected;
  expected.rotation = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ());
  expected.translation = Eigen::Vector3d{0.2, -0.1, 0.05};
  auto target = CloudPtr{new Cloud};
  for (auto point : *source) {
    const Eigen::Vector3d xyz = expected.rotation *
      point.getVector3fMap().cast<double>() + expected.translation;
    point.getVector3fMap() = xyz.cast<float>();
    target->push_back(point);
  }
  const QuatroRegistration coarse;
  const NanoGicpRegistration fine;
  const auto coarse_result = coarse.align(source, target, std::nullopt);
  // Fixed initial guess keeps fine timing comparable even when randomized
  // coarse matching chooses different correspondences in separate processes.
  auto guess = expected;
  guess.translation += Eigen::Vector3d{0.03, -0.02, 0.01};
  const auto fine_result = fine.align(source, target, guess);
  const auto translation_error = (fine_result.transform.translation - expected.translation).norm();
  const auto rotation_error = Eigen::AngleAxisd(
    expected.rotation.inverse() * fine_result.transform.rotation).angle();
  std::cout << "source_points=" << source->size() << " target_points=" << target->size()
            << " coarse_success=" << coarse_result.success
            << " coarse_ms=" << coarse_result.elapsed_ms
            << " fine_success=" << fine_result.success
            << " fine_ms=" << fine_result.elapsed_ms
            << " fine_fitness=" << fine_result.fitness
            << " translation_error_m=" << translation_error
            << " rotation_error_rad=" << rotation_error << '\n';
  return fine_result.success && translation_error < 0.02 && rotation_error < 0.01 ? 0 : 1;
}
