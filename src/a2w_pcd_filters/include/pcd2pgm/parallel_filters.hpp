#ifndef PCD2PGM__PARALLEL_FILTERS_HPP_
#define PCD2PGM__PARALLEL_FILTERS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/types.h>
#include <functional>

namespace pcd2pgm
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct GroundOptions
{
  int max_window_size = 20;
  float slope = 1.0f;
  float initial_distance = 0.5f;
  float max_distance = 3.0f;
};

// A positive upper bound, clamped to the processors available to OpenMP.
int filterThreadCount(int requested);

// Exact PCL PMF window/threshold semantics; input and its coordinates are immutable.
pcl::Indices extractGroundParallel(
  const Cloud::ConstPtr & input, const GroundOptions & options, int threads,
  const std::function<void(float, std::size_t, double)> & progress = {});

// Original 3-D K=12 neighbor mean-height selection, with stable output order.
Cloud::Ptr extractObstaclesParallel(
  const Cloud::ConstPtr & input, const Cloud::ConstPtr & ground,
  double h_min, double h_max, int threads);
}  // namespace pcd2pgm
#endif
