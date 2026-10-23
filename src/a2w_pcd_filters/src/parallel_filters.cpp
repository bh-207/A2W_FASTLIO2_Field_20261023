/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2014, RadiantBlue Technologies, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Adapted from PCL 1.12 progressive_morphological_filter.hpp and
 * morphological_filter.hpp: parallel independent queries, unchanged PMF math.
 */
#include "pcd2pgm/parallel_filters.hpp"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/search/kdtree.h>
#include <omp.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace pcd2pgm
{
int filterThreadCount(int requested)
{
  if (requested < 1) {
    throw std::invalid_argument("num_threads must be positive");
  }
  return std::min(requested, std::max(1, omp_get_num_procs()));
}

namespace
{
Cloud opening(const Cloud::ConstPtr & cloud, float window, int threads)
{
  Cloud eroded = *cloud;
  Cloud opened = *cloud;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree(window);
  tree.setInputCloud(cloud);
  tree.addPointsFromInputCloud();
  const float half = window / 2.0f;
  const float limit = std::numeric_limits<float>::max();
  const auto count = static_cast<std::int64_t>(cloud->size());

  // The built tree is immutable. Each worker owns its query scratch space.
  // Do not cache all neighborhoods: large windows would need quadratic memory.
  #pragma omp parallel num_threads(threads)
  {
    pcl::Indices neighbors;
    #pragma omp for schedule(dynamic, 128)
    for (std::int64_t i = 0; i < count; ++i) {
      const auto & p = (*cloud)[i];
      tree.boxSearch(Eigen::Vector3f(p.x - half, p.y - half, -limit),
        Eigen::Vector3f(p.x + half, p.y + half, limit), neighbors);
      if (!neighbors.empty()) {
        Eigen::Vector4f low, high;
        pcl::getMinMax3D(*cloud, neighbors, low, high);
        eroded[i].z = low.z();
      }
    }
    // Mandatory implicit barrier above: dilation reads every erosion result.
    #pragma omp for schedule(dynamic, 128)
    for (std::int64_t i = 0; i < count; ++i) {
      const auto & p = (*cloud)[i];
      tree.boxSearch(Eigen::Vector3f(p.x - half, p.y - half, -limit),
        Eigen::Vector3f(p.x + half, p.y + half, limit), neighbors);
      if (!neighbors.empty()) {
        Eigen::Vector4f low, high;
        pcl::getMinMax3D(eroded, neighbors, low, high);
        opened[i].z = high.z();
      }
    }
  }
  return opened;
}
}  // namespace

pcl::Indices extractGroundParallel(
  const Cloud::ConstPtr & input, const GroundOptions & options, int threads,
  const std::function<void(float, std::size_t, double)> & progress)
{
  threads = filterThreadCount(threads);
  pcl::Indices ground;
  if (!input || input->empty()) { return ground; }
  ground.resize(input->size());
  std::iota(ground.begin(), ground.end(), 0);
  float previous_window = 0.0f;
  int iteration = 0;
  // Preserve PCL's default cell_size=1, base=2, exponential=true.
  // max_window_size=32 intentionally processes windows 3,5,9,17,33.
  while (previous_window < options.max_window_size && !ground.empty()) {
    const auto start = std::chrono::steady_clock::now();
    const float window = 2.0f * std::pow(2.0f, iteration) + 1.0f;
    float threshold = iteration == 0 ? options.initial_distance :
      options.slope * (window - previous_window) + options.initial_distance;
    if (threshold > options.max_distance) { threshold = options.max_distance; }
    auto current = pcl::make_shared<Cloud>();
    pcl::copyPointCloud(*input, ground, *current);
    const auto filtered = opening(current, window, threads);
    pcl::Indices survivors;
    survivors.reserve(ground.size());
    for (std::size_t i = 0; i < ground.size(); ++i) {
      if ((*current)[i].z - filtered[i].z < threshold) {
        survivors.push_back(ground[i]);
      }
    }
    ground.swap(survivors);
    if (progress) {
      progress(window, ground.size(),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    }
    previous_window = window;
    ++iteration;
  }
  return ground;
}

Cloud::Ptr extractObstaclesParallel(
  const Cloud::ConstPtr & input, const Cloud::ConstPtr & ground,
  double h_min, double h_max, int threads)
{
  threads = filterThreadCount(threads);
  auto output = pcl::make_shared<Cloud>();
  if (!input || !ground || input->empty() || ground->empty()) { return output; }
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(ground);
  std::vector<std::uint8_t> keep(input->size(), 0);
  const auto count = static_cast<std::int64_t>(input->size());
  #pragma omp parallel num_threads(threads)
  {
    std::vector<int> indices;
    std::vector<float> distances;
    #pragma omp for schedule(dynamic, 128)
    for (std::int64_t i = 0; i < count; ++i) {
      const auto & p = (*input)[i];
      if (tree.nearestKSearch(p, 12, indices, distances) <= 0) { continue; }
      double z = 0.0;
      int used = 0;
      for (int index : indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= ground->size()) { continue; }
        z += (*ground)[index].z;
        ++used;
      }
      if (used == 0) { continue; }
      z /= static_cast<double>(used);
      const double height = static_cast<double>(p.z) - z;
      keep[i] = height >= h_min && height <= h_max;
    }
  }
  output->reserve(input->size());
  for (std::size_t i = 0; i < input->size(); ++i) {
    if (keep[i]) { output->push_back((*input)[i]); }
  }
  return output;
}
}  // namespace pcd2pgm
