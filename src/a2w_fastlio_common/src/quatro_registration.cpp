#include "a2w_fastlio_common/quatro_registration.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <array>
#include <cstdint>
#include <cstring>
#include <list>
#include <mutex>

#include <pcl/common/point_tests.h>
#include <quatro/quatro_module.h>

namespace a2w_fastlio_common
{
namespace
{

bool validCloud(const CloudConstPtr & cloud, const std::size_t minimum_points)
{
  if (!cloud || cloud->size() < minimum_points) {
    return false;
  }
  for (const auto & point : cloud->points) {
    if (!pcl::isFinite(point)) {
      return false;
    }
  }
  return true;
}

Pose3d poseFromMatrix(const Eigen::Matrix4d & matrix)
{
  Pose3d pose;
  pose.rotation = Eigen::Quaterniond{matrix.block<3, 3>(0, 0)}.normalized();
  pose.translation = matrix.block<3, 1>(0, 3);
  return pose;
}

}  // namespace

struct QuatroRegistration::Cache
{
  using Prepared = quatro<PointT>::PreparedCloud;
  struct Entry {
    std::uint64_t fingerprint;
    std::vector<std::array<float, 4>> points;
    std::shared_ptr<const Prepared> prepared;
  };
  mutable std::mutex mutex;
  std::list<Entry> entries;
  std::size_t hits{0U}, misses{0U};

  std::shared_ptr<const Prepared> get(
    const CloudConstPtr & cloud, quatro<PointT> & solver, std::size_t capacity)
  {
    // Include order and intensity. Exact equality below rejects hash collisions.
    // This is a content key, never a mutable cloud pointer or keyframe ID key.
    std::vector<std::array<float, 4>> points;
    points.reserve(cloud->size());
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto & p : cloud->points) {
      points.push_back({p.x, p.y, p.z, p.intensity});
      for (float value : points.back()) {
        std::uint32_t bits; std::memcpy(&bits, &value, sizeof(bits));
        for (int k=0;k<4;++k) {hash ^= (bits >> (k*8)) & 255U; hash *= 1099511628211ULL;}
      }
    }
    for (auto it=entries.begin();it!=entries.end();++it) {
      if (it->fingerprint == hash && it->points.size() == points.size() &&
        std::memcmp(it->points.data(), points.data(), points.size()*sizeof(points[0])) == 0)
      {
        ++hits;
        auto prepared=it->prepared;
        entries.splice(entries.begin(), entries, it);
        return prepared;
      }
    }
    ++misses;
    auto prepared=std::make_shared<const Prepared>(solver.prepare(*cloud));
    entries.push_front({hash, std::move(points), prepared});
    while (entries.size() > capacity) {entries.pop_back();}
    return prepared;
  }
};

QuatroRegistration::QuatroRegistration(QuatroRegistrationConfig config) : config_{config}
{
  if (!(config_.fpfh_normal_radius_m > 0.0) || !(config_.fpfh_radius_m > 0.0) ||
    !(config_.noise_bound_m > 0.0) || !(config_.rotation_gnc_factor > 1.0) ||
    !(config_.rotation_cost_threshold > 0.0) || config_.rotation_max_iterations <= 0 ||
    !(config_.descriptor_distance_threshold > 0.0) || config_.maximum_correspondences <= 0 ||
    config_.minimum_points < 3U)
  {
    throw std::invalid_argument{"invalid Quatro registration configuration"};
  }
  if (config_.feature_cache_capacity > 0U) {cache_=std::make_shared<Cache>();}
}

QuatroCacheStats QuatroRegistration::cacheStats() const
{
  if (!cache_) {return {};}
  std::lock_guard<std::mutex> lock(cache_->mutex);
  return {cache_->hits, cache_->misses, cache_->entries.size()};
}

RegistrationResult QuatroRegistration::align(
  const CloudConstPtr & source, const CloudConstPtr & target,
  const std::optional<Pose3d> & /*initial_guess*/) const
{
  RegistrationResult result;
  if (!validCloud(source, config_.minimum_points) ||
    !validCloud(target, config_.minimum_points))
  {
    result.rejection_reason = "invalid or too-small input cloud";
    return result;
  }

  const auto started = std::chrono::steady_clock::now();
  quatro<PointT> solver{
    config_.fpfh_normal_radius_m, config_.fpfh_radius_m, config_.noise_bound_m,
    config_.rotation_gnc_factor, config_.rotation_cost_threshold,
    config_.rotation_max_iterations, config_.estimate_scale, config_.optimized_matching,
    config_.descriptor_distance_threshold, config_.maximum_correspondences};
  bool valid = false;
  Eigen::Matrix4d matrix;
  if (cache_) {
    // Parameters/feature version are immutable per adapter instance. Transformed
    // coordinates encode the exact pose snapshot. A new preprocessing/config
    // requires a new adapter; changed point content always misses the cache.
    std::lock_guard<std::mutex> lock(cache_->mutex);
    const auto prepared_source=cache_->get(source, solver, config_.feature_cache_capacity);
    const auto prepared_target=cache_->get(target, solver, config_.feature_cache_capacity);
    matrix=solver.alignPrepared(*prepared_source, *prepared_target, valid);
  } else {
    matrix=solver.align(*source, *target, valid);
  }
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.transform = poseFromMatrix(matrix);
  result.success = valid && matrix.allFinite() && isFinitePose(result.transform);
  result.converged = result.success;
  result.fitness = result.success ? 0.0 : std::numeric_limits<double>::infinity();
  if (!result.success) {
    result.rejection_reason = "Quatro did not produce a valid transform";
  }
  return result;
}

}  // namespace a2w_fastlio_common
