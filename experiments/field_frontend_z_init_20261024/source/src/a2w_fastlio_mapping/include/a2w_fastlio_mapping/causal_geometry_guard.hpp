#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{

// Caller must opt in. These geometric proxy limits are not physical truth.
struct CausalGeometryGuardConfig
{
  std::size_t maximum_frames{128U};
  std::size_t maximum_points_per_frame{4096U};
  std::size_t maximum_patches_per_frame{64U};
  std::size_t maximum_frame_pairs{256U};
  std::size_t maximum_pair_enumerations{524288U};
  std::size_t maximum_patch_records{512U};
  std::size_t minimum_patch_points{8U};
  std::size_t minimum_supported_frame_pairs{2U};
  std::size_t minimum_supported_patches{8U};
  double maximum_body_range_m{12.0};
  double body_voxel_m{1.5};
  double minimum_time_separation_s{30.0};
  double maximum_raw_observer_separation_m{2.0};
  double maximum_patch_centroid_separation_m{0.75};
  double minimum_tangent_eigenvalue_m2{0.0025};
  double maximum_eigenvalue_ratio{0.08};
  double maximum_plane_residual_p95_m{0.03};
  double maximum_raw_height_gap_m{0.03};
  double maximum_raw_normal_angle_deg{10.0};
  double maximum_median_retreat_m{0.03};
  double maximum_p95_retreat_m{0.05};
  double maximum_normal_p95_retreat_deg{1.0};
};

enum class CausalGeometryGuardVerdict {SUPPORTED_PASS, REJECT, UNKNOWN};

struct CausalGeometrySeamStats
{
  std::size_t count{0U};
  double height_absolute_median_m{0.0};
  double height_absolute_p95_m{0.0};
  double normal_median_deg{0.0};
  double normal_p95_deg{0.0};
};

struct CausalGeometryPatchRecord
{
  std::uint64_t first_id{0U}, second_id{0U};
  std::vector<std::uint32_t> first_point_indices{}, second_point_indices{};
  Eigen::Vector3d raw_reference{Eigen::Vector3d::Zero()};
  Eigen::Vector3d raw_axis{Eigen::Vector3d::UnitZ()};
  double raw_signed_height_gap_m{0.0}, raw_normal_angle_deg{0.0};
  double before_signed_height_gap_m{0.0}, trial_signed_height_gap_m{0.0};
  double before_normal_angle_deg{0.0}, trial_normal_angle_deg{0.0};
};

struct CausalGeometryPairRecord
{
  std::uint64_t first_id{0U}, second_id{0U};
  std::size_t matched_patch_count{0U};
  CausalGeometrySeamStats before{}, trial{};
  std::string reason{};
};

struct CausalGeometryGuardResult
{
  CausalGeometryGuardVerdict verdict{CausalGeometryGuardVerdict::UNKNOWN};
  std::string reason{"no_reliable_independent_anchors"};
  std::uint64_t current_id{0U}, candidate_id{0U};
  std::size_t eligible_frames{0U}, sampled_frames{0U}, excluded_frames{0U};
  std::size_t tested_frame_pairs{0U}, raw_supported_frame_pairs{0U};
  std::size_t enumerated_pair_checks{0U}, eligible_frame_pairs{0U};
  std::size_t unsampled_eligible_frame_pairs{0U}, per_pair_patch_budget{0U};
  std::size_t sampled_points{0U}, invalid_points{0U}, rejected_plane_patches{0U};
  bool frame_limit_reached{false}, pair_limit_reached{false}, patch_limit_reached{false};
  bool enumeration_limit_reached{false};
  std::int64_t selected_min_stamp_ns{0}, selected_max_stamp_ns{0};
  Eigen::Vector3d selected_raw_position_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d selected_raw_position_max{Eigen::Vector3d::Zero()};
  std::set<std::uint64_t> excluded_registration_ids{};
  CausalGeometrySeamStats before{}, trial{};
  std::vector<CausalGeometryPairRecord> pairs{};
  std::vector<CausalGeometryPatchRecord> records{};
  bool accepted() const noexcept {return verdict == CausalGeometryGuardVerdict::SUPPORTED_PASS;}
};

class CausalGeometryGuard
{
public:
  explicit CausalGeometryGuard(CausalGeometryGuardConfig config = {});
  CausalGeometryGuardResult assess(
    const std::map<std::uint64_t, a2w_fastlio_common::KeyFrame> & keyframes,
    const OptimizedPoseSnapshot & before, const OptimizedPoseSnapshot & trial,
    std::uint64_t current_id, std::uint64_t candidate_id,
    const std::set<std::uint64_t> & excluded_registration_ids) const;
  const CausalGeometryGuardConfig & config() const noexcept {return config_;}
private:
  CausalGeometryGuardConfig config_;
};

const char * causalGeometryGuardVerdictName(CausalGeometryGuardVerdict verdict) noexcept;
// One JSON object per invocation, including the complete frozen point identities.
void writeCausalGeometryGuardLedger(std::ostream & output, const CausalGeometryGuardResult & result);

}  // namespace a2w_fastlio_mapping
