#pragma once
#include <array>
#include <Eigen/Core>

namespace frontend_contract {
// IKFoM state_ikfom begins [p_world, theta_right_body, ...]; SO3::boxplus
// applies R <- R * Exp(theta_right_body). geometry_msgs/PoseWithCovariance
// requires [p_world, theta_fixed_world], with the fixed axes in header.frame_id.
// theta_fixed_world = R_world_body * theta_right_body to first order. This is
// a local covariance conversion, not an Euler-angle derivative or SE(3)
// adjoint: position is already a world-frame additive state.
template<class Covariance, class Rotation>
std::array<double, 36> pose_covariance_world(const Covariance &p, const Rotation &rotation) {
  Eigen::Matrix<double, 6, 6> tangent = Eigen::Matrix<double, 6, 6>::Identity();
  tangent.template block<3, 3>(3, 3) = rotation;
  const Eigen::Matrix<double, 6, 6> covariance =
    tangent * p.template block<6, 6>(0, 0) * tangent.transpose();
  std::array<double, 36> result{};
  for (int row=0; row<6; ++row)
    for (int column=0; column<6; ++column) result[row*6+column] = covariance(row, column);
  return result;
}
}
