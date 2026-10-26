#pragma once
// Pre-commit startup state guard, quality gate only. Versioned validation
// evidence is retained under artifacts/loop_v4_20261021/frontend/.
// Include after use-ikfom.hpp. No world reorientation, bias reset or calibration.
#include <Eigen/Cholesky>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace frontend_init_v4 {
struct StateGuardConfig {
  double quaternion_norm_tolerance=1e-6;
  double rotation_orthogonality_tolerance=1e-6;
  double rotation_determinant_tolerance=1e-6;
  double initial_rotation_identity_tolerance=1e-8;
  double covariance_symmetry_tolerance=1e-10;
  double minimum_covariance_pivot=0.;
  double maximum_mean_accel_residual_m_s2=.05;
  double maximum_last_accel_residual_m_s2=.25;
  double gravity_norm_tolerance_m_s2=1e-6;
  double candidate_preservation_tolerance=1e-12;
  void validate()const {
    for(double v:{quaternion_norm_tolerance,rotation_orthogonality_tolerance,
      rotation_determinant_tolerance,initial_rotation_identity_tolerance,
      covariance_symmetry_tolerance,maximum_mean_accel_residual_m_s2,
      maximum_last_accel_residual_m_s2,gravity_norm_tolerance_m_s2,candidate_preservation_tolerance})
      if(!std::isfinite(v)||v<=0.)throw std::invalid_argument("invalid initialization state guard tolerance");
    if(!std::isfinite(minimum_covariance_pivot)||minimum_covariance_pivot<0.)
      throw std::invalid_argument("invalid covariance pivot floor");
    if(quaternion_norm_tolerance>.001||rotation_orthogonality_tolerance>.001||
      rotation_determinant_tolerance>.001||initial_rotation_identity_tolerance>1e-6)
      throw std::invalid_argument("startup state guard numerical rotation tolerances cannot disable proper/identity checks");
  }
};

struct StateGuardReport {
  bool accepted=false,mean_checked=false,last_checked=false;
  std::string reason;
  Eigen::Vector3d mean_residual=Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d last_residual=Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double pre_covariance_symmetry=std::numeric_limits<double>::quiet_NaN();
  double pre_covariance_minimum_pivot=std::numeric_limits<double>::quiet_NaN();
  double candidate_covariance_symmetry=std::numeric_limits<double>::quiet_NaN();
  double candidate_covariance_minimum_pivot=std::numeric_limits<double>::quiet_NaN();
  double raw_extrinsic_determinant=std::numeric_limits<double>::quiet_NaN();
  double raw_extrinsic_orthogonality=std::numeric_limits<double>::quiet_NaN();
};

inline bool proper_rotation(const Eigen::Matrix3d &r,const StateGuardConfig &c,
  double * determinant=nullptr,double * orthogonality=nullptr){
  if(!r.allFinite())return false;
  const double d=r.determinant(),e=(r.transpose()*r-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
  if(determinant)*determinant=d;if(orthogonality)*orthogonality=e;
  return std::isfinite(d)&&std::isfinite(e)&&std::abs(d-1.)<=c.rotation_determinant_tolerance&&e<=c.rotation_orthogonality_tolerance;
}

inline std::string state_structure_reason(const state_ikfom &s,const StateGuardConfig &c){
  for(const auto &v:std::array<Eigen::Vector3d,6>{Eigen::Vector3d(s.pos),Eigen::Vector3d(s.vel),
    Eigen::Vector3d(s.bg),Eigen::Vector3d(s.ba),Eigen::Vector3d(s.offset_T_L_I),Eigen::Vector3d(s.grav)})
    if(!v.allFinite())return "nonfinite_vector";
  const auto q=s.rot.coeffs(),ext=s.offset_R_L_I.coeffs();
  if(!q.allFinite()||!ext.allFinite()||!std::isfinite(q.norm())||!std::isfinite(ext.norm())||
    std::abs(q.norm()-1.)>c.quaternion_norm_tolerance||std::abs(ext.norm()-1.)>c.quaternion_norm_tolerance)
    return "invalid_quaternion";
  if(!proper_rotation(s.rot.toRotationMatrix(),c)||!proper_rotation(s.offset_R_L_I.toRotationMatrix(),c))return "nonproper_rotation";
  if(std::abs(Eigen::Vector3d(s.grav).norm()-9.809)>c.gravity_norm_tolerance_m_s2)return "invalid_gravity_norm";
  return {};
}

template<class Covariance>
inline std::string covariance_reason(const Covariance &p,const StateGuardConfig &c,double &symmetry,double &minimum_pivot){
  if(!p.allFinite())return "nonfinite";
  symmetry=(p-p.transpose()).cwiseAbs().maxCoeff();
  if(!std::isfinite(symmetry)||symmetry>c.covariance_symmetry_tolerance)return "asymmetric";
  // No projection, symmetrization or silent replacement of a bad existing P.
  Eigen::LDLT<Covariance> decomposition(p);
  if(decomposition.info()!=Eigen::Success||!decomposition.vectorD().allFinite())return "not_positive_definite";
  minimum_pivot=decomposition.vectorD().minCoeff();
  if(minimum_pivot<=c.minimum_covariance_pivot)return "not_positive_definite";
  return {};
}

template<class Covariance>
inline StateGuardReport check_state_inputs(const state_ikfom &before,const Covariance &before_p,
  const Eigen::Matrix3d &raw_ext_r,const Eigen::Vector3d &raw_ext_t,
  const std::array<Eigen::Vector3d,4> &noise_variances,const StateGuardConfig &c){
  StateGuardReport result;
  const auto state_reason=state_structure_reason(before,c);
  if(!state_reason.empty()){result.reason="state_pre_"+state_reason;return result;}
  if((before.rot.toRotationMatrix()-Eigen::Matrix3d::Identity()).norm()>c.initial_rotation_identity_tolerance){
    result.reason="state_requires_fresh_identity_rotation_restart";return result;}
  if(!raw_ext_t.allFinite()||!raw_ext_r.allFinite()){result.reason="state_raw_extrinsic_nonfinite";return result;}
  if(!proper_rotation(raw_ext_r,c,&result.raw_extrinsic_determinant,&result.raw_extrinsic_orthogonality)){
    result.reason="state_raw_extrinsic_nonproper_rotation";return result;}
  const auto p_reason=covariance_reason(before_p,c,result.pre_covariance_symmetry,result.pre_covariance_minimum_pivot);
  if(!p_reason.empty()){result.reason="state_pre_covariance_"+p_reason;return result;}
  for(const auto &v:noise_variances){
    if(!v.allFinite()){result.reason="state_noise_variance_nonfinite";return result;}
    if((v.array()<=0.).any()){result.reason="state_noise_variance_not_positive";return result;}}
  result.accepted=true;result.reason="state_preconditions_pass";return result;
}

template<class Covariance>
inline StateGuardReport check_state_candidate(const state_ikfom &before,const state_ikfom &candidate,
  const Covariance &candidate_p,const Eigen::Matrix3d &raw_ext_r,const Eigen::Vector3d &raw_ext_t,
  const Eigen::Vector3d &mean_acc,const Eigen::Vector3d &mean_gyro,const Eigen::Vector3d &last_acc,
  const StateGuardConfig &c,StateGuardReport result){
  result.accepted=false;
  const auto state_reason=state_structure_reason(candidate,c);
  if(!state_reason.empty()){result.reason="state_candidate_"+state_reason;return result;}
  const auto p_reason=covariance_reason(candidate_p,c,result.candidate_covariance_symmetry,result.candidate_covariance_minimum_pivot);
  if(!p_reason.empty()){result.reason="state_candidate_covariance_"+p_reason;return result;}
  const double tol=c.candidate_preservation_tolerance;
  if((candidate.pos-before.pos).norm()>tol||(candidate.vel-before.vel).norm()>tol||
    (candidate.ba-before.ba).norm()>tol||(candidate.rot.toRotationMatrix()-before.rot.toRotationMatrix()).norm()>tol){
    result.reason="state_candidate_preserved_state_changed";return result;}
  if((candidate.offset_T_L_I-raw_ext_t).norm()>tol||
    (candidate.offset_R_L_I.toRotationMatrix()-raw_ext_r).norm()>c.rotation_orthogonality_tolerance){
    result.reason="state_candidate_extrinsic_mismatch";return result;}
  if(!mean_acc.allFinite()||!mean_gyro.allFinite()||!last_acc.allFinite()||!std::isfinite(mean_acc.norm())||mean_acc.norm()<=0.){
    result.reason="state_static_input_nonfinite_or_zero";return result;}
  if((candidate.bg-mean_gyro).norm()>tol){result.reason="state_candidate_bg_window_mismatch";return result;}
  // EXACT existing prediction domain: acceleration normalized by9.81/|mean_acc|,
  // ba is preserved in that internal SI domain; no new bias or gravity estimate.
  const double scale=9.81/mean_acc.norm();
  result.mean_residual=candidate.rot*(mean_acc*scale-candidate.ba)+Eigen::Vector3d(candidate.grav);result.mean_checked=true;
  if(!result.mean_residual.allFinite()||!std::isfinite(result.mean_residual.norm())){result.reason="state_mean_residual_nonfinite";return result;}
  if(result.mean_residual.norm()>c.maximum_mean_accel_residual_m_s2){result.reason="state_mean_residual_above_threshold";return result;}
  result.last_residual=candidate.rot*(last_acc*scale-candidate.ba)+Eigen::Vector3d(candidate.grav);result.last_checked=true;
  if(!result.last_residual.allFinite()||!std::isfinite(result.last_residual.norm())){result.reason="state_last_residual_nonfinite";return result;}
  if(result.last_residual.norm()>c.maximum_last_accel_residual_m_s2){result.reason="state_last_residual_above_threshold";return result;}
  result.accepted=true;result.reason="state_consistency_pass";return result;
}
} // namespace frontend_init_v4
