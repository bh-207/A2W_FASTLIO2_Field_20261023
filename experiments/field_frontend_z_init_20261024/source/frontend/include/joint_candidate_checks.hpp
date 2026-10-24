#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <string>
namespace frontend_joint {
using Cov18=Eigen::Matrix<double,18,18>;
struct Check{bool accepted;std::string reason;};
inline Check validate(const Eigen::Matrix3d&R,const Eigen::Vector3d&g,
 const Eigen::Vector3d&ba,const Eigen::Vector3d&bg,const Cov18&P){
 if(!R.allFinite()||!g.allFinite()||!ba.allFinite()||!bg.allFinite()||!P.allFinite())return {false,"nonfinite"};
 if((R.transpose()*R-Eigen::Matrix3d::Identity()).norm()>1e-6||std::abs(R.determinant()-1)>1e-6)return {false,"rotation_invalid"};
 if(std::abs(g.norm()-9.809)>1e-6)return {false,"S2_norm_inconsistent"};
 if(ba.norm()>1.5||bg.norm()>.1)return {false,"bias_limit"};
 if((P-P.transpose()).norm()>1e-8)return {false,"covariance_asymmetric"};
 Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(P);
 if(eig.info()!=Eigen::Success||eig.eigenvalues().minCoeff()<-1e-9)return {false,"covariance_not_PSD"};
 Eigen::Matrix3d tangent=Eigen::Matrix3d::Identity()-g.normalized()*g.normalized().transpose();
 Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ge(tangent*P.block<3,3>(9,9)*tangent/(9.809*9.809));
 if(ge.info()!=Eigen::Success||ge.eigenvalues().maxCoeff()>std::pow(M_PI/180.,2))return {false,"gravity_direction_uncertainty"};
 if(P.block<3,3>(12,12).diagonal().maxCoeff()>.3*.3)return {false,"accel_bias_uncertainty"};
 if(P.block<3,3>(15,15).diagonal().maxCoeff()>.02*.02)return {false,"gyro_bias_uncertainty"};
 return {true,"joint_candidate_validated"};
}
inline Eigen::Matrix<double,23,23> map_covariance(const Cov18&P,const Eigen::Matrix<double,2,3>&N){
 Eigen::Matrix<double,23,18> G=Eigen::Matrix<double,23,18>::Zero();
 G.block<3,3>(0,0).setIdentity();G.block<3,3>(3,3).setIdentity();
 G.block<3,3>(12,6).setIdentity();G.block<2,3>(21,9)=N;
 G.block<3,3>(18,12).setIdentity();G.block<3,3>(15,15).setIdentity();
 Eigen::Matrix<double,23,23> result=G*P*G.transpose();
 // Preserve the existing fixed-extrinsic blocks, with zero new cross terms.
 result.block<6,6>(6,6).diagonal().setConstant(1e-5);
 return (result+result.transpose()).eval()*.5;
}
}
