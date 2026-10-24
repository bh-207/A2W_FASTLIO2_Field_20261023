#pragma once
#include "joint_candidate_checks.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <cstdint>
namespace frontend_joint {
struct Candidate {
 bool requested{false},active{false},committed{false};std::string reason{"disabled"},receipt;
 std::int64_t scan_id{-1},stamp_ns{0},header_ns{0};double acc_scale{1};
 Eigen::Vector3d p,v,g,ba,bg;Eigen::Matrix3d R;Cov18 covariance;
 static Eigen::Vector3d vector(const YAML::Node&n){
  if(!n.IsSequence()||n.size()!=3)throw std::invalid_argument("joint vector shape");
  Eigen::Vector3d x;for(int k=0;k<3;++k)x[k]=n[k].as<double>();if(!x.allFinite())throw std::invalid_argument("nonfinite joint vector");return x;
 }
 void load(const std::string&path,const std::string&report){
  requested=true;receipt=report;const auto n=YAML::LoadFile(path);
  if(n["schema"].as<std::string>()!="independent_lidar_imu_joint_candidate_v1")throw std::invalid_argument("joint evidence schema");
  reason=n["reason"].as<std::string>();
  if(!n["accepted"].as<bool>()){save();return;}
  if(n["g_ba_constrained_rank"].as<int>()!=5||!n["heldout_validation"]["passed"].as<bool>()||
     n["heldout_validation"]["used_in_fit"].as<bool>()||!n["frontend_world_alignment"]["applied"].as<bool>())
    throw std::invalid_argument("joint independent validation/alignment evidence missing");
  const std::vector<std::string> order{"p_WI","theta_I_right","v_WI","g_W","ba_I","bg_I"};
  if(n["covariance_order"].as<std::vector<std::string>>()!=order)throw std::invalid_argument("joint covariance tangent order");
  if(n["extrinsic_est_en"].as<bool>()||n["time_offset"].as<double>()!=0.||
     (vector(n["fixed_extrinsic_T"])-Eigen::Vector3d(-.00862,.00914,.03916)).norm()>1e-12)
    throw std::invalid_argument("joint fixed input contract differs");
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)if(std::abs(n["fixed_extrinsic_R"][i][j].as<double>()-(i==j?1.:0.))>1e-12)throw std::invalid_argument("joint extrinsic rotation differs");
  p=vector(n["p_WI_commit"]);v=vector(n["v_WI_commit"]);g=vector(n["g_W"]);ba=vector(n["ba_I"]);bg=vector(n["bg_I"]);
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)R(i,j)=n["R_WI_commit"][i][j].as<double>();
  if(n["full_covariance"].size()!=18)throw std::invalid_argument("joint covariance rows");
  for(int i=0;i<18;++i){if(n["full_covariance"][i].size()!=18)throw std::invalid_argument("joint covariance columns");for(int j=0;j<18;++j)covariance(i,j)=n["full_covariance"][i][j].as<double>();}
  acc_scale=n["acc_scale"].as<double>();scan_id=n["commit_scan_id"].as<std::int64_t>();stamp_ns=n["commit_stamp_ns"].as<std::int64_t>();header_ns=n["commit_header_stamp_ns"].as<std::int64_t>();
  if(!std::isfinite(acc_scale)||acc_scale<=0||scan_id<0||stamp_ns<=header_ns)throw std::invalid_argument("joint scale/time invalid");
  const auto check=validate(R,g,ba,bg,covariance);active=check.accepted;reason=check.reason;save();
 }
 void save(std::int64_t actual_scan=-1,std::int64_t actual_end=0,double actual_scale=0){
  const std::filesystem::path target(receipt);auto tmp=target;tmp+=".tmp";std::ofstream f(tmp);
  f<<std::setprecision(17)<<"{\"requested\":true,\"candidate_usable\":"<<(active?"true":"false")<<",\"committed\":"<<(committed?"true":"false")
   <<",\"fallback_to_original\":"<<(!active?"true":"false")<<",\"reason\":\""<<reason<<"\",\"candidate_scan_id\":"<<scan_id
   <<",\"actual_commit_scan_id\":"<<actual_scan<<",\"candidate_stamp_ns\":"<<stamp_ns<<",\"actual_commit_end_ns\":"<<actual_end
   <<",\"candidate_acc_scale\":"<<acc_scale<<",\"actual_acc_scale\":"<<actual_scale<<"}\n";
  f.close();if(!f)throw std::runtime_error("joint receipt write failure");std::filesystem::rename(tmp,target);
 }
};
}
