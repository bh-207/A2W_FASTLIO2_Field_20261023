#include "a2w_fastlio_mapping/causal_geometry_guard.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <Eigen/Eigenvalues>
namespace a2w_fastlio_mapping
{
namespace
{
using a2w_fastlio_common::KeyFrame;
using a2w_fastlio_common::Pose3d;
using Vec=Eigen::Vector3d;
constexpr double kDegrees=180.0/3.14159265358979323846;
bool finitePose(const Pose3d & p) {return p.translation.allFinite()&&p.rotation.coeffs().allFinite()&&p.rotation.norm()>1e-12;}
double percentile(std::vector<double> x,double q)
{
  if(x.empty())return 0.;
  std::sort(x.begin(),x.end());const double at=q*(x.size()-1);
  const auto lo=static_cast<std::size_t>(std::floor(at)),hi=static_cast<std::size_t>(std::ceil(at));
  return x[lo]+(x[hi]-x[lo])*(at-lo);
}
double normalAngle(const Vec & a,const Vec & b) {return kDegrees*std::atan2(a.cross(b).norm(),std::abs(a.dot(b)));}
struct Patch
{
  Vec center{Vec::Zero()},normal{Vec::UnitZ()},t1{Vec::UnitX()},t2{Vec::UnitY()};
  std::array<double,4> bounds{{0.,0.,0.,0.}};
  std::vector<std::uint32_t> ids;
};
struct Plane {Vec center,normal;};
Plane world(const Patch & p,const Pose3d & pose)
{
  const auto q=pose.rotation.normalized();return {q*p.center+pose.translation,q*p.normal};
}
bool supported(const Patch & p,const Pose3d & pose,const Vec & reference)
{
  const Vec d=pose.rotation.normalized().conjugate()*(reference-pose.translation)-p.center;
  const double a=d.dot(p.t1),b=d.dot(p.t2),slack=.05;
  return a>=p.bounds[0]-slack&&a<=p.bounds[1]+slack&&b>=p.bounds[2]-slack&&b<=p.bounds[3]+slack;
}
double gap(const Plane & a,const Plane & b,const Vec & x,const Vec & axis)
{
  const double da=a.normal.dot(axis),db=b.normal.dot(axis);
  if(std::abs(da)<.5||std::abs(db)<.5)return std::numeric_limits<double>::quiet_NaN();
  return -b.normal.dot(x-b.center)/db+a.normal.dot(x-a.center)/da;
}
std::vector<Patch> patches(const KeyFrame & frame,const CausalGeometryGuardConfig & c,CausalGeometryGuardResult & r)
{
  using Cell=std::tuple<long long,long long,long long>;
  std::map<Cell,std::vector<std::uint32_t>> cells;std::vector<Patch> out;
  if(!frame.body_cloud)return out;
  const auto total=frame.body_cloud->size(),budget=std::min(total,c.maximum_points_per_frame);
  for(std::size_t s=0;s<budget;++s){
    const auto i=s*total/budget;const auto & v=frame.body_cloud->points[i];const Vec p{v.x,v.y,v.z};
    if(!p.allFinite()){++r.invalid_points;continue;}if(p.norm()>c.maximum_body_range_m)continue;++r.sampled_points;
    cells[{static_cast<long long>(std::floor(p.x()/c.body_voxel_m)),static_cast<long long>(std::floor(p.y()/c.body_voxel_m)),static_cast<long long>(std::floor(p.z()/c.body_voxel_m))}].push_back(static_cast<std::uint32_t>(i));
  }
  for(const auto & entry:cells){
    const auto & ids=entry.second;if(ids.size()<c.minimum_patch_points){++r.rejected_plane_patches;continue;}
    Patch p;p.ids=ids;
    for(auto i:ids){const auto & v=frame.body_cloud->points[i];p.center+=Vec{v.x,v.y,v.z};}p.center/=ids.size();
    Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
    for(auto i:ids){const auto & v=frame.body_cloud->points[i];const Vec d=Vec{v.x,v.y,v.z}-p.center;cov+=d*d.transpose();}cov/=ids.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig{cov};
    if(eig.info()!=Eigen::Success||eig.eigenvalues()[1]<c.minimum_tangent_eigenvalue_m2||eig.eigenvalues()[0]/std::max(eig.eigenvalues()[1],1e-12)>c.maximum_eigenvalue_ratio){++r.rejected_plane_patches;continue;}
    p.normal=eig.eigenvectors().col(0);p.t1=eig.eigenvectors().col(1);p.t2=eig.eigenvectors().col(2);
    Eigen::Index largest;p.normal.cwiseAbs().maxCoeff(&largest);if(p.normal[largest]<0.)p.normal=-p.normal;
    p.bounds={{std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}};
    std::vector<double> residual;
    for(auto i:ids){const auto & v=frame.body_cloud->points[i];const Vec d=Vec{v.x,v.y,v.z}-p.center;residual.push_back(std::abs(d.dot(p.normal)));const double a=d.dot(p.t1),b=d.dot(p.t2);p.bounds[0]=std::min(p.bounds[0],a);p.bounds[1]=std::max(p.bounds[1],a);p.bounds[2]=std::min(p.bounds[2],b);p.bounds[3]=std::max(p.bounds[3],b);}
    if(percentile(residual,.95)>c.maximum_plane_residual_p95_m){++r.rejected_plane_patches;continue;}
    out.push_back(std::move(p));
  }
  if(out.size()>c.maximum_patches_per_frame){
    r.patch_limit_reached=true;std::vector<Patch> selected;
    for(std::size_t i=0;i<c.maximum_patches_per_frame;++i){const auto ix=c.maximum_patches_per_frame==1U?out.size()/2U:i*(out.size()-1U)/(c.maximum_patches_per_frame-1U);selected.push_back(std::move(out[ix]));}
    return selected;
  }
  return out;
}
CausalGeometrySeamStats summarize(const std::vector<CausalGeometryPatchRecord> & records,std::size_t begin,std::size_t end,bool trial)
{
  CausalGeometrySeamStats out;out.count=end-begin;std::vector<double> heights,normals;
  for(std::size_t i=begin;i<end;++i){heights.push_back(std::abs(trial?records[i].trial_signed_height_gap_m:records[i].before_signed_height_gap_m));normals.push_back(trial?records[i].trial_normal_angle_deg:records[i].before_normal_angle_deg);}
  out.height_absolute_median_m=percentile(heights,.5);out.height_absolute_p95_m=percentile(heights,.95);out.normal_median_deg=percentile(normals,.5);out.normal_p95_deg=percentile(normals,.95);return out;
}
std::string regression(const CausalGeometrySeamStats & b,const CausalGeometrySeamStats & t,const CausalGeometryGuardConfig & c)
{
  if(t.height_absolute_median_m>b.height_absolute_median_m+c.maximum_median_retreat_m)return "cross_time_plane_median_retreat";
  if(t.height_absolute_p95_m>b.height_absolute_p95_m+c.maximum_p95_retreat_m)return "cross_time_plane_p95_retreat";
  if(t.normal_p95_deg>b.normal_p95_deg+c.maximum_normal_p95_retreat_deg)return "cross_time_plane_normal_retreat";
  return {};
}
}
CausalGeometryGuard::CausalGeometryGuard(CausalGeometryGuardConfig config):config_(config)
{
  const std::array<double,13> values{{config.maximum_body_range_m,config.body_voxel_m,config.minimum_time_separation_s,config.maximum_raw_observer_separation_m,config.maximum_patch_centroid_separation_m,config.minimum_tangent_eigenvalue_m2,config.maximum_eigenvalue_ratio,config.maximum_plane_residual_p95_m,config.maximum_raw_height_gap_m,config.maximum_raw_normal_angle_deg,config.maximum_median_retreat_m,config.maximum_p95_retreat_m,config.maximum_normal_p95_retreat_deg}};
  if(config.maximum_frames<2U||config.maximum_points_per_frame<config.minimum_patch_points||config.minimum_patch_points<3U||config.maximum_patches_per_frame==0U||config.maximum_frame_pairs==0U||config.maximum_pair_enumerations==0U||config.maximum_patch_records==0U||config.minimum_supported_frame_pairs==0U||config.minimum_supported_patches==0U||std::any_of(values.begin(),values.end(),[](double v){return !std::isfinite(v)||v<=0.;}))throw std::invalid_argument{"invalid causal geometry guard configuration"};
}
CausalGeometryGuardResult CausalGeometryGuard::assess(
  const std::map<std::uint64_t,KeyFrame> & frames,const OptimizedPoseSnapshot & before,const OptimizedPoseSnapshot & trial,
  std::uint64_t current_id,std::uint64_t candidate_id,const std::set<std::uint64_t> & excluded) const
{
  CausalGeometryGuardResult out;out.current_id=current_id;out.candidate_id=candidate_id;out.excluded_registration_ids=excluded;
  std::map<std::uint64_t,Pose3d> bp,tp;
  for(const auto & p:before.poses)if(finitePose(p.pose))bp.emplace(p.id,p.pose);
  for(const auto & p:trial.poses)if(finitePose(p.pose))tp.emplace(p.id,p.pose);
  std::vector<const KeyFrame *> eligible;
  for(const auto & entry:frames){const auto id=entry.first;const auto & f=entry.second;if(id>=current_id)continue;if(excluded.count(id)){++out.excluded_frames;continue;}if(bp.count(id)&&tp.count(id)&&finitePose(f.odom_pose)&&f.body_cloud&&!f.body_cloud->empty())eligible.push_back(&f);}
  out.eligible_frames=eligible.size();out.frame_limit_reached=eligible.size()>config_.maximum_frames;
  std::vector<std::pair<std::size_t,std::size_t>> all_pairs;
  for(std::size_t a=0;a<eligible.size();++a){for(std::size_t b=a+1;b<eligible.size();++b){
    if(out.enumerated_pair_checks>=config_.maximum_pair_enumerations){out.enumeration_limit_reached=true;out.reason="pair_enumeration_budget_exhausted_coverage_unknown";return out;}
    ++out.enumerated_pair_checks;
    const auto & fa=*eligible[a];const auto & fb=*eligible[b];
    const long double dt=std::abs(static_cast<long double>(fa.stamp_ns)-static_cast<long double>(fb.stamp_ns))/1e9L;
    if(dt>=config_.minimum_time_separation_s&&(fa.odom_pose.translation-fb.odom_pose.translation).norm()<=config_.maximum_raw_observer_separation_m)all_pairs.emplace_back(a,b);
  }}
  out.eligible_frame_pairs=all_pairs.size();
  const auto frame_pair_budget=eligible.size()<=config_.maximum_frames?config_.maximum_frame_pairs:config_.maximum_frames/2U;
  const auto pair_budget=std::min({all_pairs.size(),config_.maximum_frame_pairs,frame_pair_budget,config_.maximum_patch_records/3U});
  if(pair_budget<config_.minimum_supported_frame_pairs||config_.maximum_patch_records<config_.minimum_supported_patches){out.reason="independent_anchor_budget_insufficient";return out;}
  out.unsampled_eligible_frame_pairs=all_pairs.size()-pair_budget;out.pair_limit_reached=out.unsampled_eligible_frame_pairs>0U;out.per_pair_patch_budget=config_.maximum_patch_records/pair_budget;
  std::vector<std::pair<std::size_t,std::size_t>> selected_pairs;std::set<std::size_t> selected_frames;
  for(std::size_t i=0;i<pair_budget;++i){const auto ix=pair_budget==1U?all_pairs.size()/2U:i*(all_pairs.size()-1U)/(pair_budget-1U);selected_pairs.push_back(all_pairs[ix]);selected_frames.insert(all_pairs[ix].first);selected_frames.insert(all_pairs[ix].second);}
  std::map<std::size_t,std::vector<Patch>> ps;bool first_frame=true;
  for(auto ix:selected_frames){const auto & f=*eligible[ix];ps[ix]=patches(f,config_,out);++out.sampled_frames;
    if(first_frame){out.selected_min_stamp_ns=f.stamp_ns;out.selected_max_stamp_ns=f.stamp_ns;out.selected_raw_position_min=f.odom_pose.translation;out.selected_raw_position_max=f.odom_pose.translation;first_frame=false;}
    else{out.selected_min_stamp_ns=std::min(out.selected_min_stamp_ns,f.stamp_ns);out.selected_max_stamp_ns=std::max(out.selected_max_stamp_ns,f.stamp_ns);out.selected_raw_position_min=out.selected_raw_position_min.cwiseMin(f.odom_pose.translation);out.selected_raw_position_max=out.selected_raw_position_max.cwiseMax(f.odom_pose.translation);}
  }
  bool singular=false;
  for(const auto & selected:selected_pairs){
    const auto a=selected.first,b=selected.second;const auto & fa=*eligible[a];const auto & fb=*eligible[b];++out.tested_frame_pairs;
    const auto start=out.records.size();std::set<std::size_t> used;std::vector<CausalGeometryPatchRecord> qualified;
    for(const auto & pa:ps[a]){
      const auto ra=world(pa,fa.odom_pose);std::size_t best=ps[b].size();double nearest=config_.maximum_patch_centroid_separation_m;
      for(std::size_t j=0;j<ps[b].size();++j){
        if(used.count(j)){continue;}
        const auto rb=world(ps[b][j],fb.odom_pose);const double d=(ra.center-rb.center).norm();
        if(d>nearest||normalAngle(ra.normal,rb.normal)>config_.maximum_raw_normal_angle_deg)continue;
        const Vec reference=(ra.center+rb.center)*.5;const double raw_gap=gap(ra,rb,reference,ra.normal);
        if(!std::isfinite(raw_gap)||std::abs(raw_gap)>config_.maximum_raw_height_gap_m||!supported(pa,fa.odom_pose,reference)||!supported(ps[b][j],fb.odom_pose,reference))continue;
        best=j;nearest=d;
      }
      if(best==ps[b].size()){continue;}
      used.insert(best);const auto & pb=ps[b][best];const auto rb=world(pb,fb.odom_pose);const Vec ref=(ra.center+rb.center)*.5;
      const auto ba=world(pa,bp.at(fa.id)),bb=world(pb,bp.at(fb.id)),ta=world(pa,tp.at(fa.id)),tb=world(pb,tp.at(fb.id));
      CausalGeometryPatchRecord r;r.first_id=fa.id;r.second_id=fb.id;r.first_point_indices=pa.ids;r.second_point_indices=pb.ids;r.raw_reference=ref;r.raw_axis=ra.normal;
      r.raw_signed_height_gap_m=gap(ra,rb,ref,ra.normal);r.raw_normal_angle_deg=normalAngle(ra.normal,rb.normal);r.before_signed_height_gap_m=gap(ba,bb,ref,ra.normal);r.trial_signed_height_gap_m=gap(ta,tb,ref,ra.normal);r.before_normal_angle_deg=normalAngle(ba.normal,bb.normal);r.trial_normal_angle_deg=normalAngle(ta.normal,tb.normal);
      if(!std::isfinite(r.before_signed_height_gap_m)||!std::isfinite(r.trial_signed_height_gap_m)){singular=true;continue;}
      qualified.push_back(std::move(r));
    }
    const auto take=std::min(qualified.size(),out.per_pair_patch_budget);if(take<qualified.size())out.patch_limit_reached=true;
    for(std::size_t i=0;i<take;++i){const auto ix=take==1U?qualified.size()/2U:i*(qualified.size()-1U)/(take-1U);out.records.push_back(std::move(qualified[ix]));}
    CausalGeometryPairRecord pair;pair.first_id=fa.id;pair.second_id=fb.id;pair.matched_patch_count=take;pair.before=summarize(out.records,start,out.records.size(),false);pair.trial=summarize(out.records,start,out.records.size(),true);
    if(take>=3U){++out.raw_supported_frame_pairs;pair.reason=regression(pair.before,pair.trial,config_);if(pair.reason.empty())pair.reason="supported_geometric_proxy_stable";}else pair.reason="insufficient_raw_supported_patches";
    out.pairs.push_back(std::move(pair));
  }
  out.before=summarize(out.records,0U,out.records.size(),false);out.trial=summarize(out.records,0U,out.records.size(),true);
  if(singular){out.verdict=CausalGeometryGuardVerdict::REJECT;out.reason="trial_plane_singular_relative_to_raw_axis";return out;}
  if(out.raw_supported_frame_pairs<config_.minimum_supported_frame_pairs||out.records.size()<config_.minimum_supported_patches)return out;
  for(const auto & pair:out.pairs)if(pair.matched_patch_count>=3U&&pair.reason!="supported_geometric_proxy_stable"){out.verdict=CausalGeometryGuardVerdict::REJECT;out.reason=pair.reason;return out;}
  out.reason=regression(out.before,out.trial,config_);if(!out.reason.empty()){out.verdict=CausalGeometryGuardVerdict::REJECT;return out;}
  out.verdict=CausalGeometryGuardVerdict::SUPPORTED_PASS;out.reason="uniform_raw_supported_geometric_proxies_stable_physical_identity_unknown";return out;
}
const char * causalGeometryGuardVerdictName(CausalGeometryGuardVerdict v) noexcept
{
  switch(v){case CausalGeometryGuardVerdict::SUPPORTED_PASS:return "SUPPORTED_PASS";case CausalGeometryGuardVerdict::REJECT:return "REJECT";default:return "UNKNOWN";}
}
void writeCausalGeometryGuardLedger(std::ostream & o,const CausalGeometryGuardResult & r)
{
  auto stats=[&](const CausalGeometrySeamStats & s){o<<"{\"count\":"<<s.count<<",\"height_absolute_median_m\":"<<s.height_absolute_median_m<<",\"height_absolute_p95_m\":"<<s.height_absolute_p95_m<<",\"normal_median_deg\":"<<s.normal_median_deg<<",\"normal_p95_deg\":"<<s.normal_p95_deg<<'}';};
  auto vec=[&](const Vec & v){o<<'['<<v.x()<<','<<v.y()<<','<<v.z()<<']';};
  auto ids=[&](const auto & vs){o<<'[';bool first=true;for(auto v:vs){if(!first)o<<',';first=false;o<<v;}o<<']';};
  o<<std::setprecision(17)<<"{\"current_id\":"<<r.current_id<<",\"candidate_id\":"<<r.candidate_id<<",\"verdict\":\""<<causalGeometryGuardVerdictName(r.verdict)<<"\",\"reason\":\""<<r.reason<<"\",\"eligible_frames\":"<<r.eligible_frames<<",\"sampled_frames\":"<<r.sampled_frames<<",\"excluded_frames\":"<<r.excluded_frames<<",\"tested_frame_pairs\":"<<r.tested_frame_pairs<<",\"raw_supported_frame_pairs\":"<<r.raw_supported_frame_pairs<<",\"sampled_points\":"<<r.sampled_points<<",\"invalid_points\":"<<r.invalid_points<<",\"rejected_plane_patches\":"<<r.rejected_plane_patches<<",\"frame_limit_reached\":"<<(r.frame_limit_reached?"true":"false")<<",\"pair_limit_reached\":"<<(r.pair_limit_reached?"true":"false")<<",\"patch_limit_reached\":"<<(r.patch_limit_reached?"true":"false")<<",\"physical_surface_identity\":\"unknown\",\"excluded_registration_ids\":";
  ids(r.excluded_registration_ids);o<<",\"sampling_policy\":\"uniform_pairs_uniform_patches_v2\",\"enumerated_pair_checks\":"<<r.enumerated_pair_checks<<",\"eligible_frame_pairs\":"<<r.eligible_frame_pairs<<",\"unsampled_eligible_frame_pairs\":"<<r.unsampled_eligible_frame_pairs<<",\"per_pair_patch_budget\":"<<r.per_pair_patch_budget<<",\"enumeration_limit_reached\":"<<(r.enumeration_limit_reached?"true":"false")<<",\"selected_min_stamp_ns\":"<<r.selected_min_stamp_ns<<",\"selected_max_stamp_ns\":"<<r.selected_max_stamp_ns<<",\"selected_raw_position_min\":";vec(r.selected_raw_position_min);o<<",\"selected_raw_position_max\":";vec(r.selected_raw_position_max);o<<",\"before\":";stats(r.before);o<<",\"trial\":";stats(r.trial);o<<",\"pairs\":[";
  bool first=true;for(const auto & p:r.pairs){if(!first)o<<',';first=false;o<<"{\"first_id\":"<<p.first_id<<",\"second_id\":"<<p.second_id<<",\"matched_patch_count\":"<<p.matched_patch_count<<",\"reason\":\""<<p.reason<<"\",\"before\":";stats(p.before);o<<",\"trial\":";stats(p.trial);o<<'}';}
  o<<"],\"records\":[";first=true;
  for(const auto & p:r.records){if(!first)o<<',';first=false;o<<"{\"first_id\":"<<p.first_id<<",\"second_id\":"<<p.second_id<<",\"first_point_indices\":";ids(p.first_point_indices);o<<",\"second_point_indices\":";ids(p.second_point_indices);o<<",\"raw_reference\":";vec(p.raw_reference);o<<",\"raw_axis\":";vec(p.raw_axis);o<<",\"raw_signed_height_gap_m\":"<<p.raw_signed_height_gap_m<<",\"raw_normal_angle_deg\":"<<p.raw_normal_angle_deg<<",\"before_signed_height_gap_m\":"<<p.before_signed_height_gap_m<<",\"trial_signed_height_gap_m\":"<<p.trial_signed_height_gap_m<<",\"before_normal_angle_deg\":"<<p.before_normal_angle_deg<<",\"trial_normal_angle_deg\":"<<p.trial_normal_angle_deg<<'}';}
  o<<"]}\n";
}
}
