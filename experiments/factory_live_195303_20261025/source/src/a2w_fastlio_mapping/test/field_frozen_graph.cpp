// Offline attribution of immutable NEW measurements from three historically seen pairs.
// Does not reconstruct historical factors from optimized poses or rounded diagnostics.
#define LOOP_CONSTRAINT_ABLATION_NO_MAIN
#include "loop_constraint_ablation.cpp"
#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_mapping/causal_geometry_guard.hpp"
#include "a2w_fastlio_common/registration.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include <array>
#include <numeric>

namespace field_frozen {
namespace fs=std::filesystem;
namespace common=a2w_fastlio_common;
namespace mapping=a2w_fastlio_mapping;
using ablation::Pose3d;
using ablation::Measurement;
const std::array<std::pair<std::uint64_t,std::uint64_t>,3> pairs{{{31,0},{113,82},{788,90}}};

class Provider final:public common::KeyFrameProvider {
public:
 std::vector<common::KeyFrame> frames;
 std::optional<common::KeyFrame> get(std::uint64_t id)const override {
   if(id>=frames.size())return std::nullopt; return frames[id];
 }
 std::optional<std::pair<std::uint64_t,std::uint64_t>> bounds()const override {
   if(frames.empty())return std::nullopt;return std::pair<std::uint64_t,std::uint64_t>{0,frames.size()-1};
 }
};
Pose3d compose(const Pose3d&a,const Pose3d&b) {return mapping::fromGtsamPose(mapping::toGtsamPose(a).compose(mapping::toGtsamPose(b)));}
Pose3d between(const Pose3d&a,const Pose3d&b) {return mapping::fromGtsamPose(mapping::toGtsamPose(a).between(mapping::toGtsamPose(b)));}
void poseCells(std::ostream&out,const Pose3d&p) {
 out<<std::setprecision(17)<<','<<p.translation.x()<<','<<p.translation.y()<<','<<p.translation.z()<<','
 <<p.rotation.x()<<','<<p.rotation.y()<<','<<p.rotation.z()<<','<<p.rotation.w();
}
void poseHeader(std::ostream&out,const std::string&name) {
 for(const auto*axis:{"tx","ty","tz","qx","qy","qz","qw"})out<<','<<name<<'_'<<axis;
}
std::string ids(const std::vector<std::uint64_t>&values) {
 std::ostringstream out;for(std::size_t i=0;i<values.size();++i){if(i)out<<';';out<<values[i];}return out.str();
}
common::DefaultAlgorithmSuiteConfig suiteConfig(const YAML::Node& y) {
 common::DefaultAlgorithmSuiteConfig c;
#define GET(field,name) c.field=y[name].as<decltype(c.field)>()
 GET(place_recognition.rings,"scan_context.rings"); GET(place_recognition.sectors,"scan_context.sectors");
 GET(place_recognition.max_radius_m,"scan_context.max_radius_m"); GET(place_recognition.sensor_height_m,"scan_context.sensor_height_m");
 GET(coarse.normal_radius_m,"quatro.fpfh_normal_radius_m"); GET(coarse.feature_radius_m,"quatro.fpfh_radius_m");
 GET(coarse.noise_bound_m,"quatro.noise_bound_m"); GET(coarse.rotation_gnc_factor,"quatro.rotation_gnc_factor");
 GET(coarse.rotation_cost_threshold,"quatro.rotation_cost_threshold"); GET(coarse.rotation_max_iterations,"quatro.rotation_max_iterations");
 GET(coarse.estimate_scale,"quatro.estimate_scale"); GET(coarse.optimized_matching,"quatro.optimized_matching");
 GET(coarse.descriptor_distance_threshold,"quatro.descriptor_distance_threshold"); GET(coarse.maximum_correspondences,"quatro.maximum_correspondences");
 GET(coarse.minimum_points,"quatro.minimum_points");
 GET(fine.maximum_correspondence_distance_m,"nano_gicp.maximum_correspondence_distance_m"); GET(fine.thread_count,"nano_gicp.thread_count");
 GET(fine.correspondence_randomness,"nano_gicp.correspondence_randomness"); GET(fine.maximum_iterations,"nano_gicp.maximum_iterations");
 GET(fine.transformation_epsilon,"nano_gicp.transformation_epsilon"); GET(fine.rotation_epsilon,"nano_gicp.rotation_epsilon");
 GET(fine.regularization_method,"nano_gicp.regularization_method"); GET(fine.fitness_score_max_range_m,"nano_gicp.fitness_score_max_range_m");
 GET(fine.minimum_points,"nano_gicp.minimum_points");
 GET(validation.maximum_fitness,"loop_validation.maximum_fitness"); GET(validation.minimum_overlap,"loop_validation.minimum_overlap");
 GET(validation.minimum_correspondences,"loop_validation.minimum_correspondences"); GET(validation.maximum_translation_jump_m,"loop_validation.maximum_translation_jump_m");
 GET(validation.maximum_rotation_jump_rad,"loop_validation.maximum_rotation_jump_rad");
 GET(validation.minimum_candidate_distance_separation,"loop_validation.minimum_candidate_distance_separation");
 GET(evidence_distance_m,"pipeline.evidence_distance_m");
#undef GET
 return c;
}
std::vector<Measurement> generate(const a2w_fastlio_map::MapBundle&bundle,
 const mapping::PoseGraphConfig&config,const YAML::Node& yaml,const fs::path&out) {
 auto suite=common::createDefaultAlgorithmSuite(suiteConfig(yaml));
 common::LocalMapBuilder builder{{yaml["local_map.voxel_leaf_m"].as<double>(),yaml["local_map.max_points"].as<std::size_t>()}};
 const auto before=yaml["local_map.neighbor_keyframes_before"].as<std::size_t>();
 const auto after=yaml["local_map.neighbor_keyframes_after"].as<std::size_t>();
 const auto top_k=yaml["scan_context.top_k"].as<std::size_t>();
 const auto exclude=yaml["scan_context.exclude_recent"].as<std::size_t>();
 const auto max_distance=yaml["scan_context.max_distance"].as<double>();
 Provider provider; mapping::PoseGraphOptimizer graph{config};
 std::ofstream ledger(out/"generation_factor_ledger.csv");mapping::writeFactorLedgerCsvHeader(ledger);
 graph.setFactorLedgerObserver([&](const auto&r){mapping::writeFactorLedgerCsv(ledger,r);});
 std::ofstream loops(out/"new_measurements.csv"), details(out/"measurement_provenance.csv");
 loops<<"current_id,candidate_id,relative_tx,relative_ty,relative_tz,relative_qx,relative_qy,relative_qz,relative_qw\n";
 details<<"measurement_index,current_id,candidate_id,stamp_ns,submap_frame,source_ids,target_ids,descriptor_rank,descriptor_distance,yaw_hint_rad,fitness,overlap,correspondences,translation_jump_m,rotation_jump_rad,registration_success,pose_validation_success";
 for(const auto*name:{"predicted_map_body","historical_map_body","yaw_initial_D_map_map","coarse_D_map_map","fine_D_map_map","corrected_map_body","relative_current_history"})poseHeader(details,name);
 details<<'\n'; std::vector<Measurement> result;
 for(std::uint64_t i=0;i<bundle.data.keyframes.size();++i) {
  auto frame=bundle.data.keyframes[i];
  const auto descriptor=suite.place_recognition->describe(frame.body_cloud);
  ablation::require(graph.addKeyFrame(i,frame.odom_pose));
  frame.optimized_pose=i?compose(provider.frames.back().optimized_pose,between(provider.frames.back().odom_pose,frame.odom_pose)):frame.odom_pose;
  provider.frames.push_back(frame);
  if(result.size()<pairs.size()&&i==pairs[result.size()].first) {
   auto candidates=suite.descriptor_index->queryTopK(descriptor,top_k,{i-1,exclude});
   candidates.erase(std::remove_if(candidates.begin(),candidates.end(),[&](const auto&c){return !std::isfinite(c.distance)||c.distance<0||c.distance>max_distance;}),candidates.end());
   const auto found=std::find_if(candidates.begin(),candidates.end(),[&](const auto&c){return c.keyframe_id==pairs[result.size()].second;});
   if(found==candidates.end())throw std::runtime_error("historically seen pair absent from NEW descriptor candidate query");
   common::ValidationContext context;context.best_descriptor_distance=candidates.front().distance;
   if(candidates.size()>1)context.second_descriptor_distance=candidates[1].distance;
   auto valid=suite.registration->validateCandidateContext(context);
   if(!valid.accepted)throw std::runtime_error("NEW candidate context rejected:"+valid.reason);
   const auto source=builder.build(i,before,after,provider),target=builder.build(found->keyframe_id,before,after,provider);
   Pose3d yaw;yaw.rotation=Eigen::AngleAxisd(found->yaw_hint_rad,Eigen::Vector3d::UnitZ());
   const auto registration=suite.registration->run(source.cloud,target.cloud,yaw,context);
   if(!common::isUsableRegistration(registration.fine))throw std::runtime_error("NEW registration unusable:"+registration.validation.reason);
   const auto corrected=compose(registration.fine.transform,frame.optimized_pose);
   auto pose_result=registration.fine;pose_result.transform=corrected;
   auto pose_context=context;pose_context.predicted_transform=frame.optimized_pose;
   const auto pose_validation=suite.registration->validateResult(pose_result,pose_context);
   const auto historical=provider.frames[found->keyframe_id].optimized_pose;
   const auto relative=between(corrected,historical);
   details<<std::setprecision(17)<<result.size()<<','<<i<<','<<found->keyframe_id<<','<<frame.stamp_ns<<",map,"<<ids(source.included_ids)<<','<<ids(target.included_ids)<<','<<found->rank<<','<<found->distance<<','<<found->yaw_hint_rad<<','<<registration.fine.fitness<<','<<registration.fine.overlap<<','<<registration.fine.correspondence_count<<','<<(corrected.translation-frame.optimized_pose.translation).norm()<<','<<Eigen::AngleAxisd(frame.optimized_pose.rotation.inverse()*corrected.rotation).angle()<<','<<registration.success<<','<<pose_validation.accepted;
   for(const auto&p:{frame.optimized_pose,historical,yaw,registration.coarse.transform,registration.fine.transform,corrected,relative})poseCells(details,p);details<<'\n';details.flush();
   if(!registration.success||!pose_validation.accepted)throw std::runtime_error("NEW measurement rejected:"+registration.validation.reason+":"+pose_validation.reason);
   ablation::require(graph.addLoopConstraint({i,found->keyframe_id,relative,true}));
   result.push_back({i,found->keyframe_id,relative}); loops<<i<<','<<found->keyframe_id;poseCells(loops,relative);loops<<'\n';loops.flush();
   std::cout<<"new_measurement "<<i<<" -> "<<found->keyframe_id<<" fitness="<<registration.fine.fitness<<std::endl;
  }
  suite.descriptor_index->add(i,descriptor);ablation::require(graph.update());
  for(const auto&p:graph.optimizedPoses().poses)provider.frames[p.id].optimized_pose=p.pose;
 }
 ablation::require(graph.update(true));graph.emitFactorLedger();
 ablation::save(out/"generation_final.csv",graph.optimizedPoses());
 if(result.size()!=3)throw std::runtime_error("expected exactly three new measured factors");return result;
}
void compare(const mapping::OptimizedPoseSnapshot&actual,const std::vector<ablation::Row>&old,
 const fs::path&path) {
 if(actual.poses.size()!=old.size())throw std::runtime_error("comparison node count mismatch");
 std::ofstream out(path);out<<std::setprecision(17)<<"id,translation_difference_m,rotation_difference_rad\n";
 double sum=0,max=0,angle_max=0;for(std::size_t i=0;i<old.size();++i){
  if(ablation::id(old[i].at("id"))!=i)throw std::runtime_error("comparison IDs mismatch");
  const auto p=ablation::readPose(old[i]);const double d=(actual.poses[i].pose.translation-p.translation).norm();
  const double a=Eigen::AngleAxisd(p.rotation.inverse()*actual.poses[i].pose.rotation).angle();
  out<<i<<','<<d<<','<<a<<'\n';sum+=d*d;max=std::max(max,d);angle_max=std::max(angle_max,a);
 }std::cout<<path.filename()<<" rms_m="<<std::sqrt(sum/old.size())<<" max_m="<<max<<" max_rad="<<angle_max<<std::endl;
}
using Exclusions=std::map<std::uint64_t,std::set<std::uint64_t>>;
Exclusions loadExclusions(const fs::path& provenance) {
 Exclusions result;
 for(const auto&row:ablation::readCsv(provenance)) {
  const auto current=ablation::id(row.at("current_id"));
  for(const auto*name:{"source_ids","target_ids"}) {
   std::stringstream stream(row.at(name));std::string item;
   while(std::getline(stream,item,';')) {const auto id=ablation::id(item);
    if(id>current)throw std::runtime_error("future registration provenance ID");result[current].insert(id);}
  }
 }
 return result;
}
void replay(const std::vector<Pose3d>&raw,const std::vector<Measurement>&loops,
 const mapping::PoseGraphConfig&config,unsigned mask,const fs::path&root,
 const std::vector<ablation::Row>&old,const std::vector<common::KeyFrame>& bundle_frames,
 bool guard_enabled,const Exclusions&exclusions) {
 const auto out=root/("mask_"+std::to_string(mask));fs::create_directories(out/"snapshots");
 mapping::PoseGraphOptimizer graph{config};std::ofstream ledger(out/"factor_ledger.csv");mapping::writeFactorLedgerCsvHeader(ledger);
 graph.setFactorLedgerObserver([&](const auto&r){mapping::writeFactorLedgerCsv(ledger,r);});
 std::ofstream decisions(out/"accepted_factors.csv");decisions<<"measurement_index,current_id,candidate_id,accepted,reason\n";
 std::ofstream guard_ledger;if(guard_enabled)guard_ledger.open(out/"causal_guard.jsonl");
 mapping::CausalGeometryGuard guard;
 std::map<std::uint64_t,common::KeyFrame> historical;
 for(std::uint64_t i=0;i<raw.size();++i){
  auto before=graph.optimizedPoses();ablation::require(graph.addKeyFrame(i,raw[i]));
  if(guard_enabled){auto frame=bundle_frames.at(i);frame.optimized_pose=i?compose(before.poses.back().pose,between(raw[i-1],raw[i])):raw[i];historical.emplace(i,std::move(frame));}
  std::optional<mapping::OptimizedPoseSnapshot> accepted_trial;
  for(std::size_t k=0;k<loops.size();++k){if(loops[k].current!=i)continue;
   const auto predicted=i?compose(before.poses.back().pose,between(raw[i-1],raw[i])):raw[i];
   before.poses.push_back({i,predicted});ablation::save(out/"snapshots"/("before_"+std::to_string(i)+".csv"),before);
   if(mask&(1U<<k)) {
    const mapping::LoopConstraint constraint{i,loops[k].history,loops[k].pose,true};
    bool accepted=true;std::string reason="fixed_measurement_no_guard";
    if(guard_enabled){
     const auto trial=graph.previewLoopConstraint(constraint);
     if(!trial.update.accepted)throw std::runtime_error("guard trial failed:"+trial.update.reason);
     const auto assessment=guard.assess(historical,before,trial.snapshot,i,loops[k].history,exclusions.at(i));
     mapping::writeCausalGeometryGuardLedger(guard_ledger,assessment);guard_ledger.flush();
     accepted=assessment.accepted();reason=assessment.reason;
     if(accepted)accepted_trial=trial.snapshot;
    }
    decisions<<k<<','<<i<<','<<loops[k].history<<','<<accepted<<','<<reason<<'\n';decisions.flush();
    if(accepted)ablation::require(graph.addLoopConstraint(constraint));
   }
  }
  ablation::require(graph.update());
  if(guard_enabled){const auto actual=graph.optimizedPoses();
   if(accepted_trial){if(actual.revision!=accepted_trial->revision||actual.poses.size()!=accepted_trial->poses.size())throw std::runtime_error("live trial snapshot identity mismatch");
    for(std::size_t j=0;j<actual.poses.size();++j){if((actual.poses[j].pose.translation-accepted_trial->poses[j].pose.translation).norm()!=0.||
      (actual.poses[j].pose.rotation.coeffs()-accepted_trial->poses[j].pose.rotation.coeffs()).norm()!=0.)throw std::runtime_error("live commit differs from isolated trial");}}
   for(const auto&pose:actual.poses)historical.at(pose.id).optimized_pose=pose.pose;
  }
  for(const auto&loop:loops)if(loop.current==i)ablation::save(out/"snapshots"/("after_"+std::to_string(i)+".csv"),graph.optimizedPoses());
 }
 ablation::save(out/"before_save.csv",graph.optimizedPoses());ablation::require(graph.update(true));graph.emitFactorLedger();
 const auto final=graph.optimizedPoses();ablation::save(out/"optimized_poses.csv",final);
 compare(final,old,out/"difference_from_historical_saved.csv");
}
int run(int argc,char**argv){
 if(argc<3)throw std::runtime_error("usage: field_frozen_graph BUNDLE NEW_OUTPUT_DIR [--measurements IMMUTABLE_NEW_MEASUREMENT_CSV] [--guard]");
 bool guard_enabled=false;std::optional<fs::path> measured_input;
 for(int arg=3;arg<argc;++arg){const std::string option=argv[arg];
  if(option=="--guard"){guard_enabled=true;}
  else if(option=="--measurements"&&arg+1<argc){measured_input=fs::canonical(argv[++arg]);}
  else throw std::runtime_error("unknown or incomplete option:"+option);
 }
 const auto bundle_path=fs::canonical(argv[1]),output=fs::weakly_canonical(argv[2]);
 // Reuse refusal to overwrite/overlap input; output directory is reserved only after validation.
 if(fs::exists(output))throw std::runtime_error("choose a new output directory");
 auto b=bundle_path.begin(),o=output.begin();for(;b!=bundle_path.end()&&o!=output.end()&&*b==*o;++b,++o){}
 if(b==bundle_path.end()||o==output.end())throw std::runtime_error("input/output overlap");
 const auto bundle=a2w_fastlio_map::MapBundleReader{}.read(bundle_path);
 const auto config=ablation::readConfig(bundle_path/"config/mapping_effective.yaml");
 const auto yaml=YAML::LoadFile((bundle_path/"config/mapping_effective.yaml").string());
 std::vector<Pose3d>raw;for(const auto&f:bundle.data.keyframes){if(f.id!=raw.size())throw std::runtime_error("raw IDs not contiguous");raw.push_back(f.odom_pose);}
 const auto old=ablation::readCsv(bundle_path/"optimized_poses.csv");fs::create_directories(output);
 std::ofstream meta(output/"metadata.txt");meta<<"scope=NEW_FIXED_PAIR_CONSERVATIVE_MATCHER_FACTOR_ATTRIBUTION\nhistorical_exact_measurements_recovered=false\nhistorical_reproduction_gap=true\nindependent_ground_truth=false\nsaved_optimized_used_for_initialization=false\nmeasurement_direction=T_body_current_body_history\nregistration_direction=D_target_map_source_map\nresidual_convention=GTSAM_Pose3_right_local_rx_ry_rz_rad_tx_ty_tz_m\nmask_bits=bit0_31_to_0_bit1_113_to_82_bit2_788_to_90\nnoise_unchanged=true\nsource_time_injection=true\nfinal_save_empty_update=true\nfixed_pair_replay_not_full_online_candidate_selection=true\n";meta.flush();
 std::vector<Measurement>loops;
 if(measured_input){
  for(const auto&row:ablation::readCsv(*measured_input))loops.push_back({ablation::id(row.at("current_id")),ablation::id(row.at("candidate_id")),ablation::readPose(row,"relative_")});
  fs::copy_file(*measured_input,output/"new_measurements.csv");
 }else loops=generate(bundle,config,yaml,output);
 if(loops.size()!=pairs.size())throw std::runtime_error("expected exactly three immutable factors");
 for(std::size_t k=0;k<loops.size();++k)if(loops[k].current!=pairs[k].first||loops[k].history!=pairs[k].second)throw std::runtime_error("fixed pair endpoints/order mismatch");
 Exclusions exclusions;
 if(guard_enabled){
  const auto provenance=measured_input?measured_input->parent_path()/"measurement_provenance.csv":output/"measurement_provenance.csv";
  exclusions=loadExclusions(provenance);
  for(const auto&loop:loops)if(!exclusions.count(loop.current)||exclusions.at(loop.current).empty())throw std::runtime_error("guard requires original exact registration submap provenance");
  meta<<"guard_enabled=true\nguard_unknown_policy=reject_new_factor\nguard_measurements=fixed_new_measurements_from_unguarded_generation\nguard_exclusion_source="<<provenance.string()<<'\n';
 }else meta<<"guard_enabled=false\n";
 // Baseline all-three comparison precedes every removal attribution.
 replay(raw,loops,config,7,output,old,bundle.data.keyframes,guard_enabled,exclusions);
 for(unsigned mask=0;mask<7;++mask)replay(raw,loops,config,mask,output,old,bundle.data.keyframes,guard_enabled,exclusions);
 meta<<"status=complete\n";return 0;
}
}
int main(int argc,char**argv){try{return field_frozen::run(argc,argv);}catch(const std::exception&e){std::cerr<<"field_frozen_graph: "<<e.what()<<std::endl;return 1;}}
