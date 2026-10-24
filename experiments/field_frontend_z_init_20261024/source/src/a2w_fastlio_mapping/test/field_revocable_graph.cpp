// Frozen measurements from already independent matcher runs; never rerun matcher.
#define main field_frozen_original_main
#include "field_frozen_graph.cpp"
#undef main
#include "a2w_fastlio_mapping/revocable_loop_review.hpp"
namespace revocable_frozen {
using namespace field_frozen;
bool sameSet(const std::vector<mapping::ReviewEdge>&a,const std::vector<mapping::ReviewEdge>&b){
 if(a.size()!=b.size())return false;for(const auto&e:a)if(std::none_of(b.begin(),b.end(),[&](const auto&f){return e.constraint.from_id==f.constraint.from_id&&e.constraint.to_id==f.constraint.to_id;}))return false;return true;
}
int run(int argc,char**argv){
 if(argc<4||argc>5)throw std::runtime_error("usage: field_revocable_graph BUNDLE MEASUREMENTS NEW_OUTPUT [--review]");
 const bool enabled=argc==5&&std::string(argv[4])=="--review";
 if(argc==5&&!enabled)throw std::runtime_error("unknown option");
 const auto bundle_path=fs::canonical(argv[1]),measurements=fs::canonical(argv[2]);
 const auto out=fs::weakly_canonical(argv[3]);if(fs::exists(out))throw std::runtime_error("new output required");
 if(out.string().find(bundle_path.string())==0)throw std::runtime_error("input output overlap");
 const auto bundle=a2w_fastlio_map::MapBundleReader{}.read(bundle_path);
 const auto config=ablation::readConfig(bundle_path/"config/mapping_effective.yaml");
 const auto exclusions=loadExclusions(measurements.parent_path()/"measurement_provenance.csv");
 std::vector<mapping::ReviewEdge> edges;
 for(const auto&r:ablation::readCsv(measurements)){
  const auto from=ablation::id(r.at("current_id")),to=ablation::id(r.at("candidate_id"));
  edges.push_back({{from,to,ablation::readPose(r,"relative_"),true},exclusions.at(from)});
 }
 fs::create_directories(out/"snapshots");
 fs::copy_file(measurements,out/"new_measurements.csv");
 fs::copy_file(measurements.parent_path()/"measurement_provenance.csv",out/"measurement_provenance.csv");
 std::ofstream ledger(out/"factor_ledger.csv"),review(out/"revocable_review.jsonl"),decisions(out/"decisions.csv");
 mapping::writeFactorLedgerCsvHeader(ledger);decisions<<"current_id,selected,changed,active_count,reason\n";
 mapping::PoseGraphOptimizer graph{config};graph.setFactorLedgerObserver([&](const auto&r){mapping::writeFactorLedgerCsv(ledger,r);if(!ledger)throw std::runtime_error("frozen_factor_ledger_write_failed");});
 std::map<std::uint64_t,common::KeyFrame> frames;std::vector<mapping::ReviewEdge> pool,active;
 for(const auto&frame:bundle.data.keyframes){
  if(frame.id!=frames.size())throw std::runtime_error("raw frame IDs not contiguous");
  frames.emplace(frame.id,frame);ablation::require(graph.addKeyFrame(frame.id,frame.odom_pose));
  bool replaced=false;
  for(const auto&edge:edges)if(edge.constraint.from_id==frame.id){
   ablation::save(out/"snapshots"/("before_"+std::to_string(frame.id)+".csv"),graph.optimizedPoses());
   if(enabled&&!mapping::appendReviewEdgeBounded(pool,edge,active,frame.id,review))throw std::runtime_error("review proposal cache admission unknown");
   if(enabled){const auto result=mapping::reviewLoopSets(graph,frames,frame.id,pool,active,review);
    if(result.selected&&!sameSet(result.selected_edges,active)){
     std::vector<mapping::LoopConstraint> cs;for(const auto&e:result.selected_edges)cs.push_back(e.constraint);
     const auto update=mapping::commitReviewLoopSet(graph,active,result.selected_edges,frame.id,review);ablation::require(update);replaced=true;
     const auto actual=graph.optimizedPoses();
     if(actual.poses.size()!=result.snapshot.poses.size())throw std::runtime_error("preview commit node count mismatch");
     for(std::size_t j=0;j<actual.poses.size();++j)if((actual.poses[j].pose.translation-result.snapshot.poses[j].pose.translation).norm()!=0.||(actual.poses[j].pose.rotation.coeffs()-result.snapshot.poses[j].pose.rotation.coeffs()).norm()!=0.)throw std::runtime_error("preview commit exact equality failed");

    }
    decisions<<frame.id<<','<<result.selected<<','<<replaced<<','<<active.size()<<','<<result.reason<<'\n';
   }else{ablation::require(graph.addLoopConstraint(edge.constraint));active.push_back(edge);}
  }
  if(!replaced)ablation::require(graph.update());
  for(const auto&edge:edges)if(edge.constraint.from_id==frame.id)ablation::save(out/"snapshots"/("after_"+std::to_string(frame.id)+".csv"),graph.optimizedPoses());
 }
 ablation::save(out/"before_save.csv",graph.optimizedPoses());ablation::require(graph.update(true));graph.emitFactorLedger();
 ledger.flush();review.flush();if(!ledger||!review)throw std::runtime_error("frozen_ledger_flush_failed");
 ablation::save(out/"optimized_poses.csv",graph.optimizedPoses());
 std::ofstream accepted(out/"active_constraints.csv");accepted<<"from_id,to_id,tx,ty,tz,qx,qy,qz,qw\n";
 for(const auto&e:active){accepted<<e.constraint.from_id<<','<<e.constraint.to_id;poseCells(accepted,e.constraint.relative_pose);accepted<<'\n';}
 std::ofstream meta(out/"metadata.txt");meta<<"revocable_review_enabled="<<enabled<<"\nnoise_unchanged=true\nmatcher_repeated=false\ncausal_independent_support_proxy_physical_identity=unknown\nsource_measurements="<<measurements.string()<<"\nraw_bundle="<<bundle_path.string()<<"\nstatus=complete\n";
 std::cout<<"complete active="<<active.size()<<" output="<<out<<std::endl;return 0;
}
}
int main(int argc,char**argv){try{return revocable_frozen::run(argc,argv);}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
