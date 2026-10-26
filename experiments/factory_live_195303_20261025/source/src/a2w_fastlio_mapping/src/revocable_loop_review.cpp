#include "a2w_fastlio_mapping/revocable_loop_review.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <ostream>
#include <tuple>
namespace a2w_fastlio_mapping {
namespace {
bool same(const ReviewEdge&a,const ReviewEdge&b) {
 return a.constraint.from_id==b.constraint.from_id&&a.constraint.to_id==b.constraint.to_id;
}
double score(const CausalGeometryGuardResult& g) {
 double sum=0;std::size_t n=0;
 // Equal frame-pair weight: a stationary scan cluster cannot win by point count.
 for(const auto&p:g.pairs)if(p.matched_patch_count>=3){sum+=p.trial.height_absolute_p95_m;++n;}
 return n?sum/n:std::numeric_limits<double>::infinity();
}
std::vector<LoopConstraint> constraints(const std::vector<ReviewEdge>& edges) {
 std::vector<LoopConstraint> out;for(const auto&e:edges)out.push_back(e.constraint);return out;
}
bool responds(const CausalGeometryGuardResult& full,const CausalGeometryGuardResult& minus) {
 if(full.records.size()!=minus.records.size())return false;
 std::size_t count=0;std::set<std::pair<std::uint64_t,std::uint64_t>> pairs;
 for(std::size_t i=0;i<full.records.size();++i){const auto&a=full.records[i];const auto&b=minus.records[i];
  if(a.first_id!=b.first_id||a.second_id!=b.second_id||a.first_point_indices!=b.first_point_indices||a.second_point_indices!=b.second_point_indices)return false;
  // Numerical zero only; no physical acceptance tolerance is introduced.
  if(std::abs(a.trial_signed_height_gap_m-b.trial_signed_height_gap_m)>1e-9||std::abs(a.trial_normal_angle_deg-b.trial_normal_angle_deg)>1e-9){++count;pairs.emplace(a.first_id,a.second_id);}
 }
 return count>=CausalGeometryGuardConfig{}.minimum_supported_patches&&pairs.size()>=CausalGeometryGuardConfig{}.minimum_supported_frame_pairs;
}
void writeEdges(std::ostream&o,const std::vector<ReviewEdge>& edges) {
 o<<'[';bool first=true;for(const auto&e:edges){if(!first)o<<',';first=false;const auto&c=e.constraint;
 o<<"{\"from_id\":"<<c.from_id<<",\"to_id\":"<<c.to_id<<",\"measurement\":["<<c.relative_pose.translation.x()<<','<<c.relative_pose.translation.y()<<','<<c.relative_pose.translation.z()<<','<<c.relative_pose.rotation.x()<<','<<c.relative_pose.rotation.y()<<','<<c.relative_pose.rotation.z()<<','<<c.relative_pose.rotation.w()<<"]}";}
 o<<']';
}
}
bool appendReviewEdgeBounded(std::vector<ReviewEdge>& pool,const ReviewEdge& proposed,
 const std::vector<ReviewEdge>& active,std::uint64_t current_id,std::ostream& ledger) {
 if(std::any_of(pool.begin(),pool.end(),[&](const auto&e){return same(e,proposed);}))return true;
 auto staged=pool;
 if(staged.size()>=4U){
  const auto oldest=std::find_if(staged.begin(),staged.end(),[&](const auto&e){return std::none_of(active.begin(),active.end(),[&](const auto&a){return same(e,a);});});
  if(oldest==staged.end()){
   ledger<<"{\"type\":\"review_pending_admission\",\"current_id\":"<<current_id<<",\"admitted\":false,\"reason\":\"active_set_capacity_unknown\"}\n";ledger.flush();return false;
  }
  ledger<<std::setprecision(17)<<"{\"type\":\"review_pending_evicted\",\"current_id\":"<<current_id<<",\"reason\":\"capacity_fifo_oldest_deferred_not_physical_rejection\",\"evicted\":";
  writeEdges(ledger,{*oldest});ledger<<",\"incoming\":";writeEdges(ledger,{proposed});ledger<<"}\n";ledger.flush();
  if(!ledger)return false;
  staged.erase(oldest);
 }
 staged.push_back(proposed);pool=std::move(staged);return true;
}
GraphUpdate commitReviewLoopSet(PoseGraphOptimizer& graph,std::vector<ReviewEdge>& active,
 const std::vector<ReviewEdge>& selected,std::uint64_t current_id,std::ostream& ledger) {
 auto next=selected;std::vector<ReviewEdge> withdrawn;
 for(const auto&e:active)if(std::none_of(next.begin(),next.end(),[&](const auto&n){return same(e,n);}))withdrawn.push_back(e);
 const auto cs=constraints(next);
 const auto committed=graph.replaceLoopSet(cs,[&](const OptimizedPoseSnapshot& snapshot){
  ledger<<std::setprecision(17)<<"{\"type\":\"review_commit\",\"current_id\":"<<current_id<<",\"revision\":"<<snapshot.revision<<",\"withdrawn_edges\":";
  writeEdges(ledger,withdrawn);ledger<<",\"active_edges\":";writeEdges(ledger,next);ledger<<"}\n";ledger.flush();
  if(!ledger)throw LoopReviewPersistenceError{"review_commit_flush_failed"};
 });
 if(!committed.accepted&&(committed.reason.rfind("replacement_factor_ledger_failed:",0)==0||committed.reason.rfind("replacement_commit_barrier_failed:",0)==0))throw LoopReviewPersistenceError{committed.reason};
 if(committed.accepted)active.swap(next); // noexcept after the durable commit barrier
 return committed;
}
std::optional<std::size_t> selectReviewHypothesis(const std::vector<ReviewHypothesis>& hs) {
 std::optional<std::size_t> best;bool tied=false;
 for(std::size_t i=0;i<hs.size();++i){const auto&h=hs[i];
  if(!h.loop_count||!h.independently_observable||!h.geometry.accepted()||!std::isfinite(score(h.geometry)))continue;
  if(!best||h.loop_count>hs[*best].loop_count||(h.loop_count==hs[*best].loop_count&&score(h.geometry)<score(hs[*best].geometry)-1e-12)){best=i;tied=false;}
  else if(h.loop_count==hs[*best].loop_count&&std::abs(score(h.geometry)-score(hs[*best].geometry))<=1e-12)tied=true;
 }
 return tied?std::nullopt:best;
}
RevocableReviewResult reviewLoopSets(const PoseGraphOptimizer& graph,
 const std::map<std::uint64_t,a2w_fastlio_common::KeyFrame>& frames,
 std::uint64_t current_id,const std::vector<ReviewEdge>& pool,
 const std::vector<ReviewEdge>& active,std::ostream& ledger) {
 RevocableReviewResult result;ledger<<std::setprecision(17);
 auto unknown=[&](const char*reason){result.reason=reason;ledger<<"{\"type\":\"review_decision\",\"current_id\":"<<current_id<<",\"selected\":false,\"reason\":\""<<reason<<"\"}\n";ledger.flush();return result;};
 if(pool.empty()||pool.size()>4)return unknown("review_pool_budget_exceeded_or_empty");
 for(const auto&e:pool)if(e.constraint.from_id>current_id||e.registration_ids.empty())return unknown("review_noncausal_or_missing_provenance");
 std::set<std::uint64_t> excluded;
 for(const auto&e:pool)excluded.insert(e.registration_ids.begin(),e.registration_ids.end());
 const auto baseline=graph.previewLoopSet({});
 if(!baseline.update.accepted)return unknown("review_baseline_trial_failed");
 std::vector<ReviewHypothesis> hs;std::vector<OptimizedPoseSnapshot> snapshots;
 CausalGeometryGuard guard;
 for(unsigned mask=0;mask<(1U<<pool.size());++mask){
  std::vector<ReviewEdge> edges;for(unsigned k=0;k<pool.size();++k)if(mask&(1U<<k))edges.push_back(pool[k]);
  ReviewHypothesis h;h.mask=mask;h.loop_count=edges.size();
  const auto trial=mask?graph.previewLoopSet(constraints(edges)):baseline;
  if(trial.update.accepted)h.geometry=guard.assess(frames,baseline.snapshot,trial.snapshot,current_id,0,excluded);
  else h.geometry.reason="review_transaction_trial_failed";
  ledger<<"{\"type\":\"review_hypothesis\",\"current_id\":"<<current_id<<",\"mask\":"<<mask<<",\"score_pair_mean_p95_m\":";
  const auto cost=score(h.geometry);if(std::isfinite(cost))ledger<<cost;else ledger<<"null";
  ledger<<",\"edges\":";writeEdges(ledger,edges);ledger<<"}\n";
  writeCausalGeometryGuardLedger(ledger,h.geometry);hs.push_back(std::move(h));snapshots.push_back(trial.snapshot);++result.hypotheses_tested;
 }
 for(auto&h:hs){
  std::size_t withdrawals=0;
  for(const auto&a:active){bool retained=false;for(unsigned k=0;k<pool.size();++k)if((h.mask&(1U<<k))&&same(a,pool[k]))retained=true;if(!retained)++withdrawals;}
  if(withdrawals>1)continue;
  h.independently_observable=true;
  for(unsigned k=0;k<pool.size();++k)if(h.mask&(1U<<k))if(!responds(h.geometry,hs[h.mask&~(1U<<k)].geometry)){h.independently_observable=false;break;}
  ledger<<"{\"type\":\"review_observability\",\"current_id\":"<<current_id<<",\"mask\":"<<h.mask<<",\"observable\":"<<(h.independently_observable?"true":"false")<<",\"withdrawals\":"<<withdrawals<<"}\n";
 }
 const auto choice=selectReviewHypothesis(hs);
 if(choice){result.selected=true;result.reason="causal_proxy_supported_unique_set";result.snapshot=snapshots[*choice];
  for(unsigned k=0;k<pool.size();++k)if(hs[*choice].mask&(1U<<k))result.selected_edges.push_back(pool[k]);
 }else result.reason="review_unknown_unobservable_or_ambiguous";
 ledger<<"{\"type\":\"review_decision\",\"current_id\":"<<current_id<<",\"selected\":"<<(result.selected?"true":"false")<<",\"reason\":\""<<result.reason<<"\",\"physical_identity\":\"unknown\",\"previous_active\":";writeEdges(ledger,active);ledger<<",\"selected_edges\":";writeEdges(ledger,result.selected_edges);ledger<<"}\n";ledger.flush();
 if(!ledger){result.selected=false;result.reason="review_ledger_write_failed";result.selected_edges.clear();}
 return result;
}
}
