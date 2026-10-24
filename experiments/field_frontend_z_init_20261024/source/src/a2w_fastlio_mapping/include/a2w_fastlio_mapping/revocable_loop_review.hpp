#pragma once
#include <optional>
#include <stdexcept>
#include <set>
#include "a2w_fastlio_mapping/causal_geometry_guard.hpp"
namespace a2w_fastlio_mapping {
class LoopReviewPersistenceError : public std::runtime_error {public: using std::runtime_error::runtime_error;};
struct ReviewHypothesis {
 unsigned mask{0}; std::size_t loop_count{0}; bool independently_observable{false};
 CausalGeometryGuardResult geometry;
};
// No arrival order or endpoint ID participates in the ranking. Ties defer.
std::optional<std::size_t> selectReviewHypothesis(const std::vector<ReviewHypothesis>& hypotheses);
struct ReviewEdge {LoopConstraint constraint;std::set<std::uint64_t> registration_ids;};
bool appendReviewEdgeBounded(std::vector<ReviewEdge>& pool, const ReviewEdge& proposed,
 const std::vector<ReviewEdge>& active, std::uint64_t current_id, std::ostream& ledger);
GraphUpdate commitReviewLoopSet(PoseGraphOptimizer& graph, std::vector<ReviewEdge>& active,
 const std::vector<ReviewEdge>& selected, std::uint64_t current_id, std::ostream& ledger);
struct RevocableReviewResult {
 bool selected{false};std::string reason{"unknown"};std::vector<ReviewEdge> selected_edges;
 OptimizedPoseSnapshot snapshot;std::size_t hypotheses_tested{0};
};
RevocableReviewResult reviewLoopSets(const PoseGraphOptimizer& graph,
 const std::map<std::uint64_t,a2w_fastlio_common::KeyFrame>& frames,
 std::uint64_t current_id,const std::vector<ReviewEdge>& pool,
 const std::vector<ReviewEdge>& active,std::ostream& ledger);
}
