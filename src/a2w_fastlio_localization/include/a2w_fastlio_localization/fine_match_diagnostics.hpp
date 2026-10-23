#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "a2w_fastlio_localization/localization_manager.hpp"

namespace a2w_fastlio_localization
{

// Metadata only: never retain source clouds or rebuild a Local Map for logging.
struct FineSelectionDiagnostic
{
  std::int64_t stamp_ns{-1};
  std::int64_t source_age_at_start_ns{0};
  a2w_fastlio_common::Pose3d initial_guess{};
  std::vector<std::pair<std::uint64_t, double>> keyframes;
};

FineSelectionDiagnostic captureFineSelection(
  std::int64_t stamp_ns, std::int64_t source_age_at_start_ns,
  const SelectionResult & selection,
  const std::vector<a2w_fastlio_common::KeyFrame> & keyframes);

struct FineMatchRecord
{
  LocalizationOutput output;
  FineSelectionDiagnostic selection;
  a2w_fastlio_common::Pose3d odom_pose{};
  a2w_fastlio_common::CloudConstPtr source_cloud{};
  std::int64_t source_age_ns{0};
  std::int64_t correction_accepted_age_ns{0};
  std::int64_t prediction_correction_accepted_age_ns{-1};
  double match_latency_ms{0.0};
};

// Used only by the single normal-matching worker, including its result callback.
class FineMatchDiagnostics
{
public:
  std::optional<std::string> observe(const FineMatchRecord & record);

private:
  std::optional<FineMatchRecord> previous_success_;
  std::optional<std::int64_t> last_observed_stamp_;
};

}  // namespace a2w_fastlio_localization
