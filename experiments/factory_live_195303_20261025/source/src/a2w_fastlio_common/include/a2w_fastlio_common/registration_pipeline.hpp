#pragma once

#include <memory>
#include <optional>

#include "a2w_fastlio_common/match_validator.hpp"

namespace a2w_fastlio_common
{

struct RegistrationPipelineConfig
{
  double evidence_distance_m{0.5};
};

struct PipelineResult
{
  bool success{false};
  RegistrationResult coarse{};
  RegistrationResult fine{};
  ValidationResult validation{};
  Pose3d final_transform{};
  double coarse_evidence_ms{0.0};
  double fine_evidence_ms{0.0};
  double validation_ms{0.0};
  double total_ms{0.0};
};

class RegistrationPipeline
{
public:
  RegistrationPipeline(
    std::shared_ptr<const CoarseRegistration> coarse,
    std::shared_ptr<const FineRegistration> fine,
    MatchValidator validator,
    RegistrationPipelineConfig config);

  PipelineResult run(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & yaw_hint,
    const ValidationContext & context = {}) const;

  PipelineResult runFineOnly(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const Pose3d & initial_guess,
    const ValidationContext & context = {}) const;

  ValidationResult validateCandidateContext(const ValidationContext & context) const;

  // Revalidate an already scored registration expressed in the caller's pose frame.
  // Does not rerun registration/evidence or change Localization's existing run methods.
  ValidationResult validateResult(
    const RegistrationResult & result, const ValidationContext & context) const
  {return validator_.validate(result, context);}

  const MatchValidationConfig & validationConfig() const noexcept {return validator_.config();}

private:
  RegistrationResult withGeometricEvidence(
    RegistrationResult result, const CloudConstPtr & source,
    const CloudConstPtr & target) const;

  std::shared_ptr<const CoarseRegistration> coarse_;
  std::shared_ptr<const FineRegistration> fine_;
  MatchValidator validator_;
  RegistrationPipelineConfig config_;
};

}  // namespace a2w_fastlio_common
