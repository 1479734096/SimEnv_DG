#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>

namespace danger_search_localization {

enum class AdaptiveTranslationSource {
  kNone = 0,
  kRgbd,
  kLivox,
};

inline const char *AdaptiveTranslationSourceName(
    AdaptiveTranslationSource source) {
  switch (source) {
    case AdaptiveTranslationSource::kNone:
      return "none";
    case AdaptiveTranslationSource::kRgbd:
      return "rgbd";
    case AdaptiveTranslationSource::kLivox:
      return "livox";
  }
  return "unknown";
}

enum class AdaptiveTranslationReason {
  kNoValidProposal = 0,
  kAgreementRgbd,
  kDivergenceLivox,
  kRgbdOnly,
  kLivoxOnly,
};

inline const char *AdaptiveTranslationReasonName(
    AdaptiveTranslationReason reason) {
  switch (reason) {
    case AdaptiveTranslationReason::kNoValidProposal:
      return "no_valid_proposal";
    case AdaptiveTranslationReason::kAgreementRgbd:
      return "agreement_rgbd";
    case AdaptiveTranslationReason::kDivergenceLivox:
      return "divergence_livox";
    case AdaptiveTranslationReason::kRgbdOnly:
      return "rgbd_only";
    case AdaptiveTranslationReason::kLivoxOnly:
      return "livox_only";
  }
  return "unknown";
}

struct TranslationProposal {
  bool accepted = false;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double age_seconds = std::numeric_limits<double>::infinity();
  std::uint64_t map_generation = 0;
};

struct AdaptiveTranslationConfig {
  double maximum_proposal_age = 0.35;
  double maximum_source_divergence = 0.05;
  double maximum_map_commit_difference = 0.05;
};

struct AdaptiveTranslationDecision {
  bool accepted = false;
  AdaptiveTranslationSource source = AdaptiveTranslationSource::kNone;
  AdaptiveTranslationReason reason =
      AdaptiveTranslationReason::kNoValidProposal;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double source_divergence = std::numeric_limits<double>::quiet_NaN();
  bool rgbd_valid = false;
  bool livox_valid = false;
  bool commit_livox_map = false;
};

inline bool IsTranslationProposalValid(
    const TranslationProposal &proposal, std::uint64_t current_map_generation,
    double maximum_age) {
  return proposal.accepted && proposal.translation.allFinite() &&
         std::isfinite(proposal.age_seconds) && proposal.age_seconds >= 0.0 &&
         proposal.age_seconds <= maximum_age &&
         proposal.map_generation == current_map_generation;
}

struct LivoxOnlyGuardDecision {
  bool allow_proposal = false;
  bool abstained = false;
  double planar_delta = std::numeric_limits<double>::quiet_NaN();
  std::uint32_t consecutive_vetoes = 0;
};

inline LivoxOnlyGuardDecision EvaluateLivoxOnlyGuard(
    const TranslationProposal &proposal,
    const Eigen::Vector3d &reference_translation, double maximum_delta,
    std::uint32_t previous_consecutive_vetoes,
    std::uint32_t maximum_consecutive_vetoes) {
  if (!std::isfinite(maximum_delta) || maximum_delta <= 0.0 ||
      maximum_consecutive_vetoes == 0) {
    throw std::invalid_argument("Livox-only 守卫参数无效");
  }

  LivoxOnlyGuardDecision decision;
  if (!proposal.accepted || !proposal.translation.allFinite() ||
      !reference_translation.allFinite()) {
    return decision;
  }

  decision.planar_delta =
      (proposal.translation - reference_translation).head<2>().norm();
  if (decision.planar_delta <= maximum_delta) {
    decision.allow_proposal = true;
    return decision;
  }

  if (previous_consecutive_vetoes >= maximum_consecutive_vetoes - 1) {
    decision.allow_proposal = true;
    decision.abstained = true;
    return decision;
  }

  decision.consecutive_vetoes = previous_consecutive_vetoes + 1;
  return decision;
}

inline AdaptiveTranslationDecision DecideAdaptiveTranslation(
    const TranslationProposal &rgbd, const TranslationProposal &livox,
    std::uint64_t current_map_generation,
    const AdaptiveTranslationConfig &config = AdaptiveTranslationConfig()) {
  if (!std::isfinite(config.maximum_proposal_age) ||
      config.maximum_proposal_age <= 0.0 ||
      !std::isfinite(config.maximum_source_divergence) ||
      config.maximum_source_divergence <= 0.0 ||
      !std::isfinite(config.maximum_map_commit_difference) ||
      config.maximum_map_commit_difference <= 0.0) {
    throw std::invalid_argument("自适应平移选择参数必须为正有限数");
  }

  AdaptiveTranslationDecision decision;
  decision.rgbd_valid = IsTranslationProposalValid(
      rgbd, current_map_generation, config.maximum_proposal_age);
  decision.livox_valid = IsTranslationProposalValid(
      livox, current_map_generation, config.maximum_proposal_age);

  if (decision.rgbd_valid && decision.livox_valid) {
    decision.source_divergence =
        (rgbd.translation - livox.translation).head<2>().norm();
    if (decision.source_divergence <= config.maximum_source_divergence) {
      decision.source = AdaptiveTranslationSource::kRgbd;
      decision.reason = AdaptiveTranslationReason::kAgreementRgbd;
      decision.translation = rgbd.translation;
    } else {
      decision.source = AdaptiveTranslationSource::kLivox;
      decision.reason = AdaptiveTranslationReason::kDivergenceLivox;
      decision.translation = livox.translation;
    }
  } else if (decision.rgbd_valid) {
    decision.source = AdaptiveTranslationSource::kRgbd;
    decision.reason = AdaptiveTranslationReason::kRgbdOnly;
    decision.translation = rgbd.translation;
  } else if (decision.livox_valid) {
    decision.source = AdaptiveTranslationSource::kLivox;
    decision.reason = AdaptiveTranslationReason::kLivoxOnly;
    decision.translation = livox.translation;
  } else {
    return decision;
  }

  decision.accepted = true;
  decision.commit_livox_map =
      decision.livox_valid &&
      (decision.translation - livox.translation).head<2>().norm() <=
          config.maximum_map_commit_difference;
  return decision;
}

}  // namespace danger_search_localization
