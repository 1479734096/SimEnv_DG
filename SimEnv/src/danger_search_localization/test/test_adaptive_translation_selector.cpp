#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>

#include <danger_search_localization/adaptive_translation_selector.hpp>

namespace danger_search_localization {
namespace {

TranslationProposal Proposal(double x, double y, std::uint64_t generation,
                             double age = 0.0) {
  TranslationProposal proposal;
  proposal.accepted = true;
  proposal.translation = Eigen::Vector3d(x, y, 0.35);
  proposal.age_seconds = age;
  proposal.map_generation = generation;
  return proposal;
}

double KinematicTranslationLimit(double frame_interval, double maximum_speed,
                                 double margin, double absolute_cap) {
  return std::min(absolute_cap, maximum_speed * frame_interval + margin);
}

TEST(AdaptiveTranslationSelector, ChoosesRgbdWhenBothSourcesAgree) {
  const auto decision = DecideAdaptiveTranslation(
      Proposal(1.0, 2.0, 7), Proposal(1.03, 2.0, 7), 7);

  ASSERT_TRUE(decision.accepted);
  EXPECT_EQ(decision.source, AdaptiveTranslationSource::kRgbd);
  EXPECT_EQ(decision.reason, AdaptiveTranslationReason::kAgreementRgbd);
  EXPECT_TRUE(decision.commit_livox_map);
  EXPECT_NEAR(decision.source_divergence, 0.03, 1e-12);
}

TEST(AdaptiveTranslationSelector, ChoosesLivoxWhenSourcesDiverge) {
  const auto decision = DecideAdaptiveTranslation(
      Proposal(1.0, 2.0, 7), Proposal(1.08, 2.0, 7), 7);

  ASSERT_TRUE(decision.accepted);
  EXPECT_EQ(decision.source, AdaptiveTranslationSource::kLivox);
  EXPECT_EQ(decision.reason, AdaptiveTranslationReason::kDivergenceLivox);
  EXPECT_TRUE(decision.commit_livox_map);
}

TEST(AdaptiveTranslationSelector, UsesSingleValidSourceWithoutFalseStop) {
  TranslationProposal invalid;
  const auto rgbd_only = DecideAdaptiveTranslation(
      Proposal(1.0, 2.0, 7), invalid, 7);
  const auto livox_only = DecideAdaptiveTranslation(
      invalid, Proposal(1.0, 2.0, 7), 7);

  ASSERT_TRUE(rgbd_only.accepted);
  EXPECT_EQ(rgbd_only.source, AdaptiveTranslationSource::kRgbd);
  EXPECT_FALSE(rgbd_only.commit_livox_map);
  ASSERT_TRUE(livox_only.accepted);
  EXPECT_EQ(livox_only.source, AdaptiveTranslationSource::kLivox);
  EXPECT_TRUE(livox_only.commit_livox_map);
}

TEST(AdaptiveTranslationSelector,
     RecoversFromRepeatedLivoxOnlyVetoOnThirdCandidate) {
  const Eigen::Vector3d reference(1.0, 2.35, 0.244);
  const std::array<double, 3> frame_intervals = {0.20, 0.30, 0.40};
  const std::array<double, 3> candidate_deltas = {0.159, 0.229, 0.288};
  std::uint32_t consecutive_vetoes = 0;

  for (std::size_t index = 0; index < frame_intervals.size(); ++index) {
    const double upstream_limit = KinematicTranslationLimit(
        frame_intervals[index], 0.70, 0.02, 0.30);
    const double livox_only_limit = KinematicTranslationLimit(
        frame_intervals[index], 0.60, 0.02, 0.30);
    ASSERT_LE(candidate_deltas[index], upstream_limit);

    const auto decision = EvaluateLivoxOnlyGuard(
        Proposal(reference.x() + candidate_deltas[index], reference.y(), 7),
        reference, livox_only_limit, consecutive_vetoes, 3);
    consecutive_vetoes = decision.consecutive_vetoes;

    if (index < 2) {
      EXPECT_FALSE(decision.allow_proposal);
      EXPECT_FALSE(decision.abstained);
      EXPECT_EQ(consecutive_vetoes, index + 1);
    } else {
      EXPECT_TRUE(decision.allow_proposal);
      EXPECT_TRUE(decision.abstained);
      EXPECT_EQ(consecutive_vetoes, 0U);
    }
  }
}

TEST(AdaptiveTranslationSelector, AcceptsPlausibleLivoxOnlyMotionAcrossDt) {
  const Eigen::Vector3d reference(1.0, 2.35, 0.244);
  const std::array<double, 4> frame_intervals = {0.05, 0.10, 0.30, 0.58};

  for (const double frame_interval : frame_intervals) {
    const double upstream_limit =
        KinematicTranslationLimit(frame_interval, 0.70, 0.02, 0.30);
    const double livox_only_limit =
        KinematicTranslationLimit(frame_interval, 0.60, 0.02, 0.30);
    ASSERT_LE(livox_only_limit, upstream_limit);

    const auto decision = EvaluateLivoxOnlyGuard(
        Proposal(reference.x() + 0.50 * frame_interval, reference.y(), 7),
        reference, livox_only_limit, 0, 3);
    EXPECT_TRUE(decision.allow_proposal);
    EXPECT_FALSE(decision.abstained);
    EXPECT_EQ(decision.consecutive_vetoes, 0U);
  }
}

TEST(AdaptiveTranslationSelector, NeverOverridesUpstreamTranslationLimit) {
  const Eigen::Vector3d reference(1.0, 2.35, 0.244);
  auto rejected_jump = Proposal(1.30, reference.y(), 7);
  rejected_jump.accepted = false;
  const auto short_dt = EvaluateLivoxOnlyGuard(
      rejected_jump, reference,
      KinematicTranslationLimit(0.10, 0.60, 0.02, 0.30), 2, 3);
  EXPECT_FALSE(short_dt.allow_proposal);
  EXPECT_FALSE(short_dt.abstained);
  EXPECT_EQ(short_dt.consecutive_vetoes, 0U);

  auto rejected_long_gap = Proposal(1.50, reference.y(), 7);
  rejected_long_gap.accepted = false;
  EXPECT_NEAR(KinematicTranslationLimit(1.0, 0.70, 0.02, 0.30), 0.30,
              1e-12);
  const auto long_dt = EvaluateLivoxOnlyGuard(
      rejected_long_gap, reference,
      KinematicTranslationLimit(1.0, 0.60, 0.02, 0.30), 2, 3);
  EXPECT_FALSE(long_dt.allow_proposal);
  EXPECT_FALSE(long_dt.abstained);
  EXPECT_EQ(long_dt.consecutive_vetoes, 0U);
}

TEST(AdaptiveTranslationSelector, ResetsLivoxOnlyVetoesAfterNormalAcceptance) {
  const Eigen::Vector3d reference(1.0, 2.35, 0.244);
  const auto decision = EvaluateLivoxOnlyGuard(
      Proposal(1.05, reference.y(), 7), reference, 0.08, 2, 3);
  EXPECT_TRUE(decision.allow_proposal);
  EXPECT_FALSE(decision.abstained);
  EXPECT_EQ(decision.consecutive_vetoes, 0U);
}

TEST(AdaptiveTranslationSelector, RejectsInvalidLivoxOnlyGuardConfiguration) {
  const Eigen::Vector3d reference(1.0, 2.35, 0.244);
  const auto proposal = Proposal(1.05, reference.y(), 7);
  EXPECT_THROW(EvaluateLivoxOnlyGuard(proposal, reference, 0.0, 0, 3),
               std::invalid_argument);
  EXPECT_THROW(EvaluateLivoxOnlyGuard(proposal, reference, 0.08, 0, 0),
               std::invalid_argument);
}

TEST(AdaptiveTranslationSelector, RejectsStaleWrongGenerationAndNonfiniteProposals) {
  TranslationProposal stale = Proposal(1.0, 2.0, 7, 0.36);
  TranslationProposal wrong_generation = Proposal(1.0, 2.0, 6);
  TranslationProposal nonfinite = Proposal(1.0, 2.0, 7);
  nonfinite.translation.x() = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(DecideAdaptiveTranslation(stale, wrong_generation, 7).accepted);
  EXPECT_FALSE(DecideAdaptiveTranslation(nonfinite, wrong_generation, 7).accepted);
}

TEST(AdaptiveTranslationSelector, PreventsMapCommitWhenChosenPoseIsTooFarFromLivox) {
  AdaptiveTranslationConfig config;
  config.maximum_source_divergence = 0.10;
  config.maximum_map_commit_difference = 0.05;

  const auto decision = DecideAdaptiveTranslation(
      Proposal(1.0, 2.0, 7), Proposal(1.08, 2.0, 7), 7, config);

  ASSERT_TRUE(decision.accepted);
  EXPECT_EQ(decision.source, AdaptiveTranslationSource::kRgbd);
  EXPECT_FALSE(decision.commit_livox_map);
}

TEST(AdaptiveTranslationSelector, RejectsInvalidConfiguration) {
  AdaptiveTranslationConfig config;
  config.maximum_proposal_age = 0.0;
  EXPECT_THROW(
      DecideAdaptiveTranslation(Proposal(1.0, 2.0, 7),
                                Proposal(1.0, 2.0, 7), 7, config),
      std::invalid_argument);
}

}  // namespace
}  // namespace danger_search_localization
