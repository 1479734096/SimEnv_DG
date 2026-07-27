#include <gtest/gtest.h>

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
