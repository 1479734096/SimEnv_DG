#include <gtest/gtest.h>

#include <danger_search_localization/stance_initialization_gate.hpp>

namespace danger_search_localization {
namespace {

StanceObservation Standing(double height = 0.31) {
  return StanceObservation{height, 0.99, 0.99, 0.02};
}

TEST(StanceInitializationGate, RejectsProneHeightEvenWhenDirectionsAreVertical) {
  StanceInitializationGate gate;
  for (int frame = 0; frame < 20; ++frame) {
    EXPECT_FALSE(gate.Update(StanceObservation{0.077, 0.997, 0.999, 0.01}));
  }
  EXPECT_EQ(gate.consecutive_frames(), 0U);
}

TEST(StanceInitializationGate, RequiresAContinuousStableWindow) {
  StanceInitializationConfig config;
  config.required_consecutive_frames = 4;
  config.maximum_height_span = 0.03;
  StanceInitializationGate gate(config);

  EXPECT_FALSE(gate.Update(Standing(0.30)));
  EXPECT_FALSE(gate.Update(Standing(0.31)));
  EXPECT_FALSE(gate.Update(Standing(0.32)));
  EXPECT_TRUE(gate.Update(Standing(0.31)));

  EXPECT_FALSE(gate.Update(Standing(0.40)));
  EXPECT_FALSE(gate.Update(Standing(0.31)));
}

TEST(StanceInitializationGate, MotionOrTiltClearsAccumulatedEvidence) {
  StanceInitializationConfig config;
  config.required_consecutive_frames = 3;
  StanceInitializationGate gate(config);

  EXPECT_FALSE(gate.Update(Standing()));
  EXPECT_FALSE(gate.Update(Standing()));
  EXPECT_FALSE(gate.Update(StanceObservation{0.31, 0.99, 0.99, 0.8}));
  EXPECT_EQ(gate.consecutive_frames(), 0U);
  EXPECT_FALSE(gate.Update(Standing()));
  EXPECT_FALSE(gate.Update(StanceObservation{0.31, 0.70, 0.99, 0.01}));
  EXPECT_EQ(gate.consecutive_frames(), 0U);
}

TEST(StanceInitializationGate, RejectsInvalidConfiguration) {
  StanceInitializationConfig config;
  config.minimum_base_height = config.maximum_base_height;
  EXPECT_THROW(StanceInitializationGate gate(config), std::invalid_argument);
}

}  // namespace
}  // namespace danger_search_localization
