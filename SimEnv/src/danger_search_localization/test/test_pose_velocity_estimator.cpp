#include <gtest/gtest.h>

#include <danger_search_localization/pose_velocity_estimator.hpp>

namespace danger_search_localization {
namespace {

TEST(PoseVelocityEstimator, SuppressesBoundedStaticRegistrationJitter) {
  PoseVelocityConfig config;
  config.window_seconds = 0.5;
  config.minimum_interval = 0.4;
  config.linear_deadband = 0.05;
  PoseVelocityEstimator estimator(config);

  PoseVelocityEstimate result;
  for (int index = 0; index <= 10; ++index) {
    const double jitter = index % 2 == 0 ? -0.01 : 0.01;
    result = estimator.Update(0.1 * index,
                              Eigen::Vector3d(jitter, 2.0, 0.3), 0.0);
  }
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.linear_world.x(), 0.0);
  EXPECT_DOUBLE_EQ(result.linear_world.y(), 0.0);
  EXPECT_DOUBLE_EQ(result.linear_world.z(), 0.0);
}

TEST(PoseVelocityEstimator, PreservesSustainedMotionAndWrapsYaw) {
  PoseVelocityEstimator estimator;
  estimator.Update(1.0, Eigen::Vector3d(0.0, 0.0, 0.3), 3.10);
  const auto result = estimator.Update(
      2.0, Eigen::Vector3d(0.20, -0.10, 0.3), -3.10);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.linear_world.x(), 0.20, 1e-9);
  EXPECT_NEAR(result.linear_world.y(), -0.10, 1e-9);
  EXPECT_NEAR(result.yaw_rate, 0.083185307, 1e-6);
}

TEST(PoseVelocityEstimator, NeedsEnoughTimeAndResetsOnNonMonotonicStamp) {
  PoseVelocityEstimator estimator;
  EXPECT_FALSE(estimator.Update(
      1.0, Eigen::Vector3d::Zero(), 0.0).valid);
  EXPECT_FALSE(estimator.Update(
      1.1, Eigen::Vector3d::Zero(), 0.0).valid);
  EXPECT_FALSE(estimator.Update(
      0.5, Eigen::Vector3d::Zero(), 0.0).valid);
}

TEST(PoseVelocityEstimator, RejectsInvalidConfiguration) {
  PoseVelocityConfig config;
  config.minimum_interval = config.window_seconds + 0.1;
  EXPECT_THROW(PoseVelocityEstimator estimator(config), std::invalid_argument);
}

}  // namespace
}  // namespace danger_search_localization
