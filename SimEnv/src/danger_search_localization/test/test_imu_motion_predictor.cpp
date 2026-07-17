#include <gtest/gtest.h>

#include <Eigen/Core>

#include <danger_search_localization/imu_motion_predictor.hpp>

namespace danger_search_localization {
namespace {

void CalibrateAtRest(ImuMotionPredictor &predictor) {
  for (int index = 0; index <= 50; ++index) {
    predictor.AddSample(
        1.0 + 0.01 * index, Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.03, -0.02, 9.81), Eigen::Vector3d::Zero());
  }
}

TEST(ImuMotionPredictor, RemovesStationaryBiasBeforeIntegratingMotion) {
  ImuMotionPredictor predictor;
  CalibrateAtRest(predictor);
  ASSERT_TRUE(predictor.calibrated());
  EXPECT_NEAR(predictor.acceleration_bias().x(), 0.03, 1e-9);
  EXPECT_NEAR(predictor.acceleration_bias().y(), -0.02, 1e-9);

  for (int index = 1; index <= 100; ++index) {
    predictor.AddSample(
        1.5 + 0.01 * index, Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(1.03, -0.02, 9.81),
        Eigen::Vector3d(0.0, 0.0, 0.2));
  }

  EXPECT_NEAR(predictor.displacement().x(), 0.5, 0.015);
  EXPECT_NEAR(predictor.displacement().y(), 0.0, 1e-9);
  EXPECT_NEAR(predictor.velocity().x(), 1.0, 0.015);
}

TEST(ImuMotionPredictor, ClearsResidualVelocityAfterStableWindow) {
  ImuMotionPredictor predictor;
  CalibrateAtRest(predictor);
  for (int index = 1; index <= 20; ++index) {
    predictor.AddSample(
        1.5 + 0.01 * index, Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.53, -0.02, 9.81),
        Eigen::Vector3d(0.0, 0.0, 0.2));
  }
  ASSERT_GT(predictor.velocity().norm(), 0.05);
  for (int index = 1; index <= 30; ++index) {
    predictor.AddSample(
        1.7 + 0.01 * index, Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.03, -0.02, 9.81), Eigen::Vector3d::Zero());
  }
  EXPECT_NEAR(predictor.velocity().norm(), 0.0, 1e-12);
}

}  // namespace
}  // namespace danger_search_localization
