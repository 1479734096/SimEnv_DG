#include <danger_search_localization/planar_gicp_odometry.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <vector>

namespace {

std::vector<Eigen::Vector3d> MakeAsymmetricRoomPoints() {
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < 18; ++x) {
    for (int z = 0; z < 9; ++z) {
      points.emplace_back(0.08 * x, -0.7, 0.07 * z);
    }
  }
  for (int y = 0; y < 15; ++y) {
    for (int z = 0; z < 8; ++z) {
      points.emplace_back(1.35, -0.65 + 0.08 * y, 0.09 * z);
    }
  }
  for (int x = 0; x < 12; ++x) {
    points.emplace_back(0.11 * x, 0.43 + 0.013 * x, 0.31);
  }
  return points;
}

TEST(PlanarGicpOdometryTest, RecoversBaseTranslationFromConsecutiveFrames) {
  danger_search_localization::PlanarGicpConfig config;
  config.voxel_size = 0.025;
  config.maximum_correspondence_distance = 0.30;
  config.maximum_frame_translation = 0.20;
  config.maximum_fitness = 0.02;
  config.minimum_points = 60;
  danger_search_localization::PlanarGicpOdometry odometry(config);

  const auto first = MakeAsymmetricRoomPoints();
  auto second = first;
  const Eigen::Vector3d base_motion(0.08, -0.035, 0.0);
  for (auto &point : second) {
    point -= base_motion;
  }

  const auto initial = odometry.RegisterFrame(first);
  ASSERT_TRUE(initial.accepted);
  EXPECT_FALSE(initial.has_delta);
  const auto moved = odometry.RegisterFrame(second);
  ASSERT_TRUE(moved.accepted);
  ASSERT_TRUE(moved.has_delta);
  EXPECT_NEAR(moved.delta.x(), base_motion.x(), 0.015);
  EXPECT_NEAR(moved.delta.y(), base_motion.y(), 0.015);
}

TEST(PlanarGicpOdometryTest, RejectsInvalidConfiguration) {
  danger_search_localization::PlanarGicpConfig config;
  config.voxel_size = 0.0;
  EXPECT_THROW(
      danger_search_localization::PlanarGicpOdometry odometry(config),
      std::invalid_argument);
}

TEST(PlanarGicpOdometryTest, UsesMotionPriorAlongWeaklyObservedWallDirection) {
  danger_search_localization::PlanarGicpConfig config;
  config.voxel_size = 0.04;
  config.maximum_correspondence_distance = 0.30;
  config.maximum_frame_translation = 0.20;
  config.maximum_fitness = 0.03;
  config.minimum_points = 50;
  danger_search_localization::PlanarGicpOdometry odometry(config);

  std::vector<Eigen::Vector3d> first;
  for (int x = 0; x < 80; ++x) {
    for (int z = 0; z < 5; ++z) {
      first.emplace_back(0.05 * x, 0.0, 0.12 * z);
    }
  }
  auto second = first;
  for (auto &point : second) {
    point.x() -= 0.09;
  }

  ASSERT_TRUE(odometry.RegisterFrame(first).accepted);
  const auto moved = odometry.RegisterFrame(second, Eigen::Vector2d(0.09, 0.0));
  ASSERT_TRUE(moved.accepted);
  EXPECT_NEAR(moved.delta.x(), 0.09, 0.025);
  EXPECT_NEAR(moved.delta.y(), 0.0, 0.015);
}

}  // namespace
