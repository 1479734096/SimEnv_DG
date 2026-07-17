#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <danger_search_localization/ground_plane_initializer.hpp>
#include <Eigen/Geometry>

namespace danger_search_localization {
namespace {

std::vector<Eigen::Vector3d> SyntheticGroundPoints(
    const Eigen::Vector3d &normal_base, double distance_from_base) {
  std::vector<Eigen::Vector3d> points;
  for (int ix = -20; ix <= 40; ++ix) {
    for (int iy = -20; iy <= 20; ++iy) {
      const double x = 0.1 * ix;
      const double y = 0.1 * iy;
      const double z =
          (-distance_from_base - normal_base.x() * x - normal_base.y() * y) /
          normal_base.z();
      points.emplace_back(x, y, z);
    }
  }
  // 混入竖直墙面和离群点，确保初始化不把“点最多的任意平面”当作楼面。
  for (int iy = -20; iy <= 20; ++iy) {
    for (int iz = 0; iz <= 20; ++iz) {
      points.emplace_back(3.0, 0.1 * iy, 0.1 * iz - 0.5);
    }
  }
  return points;
}

TEST(GroundPlaneInitializer, RecoversTiltAndStandingHeightFromFloorReturns) {
  const Eigen::Matrix3d known_rotation =
      (Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(-0.31, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  const Eigen::Vector3d normal_base = known_rotation.transpose() * Eigen::Vector3d::UnitZ();
  const auto points = SyntheticGroundPoints(normal_base, 0.14);

  GroundPlaneOptions options;
  options.minimum_inliers = 300;
  options.minimum_inlier_ratio = 0.20;
  const auto estimate = EstimateGroundPlane(points, options);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_GT(estimate->normal_base.dot(normal_base), 0.999);
  EXPECT_NEAR(estimate->distance_from_base, 0.14, 0.01);
  EXPECT_GT(estimate->inlier_count, 1000U);
}

TEST(GroundPlaneInitializer, BuildsWorldPoseWithRequestedHeadingAndFloorAtZero) {
  const Eigen::Vector3d normal_base(0.28, 0.19, 0.941);
  const Eigen::Vector3d unit_normal = normal_base.normalized();
  const double distance_from_base = 0.13;
  const double requested_yaw = 1.2;
  const Eigen::Isometry3d world_base = BuildWorldBaseFromGroundPlane(
      unit_normal, distance_from_base, 2.0, -3.0, requested_yaw);

  EXPECT_NEAR(world_base.translation().x(), 2.0, 1e-9);
  EXPECT_NEAR(world_base.translation().y(), -3.0, 1e-9);
  EXPECT_NEAR(world_base.translation().z(), distance_from_base, 1e-9);
  EXPECT_NEAR((world_base.linear() * unit_normal).x(), 0.0, 1e-9);
  EXPECT_NEAR((world_base.linear() * unit_normal).y(), 0.0, 1e-9);
  EXPECT_NEAR((world_base.linear() * unit_normal).z(), 1.0, 1e-9);
  const Eigen::Vector3d world_forward = world_base.linear() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(std::atan2(world_forward.y(), world_forward.x()), requested_yaw, 1e-9);
}

TEST(GroundPlaneInitializer, RejectsVerticalOrInsufficientEvidence) {
  std::vector<Eigen::Vector3d> wall;
  for (int iy = -10; iy <= 10; ++iy) {
    for (int iz = 0; iz <= 20; ++iz) {
      wall.emplace_back(1.0, 0.1 * iy, 0.1 * iz - 0.5);
    }
  }
  GroundPlaneOptions options;
  options.minimum_inliers = 100;
  EXPECT_FALSE(EstimateGroundPlane(wall, options).has_value());
  EXPECT_FALSE(EstimateGroundPlane({Eigen::Vector3d::Zero()}, options).has_value());
}

}  // namespace
}  // namespace danger_search_localization
