#include <gtest/gtest.h>

#include <danger_search_localization/velocity_transform.hpp>
#include <Eigen/Geometry>

namespace danger_search_localization {
namespace {

TEST(VelocityTransform, IdentityRotationPreservesVelocity) {
  const Eigen::Vector3d velocity_world(1.0, 2.0, 3.0);
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d velocity_base = WorldVelocityToBase(velocity_world, rotation);
  EXPECT_NEAR(velocity_base.x(), 1.0, 1e-9);
  EXPECT_NEAR(velocity_base.y(), 2.0, 1e-9);
  EXPECT_NEAR(velocity_base.z(), 3.0, 1e-9);
}

TEST(VelocityTransform, Yaw90RotationMapsWorldForwardToBaseRight) {
  // 世界系下沿 +X 前进，base 相对于 world 逆时针转 90°，
  // 等价于 base 的 +X 指向 world 的 +Y，因此 world +X 在 base 中为 -Y。
  const Eigen::Vector3d velocity_world(1.0, 0.0, 0.0);
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d velocity_base = WorldVelocityToBase(velocity_world, rotation);
  EXPECT_NEAR(velocity_base.x(), 0.0, 1e-9);
  EXPECT_NEAR(velocity_base.y(), -1.0, 1e-9);
  EXPECT_NEAR(velocity_base.z(), 0.0, 1e-9);
}

TEST(VelocityTransform, YawNegative90RotationMapsWorldLeftToBaseBackward) {
  // base 相对于 world 顺时针转 90°，base 的 +X 指向 world 的 -Y，
  // 因此 world +Y 在 base 中为 -X。
  const Eigen::Vector3d velocity_world(0.0, 1.0, 0.0);
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d velocity_base = WorldVelocityToBase(velocity_world, rotation);
  EXPECT_NEAR(velocity_base.x(), -1.0, 1e-9);
  EXPECT_NEAR(velocity_base.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity_base.z(), 0.0, 1e-9);
}

TEST(VelocityTransform, PitchRotationLeavesZComponentInBase) {
  const Eigen::Vector3d velocity_world(0.0, 0.0, 1.0);
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d velocity_base = WorldVelocityToBase(velocity_world, rotation);
  EXPECT_NEAR(velocity_base.x(), -std::sin(M_PI / 4.0), 1e-9);
  EXPECT_NEAR(velocity_base.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity_base.z(), std::cos(M_PI / 4.0), 1e-9);
}

}  // namespace
}  // namespace danger_search_localization
