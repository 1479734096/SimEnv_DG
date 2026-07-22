#include <gtest/gtest.h>

#include <Eigen/Core>
#include <random>
#include <vector>

#include <danger_search_localization/imu_constrained_odometry.hpp>

namespace danger_search_localization {
namespace {

TEST(ImuConstrainedOdometry, AnchorsRelativeImuRotationToPublicInitialPose) {
  const Eigen::Matrix3d initial_world_base =
      Eigen::AngleAxisd(1.20, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d initial_imu =
      Eigen::AngleAxisd(0.80, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d current_imu =
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const Eigen::Matrix3d anchored = AnchorImuOrientation(
      initial_world_base, initial_imu, current_imu);

  EXPECT_NEAR(std::atan2(anchored(1, 0), anchored(0, 0)), 0.75, 1e-9);
  EXPECT_NEAR(anchored.determinant(), 1.0, 1e-9);
}

std::vector<Eigen::Vector3d> MakeAsymmetricStructure() {
  std::mt19937 generator(42);
  std::uniform_real_distribution<double> x_distribution(-2.5, 4.0);
  std::uniform_real_distribution<double> y_distribution(-3.0, 3.5);
  std::uniform_real_distribution<double> z_distribution(0.25, 1.8);
  std::vector<Eigen::Vector3d> points;
  points.reserve(1600);
  for (int index = 0; index < 800; ++index) {
    points.emplace_back(4.0, y_distribution(generator), z_distribution(generator));
    points.emplace_back(x_distribution(generator), 3.5, z_distribution(generator));
  }
  return points;
}

std::vector<Eigen::Vector3d> SensorVectors(
    const std::vector<Eigen::Vector3d> &world_points,
    const Eigen::Vector3d &sensor_translation) {
  std::vector<Eigen::Vector3d> vectors;
  vectors.reserve(world_points.size());
  for (const auto &point : world_points) {
    vectors.push_back(point - sensor_translation);
  }
  return vectors;
}

TEST(ImuConstrainedOdometry, RecoversPlanarTranslationWithKnownOrientation) {
  ImuConstrainedOdometryConfig config;
  config.minimum_correspondences = 80;
  config.maximum_correspondence_distance = 0.45;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(0.2, -3.1, 0.35);
  const Eigen::Vector3d moved_translation(0.27, -3.13, 0.36);

  const auto initialized = odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation);
  const Eigen::Vector3d current_height_seed(
      initial_translation.x(), initial_translation.y(), moved_translation.z());
  const auto moved = odometry.RegisterFrame(
      SensorVectors(world_points, moved_translation), current_height_seed);

  ASSERT_TRUE(initialized.accepted);
  EXPECT_EQ(initialized.status, ImuConstrainedOdometryStatus::kInitialFrame);
  ASSERT_TRUE(moved.accepted);
  EXPECT_EQ(moved.status, ImuConstrainedOdometryStatus::kAccepted);
  EXPECT_FALSE(moved.correction_was_clamped);
  EXPECT_GT(moved.correspondence_count, config.minimum_correspondences);
  EXPECT_NEAR(moved.translation.x(), moved_translation.x(), 0.01);
  EXPECT_NEAR(moved.translation.y(), moved_translation.y(), 0.01);
  EXPECT_DOUBLE_EQ(moved.translation.z(), moved_translation.z());
}

TEST(ImuConstrainedOdometry, RejectsFrameWithoutEnoughStructure) {
  ImuConstrainedOdometryConfig config;
  config.minimum_correspondences = 80;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(0.2, -3.1, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);

  std::vector<Eigen::Vector3d> isolated_points{
      Eigen::Vector3d(30.0, 30.0, 0.5),
      Eigen::Vector3d(31.0, 30.0, 0.7),
  };
  const auto rejected = odometry.RegisterFrame(isolated_points, initial_translation);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.status,
            ImuConstrainedOdometryStatus::kInsufficientSource);
  EXPECT_STREQ(ImuConstrainedOdometryStatusName(rejected.status),
               "insufficient_source");
  EXPECT_EQ(rejected.correspondence_count, 0U);
  EXPECT_NEAR(rejected.translation.x(), initial_translation.x(), 1e-12);
  EXPECT_NEAR(rejected.translation.y(), initial_translation.y(), 1e-12);
}

TEST(ImuConstrainedOdometry, SearchesTheFullConfiguredCorrespondenceRadius) {
  ImuConstrainedOdometryConfig config;
  config.minimum_correspondences = 80;
  config.maximum_correspondence_distance = 0.55;
  config.maximum_correction_from_prior = 0.50;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(-0.20, -3.10, 0.35);
  const Eigen::Vector3d moved_translation(0.15, -3.10, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);

  const auto moved = odometry.RegisterFrame(
      SensorVectors(world_points, moved_translation), initial_translation);

  ASSERT_TRUE(moved.accepted);
  EXPECT_NEAR(moved.translation.x(), moved_translation.x(), 0.02);
  EXPECT_NEAR(moved.translation.y(), moved_translation.y(), 0.02);
}

TEST(ImuConstrainedOdometry, ReportsUnclampedCorrectionWhenSafetyLimitApplies) {
  ImuConstrainedOdometryConfig config;
  config.minimum_correspondences = 80;
  config.maximum_correspondence_distance = 0.55;
  config.maximum_correction_from_prior = 0.005;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(-0.20, -3.10, 0.35);
  const Eigen::Vector3d moved_translation(0.15, -3.10, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);

  const auto moved = odometry.RegisterFrame(
      SensorVectors(world_points, moved_translation), initial_translation);

  ASSERT_TRUE(moved.accepted);
  EXPECT_EQ(moved.status, ImuConstrainedOdometryStatus::kAccepted);
  EXPECT_TRUE(moved.correction_was_clamped);
  EXPECT_GT(moved.unclamped_correction.norm(),
            config.maximum_correction_from_prior);
  EXPECT_NEAR((moved.translation - initial_translation).head<2>().norm(),
              config.maximum_correction_from_prior, 1e-9);
}

}  // namespace
}  // namespace danger_search_localization
