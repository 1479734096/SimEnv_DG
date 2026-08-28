#include <gtest/gtest.h>

#include <cmath>
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
  EXPECT_EQ(initialized.source_point_count, initialized.correspondence_count);
  EXPECT_EQ(initialized.matched_correspondence_count, 0U);
  EXPECT_GT(initialized.map_point_count, 0U);
  ASSERT_TRUE(moved.accepted);
  EXPECT_EQ(moved.status, ImuConstrainedOdometryStatus::kAccepted);
  EXPECT_FALSE(moved.correction_was_clamped);
  EXPECT_GT(moved.correspondence_count, config.minimum_correspondences);
  EXPECT_GE(moved.source_point_count, moved.matched_correspondence_count);
  EXPECT_GE(moved.matched_correspondence_count, moved.correspondence_count);
  EXPECT_GT(moved.map_point_count, 0U);
  EXPECT_GE(moved.iteration_count, 1);
  EXPECT_TRUE(std::isfinite(moved.residual_p50));
  EXPECT_TRUE(std::isfinite(moved.residual_p70));
  EXPECT_TRUE(std::isfinite(moved.residual_p90));
  EXPECT_LE(moved.residual_p50, moved.residual_p70);
  EXPECT_LE(moved.residual_p70, moved.residual_p90);
  EXPECT_TRUE(moved.residual_mean.allFinite());
  EXPECT_TRUE(moved.residual_median.allFinite());
  EXPECT_TRUE(moved.residual_mad.allFinite());
  EXPECT_GE(moved.residual_mad.minCoeff(), 0.0);
  EXPECT_GE(moved.residual_positive_fraction.minCoeff(), 0.0);
  EXPECT_LE(moved.residual_positive_fraction.maxCoeff(), 1.0);
  EXPECT_GE(moved.residual_negative_fraction.minCoeff(), 0.0);
  EXPECT_LE(moved.residual_negative_fraction.maxCoeff(), 1.0);
  EXPECT_GE(moved.residual_covariance_min_eigenvalue, -1e-12);
  EXPECT_GE(moved.residual_covariance_max_eigenvalue,
            moved.residual_covariance_min_eigenvalue);
  EXPECT_GE(moved.residual_anisotropy, 0.0);
  EXPECT_LE(moved.residual_anisotropy, 1.0);
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
  EXPECT_GT(rejected.map_point_count, 0U);
  EXPECT_LT(rejected.source_point_count, config.minimum_correspondences);
  EXPECT_EQ(rejected.matched_correspondence_count, 0U);
  EXPECT_TRUE(std::isnan(rejected.residual_p50));
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
  config.maximum_correction_from_prior = 0.10;
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

TEST(ImuConstrainedOdometry, ComputesCappedKinematicTranslationLimit) {
  EXPECT_NEAR(PlanarTranslationLimit(0.20, 0.70, 0.02, 0.30), 0.16, 1e-12);
  EXPECT_NEAR(PlanarTranslationLimit(1.00, 0.70, 0.02, 0.30), 0.30, 1e-12);
  EXPECT_TRUE(std::isinf(PlanarTranslationLimit(
      std::numeric_limits<double>::quiet_NaN(), 0.70, 0.02, 0.30)));
  EXPECT_THROW(PlanarTranslationLimit(0.10, 0.0, 0.02, 0.30),
               std::invalid_argument);
}

TEST(ImuConstrainedOdometry, RejectsTranslationBeforeUpdatingLocalMap) {
  ImuConstrainedOdometryConfig config;
  config.minimum_correspondences = 80;
  config.maximum_correspondence_distance = 0.55;
  config.maximum_correction_from_prior = 0.50;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(0.0, -3.0, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);

  const Eigen::Vector3d excessive_translation(0.20, -3.0, 0.35);
  const auto rejected = odometry.RegisterFrame(
      SensorVectors(world_points, excessive_translation), excessive_translation,
      0.10);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.status, ImuConstrainedOdometryStatus::kTranslationLimit);
  EXPECT_STREQ(ImuConstrainedOdometryStatusName(rejected.status),
               "translation_limit");
  EXPECT_NEAR(rejected.translation.x(), initial_translation.x(), 1e-12);
  EXPECT_NEAR(rejected.translation.y(), initial_translation.y(), 1e-12);

  const Eigen::Vector3d valid_translation(0.05, -3.0, 0.35);
  const auto accepted = odometry.RegisterFrame(
      SensorVectors(world_points, valid_translation), valid_translation, 0.10);
  ASSERT_TRUE(accepted.accepted);
  EXPECT_NEAR(accepted.translation.x(), valid_translation.x(), 0.01);
  EXPECT_NEAR(accepted.translation.y(), valid_translation.y(), 0.01);
}

TEST(ImuConstrainedOdometry, EstimatesWithoutMutatingMapUntilExplicitCommit) {
  ImuConstrainedOdometryConfig config;
  config.minimum_correspondences = 80;
  config.maximum_correspondence_distance = 0.55;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(0.0, -3.0, 0.35);
  const Eigen::Vector3d moved_translation(0.06, -3.02, 0.36);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);
  const std::uint64_t generation_before = odometry.map_generation();
  const std::size_t map_points_before = odometry.map_point_count();

  const auto estimated = odometry.EstimateFrame(
      SensorVectors(world_points, moved_translation), moved_translation,
      initial_translation, 0.10);

  ASSERT_TRUE(estimated.accepted);
  EXPECT_EQ(estimated.map_generation, generation_before);
  EXPECT_EQ(odometry.map_generation(), generation_before);
  EXPECT_EQ(odometry.map_point_count(), map_points_before);
  EXPECT_TRUE(odometry.last_committed_translation().isApprox(
      initial_translation, 1e-12));
  ASSERT_TRUE(odometry.CommitFrame(
      SensorVectors(world_points, moved_translation), estimated.translation,
      estimated.map_generation));
  EXPECT_EQ(odometry.map_generation(), generation_before + 1U);
  EXPECT_TRUE(odometry.last_committed_translation().isApprox(
      estimated.translation, 1e-12));
}

TEST(ImuConstrainedOdometry, RejectsStaleMapGenerationWithoutMutation) {
  ImuConstrainedOdometry odometry;
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(0.0, -3.0, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);
  const std::uint64_t current_generation = odometry.map_generation();
  const std::size_t map_points_before = odometry.map_point_count();

  EXPECT_FALSE(odometry.CommitFrame(
      SensorVectors(world_points, initial_translation), initial_translation,
      current_generation - 1U));
  EXPECT_EQ(odometry.map_generation(), current_generation);
  EXPECT_EQ(odometry.map_point_count(), map_points_before);
}

TEST(ImuConstrainedOdometry, TranslatesPoseAndLocalMapInOneGeneration) {
  ImuConstrainedOdometry odometry;
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d initial_translation(0.0, -3.0, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, initial_translation), initial_translation).accepted);
  const std::uint64_t generation = odometry.map_generation();
  const std::size_t point_count = odometry.map_point_count();

  ASSERT_TRUE(odometry.TranslatePlanar(Eigen::Vector2d(0.45, -0.30)));

  EXPECT_EQ(odometry.map_generation(), generation + 1U);
  // 平移后需按固定 world 体素重建索引，落入同一体素的点可合并；
  // 这不能改变位姿与地图的同一次原子提交。
  EXPECT_GE(odometry.map_point_count(), point_count * 4U / 5U);
  EXPECT_NEAR(odometry.last_committed_translation().x(), 0.45, 1e-12);
  EXPECT_NEAR(odometry.last_committed_translation().y(), -3.30, 1e-12);
  EXPECT_NEAR(odometry.last_committed_translation().z(), 0.35, 1e-12);
}

TEST(ImuConstrainedOdometry, UsesExplicitPublishedPoseForMotionLimit) {
  ImuConstrainedOdometryConfig config;
  config.maximum_correspondence_distance = 0.55;
  config.maximum_correction_from_prior = 0.50;
  ImuConstrainedOdometry odometry(config);
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d map_commit_translation(0.0, -3.0, 0.35);
  ASSERT_TRUE(odometry.RegisterFrame(
      SensorVectors(world_points, map_commit_translation),
      map_commit_translation).accepted);
  const Eigen::Vector3d latest_published_translation(0.20, -3.0, 0.35);
  const Eigen::Vector3d current_translation(0.25, -3.0, 0.35);

  const auto estimated = odometry.EstimateFrame(
      SensorVectors(world_points, current_translation), current_translation,
      latest_published_translation, 0.10);

  ASSERT_TRUE(estimated.accepted);
  EXPECT_NEAR(estimated.translation.x(), current_translation.x(), 0.02);
}

TEST(ImuConstrainedOdometry, ReportsUninitializedMapForReadOnlyEstimate) {
  ImuConstrainedOdometry odometry;
  const auto world_points = MakeAsymmetricStructure();
  const Eigen::Vector3d translation(0.0, -3.0, 0.35);

  const auto estimated = odometry.EstimateFrame(
      SensorVectors(world_points, translation), translation, translation);

  EXPECT_FALSE(estimated.accepted);
  EXPECT_EQ(estimated.status,
            ImuConstrainedOdometryStatus::kMapUninitialized);
  EXPECT_EQ(estimated.map_generation, 0U);
}

}  // namespace
}  // namespace danger_search_localization
