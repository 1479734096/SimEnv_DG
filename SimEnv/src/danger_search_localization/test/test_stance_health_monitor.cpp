#include <gtest/gtest.h>

#include <limits>

#include <danger_search_localization/stance_health_monitor.hpp>

namespace danger_search_localization {
namespace {

StanceObservation Good(double base_up_z = 0.99, double angular_speed = 0.02) {
  return StanceObservation{0.31, 0.99, base_up_z, angular_speed};
}

StanceObservation BadBaseUp(double base_up_z = 0.30) {
  return StanceObservation{0.31, 0.99, base_up_z, 0.02};
}

StanceObservation BadAngular(double angular_speed = 1.0) {
  return StanceObservation{0.31, 0.99, 0.99, angular_speed};
}

TEST(StanceHealthMonitor, StartsUnhealthyAndRequiresConsecutiveGoodFrames) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 3;
  config.required_unhealthy_frames = 2;
  StanceHealthMonitor monitor(config);

  EXPECT_FALSE(monitor.healthy());
  EXPECT_FALSE(monitor.Update(Good()));
  EXPECT_FALSE(monitor.Update(Good()));
  EXPECT_TRUE(monitor.Update(Good()));
  EXPECT_TRUE(monitor.healthy());
}

TEST(StanceHealthMonitor, RequiresConsecutiveBadFramesToBecomeUnhealthy) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 2;
  config.required_unhealthy_frames = 3;
  StanceHealthMonitor monitor(config);

  // 先进入健康状态
  monitor.Update(Good());
  monitor.Update(Good());
  ASSERT_TRUE(monitor.healthy());

  // 单帧异常不应立即翻转
  EXPECT_TRUE(monitor.Update(BadBaseUp()));
  EXPECT_TRUE(monitor.Update(BadBaseUp()));
  EXPECT_FALSE(monitor.Update(BadBaseUp()));
  EXPECT_FALSE(monitor.healthy());
}

TEST(StanceHealthMonitor, LatchesUnhealthyByDefault) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 2;
  config.required_unhealthy_frames = 2;
  config.latch_unhealthy = true;
  StanceHealthMonitor monitor(config);

  monitor.Update(Good());
  monitor.Update(Good());
  ASSERT_TRUE(monitor.healthy());

  monitor.Update(BadBaseUp());
  monitor.Update(BadBaseUp());
  ASSERT_FALSE(monitor.healthy());

  // 锁存模式下即使恢复站立也不再变健康
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(monitor.Update(Good()));
  }
}

TEST(StanceHealthMonitor, AllowsRecoveryWhenLatchDisabled) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 2;
  config.required_unhealthy_frames = 2;
  config.latch_unhealthy = false;
  StanceHealthMonitor monitor(config);

  monitor.Update(Good());
  monitor.Update(Good());
  ASSERT_TRUE(monitor.healthy());

  monitor.Update(BadBaseUp());
  monitor.Update(BadBaseUp());
  ASSERT_FALSE(monitor.healthy());

  monitor.Update(Good());
  EXPECT_TRUE(monitor.Update(Good()));
}

TEST(StanceHealthMonitor, RejectsInvalidConfiguration) {
  StanceHealthMonitorConfig config;
  config.minimum_base_height = config.maximum_base_height;
  EXPECT_THROW(StanceHealthMonitor monitor(config), std::invalid_argument);
}

TEST(StanceHealthMonitor, TreatsConsecutiveNonFiniteObservationsAsUnhealthy) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 2;
  config.required_unhealthy_frames = 2;
  config.required_unknown_frames = 2;
  StanceHealthMonitor monitor(config);

  monitor.Update(Good());
  monitor.Update(Good());
  ASSERT_TRUE(monitor.healthy());
  const StanceObservation invalid{
      std::numeric_limits<double>::quiet_NaN(), 0.99, 0.99, 0.02};
  EXPECT_TRUE(monitor.Update(invalid));
  EXPECT_FALSE(monitor.Update(invalid));
}

TEST(StanceHealthMonitor, UprightStableImuKeepsHealthWhenGroundIsAbsent) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 2;
  config.required_unhealthy_frames = 2;
  config.required_unknown_frames = 5;
  StanceHealthMonitor monitor(config);

  monitor.Update(Good());
  monitor.Update(Good());
  ASSERT_TRUE(monitor.healthy());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const StanceObservation missing_ground{nan, nan, 1.0, 0.33};
  EXPECT_TRUE(monitor.Update(missing_ground));
  EXPECT_TRUE(monitor.Update(missing_ground));
  EXPECT_TRUE(monitor.Update(missing_ground));
  EXPECT_EQ(monitor.consecutive_unknown_frames(), 0U);

  EXPECT_TRUE(monitor.Update(Good()));
  EXPECT_EQ(monitor.consecutive_unknown_frames(), 0U);
}

TEST(StanceHealthMonitor, MissingGroundNeverMasksExplicitImuTilt) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 2;
  StanceHealthMonitor monitor(config);

  ASSERT_TRUE(monitor.Update(Good()));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const StanceObservation tilted_without_ground{nan, nan, 0.30, 0.02};
  EXPECT_TRUE(monitor.Update(tilted_without_ground));
  EXPECT_FALSE(monitor.Update(tilted_without_ground));
}

TEST(StanceHealthMonitor, ExplicitImuTiltStillFailsWithoutGroundPlane) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 2;
  config.required_unknown_frames = 30;
  StanceHealthMonitor monitor(config);

  ASSERT_TRUE(monitor.Update(Good()));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const StanceObservation tilted{nan, nan, 0.30, 0.02};
  EXPECT_TRUE(monitor.Update(tilted));
  EXPECT_FALSE(monitor.Update(tilted));
  EXPECT_EQ(monitor.consecutive_bad_frames(), 2U);
}

TEST(StanceHealthMonitor, InconsistentGroundPlaneDoesNotOverrideUprightImu) {
  StanceHealthMonitorConfig config;
  config.minimum_base_height = 0.14;
  config.maximum_base_height = 0.60;
  config.minimum_ground_normal_z = 0.85;
  config.minimum_base_up_z = 0.71;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 3;
  StanceHealthMonitor monitor(config);

  ASSERT_TRUE(monitor.Update(Good()));
  const StanceObservation stage13_false_positive{0.483, 0.797, 1.0, 0.407};
  EXPECT_TRUE(monitor.Update(stage13_false_positive));
  EXPECT_TRUE(monitor.Update(stage13_false_positive));
  EXPECT_TRUE(monitor.Update(stage13_false_positive));
  EXPECT_EQ(monitor.consecutive_bad_frames(), 0U);
}

TEST(StanceHealthMonitor, InconsistentGroundNeverMasksExplicitImuTilt) {
  StanceHealthMonitorConfig config;
  config.minimum_base_height = 0.14;
  config.maximum_base_height = 0.60;
  config.minimum_ground_normal_z = 0.85;
  config.minimum_base_up_z = 0.71;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 3;
  StanceHealthMonitor monitor(config);

  ASSERT_TRUE(monitor.Update(Good()));
  const StanceObservation tilted{0.483, 0.797, 0.60, 0.407};
  EXPECT_TRUE(monitor.Update(tilted));
  EXPECT_TRUE(monitor.Update(tilted));
  EXPECT_FALSE(monitor.Update(tilted));
  EXPECT_EQ(monitor.consecutive_bad_frames(), 3U);
}

TEST(StanceHealthMonitor, ReliableGroundHeightStillDetectsProneRobot) {
  StanceHealthMonitorConfig config;
  config.minimum_base_height = 0.14;
  config.maximum_base_height = 0.60;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 3;
  StanceHealthMonitor monitor(config);

  ASSERT_TRUE(monitor.Update(Good()));
  const StanceObservation prone{0.058, 0.999, 1.0, 0.02};
  EXPECT_TRUE(monitor.Update(prone));
  EXPECT_TRUE(monitor.Update(prone));
  EXPECT_FALSE(monitor.Update(prone));
  EXPECT_EQ(monitor.consecutive_bad_frames(), 3U);
}

TEST(StanceHealthMonitor, UprightDynamicAngularSpikeIsBoundedUnknown) {
  StanceHealthMonitorConfig config;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 3;
  config.required_unknown_frames = 5;
  config.maximum_angular_speed = 1.5;
  StanceHealthMonitor monitor(config);

  ASSERT_TRUE(monitor.Update(Good()));
  const StanceObservation upright_dynamic{0.239, 0.999, 0.998, 1.582};
  EXPECT_TRUE(monitor.Update(upright_dynamic));
  EXPECT_TRUE(monitor.Update(upright_dynamic));
  EXPECT_TRUE(monitor.Update(upright_dynamic));
  EXPECT_EQ(monitor.consecutive_bad_frames(), 0U);
  EXPECT_EQ(monitor.consecutive_unknown_frames(), 3U);

  // 同样的动态角速度若伴随明确倾角，仍按跌倒证据立即累计 BAD。
  const StanceObservation tilted_dynamic{0.239, 0.999, 0.578, 4.827};
  EXPECT_TRUE(monitor.Update(tilted_dynamic));
  EXPECT_TRUE(monitor.Update(tilted_dynamic));
  EXPECT_FALSE(monitor.Update(tilted_dynamic));
}

TEST(StanceHealthMonitor, RaisedThresholdRemainsHealthyButProneFails) {
  StanceHealthMonitorConfig config;
  config.minimum_base_height = 0.14;
  config.maximum_base_height = 0.60;
  config.required_healthy_frames = 1;
  config.required_unhealthy_frames = 1;
  config.latch_unhealthy = false;
  StanceHealthMonitor monitor(config);

  EXPECT_TRUE(monitor.Update(StanceObservation{0.175, 0.999, 1.0, 0.087}));
  EXPECT_FALSE(monitor.Update(StanceObservation{0.058, 0.999, 1.0, 0.0}));
}

}  // namespace
}  // namespace danger_search_localization
