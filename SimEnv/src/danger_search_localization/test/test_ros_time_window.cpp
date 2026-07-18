#include <gtest/gtest.h>

#include "danger_search_localization/ros_time_window.hpp"

TEST(RosTimeWindow, EarlySimulationTimeClampsToZero) {
  const ros::Time stamp(4, 200000000);

  const ros::Time start =
      danger_search_localization::RetentionWindowStart(stamp, 5.0);

  EXPECT_TRUE(start.isZero());
}

TEST(RosTimeWindow, EstablishedSimulationTimeKeepsRequestedWindow) {
  const ros::Time stamp(8, 200000000);

  const ros::Time start =
      danger_search_localization::RetentionWindowStart(stamp, 5.0);

  EXPECT_NEAR(start.toSec(), 3.2, 1e-9);
}

TEST(RosTimeWindow, InvalidWindowFailsClosedAtZero) {
  const ros::Time stamp(8, 200000000);

  EXPECT_TRUE(danger_search_localization::RetentionWindowStart(stamp, -1.0)
                  .isZero());
}

TEST(RosTimeWindow, NearZeroWindowDoesNotThrow) {
  // 该场景直接触发底层 Time - Duration 的负时间溢出，必须被 fromSec 路径规避。
  ros::Time stamp;
  stamp.fromSec(4.1);

  const ros::Time start =
      danger_search_localization::RetentionWindowStart(stamp, 5.0);

  EXPECT_TRUE(start.isZero());
}

TEST(RosTimeWindow, BoundaryBetweenStampAndWindowIsClamped) {
  ros::Time stamp;
  stamp.fromSec(4.999999999);

  const ros::Time start =
      danger_search_localization::RetentionWindowStart(stamp, 5.0);

  EXPECT_TRUE(start.isZero());
}

TEST(RosTimeWindow, JustAboveWindowReturnsPositiveStart) {
  ros::Time stamp;
  stamp.fromSec(5.001);

  const ros::Time start =
      danger_search_localization::RetentionWindowStart(stamp, 5.0);

  EXPECT_NEAR(start.toSec(), 0.001, 1e-9);
}
