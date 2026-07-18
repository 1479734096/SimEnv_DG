#pragma once

#include <algorithm>
#include <cmath>

#include <ros/time.h>

namespace danger_search_localization {

// ROS Noetic 的 ros::Time - ros::Duration 在底层实现为 Time + (-Duration)；
// 当窗口跨越零时刻时，-Duration 的负秒被解释成极大无符号数，normalize 会抛出
// "Time is out of dual 32-bit range"。因此一律用 fromSec 构造结果。
inline ros::Time RetentionWindowStart(const ros::Time &stamp,
                                      const double window_seconds) {
  if (stamp.isZero() || !std::isfinite(window_seconds) ||
      window_seconds <= 0.0 || stamp.toSec() <= window_seconds) {
    return ros::Time(0);
  }
  const double start_sec =
      std::max(0.0, stamp.toSec() - window_seconds);
  ros::Time start;
  start.fromSec(start_sec);
  return start;
}

}  // namespace danger_search_localization
