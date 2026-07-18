#pragma once

#include <algorithm>
#include <cmath>

#include <ros/time.h>

namespace danger_search_localization {

/// 返回保留窗口起点，并在窗口跨越仿真零时刻时钳制为 ros::Time(0)。
///
/// ROS Noetic 的 ros::Time - ros::Duration 在底层实现为 Time + (-Duration)；
/// 负秒可能被解释为极大无符号数并抛出范围异常，因此一律用秒数计算和
/// ``fromSec`` 构造。IMU/命令缓存均通过本函数清理旧样本。
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
