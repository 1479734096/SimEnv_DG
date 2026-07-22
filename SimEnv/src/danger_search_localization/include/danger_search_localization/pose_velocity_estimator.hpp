#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

#include <Eigen/Core>

namespace danger_search_localization {

struct PoseVelocityConfig {
  double window_seconds = 1.00;
  double minimum_interval = 0.80;
  double linear_deadband = 0.05;
  double angular_deadband = 0.03;
};

struct PoseVelocityEstimate {
  bool valid = false;
  Eigen::Vector3d linear_world = Eigen::Vector3d::Zero();
  double yaw_rate = 0.0;
  double interval = 0.0;
};

/// 以固定时间窗口估计位姿速度，避免把单帧点云配准抖动放大成速度尖峰。
class PoseVelocityEstimator {
 public:
  explicit PoseVelocityEstimator(
      const PoseVelocityConfig &config = PoseVelocityConfig())
      : config_(Validate(config)) {}

  PoseVelocityEstimate Update(double stamp, const Eigen::Vector3d &position,
                              double yaw) {
    if (!std::isfinite(stamp) || !position.allFinite() || !std::isfinite(yaw)) {
      return {};
    }
    if (!samples_.empty() && stamp <= samples_.back().stamp) {
      samples_.clear();
    }
    samples_.push_back(Sample{stamp, position, yaw});
    const double cutoff = stamp - config_.window_seconds;
    // 保留截止点之前最后一帧，使实际估计窗口尽量接近配置值。
    while (samples_.size() > 2 && samples_[1].stamp <= cutoff) {
      samples_.pop_front();
    }
    const Sample &oldest = samples_.front();
    const double interval = stamp - oldest.stamp;
    if (interval < config_.minimum_interval) {
      return {};
    }
    Eigen::Vector3d linear = (position - oldest.position) / interval;
    double yaw_rate = WrapAngle(yaw - oldest.yaw) / interval;
    if (linear.head<2>().norm() < config_.linear_deadband) {
      linear.x() = 0.0;
      linear.y() = 0.0;
    }
    if (std::abs(linear.z()) < config_.linear_deadband) {
      linear.z() = 0.0;
    }
    if (std::abs(yaw_rate) < config_.angular_deadband) {
      yaw_rate = 0.0;
    }
    return {true, linear, yaw_rate, interval};
  }

  void Reset() { samples_.clear(); }

 private:
  struct Sample {
    double stamp;
    Eigen::Vector3d position;
    double yaw;
  };

  static double WrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static PoseVelocityConfig Validate(const PoseVelocityConfig &config) {
    if (!std::isfinite(config.window_seconds) ||
        config.window_seconds <= 0.0 ||
        !std::isfinite(config.minimum_interval) ||
        config.minimum_interval <= 0.0 ||
        config.minimum_interval > config.window_seconds ||
        !std::isfinite(config.linear_deadband) ||
        config.linear_deadband < 0.0 ||
        !std::isfinite(config.angular_deadband) ||
        config.angular_deadband < 0.0) {
      throw std::invalid_argument("位姿速度窗口参数无效");
    }
    return config;
  }

  PoseVelocityConfig config_;
  std::deque<Sample> samples_;
};

}  // namespace danger_search_localization
