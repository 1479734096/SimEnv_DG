#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace danger_search_localization {

/// 世界系初始化前的站立稳定性门槛。
struct StanceInitializationConfig {
  double minimum_base_height = 0.20;
  double maximum_base_height = 0.55;
  double minimum_ground_normal_z = 0.94;
  double minimum_base_up_z = 0.90;
  double maximum_angular_speed = 0.30;
  double maximum_height_span = 0.04;
  std::size_t required_consecutive_frames = 8;
};

/// 一帧由公开点云和机体 IMU 派生的站立证据。
struct StanceObservation {
  double base_height = 0.0;
  double ground_normal_z = 0.0;
  double base_up_z = 0.0;
  double angular_speed = 0.0;
};

/// 只有连续多帧满足高度、方向和动态约束时才允许建立世界坐标。
///
/// 腹部着地时 IMU 和楼面法向仍可能近似竖直，因此 base 高度是不可省略的
/// 判据；连续窗口用于排除起立过程中短暂经过合法高度区间的帧。
class StanceInitializationGate {
 public:
  explicit StanceInitializationGate(
      const StanceInitializationConfig &config = StanceInitializationConfig())
      : config_(Validate(config)) {}

  bool Update(const StanceObservation &observation) {
    if (!IsCandidate(observation)) {
      heights_.clear();
      return false;
    }
    heights_.push_back(observation.base_height);
    while (heights_.size() > config_.required_consecutive_frames) {
      heights_.pop_front();
    }
    if (heights_.size() < config_.required_consecutive_frames) {
      return false;
    }
    const auto bounds = std::minmax_element(heights_.begin(), heights_.end());
    return *bounds.second - *bounds.first <= config_.maximum_height_span;
  }

  std::size_t consecutive_frames() const { return heights_.size(); }

  void Reset() { heights_.clear(); }

 private:
  static StanceInitializationConfig Validate(
      const StanceInitializationConfig &config) {
    if (!std::isfinite(config.minimum_base_height) ||
        !std::isfinite(config.maximum_base_height) ||
        config.minimum_base_height <= 0.0 ||
        config.maximum_base_height <= config.minimum_base_height ||
        !std::isfinite(config.minimum_ground_normal_z) ||
        config.minimum_ground_normal_z <= 0.0 ||
        config.minimum_ground_normal_z > 1.0 ||
        !std::isfinite(config.minimum_base_up_z) ||
        config.minimum_base_up_z <= 0.0 || config.minimum_base_up_z > 1.0 ||
        !std::isfinite(config.maximum_angular_speed) ||
        config.maximum_angular_speed <= 0.0 ||
        !std::isfinite(config.maximum_height_span) ||
        config.maximum_height_span < 0.0 ||
        config.required_consecutive_frames == 0) {
      throw std::invalid_argument("站立初始化门参数无效");
    }
    return config;
  }

  bool IsCandidate(const StanceObservation &observation) const {
    return std::isfinite(observation.base_height) &&
           observation.base_height >= config_.minimum_base_height &&
           observation.base_height <= config_.maximum_base_height &&
           std::isfinite(observation.ground_normal_z) &&
           observation.ground_normal_z >= config_.minimum_ground_normal_z &&
           std::isfinite(observation.base_up_z) &&
           observation.base_up_z >= config_.minimum_base_up_z &&
           std::isfinite(observation.angular_speed) &&
           observation.angular_speed <= config_.maximum_angular_speed;
  }

  StanceInitializationConfig config_;
  std::deque<double> heights_;
};

}  // namespace danger_search_localization
