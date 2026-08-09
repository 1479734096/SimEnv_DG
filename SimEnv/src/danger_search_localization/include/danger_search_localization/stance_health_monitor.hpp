#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "danger_search_localization/stance_initialization_gate.hpp"

namespace danger_search_localization {

/// 世界系建立后的持续姿态健康监控参数。
///
/// 判据复用 ``StanceObservation``：base 高度、楼面法向竖直分量、机身竖直分量和
/// 角速度。明确姿态/高度异常连续出现时认为机器人已倾倒；只有动态角速度偏高
/// 时视为暂时不可判定，避免正常步态和跨门冲击被误报为跌倒；楼面估计的
/// 高度与法向同时缺失、但 IMU 明确直立稳定时由负障安全门负责路径安全，
/// 不能把“前方是空洞”误报成“机身已经倾倒”；
/// 一旦置为不健康默认保持锁存，避免重锚定导致世界系跳变。
struct StanceHealthMonitorConfig {
  double minimum_base_height = 0.20;
  double maximum_base_height = 0.55;
  double minimum_ground_normal_z = 0.94;
  // 只有楼面法向与 IMU 竖直方向足够一致时，才采信楼面高度越界。
  double maximum_ground_normal_alignment_error = 0.02;
  double minimum_base_up_z = 0.90;
  double maximum_angular_speed = 1.50;
  std::size_t required_healthy_frames = 8;
  std::size_t required_unhealthy_frames = 3;
  std::size_t required_unknown_frames = 30;
  bool latch_unhealthy = true;
};

/// 持续监控机体是否仍处于可继续任务的站立/稳定姿态。
///
/// 与初始化门不同：初始化门在建立世界系前等待连续站立证据；本监控器在已经建立
/// 世界系后持续工作，一旦检测到连续异常帧就发布 health=false，且默认不再恢复，
/// 防止跌倒后定位继续报告虚假 ready 而任务继续推进。
class StanceHealthMonitor {
 public:
  explicit StanceHealthMonitor(
      const StanceHealthMonitorConfig &config = StanceHealthMonitorConfig())
      : config_(Validate(config)) {}

  /// 用一帧公开传感器派生的站立证据更新健康状态。
  bool Update(const StanceObservation &observation) {
    // 一旦触发不健康锁存，不再接受任何恢复证据，避免跌倒后短暂抖动让任务继续。
    if (latched_) {
      return false;
    }
    const Evidence evidence = Classify(observation);
    if (evidence == Evidence::kGood) {
      consecutive_bad_frames_ = 0;
      consecutive_unknown_frames_ = 0;
      if (!healthy_) {
        ++consecutive_good_frames_;
        if (consecutive_good_frames_ >= config_.required_healthy_frames) {
          healthy_ = true;
        }
      }
    } else if (evidence == Evidence::kBad) {
      consecutive_good_frames_ = 0;
      consecutive_unknown_frames_ = 0;
      ++consecutive_bad_frames_;
      if (consecutive_bad_frames_ >= config_.required_unhealthy_frames) {
        if (config_.latch_unhealthy && healthy_) {
          latched_ = true;
        }
        healthy_ = false;
      }
    } else {
      // 楼面 RANSAC 在转弯或稀疏回波中可能短时无解。此时没有“已跌倒”
      // 证据，保持既有健康状态；持续失明达到独立上限后仍 fail-closed。
      consecutive_good_frames_ = 0;
      consecutive_bad_frames_ = 0;
      ++consecutive_unknown_frames_;
      if (consecutive_unknown_frames_ >= config_.required_unknown_frames) {
        if (config_.latch_unhealthy && healthy_) {
          latched_ = true;
        }
        healthy_ = false;
      }
    }
    return healthy_;
  }

  bool healthy() const { return healthy_; }

  void Reset() {
    healthy_ = false;
    latched_ = false;
    consecutive_good_frames_ = 0;
    consecutive_bad_frames_ = 0;
    consecutive_unknown_frames_ = 0;
  }

  std::size_t consecutive_good_frames() const { return consecutive_good_frames_; }
  std::size_t consecutive_bad_frames() const { return consecutive_bad_frames_; }
  std::size_t consecutive_unknown_frames() const {
    return consecutive_unknown_frames_;
  }

 private:
  static StanceHealthMonitorConfig Validate(
      const StanceHealthMonitorConfig &config) {
    if (!std::isfinite(config.minimum_base_height) ||
        config.minimum_base_height <= 0.0 ||
        !std::isfinite(config.maximum_base_height) ||
        config.maximum_base_height <= config.minimum_base_height ||
        !std::isfinite(config.minimum_ground_normal_z) ||
        config.minimum_ground_normal_z <= 0.0 ||
        config.minimum_ground_normal_z > 1.0 ||
        !std::isfinite(config.maximum_ground_normal_alignment_error) ||
        config.maximum_ground_normal_alignment_error < 0.0 ||
        config.maximum_ground_normal_alignment_error > 1.0 ||
        !std::isfinite(config.minimum_base_up_z) ||
        config.minimum_base_up_z <= 0.0 || config.minimum_base_up_z > 1.0 ||
        !std::isfinite(config.maximum_angular_speed) ||
        config.maximum_angular_speed <= 0.0 ||
        config.required_healthy_frames == 0 ||
        config.required_unhealthy_frames == 0 ||
        config.required_unknown_frames == 0) {
      throw std::invalid_argument("姿态健康监控参数无效");
    }
    return config;
  }

  enum class Evidence { kGood, kBad, kUnknown };

  Evidence Classify(const StanceObservation &observation) const {
    const bool height_known = std::isfinite(observation.base_height);
    const bool ground_known = std::isfinite(observation.ground_normal_z);
    const bool up_known = std::isfinite(observation.base_up_z);
    const bool angular_known = std::isfinite(observation.angular_speed);
    if (up_known && observation.base_up_z < config_.minimum_base_up_z) {
      return Evidence::kBad;
    }
    const bool normal_aligned_with_imu =
        !up_known ||
        std::abs(observation.ground_normal_z - observation.base_up_z) <=
            config_.maximum_ground_normal_alignment_error;
    const bool reliable_ground =
        height_known && ground_known &&
        observation.ground_normal_z >= config_.minimum_ground_normal_z &&
        normal_aligned_with_imu;
    if (reliable_ground &&
        (observation.base_height < config_.minimum_base_height ||
         observation.base_height > config_.maximum_base_height)) {
      return Evidence::kBad;
    }
    // 角速度本身只能说明机身正在快速运动，不能证明已经倾倒。姿态和高度均
    // 正常时把它放入有界 UNKNOWN；若持续超过 unknown 上限仍会 fail-closed。
    if (angular_known &&
        observation.angular_speed > config_.maximum_angular_speed) {
      return Evidence::kUnknown;
    }
    // 楼面 RANSAC 缺失，或返回与直立 IMU 明显不一致的斜面时，由同一拟合
    // 派生的法向和高度都不是机身倾倒证据。路径与足印支撑由独立安全门
    // fail-closed，姿态监控仍会对任何明确 IMU 倾角立即累计 BAD。
    const bool absent_ground = !height_known && !ground_known;
    const bool inconsistent_ground =
        height_known && ground_known && !reliable_ground;
    if ((absent_ground || inconsistent_ground) && up_known && angular_known) {
      return Evidence::kGood;
    }
    return reliable_ground && up_known && angular_known ? Evidence::kGood
                                                        : Evidence::kUnknown;
  }

  StanceHealthMonitorConfig config_;
  bool healthy_ = false;
  bool latched_ = false;  // 不健康锁存已触发，之后不再恢复。
  std::size_t consecutive_good_frames_ = 0;
  std::size_t consecutive_bad_frames_ = 0;
  std::size_t consecutive_unknown_frames_ = 0;
};

}  // namespace danger_search_localization
