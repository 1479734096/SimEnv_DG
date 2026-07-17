#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Core>

namespace danger_search_localization {

struct ImuMotionPredictorConfig {
  double gravity = 9.81;
  double calibration_duration = 0.40;
  double maximum_sample_gap = 0.03;
  double stationary_acceleration = 0.18;
  double stationary_angular_velocity = 0.10;
  double stationary_duration = 0.25;
};

// 利用公开机身 IMU 为点云配准提供短时平移先验。它不单独发布里程计；
// 点云配准仍负责约束长期漂移，IMU 只补足长走廊中的平移退化方向。
class ImuMotionPredictor {
 public:
  explicit ImuMotionPredictor(
      const ImuMotionPredictorConfig &config = ImuMotionPredictorConfig())
      : config_(Validate(config)) {}

  void Reset() {
    first_stamp_ = 0.0;
    last_stamp_ = 0.0;
    acceleration_sum_.setZero();
    acceleration_samples_ = 0;
    acceleration_bias_.setZero();
    displacement_.setZero();
    velocity_.setZero();
    stationary_elapsed_ = 0.0;
    calibrated_ = false;
  }

  void AddSample(double stamp, const Eigen::Matrix3d &world_base_rotation,
                 const Eigen::Vector3d &linear_acceleration_base,
                 const Eigen::Vector3d &angular_velocity_base) {
    if (!std::isfinite(stamp) || stamp <= 0.0 ||
        !world_base_rotation.allFinite() ||
        !linear_acceleration_base.allFinite() ||
        !angular_velocity_base.allFinite()) {
      return;
    }
    const Eigen::Vector3d acceleration_world =
        world_base_rotation * linear_acceleration_base -
        Eigen::Vector3d(0.0, 0.0, config_.gravity);

    if (last_stamp_ <= 0.0) {
      first_stamp_ = stamp;
      last_stamp_ = stamp;
      acceleration_sum_ += acceleration_world.head<2>();
      ++acceleration_samples_;
      return;
    }
    const double dt = stamp - last_stamp_;
    if (dt <= 0.0) {
      return;
    }
    last_stamp_ = stamp;

    if (!calibrated_) {
      acceleration_sum_ += acceleration_world.head<2>();
      ++acceleration_samples_;
      if (stamp - first_stamp_ >= config_.calibration_duration &&
          acceleration_samples_ > 1) {
        acceleration_bias_ =
            acceleration_sum_ / static_cast<double>(acceleration_samples_);
        calibrated_ = true;
        velocity_.setZero();
        displacement_.setZero();
      }
      return;
    }

    if (dt > config_.maximum_sample_gap) {
      stationary_elapsed_ = 0.0;
      return;
    }

    const Eigen::Vector2d planar_acceleration =
        acceleration_world.head<2>() - acceleration_bias_;
    displacement_ += velocity_ * dt + 0.5 * planar_acceleration * dt * dt;
    velocity_ += planar_acceleration * dt;

    if (planar_acceleration.norm() <= config_.stationary_acceleration &&
        angular_velocity_base.norm() <=
            config_.stationary_angular_velocity) {
      stationary_elapsed_ += dt;
      if (stationary_elapsed_ >= config_.stationary_duration) {
        velocity_.setZero();
      }
    } else {
      stationary_elapsed_ = 0.0;
    }
  }

  bool calibrated() const { return calibrated_; }
  const Eigen::Vector2d &displacement() const { return displacement_; }
  const Eigen::Vector2d &velocity() const { return velocity_; }
  const Eigen::Vector2d &acceleration_bias() const {
    return acceleration_bias_;
  }

 private:
  static ImuMotionPredictorConfig Validate(
      const ImuMotionPredictorConfig &config) {
    if (!std::isfinite(config.gravity) || config.gravity <= 0.0 ||
        !std::isfinite(config.calibration_duration) ||
        config.calibration_duration <= 0.0 ||
        !std::isfinite(config.maximum_sample_gap) ||
        config.maximum_sample_gap <= 0.0 ||
        !std::isfinite(config.stationary_acceleration) ||
        config.stationary_acceleration < 0.0 ||
        !std::isfinite(config.stationary_angular_velocity) ||
        config.stationary_angular_velocity < 0.0 ||
        !std::isfinite(config.stationary_duration) ||
        config.stationary_duration <= 0.0) {
      throw std::invalid_argument("IMU 平移先验参数无效");
    }
    return config;
  }

  ImuMotionPredictorConfig config_;
  double first_stamp_ = 0.0;
  double last_stamp_ = 0.0;
  Eigen::Vector2d acceleration_sum_ = Eigen::Vector2d::Zero();
  std::size_t acceleration_samples_ = 0;
  Eigen::Vector2d acceleration_bias_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d displacement_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity_ = Eigen::Vector2d::Zero();
  double stationary_elapsed_ = 0.0;
  bool calibrated_ = false;
};

}  // namespace danger_search_localization
