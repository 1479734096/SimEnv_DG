#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kiss_icp/core/Preprocessing.hpp"

namespace danger_search_localization {

/// 把当前 IMU 姿态锚定到初始化时的 world->base 姿态。
/// IMU 的绝对参考系无需等于 world；初始相对旋转会在该公式中抵消。
inline Eigen::Matrix3d AnchorImuOrientation(
    const Eigen::Matrix3d &initial_world_base_rotation,
    const Eigen::Matrix3d &initial_imu_rotation,
    const Eigen::Matrix3d &current_imu_rotation) {
  return initial_world_base_rotation * initial_imu_rotation.transpose() *
         current_imu_rotation;
}

/// 局部结构地图、对应点筛选和每帧平移校正的保守参数。
struct ImuConstrainedOdometryConfig {
  double source_voxel_size = 0.16;
  double map_voxel_size = 0.12;
  double maximum_map_distance = 12.0;
  double maximum_correspondence_distance = 0.55;
  double residual_quantile = 0.70;
  double minimum_residual_cutoff = 0.08;
  double maximum_iteration_step = 0.12;
  double maximum_correction_from_prior = 0.25;
  double convergence_threshold = 0.0002;
  std::size_t minimum_correspondences = 80;
  int maximum_iterations = 12;
};

/// 局部地图校正的稳定诊断状态；只暴露既有分支，不改变验收策略。
enum class ImuConstrainedOdometryStatus {
  kAccepted = 0,
  kInitialFrame,
  kInsufficientSource,
  kInsufficientCorrespondences,
  kTranslationLimit,
  kMapUninitialized,
  kMapGenerationMismatch,
  kCount,
};

inline const char *ImuConstrainedOdometryStatusName(
    ImuConstrainedOdometryStatus status) {
  switch (status) {
    case ImuConstrainedOdometryStatus::kAccepted:
      return "accepted";
    case ImuConstrainedOdometryStatus::kInitialFrame:
      return "initial_frame";
    case ImuConstrainedOdometryStatus::kInsufficientSource:
      return "insufficient_source";
    case ImuConstrainedOdometryStatus::kInsufficientCorrespondences:
      return "insufficient_correspondences";
    case ImuConstrainedOdometryStatus::kTranslationLimit:
      return "translation_limit";
    case ImuConstrainedOdometryStatus::kMapUninitialized:
      return "map_uninitialized";
    case ImuConstrainedOdometryStatus::kMapGenerationMismatch:
      return "map_generation_mismatch";
    case ImuConstrainedOdometryStatus::kCount:
      return "count";
  }
  return "unknown";
}

inline double PlanarTranslationLimit(double frame_interval,
                                     double maximum_speed,
                                     double margin,
                                     double absolute_cap) {
  if (!std::isfinite(maximum_speed) || maximum_speed <= 0.0 ||
      !std::isfinite(margin) || margin < 0.0 ||
      !std::isfinite(absolute_cap) || absolute_cap <= 0.0) {
    throw std::invalid_argument("平面运动学上界参数无效");
  }
  if (!std::isfinite(frame_interval) || frame_interval <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::min(absolute_cap, maximum_speed * frame_interval + margin);
}

/// 一帧局部地图配准的验收结果；拒绝时仍返回安全的上一平移估计。
struct ImuConstrainedOdometryResult {
  bool accepted = false;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  std::size_t correspondence_count = 0;
  ImuConstrainedOdometryStatus status =
      ImuConstrainedOdometryStatus::kInsufficientSource;
  Eigen::Vector2d unclamped_correction = Eigen::Vector2d::Zero();
  bool correction_was_clamped = false;
  std::size_t source_point_count = 0;
  std::size_t matched_correspondence_count = 0;
  std::size_t map_point_count = 0;
  int iteration_count = 0;
  double residual_p50 = std::numeric_limits<double>::quiet_NaN();
  double residual_p70 = std::numeric_limits<double>::quiet_NaN();
  double residual_p90 = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector2d residual_mean = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d residual_median = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d residual_mad = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d residual_positive_fraction = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d residual_negative_fraction = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double residual_covariance_min_eigenvalue =
      std::numeric_limits<double>::quiet_NaN();
  double residual_covariance_max_eigenvalue =
      std::numeric_limits<double>::quiet_NaN();
  double residual_anisotropy = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t map_generation = 0;
};

/// 在 IMU 已固定旋转的条件下，仅估计平面平移的局部扫描匹配器。
///
/// 输入点已经旋转到 world 方向，本类把先验平移后的点与稀疏体素地图做最近邻
/// 对应，以残差分位数去除离群点，再用 XY 中位残差迭代修正。Z 始终来自楼面
/// 先验，单帧校正也被限制在有限半径，避免走廊退化或动态物体造成定位跳变。
class ImuConstrainedOdometry {
 public:
  explicit ImuConstrainedOdometry(
      const ImuConstrainedOdometryConfig &config = ImuConstrainedOdometryConfig())
      : config_(Validate(config)) {}

  ImuConstrainedOdometryResult RegisterFrame(
      const std::vector<Eigen::Vector3d> &world_oriented_vectors,
      const Eigen::Vector3d &translation_prior,
      double maximum_translation_from_last =
          std::numeric_limits<double>::infinity()) {
    ValidateTranslationLimit(maximum_translation_from_last);
    const auto map_frame = kiss_icp::VoxelDownsample(
        world_oriented_vectors, config_.map_voxel_size);
    const auto source = kiss_icp::VoxelDownsample(
        map_frame, config_.source_voxel_size);
    if (source.size() < config_.minimum_correspondences) {
      auto rejected = Rejected(
          RejectionFallback(
              initialized_ ? last_translation_ : translation_prior,
              translation_prior),
          0,
          ImuConstrainedOdometryStatus::kInsufficientSource);
      rejected.source_point_count = source.size();
      rejected.map_point_count = map_points_.size();
      rejected.map_generation = map_generation_;
      return rejected;
    }
    if (!initialized_) {
      const std::uint64_t expected_generation = map_generation_;
      if (!CommitFrame(world_oriented_vectors, translation_prior,
                       expected_generation)) {
        auto rejected = Rejected(
            translation_prior, 0,
            ImuConstrainedOdometryStatus::kMapGenerationMismatch);
        rejected.source_point_count = source.size();
        rejected.map_point_count = map_points_.size();
        rejected.map_generation = map_generation_;
        return rejected;
      }
      ImuConstrainedOdometryResult result;
      result.accepted = true;
      result.translation = translation_prior;
      result.correspondence_count = source.size();
      result.status = ImuConstrainedOdometryStatus::kInitialFrame;
      result.source_point_count = source.size();
      result.map_point_count = map_points_.size();
      result.map_generation = map_generation_;
      return result;
    }

    auto result = EstimateFrame(
        world_oriented_vectors, translation_prior, last_translation_,
        maximum_translation_from_last);
    if (!result.accepted) {
      return result;
    }
    if (!CommitFrame(world_oriented_vectors, result.translation,
                     result.map_generation)) {
      auto rejected = Rejected(
          last_translation_, result.correspondence_count,
          ImuConstrainedOdometryStatus::kMapGenerationMismatch);
      rejected.source_point_count = result.source_point_count;
      rejected.matched_correspondence_count =
          result.matched_correspondence_count;
      rejected.map_point_count = map_points_.size();
      rejected.map_generation = map_generation_;
      return rejected;
    }
    result.map_point_count = map_points_.size();
    result.map_generation = map_generation_;
    return result;
  }

  /// 只读估计当前帧平移；运动学上界始终相对上一正式发布位姿，而不是地图提交位姿。
  ImuConstrainedOdometryResult EstimateFrame(
      const std::vector<Eigen::Vector3d> &world_oriented_vectors,
      const Eigen::Vector3d &translation_prior,
      const Eigen::Vector3d &motion_reference_translation,
      double maximum_translation_from_reference =
          std::numeric_limits<double>::infinity()) const {
    ValidateTranslationLimit(maximum_translation_from_reference);
    if (!translation_prior.allFinite() ||
        !motion_reference_translation.allFinite()) {
      throw std::invalid_argument("平移先验和运动参考必须为有限数");
    }
    // 两级体素分别控制地图密度与当前帧计算量；首个有效帧只初始化局部地图，
    // 没有上一地图时不虚构相对运动。
    const auto map_frame = kiss_icp::VoxelDownsample(
        world_oriented_vectors, config_.map_voxel_size);
    const auto source = kiss_icp::VoxelDownsample(
        map_frame, config_.source_voxel_size);
    if (source.size() < config_.minimum_correspondences) {
      auto rejected = Rejected(
          RejectionFallback(
              motion_reference_translation, translation_prior),
          0,
          ImuConstrainedOdometryStatus::kInsufficientSource);
      rejected.source_point_count = source.size();
      rejected.map_point_count = map_points_.size();
      return rejected;
    }

    if (!initialized_) {
      auto rejected = Rejected(
          RejectionFallback(
              motion_reference_translation, translation_prior),
          0,
          ImuConstrainedOdometryStatus::kMapUninitialized);
      rejected.source_point_count = source.size();
      rejected.map_generation = map_generation_;
      return rejected;
    }

    Eigen::Vector3d estimate = translation_prior;
    // 当前楼面观测负责机身高度；扫描匹配只估计平面平移，避免地面点或
    // 竖直墙面把 z 与 xy 的退化方向互相污染。
    estimate.z() = translation_prior.z();
    std::size_t retained_count = 0;
    std::size_t matched_count = 0;
    int iteration_count = 0;
    double residual_p50 = std::numeric_limits<double>::quiet_NaN();
    double residual_p70 = std::numeric_limits<double>::quiet_NaN();
    double residual_p90 = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector2d residual_mean = Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d residual_median = Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d residual_mad = Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d residual_positive_fraction = Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d residual_negative_fraction = Eigen::Vector2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double residual_covariance_min_eigenvalue =
        std::numeric_limits<double>::quiet_NaN();
    double residual_covariance_max_eigenvalue =
        std::numeric_limits<double>::quiet_NaN();
    double residual_anisotropy = std::numeric_limits<double>::quiet_NaN();
    bool has_valid_iteration = false;
    for (int iteration = 0; iteration < config_.maximum_iterations; ++iteration) {
      iteration_count = iteration + 1;
      std::vector<Eigen::Vector3d> world_source;
      world_source.reserve(source.size());
      for (const auto &point : source) {
        world_source.push_back(point + estimate);
      }
      const auto correspondences = FindCorrespondences(world_source);
      const auto &matched_source = correspondences.first;
      const auto &matched_target = correspondences.second;
      matched_count = matched_source.size();
      if (matched_source.size() < config_.minimum_correspondences) {
        break;
      }

      std::vector<double> residual_norms;
      residual_norms.reserve(matched_source.size());
      for (std::size_t index = 0; index < matched_source.size(); ++index) {
        residual_norms.push_back(
            (matched_target[index] - matched_source[index]).head<2>().norm());
      }
      residual_p50 = Quantile(residual_norms, 0.50);
      residual_p70 = Quantile(residual_norms, 0.70);
      residual_p90 = Quantile(residual_norms, 0.90);
      const double quantile =
          Quantile(residual_norms, config_.residual_quantile);
      const double cutoff = std::max(config_.minimum_residual_cutoff, quantile);
      std::vector<double> residual_x;
      std::vector<double> residual_y;
      residual_x.reserve(matched_source.size());
      residual_y.reserve(matched_source.size());
      for (std::size_t index = 0; index < matched_source.size(); ++index) {
        const Eigen::Vector3d residual = matched_target[index] - matched_source[index];
        if (residual.head<2>().norm() <= cutoff) {
          residual_x.push_back(residual.x());
          residual_y.push_back(residual.y());
        }
      }
      retained_count = residual_x.size();
      if (retained_count < config_.minimum_correspondences) {
        break;
      }

      residual_median =
          Eigen::Vector2d(Median(residual_x), Median(residual_y));
      residual_mean.setZero();
      residual_positive_fraction.setZero();
      residual_negative_fraction.setZero();
      for (std::size_t index = 0; index < retained_count; ++index) {
        residual_mean.x() += residual_x[index];
        residual_mean.y() += residual_y[index];
        residual_positive_fraction.x() += residual_x[index] > 0.0 ? 1.0 : 0.0;
        residual_positive_fraction.y() += residual_y[index] > 0.0 ? 1.0 : 0.0;
        residual_negative_fraction.x() += residual_x[index] < 0.0 ? 1.0 : 0.0;
        residual_negative_fraction.y() += residual_y[index] < 0.0 ? 1.0 : 0.0;
      }
      const double retained_denominator = static_cast<double>(retained_count);
      residual_mean /= retained_denominator;
      residual_positive_fraction /= retained_denominator;
      residual_negative_fraction /= retained_denominator;

      std::vector<double> absolute_deviation_x;
      std::vector<double> absolute_deviation_y;
      absolute_deviation_x.reserve(retained_count);
      absolute_deviation_y.reserve(retained_count);
      double covariance_xx = 0.0;
      double covariance_xy = 0.0;
      double covariance_yy = 0.0;
      for (std::size_t index = 0; index < retained_count; ++index) {
        absolute_deviation_x.push_back(
            std::abs(residual_x[index] - residual_median.x()));
        absolute_deviation_y.push_back(
            std::abs(residual_y[index] - residual_median.y()));
        const double centered_x = residual_x[index] - residual_mean.x();
        const double centered_y = residual_y[index] - residual_mean.y();
        covariance_xx += centered_x * centered_x;
        covariance_xy += centered_x * centered_y;
        covariance_yy += centered_y * centered_y;
      }
      residual_mad = Eigen::Vector2d(
          Median(absolute_deviation_x), Median(absolute_deviation_y));
      covariance_xx /= retained_denominator;
      covariance_xy /= retained_denominator;
      covariance_yy /= retained_denominator;
      const double covariance_trace = covariance_xx + covariance_yy;
      const double covariance_discriminant = std::sqrt(std::max(
          0.0, (covariance_xx - covariance_yy) *
                       (covariance_xx - covariance_yy) +
                   4.0 * covariance_xy * covariance_xy));
      residual_covariance_min_eigenvalue =
          0.5 * (covariance_trace - covariance_discriminant);
      residual_covariance_max_eigenvalue =
          0.5 * (covariance_trace + covariance_discriminant);
      residual_anisotropy = residual_covariance_max_eigenvalue > 1e-12
                                ? residual_covariance_min_eigenvalue /
                                      residual_covariance_max_eigenvalue
                                : 0.0;

      Eigen::Vector2d delta = residual_median;
      if (delta.norm() > config_.maximum_iteration_step) {
        delta *= config_.maximum_iteration_step / delta.norm();
      }
      estimate.x() += delta.x();
      estimate.y() += delta.y();
      has_valid_iteration = true;
      if (delta.norm() < config_.convergence_threshold) {
        break;
      }
    }
    if (!has_valid_iteration || retained_count < config_.minimum_correspondences) {
      auto rejected = Rejected(
          RejectionFallback(
              motion_reference_translation, translation_prior),
          retained_count,
          ImuConstrainedOdometryStatus::kInsufficientCorrespondences);
      rejected.source_point_count = source.size();
      rejected.matched_correspondence_count = matched_count;
      rejected.map_point_count = map_points_.size();
      rejected.iteration_count = iteration_count;
      return rejected;
    }

    Eigen::Vector2d correction =
        estimate.head<2>() - translation_prior.head<2>();
    const Eigen::Vector2d unclamped_correction = correction;
    bool correction_was_clamped = false;
    if (correction.norm() > config_.maximum_correction_from_prior) {
      correction *= config_.maximum_correction_from_prior / correction.norm();
      estimate.x() = translation_prior.x() + correction.x();
      estimate.y() = translation_prior.y() + correction.y();
      correction_was_clamped = true;
    }

    if (std::isfinite(maximum_translation_from_reference) &&
        (estimate - motion_reference_translation).head<2>().norm() >
            maximum_translation_from_reference) {
      auto rejected = Rejected(
          RejectionFallback(
              motion_reference_translation, translation_prior),
          retained_count,
          ImuConstrainedOdometryStatus::kTranslationLimit);
      rejected.unclamped_correction = unclamped_correction;
      rejected.correction_was_clamped = correction_was_clamped;
      rejected.source_point_count = source.size();
      rejected.matched_correspondence_count = matched_count;
      rejected.map_point_count = map_points_.size();
      rejected.iteration_count = iteration_count;
      rejected.residual_p50 = residual_p50;
      rejected.residual_p70 = residual_p70;
      rejected.residual_p90 = residual_p90;
      return rejected;
    }

    ImuConstrainedOdometryResult result;
    result.accepted = true;
    result.translation = estimate;
    result.correspondence_count = retained_count;
    result.status = ImuConstrainedOdometryStatus::kAccepted;
    result.unclamped_correction = unclamped_correction;
    result.correction_was_clamped = correction_was_clamped;
    result.source_point_count = source.size();
    result.matched_correspondence_count = matched_count;
    result.map_point_count = map_points_.size();
    result.iteration_count = iteration_count;
    result.residual_p50 = residual_p50;
    result.residual_p70 = residual_p70;
    result.residual_p90 = residual_p90;
    result.residual_mean = residual_mean;
    result.residual_median = residual_median;
    result.residual_mad = residual_mad;
    result.residual_positive_fraction = residual_positive_fraction;
    result.residual_negative_fraction = residual_negative_fraction;
    result.residual_covariance_min_eigenvalue =
        residual_covariance_min_eigenvalue;
    result.residual_covariance_max_eigenvalue =
        residual_covariance_max_eigenvalue;
    result.residual_anisotropy = residual_anisotropy;
    result.map_generation = map_generation_;
    return result;
  }

  /// 仅在地图代次仍与候选估计一致时提交一帧；调用方负责把位姿选择和地图写入串行化。
  bool CommitFrame(
      const std::vector<Eigen::Vector3d> &world_oriented_vectors,
      const Eigen::Vector3d &translation, std::uint64_t expected_generation) {
    if (!translation.allFinite() || expected_generation != map_generation_) {
      return false;
    }
    const auto map_frame = kiss_icp::VoxelDownsample(
        world_oriented_vectors, config_.map_voxel_size);
    const auto source = kiss_icp::VoxelDownsample(
        map_frame, config_.source_voxel_size);
    if (source.size() < config_.minimum_correspondences) {
      return false;
    }
    UpdateMap(map_frame, translation);
    last_translation_ = translation;
    initialized_ = true;
    ++map_generation_;
    return true;
  }

  bool initialized() const { return initialized_; }

  std::uint64_t map_generation() const { return map_generation_; }

  std::size_t map_point_count() const { return map_points_.size(); }

  const Eigen::Vector3d &last_committed_translation() const {
    return last_translation_;
  }

  /// 将当前局部地图和最后提交位姿作为一个事务平移到同一 world 坐标。
  bool TranslatePlanar(const Eigen::Vector2d &correction) {
    if (!initialized_ || !correction.allFinite()) {
      return false;
    }
    PointMap translated;
    translated.reserve(map_points_.size());
    for (const auto &item : map_points_) {
      Eigen::Vector3d point = item.second;
      point.head<2>() += correction;
      translated.try_emplace(KeyFor(point, config_.map_voxel_size), point);
    }
    map_points_ = std::move(translated);
    last_translation_.head<2>() += correction;
    ++map_generation_;
    RebuildSearchIndex();
    return true;
  }

 private:
  struct GridKey {
    int x;
    int y;
    int z;

    bool operator==(const GridKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct GridKeyHash {
    std::size_t operator()(const GridKey &key) const {
      const std::size_t x = std::hash<int>{}(key.x);
      const std::size_t y = std::hash<int>{}(key.y);
      const std::size_t z = std::hash<int>{}(key.z);
      return x ^ (y << 1U) ^ (z << 2U);
    }
  };

  using PointMap = std::unordered_map<GridKey, Eigen::Vector3d, GridKeyHash>;
  using SearchCloud = pcl::PointCloud<pcl::PointXYZ>;

  static GridKey KeyFor(const Eigen::Vector3d &point, double size) {
    return GridKey{
        static_cast<int>(std::floor(point.x() / size)),
        static_cast<int>(std::floor(point.y() / size)),
        static_cast<int>(std::floor(point.z() / size)),
    };
  }

  static pcl::PointXYZ ToSearchPoint(const Eigen::Vector3d &point) {
    return pcl::PointXYZ(
        static_cast<float>(point.x()), static_cast<float>(point.y()),
        static_cast<float>(point.z()));
  }

  static ImuConstrainedOdometryConfig Validate(
      const ImuConstrainedOdometryConfig &config) {
    if (!std::isfinite(config.source_voxel_size) ||
        !std::isfinite(config.map_voxel_size) ||
        !std::isfinite(config.maximum_map_distance) ||
        !std::isfinite(config.maximum_correspondence_distance) ||
        !std::isfinite(config.residual_quantile) ||
        !std::isfinite(config.minimum_residual_cutoff) ||
        !std::isfinite(config.maximum_iteration_step) ||
        !std::isfinite(config.maximum_correction_from_prior) ||
        !std::isfinite(config.convergence_threshold) ||
        config.source_voxel_size <= 0.0 || config.map_voxel_size <= 0.0 ||
        config.maximum_map_distance <= 0.0 ||
        config.maximum_correspondence_distance <= 0.0 ||
        config.residual_quantile <= 0.0 || config.residual_quantile > 1.0 ||
        config.minimum_residual_cutoff <= 0.0 ||
        config.maximum_iteration_step <= 0.0 ||
        config.maximum_correction_from_prior <= 0.0 ||
        config.convergence_threshold <= 0.0 ||
        config.minimum_correspondences == 0 || config.maximum_iterations <= 0) {
      throw std::invalid_argument("IMU 约束定位参数无效");
    }
    return config;
  }

  static void ValidateTranslationLimit(double maximum_translation) {
    if ((!std::isfinite(maximum_translation) &&
         !std::isinf(maximum_translation)) ||
        maximum_translation <= 0.0) {
      throw std::invalid_argument("单帧平移上界必须为正数或正无穷");
    }
  }

  static Eigen::Vector3d RejectionFallback(
      const Eigen::Vector3d &motion_reference_translation,
      const Eigen::Vector3d &translation_prior) {
    Eigen::Vector3d fallback = motion_reference_translation;
    fallback.z() = translation_prior.z();
    return fallback;
  }

  static double Quantile(std::vector<double> values, double quantile) {
    const std::size_t index = static_cast<std::size_t>(
        std::floor(quantile * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
  }

  static double Median(std::vector<double> values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0) {
      return upper;
    }
    std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
    return 0.5 * (values[middle - 1] + upper);
  }

  ImuConstrainedOdometryResult Rejected(
      const Eigen::Vector3d &fallback_translation,
      std::size_t correspondence_count,
      ImuConstrainedOdometryStatus status) const {
    ImuConstrainedOdometryResult result;
    result.accepted = false;
    result.translation = fallback_translation;
    result.correspondence_count = correspondence_count;
    result.status = status;
    result.map_generation = map_generation_;
    return result;
  }

  std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
  FindCorrespondences(const std::vector<Eigen::Vector3d> &source) const {
    std::vector<Eigen::Vector3d> matched_source;
    std::vector<Eigen::Vector3d> matched_target;
    matched_source.reserve(source.size());
    matched_target.reserve(source.size());
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_distance_squared(1);
    const double maximum_distance_squared =
        config_.maximum_correspondence_distance *
        config_.maximum_correspondence_distance;
    for (const auto &point : source) {
      if (search_tree_.nearestKSearch(
              ToSearchPoint(point), 1, nearest_index,
              nearest_distance_squared) > 0) {
        if (nearest_distance_squared.front() >= maximum_distance_squared) {
          continue;
        }
        const Eigen::Vector3d &candidate =
            search_points_[static_cast<std::size_t>(nearest_index.front())];
        matched_source.push_back(point);
        matched_target.push_back(candidate);
      }
    }
    return {std::move(matched_source), std::move(matched_target)};
  }

  void UpdateMap(const std::vector<Eigen::Vector3d> &frame,
                 const Eigen::Vector3d &translation) {
    // 地图以体素键去重，并围绕当前位置保留固定半径；随后重建 KD-tree，保证
    // 下一帧的对应点只来自近期局部结构而不是无限增长的历史地图。
    for (const auto &point : frame) {
      const Eigen::Vector3d world_point = point + translation;
      map_points_.try_emplace(
          KeyFor(world_point, config_.map_voxel_size), world_point);
    }
    const double maximum_distance_squared =
        config_.maximum_map_distance * config_.maximum_map_distance;
    for (auto item = map_points_.begin(); item != map_points_.end();) {
      if ((item->second - translation).squaredNorm() > maximum_distance_squared) {
        item = map_points_.erase(item);
      } else {
        ++item;
      }
    }
    RebuildSearchIndex();
  }

  void RebuildSearchIndex() {
    search_points_.clear();
    search_points_.reserve(map_points_.size());
    search_cloud_->clear();
    search_cloud_->reserve(map_points_.size());
    for (const auto &item : map_points_) {
      const Eigen::Vector3d &world_point = item.second;
      search_points_.push_back(world_point);
      search_cloud_->push_back(ToSearchPoint(world_point));
    }
    search_cloud_->width = static_cast<std::uint32_t>(search_cloud_->size());
    search_cloud_->height = 1;
    search_cloud_->is_dense = true;
    search_tree_.setInputCloud(search_cloud_);
  }

  ImuConstrainedOdometryConfig config_;
  PointMap map_points_;
  std::vector<Eigen::Vector3d> search_points_;
  SearchCloud::Ptr search_cloud_{new SearchCloud};
  pcl::KdTreeFLANN<pcl::PointXYZ> search_tree_;
  bool initialized_ = false;
  Eigen::Vector3d last_translation_ = Eigen::Vector3d::Zero();
  std::uint64_t map_generation_ = 0;
};

}  // namespace danger_search_localization
