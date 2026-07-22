#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace danger_search_localization {

/// 相邻帧 GICP 的降采样、对应距离、运动上限和质量门槛。
struct PlanarGicpConfig {
  double voxel_size = 0.18;
  double maximum_correspondence_distance = 0.55;
  double maximum_frame_translation = 0.30;
  double maximum_fitness = 0.12;
  std::size_t minimum_points = 80;
  int maximum_iterations = 35;
};

/// 相邻帧配准的稳定诊断状态；只描述验收结果，不参与融合决策。
enum class PlanarGicpStatus {
  kAccepted = 0,
  kInitialFrame,
  kInvalidPrior,
  kInsufficientPoints,
  kNotConverged,
  kNonFiniteDelta,
  kNonFiniteFitness,
  kTranslationLimit,
  kFitnessLimit,
  kCount,
};

inline const char *PlanarGicpStatusName(PlanarGicpStatus status) {
  switch (status) {
    case PlanarGicpStatus::kAccepted:
      return "accepted";
    case PlanarGicpStatus::kInitialFrame:
      return "initial_frame";
    case PlanarGicpStatus::kInvalidPrior:
      return "invalid_prior";
    case PlanarGicpStatus::kInsufficientPoints:
      return "insufficient_points";
    case PlanarGicpStatus::kNotConverged:
      return "not_converged";
    case PlanarGicpStatus::kNonFiniteDelta:
      return "nonfinite_delta";
    case PlanarGicpStatus::kNonFiniteFitness:
      return "nonfinite_fitness";
    case PlanarGicpStatus::kTranslationLimit:
      return "translation_limit";
    case PlanarGicpStatus::kFitnessLimit:
      return "fitness_limit";
    case PlanarGicpStatus::kCount:
      return "count";
  }
  return "unknown";
}

/// 相邻两帧的平面位移；首帧 accepted 但 ``has_delta=false``。
struct PlanarGicpResult {
  bool accepted = false;
  bool has_delta = false;
  Eigen::Vector2d delta = Eigen::Vector2d::Zero();
  Eigen::Vector2d candidate_delta = Eigen::Vector2d::Zero();
  double fitness = 0.0;
  std::size_t point_count = 0;
  PlanarGicpStatus status = PlanarGicpStatus::kInsufficientPoints;
};

/// 对已统一旋转的相邻点云执行 GICP，并且只消费平面平移。
///
/// 输入点已由合法 IMU 旋到统一 world 方向，GICP 的旋转与高度结果不会反向
/// 污染姿态。命令积分只作为初始猜测；最终位移还必须通过收敛、fitness 和
/// 单帧位移上限。配准失败仍推进参考帧，避免恢复时长基线永久落在局部配准域外。
class PlanarGicpOdometry {
 public:
  explicit PlanarGicpOdometry(
      const PlanarGicpConfig &config = PlanarGicpConfig())
      : config_(Validate(config)) {}

  PlanarGicpResult RegisterFrame(
      const std::vector<Eigen::Vector3d> &world_oriented_points,
      const Eigen::Vector2d &translation_prior = Eigen::Vector2d::Zero()) {
    if (!translation_prior.allFinite()) {
      return {false, false, Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero(), 0.0, 0,
              PlanarGicpStatus::kInvalidPrior};
    }
    Cloud::Ptr current = Downsample(world_oriented_points);
    if (current->size() < config_.minimum_points) {
      return {false, false, Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero(), 0.0, current->size(),
              PlanarGicpStatus::kInsufficientPoints};
    }
    if (!previous_) {
      previous_ = current;
      return {true, false, Eigen::Vector2d::Zero(),
              Eigen::Vector2d::Zero(), 0.0, current->size(),
              PlanarGicpStatus::kInitialFrame};
    }

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(config_.maximum_iterations);
    gicp.setMaxCorrespondenceDistance(
        config_.maximum_correspondence_distance);
    gicp.setTransformationEpsilon(1e-4);
    gicp.setEuclideanFitnessEpsilon(1e-4);
    gicp.setInputSource(current);
    gicp.setInputTarget(previous_);
    Cloud aligned;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess(0, 3) = static_cast<float>(translation_prior.x());
    initial_guess(1, 3) = static_cast<float>(translation_prior.y());
    gicp.align(aligned, initial_guess);
    const Eigen::Matrix4f relative = gicp.getFinalTransformation();
    const Eigen::Vector2d delta =
        relative.block<2, 1>(0, 3).cast<double>();
    const double fitness = gicp.getFitnessScore(
        config_.maximum_correspondence_distance);
    PlanarGicpStatus status = PlanarGicpStatus::kAccepted;
    if (!gicp.hasConverged()) {
      status = PlanarGicpStatus::kNotConverged;
    } else if (!delta.allFinite()) {
      status = PlanarGicpStatus::kNonFiniteDelta;
    } else if (!std::isfinite(fitness)) {
      status = PlanarGicpStatus::kNonFiniteFitness;
    } else if (delta.norm() > config_.maximum_frame_translation) {
      status = PlanarGicpStatus::kTranslationLimit;
    } else if (fitness > config_.maximum_fitness) {
      status = PlanarGicpStatus::kFitnessLimit;
    }
    const bool accepted = status == PlanarGicpStatus::kAccepted;
    // 当前帧证据充足但配准失败时仍推进参考帧，让另一传感器承担缺口；
    // 否则恢复后的长基线会永久无法重新进入局部配准域。
    previous_ = current;
    return {accepted, accepted, accepted ? delta : Eigen::Vector2d::Zero(),
            delta, fitness, current->size(), status};
  }

 private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  static PlanarGicpConfig Validate(const PlanarGicpConfig &config) {
    if (!std::isfinite(config.voxel_size) || config.voxel_size <= 0.0 ||
        !std::isfinite(config.maximum_correspondence_distance) ||
        config.maximum_correspondence_distance <= 0.0 ||
        !std::isfinite(config.maximum_frame_translation) ||
        config.maximum_frame_translation <= 0.0 ||
        !std::isfinite(config.maximum_fitness) ||
        config.maximum_fitness <= 0.0 || config.minimum_points == 0 ||
        config.maximum_iterations <= 0) {
      throw std::invalid_argument("平面 GICP 参数无效");
    }
    return config;
  }

  Cloud::Ptr Downsample(
      const std::vector<Eigen::Vector3d> &points) const {
    Cloud::Ptr raw(new Cloud);
    raw->reserve(points.size());
    for (const auto &point : points) {
      if (!point.allFinite()) {
        continue;
      }
      raw->emplace_back(
          static_cast<float>(point.x()), static_cast<float>(point.y()),
          static_cast<float>(point.z()));
    }
    Cloud::Ptr filtered(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    const float leaf = static_cast<float>(config_.voxel_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(raw);
    voxel.filter(*filtered);
    return filtered;
  }

  PlanarGicpConfig config_;
  Cloud::Ptr previous_;
};

}  // namespace danger_search_localization
