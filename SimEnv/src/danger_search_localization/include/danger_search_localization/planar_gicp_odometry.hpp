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

struct PlanarGicpConfig {
  double voxel_size = 0.18;
  double maximum_correspondence_distance = 0.55;
  double maximum_frame_translation = 0.30;
  double maximum_fitness = 0.12;
  std::size_t minimum_points = 80;
  int maximum_iterations = 35;
};

struct PlanarGicpResult {
  bool accepted = false;
  bool has_delta = false;
  Eigen::Vector2d delta = Eigen::Vector2d::Zero();
  double fitness = 0.0;
  std::size_t point_count = 0;
};

// 输入点已由合法 IMU 旋到统一世界方向，因此这里只消费 GICP 的平面平移；
// 旋转与高度不会由退化点云反向污染。
class PlanarGicpOdometry {
 public:
  explicit PlanarGicpOdometry(
      const PlanarGicpConfig &config = PlanarGicpConfig())
      : config_(Validate(config)) {}

  PlanarGicpResult RegisterFrame(
      const std::vector<Eigen::Vector3d> &world_oriented_points,
      const Eigen::Vector2d &translation_prior = Eigen::Vector2d::Zero()) {
    if (!translation_prior.allFinite()) {
      return {false, false, Eigen::Vector2d::Zero(), 0.0, 0};
    }
    Cloud::Ptr current = Downsample(world_oriented_points);
    if (current->size() < config_.minimum_points) {
      return {false, false, Eigen::Vector2d::Zero(), 0.0, current->size()};
    }
    if (!previous_) {
      previous_ = current;
      return {true, false, Eigen::Vector2d::Zero(), 0.0, current->size()};
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
    const bool accepted = gicp.hasConverged() && delta.allFinite() &&
                          std::isfinite(fitness) &&
                          delta.norm() <= config_.maximum_frame_translation &&
                          fitness <= config_.maximum_fitness;
    // 当前帧证据充足但配准失败时仍推进参考帧，让另一传感器承担缺口；
    // 否则恢复后的长基线会永久无法重新进入局部配准域。
    previous_ = current;
    return {accepted, accepted, accepted ? delta : Eigen::Vector2d::Zero(),
            fitness, current->size()};
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
