#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
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

/// 一帧局部地图配准的验收结果；拒绝时仍返回安全的上一平移估计。
struct ImuConstrainedOdometryResult {
  bool accepted = false;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  std::size_t correspondence_count = 0;
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
      const Eigen::Vector3d &translation_prior) {
    // 两级体素分别控制地图密度与当前帧计算量；首个有效帧只初始化局部地图，
    // 没有上一地图时不虚构相对运动。
    const auto map_frame = kiss_icp::VoxelDownsample(
        world_oriented_vectors, config_.map_voxel_size);
    const auto source = kiss_icp::VoxelDownsample(
        map_frame, config_.source_voxel_size);
    if (source.size() < config_.minimum_correspondences) {
      return Rejected(translation_prior, 0);
    }

    if (!initialized_) {
      UpdateMap(map_frame, translation_prior);
      last_translation_ = translation_prior;
      initialized_ = true;
      return {true, translation_prior, source.size()};
    }

    Eigen::Vector3d estimate = translation_prior;
    // 当前楼面观测负责机身高度；扫描匹配只估计平面平移，避免地面点或
    // 竖直墙面把 z 与 xy 的退化方向互相污染。
    estimate.z() = translation_prior.z();
    std::size_t retained_count = 0;
    bool has_valid_iteration = false;
    for (int iteration = 0; iteration < config_.maximum_iterations; ++iteration) {
      std::vector<Eigen::Vector3d> world_source;
      world_source.reserve(source.size());
      for (const auto &point : source) {
        world_source.push_back(point + estimate);
      }
      const auto correspondences = FindCorrespondences(world_source);
      const auto &matched_source = correspondences.first;
      const auto &matched_target = correspondences.second;
      if (matched_source.size() < config_.minimum_correspondences) {
        break;
      }

      std::vector<double> residual_norms;
      residual_norms.reserve(matched_source.size());
      for (std::size_t index = 0; index < matched_source.size(); ++index) {
        residual_norms.push_back(
            (matched_target[index] - matched_source[index]).head<2>().norm());
      }
      const double quantile = Quantile(residual_norms, config_.residual_quantile);
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

      Eigen::Vector2d delta(Median(residual_x), Median(residual_y));
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
      return Rejected(translation_prior, retained_count);
    }

    Eigen::Vector2d correction =
        estimate.head<2>() - translation_prior.head<2>();
    if (correction.norm() > config_.maximum_correction_from_prior) {
      correction *= config_.maximum_correction_from_prior / correction.norm();
      estimate.x() = translation_prior.x() + correction.x();
      estimate.y() = translation_prior.y() + correction.y();
    }

    UpdateMap(map_frame, estimate);
    last_translation_ = estimate;
    return {true, estimate, retained_count};
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
      const Eigen::Vector3d &translation_prior,
      std::size_t correspondence_count) const {
    Eigen::Vector3d translation = initialized_
                                      ? last_translation_
                                      : translation_prior;
    translation.z() = translation_prior.z();
    return {false, translation, correspondence_count};
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
};

}  // namespace danger_search_localization
