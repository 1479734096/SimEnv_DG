#pragma once

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <random>
#include <vector>

namespace danger_search_localization {

/// 首帧楼面 RANSAC 的候选范围、几何阈值和最小证据量。
struct GroundPlaneOptions {
  double minimum_candidate_z = -1.0;
  double maximum_candidate_z = 0.20;
  double minimum_planar_range = 0.25;
  double maximum_planar_range = 6.0;
  double inlier_distance = 0.04;
  double maximum_tilt_radians = 0.75;
  double minimum_distance_from_base = 0.05;
  double maximum_distance_from_base = 0.80;
  std::size_t minimum_inliers = 300;
  double minimum_inlier_ratio = 0.20;
  std::size_t ransac_iterations = 256;
};

/// 由当前 Livox 帧估计的 base 系楼面法向与机身离地距离。
struct GroundPlaneEstimate {
  Eigen::Vector3d normal_base = Eigen::Vector3d::UnitZ();
  double distance_from_base = 0.0;
  std::size_t inlier_count = 0;
  std::size_t candidate_count = 0;
};

/// 从 base 系点云估计当前楼面；证据不足或几何不合理时返回空值。
///
/// 算法先在预期高度/量程内做固定种子 RANSAC，再用最佳内点的协方差最小
/// 特征向量精炼法向。法向必须朝上且倾角、离地距离、内点数和比例同时通过，
/// 防止把墙面、台阶边缘或稀疏家具表面误当作出生楼面。
inline std::optional<GroundPlaneEstimate> EstimateGroundPlane(
    const std::vector<Eigen::Vector3d> &base_points,
    const GroundPlaneOptions &options = GroundPlaneOptions()) {
  if (!std::isfinite(options.inlier_distance) || options.inlier_distance <= 0.0 ||
      !std::isfinite(options.maximum_tilt_radians) ||
      options.maximum_tilt_radians <= 0.0 ||
      options.maximum_tilt_radians >= M_PI_2 ||
      options.minimum_inliers < 3 || options.ransac_iterations == 0) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> candidates;
  candidates.reserve(base_points.size());
  for (const auto &point : base_points) {
    if (!point.allFinite() || point.z() < options.minimum_candidate_z ||
        point.z() > options.maximum_candidate_z) {
      continue;
    }
    const double planar_range = std::hypot(point.x(), point.y());
    if (planar_range < options.minimum_planar_range ||
        planar_range > options.maximum_planar_range) {
      continue;
    }
    candidates.push_back(point);
  }
  if (candidates.size() < options.minimum_inliers) {
    return std::nullopt;
  }

  const double minimum_normal_z = std::cos(options.maximum_tilt_radians);
  std::mt19937 generator(42U);
  std::uniform_int_distribution<std::size_t> index(0, candidates.size() - 1);
  Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
  double best_offset = 0.0;
  std::size_t best_count = 0;
  for (std::size_t iteration = 0; iteration < options.ransac_iterations; ++iteration) {
    const std::size_t i0 = index(generator);
    const std::size_t i1 = index(generator);
    const std::size_t i2 = index(generator);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }
    Eigen::Vector3d normal =
        (candidates[i1] - candidates[i0]).cross(candidates[i2] - candidates[i0]);
    const double norm = normal.norm();
    if (!std::isfinite(norm) || norm <= 1e-9) {
      continue;
    }
    normal /= norm;
    if (normal.z() < 0.0) {
      normal = -normal;
    }
    if (normal.z() < minimum_normal_z) {
      continue;
    }
    const double offset = -normal.dot(candidates[i0]);
    std::size_t count = 0;
    for (const auto &point : candidates) {
      if (std::abs(normal.dot(point) + offset) <= options.inlier_distance) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best_normal = normal;
      best_offset = offset;
    }
  }

  const std::size_t required_by_ratio = static_cast<std::size_t>(std::ceil(
      std::clamp(options.minimum_inlier_ratio, 0.0, 1.0) * candidates.size()));
  const std::size_t required_count = std::max(options.minimum_inliers, required_by_ratio);
  if (best_count < required_count) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(best_count);
  for (const auto &point : candidates) {
    if (std::abs(best_normal.dot(point) + best_offset) <= options.inlier_distance) {
      inliers.push_back(point);
    }
  }
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto &point : inliers) {
    centroid += point;
  }
  centroid /= static_cast<double>(inliers.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto &point : inliers) {
    const Eigen::Vector3d centered = point - centroid;
    covariance += centered * centered.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }
  Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  const double distance_from_base = -normal.dot(centroid);
  if (normal.z() < minimum_normal_z || !std::isfinite(distance_from_base) ||
      distance_from_base < options.minimum_distance_from_base ||
      distance_from_base > options.maximum_distance_from_base) {
    return std::nullopt;
  }

  std::size_t refined_count = 0;
  for (const auto &point : candidates) {
    if (std::abs(normal.dot(point) + distance_from_base) <= options.inlier_distance) {
      ++refined_count;
    }
  }
  if (refined_count < required_count) {
    return std::nullopt;
  }
  return GroundPlaneEstimate{
      normal, distance_from_base, refined_count, candidates.size()};
}

/// 用实测楼面确定 roll/pitch/z，同时保持公开起点的 world x/y/yaw。
///
/// 这一步把“公开场景提供的平面起点”和“传感器测得的机身离地姿态”组合为
/// 初始 world->base，不使用 Gazebo 出生高度或真值 TF。
inline Eigen::Isometry3d BuildWorldBaseFromGroundPlane(
    const Eigen::Vector3d &normal_base, double distance_from_base,
    double world_x, double world_y, double requested_yaw) {
  const Eigen::Vector3d unit_normal = normal_base.normalized();
  const Eigen::Matrix3d align_ground =
      Eigen::Quaterniond::FromTwoVectors(unit_normal, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d aligned_forward = align_ground * Eigen::Vector3d::UnitX();
  const double aligned_yaw = std::atan2(aligned_forward.y(), aligned_forward.x());
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(requested_yaw - aligned_yaw, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      align_ground;
  Eigen::Isometry3d world_base = Eigen::Isometry3d::Identity();
  world_base.linear() = rotation;
  world_base.translation() = Eigen::Vector3d(world_x, world_y, distance_from_base);
  return world_base;
}

}  // namespace danger_search_localization
