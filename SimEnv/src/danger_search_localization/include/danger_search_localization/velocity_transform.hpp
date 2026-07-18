#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace danger_search_localization {

/// 将 world 下的线速度转换到 base 坐标系。
///
/// 发布的 Odometry 使用 ``child_frame_id=base``，ROS 约定 twist 必须在 child
/// frame 表达；这里使用 world->base 旋转的逆变换，避免下游把世界速度再次旋转。
inline Eigen::Vector3d WorldVelocityToBase(
    const Eigen::Vector3d &velocity_world,
    const Eigen::Matrix3d &world_base_rotation) {
  return world_base_rotation.inverse() * velocity_world;
}

}  // namespace danger_search_localization
