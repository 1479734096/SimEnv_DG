#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace danger_search_localization {

// 将世界坐标系下的线速度转换到 base 坐标系。
// Odometry.child_frame_id="base"，因此 twist.linear 必须在 base 下表达。
inline Eigen::Vector3d WorldVelocityToBase(
    const Eigen::Vector3d &velocity_world,
    const Eigen::Matrix3d &world_base_rotation) {
  return world_base_rotation.inverse() * velocity_world;
}

}  // namespace danger_search_localization
