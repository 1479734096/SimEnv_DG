#pragma once

#include <cmath>

#include <Eigen/Core>

namespace danger_search_localization {

/// 检查有限 XYZ 点是否严格位于定位有效量程内。
///
/// 定位输入和注册地图输出复用同一过滤规则，避免近场无效回波被定位丢弃后仍
/// 写进 Python Mapper。参数非法时 fail-closed，不尝试交换上下界或使用默认值。
inline bool IsPointInLocalizationRange(
    const Eigen::Vector3d &point,
    double min_range,
    double max_range) {
  if (!point.allFinite() || !std::isfinite(min_range) || !std::isfinite(max_range) ||
      min_range < 0.0 || max_range < min_range) {
    return false;
  }
  const double distance = point.norm();
  return std::isfinite(distance) && distance > min_range && distance < max_range;
}

}  // namespace danger_search_localization
