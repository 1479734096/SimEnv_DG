#pragma once

#include <cmath>

#include <Eigen/Core>

namespace danger_search_localization {

// 定位和注册地图输出必须使用同一有效量程，避免传感器近距离无效回波写入占据地图。
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
