#include <gtest/gtest.h>

#include <limits>

#include <danger_search_localization/point_range_filter.hpp>
#include <Eigen/Core>

namespace danger_search_localization {
namespace {

TEST(PointRangeFilter, RetainsOnlyPointsInTheLocalizationRange) {
  const Eigen::Vector3d below_minimum(0.499, 0.0, 0.0);
  const Eigen::Vector3d at_minimum(0.5, 0.0, 0.0);
  const Eigen::Vector3d at_maximum(0.0, 15.0, 0.0);
  const Eigen::Vector3d above_maximum(15.001, 0.0, 0.0);

  EXPECT_FALSE(IsPointInLocalizationRange(below_minimum, 0.5, 15.0));
  EXPECT_FALSE(IsPointInLocalizationRange(at_minimum, 0.5, 15.0));
  EXPECT_FALSE(IsPointInLocalizationRange(at_maximum, 0.5, 15.0));
  EXPECT_FALSE(IsPointInLocalizationRange(above_maximum, 0.5, 15.0));
}

TEST(PointRangeFilter, RejectsNonFiniteAndInvalidRangeConfiguration) {
  const Eigen::Vector3d finite_point(1.0, 0.0, 0.0);
  const Eigen::Vector3d non_finite_point(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);

  EXPECT_FALSE(IsPointInLocalizationRange(non_finite_point, 0.5, 15.0));
  EXPECT_FALSE(IsPointInLocalizationRange(finite_point, -0.1, 15.0));
  EXPECT_FALSE(IsPointInLocalizationRange(finite_point, 2.0, 1.0));
}

}  // namespace
}  // namespace danger_search_localization
