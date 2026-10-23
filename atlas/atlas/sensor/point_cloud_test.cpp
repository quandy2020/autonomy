#include "sensor/point_cloud.hpp"

#include <gtest/gtest.h>

#include "sensor/internal/voxel_filter.hpp"
#include "sensor/range_data.hpp"

namespace atlas {
namespace sensor {
namespace {

TEST(PointCloud, TransformAndCrop) {
  PointCloud cloud({{RangefinderPoint{Eigen::Vector3f(1.f, 0.f, 1.f)}},
                    {RangefinderPoint{Eigen::Vector3f(1.f, 0.f, 5.f)}}},
                   {2.f, 3.f});
  const transform::Rigid3f shift =
      transform::Rigid3f::Translation(Eigen::Vector3f(0.f, 0.f, 1.f));
  const PointCloud moved = TransformPointCloud(cloud, shift);
  EXPECT_FLOAT_EQ(moved[0].position.z(), 2.f);
  EXPECT_FLOAT_EQ(moved.intensities()[1], 3.f);
  const PointCloud cropped = CropPointCloud(moved, 0.f, 3.f);
  ASSERT_EQ(cropped.size(), 1u);
  EXPECT_FLOAT_EQ(cropped.intensities()[0], 2.f);
}

TEST(PointCloud, TimedTransformKeepsRelativeTime) {
  TimedPointCloud cloud{{Eigen::Vector3f(1.f, 0.f, 0.f), -0.1f}};
  const TimedPointCloud moved = TransformTimedPointCloud(
      cloud, transform::Rigid3f::Translation(Eigen::Vector3f(1.f, 0.f, 0.f)));
  EXPECT_FLOAT_EQ(moved[0].position.x(), 2.f);
  EXPECT_FLOAT_EQ(moved[0].time, -0.1f);
}

TEST(RangeData, TransformMovesOriginAndReturns) {
  RangeData data;
  data.origin = Eigen::Vector3f(1.f, 0.f, 0.f);
  data.returns = PointCloud({RangefinderPoint{Eigen::Vector3f(2.f, 0.f, 0.f)}});
  const RangeData moved = TransformRangeData(
      data, transform::Rigid3f::Translation(Eigen::Vector3f(0.f, 3.f, 0.f)));
  EXPECT_FLOAT_EQ(moved.origin.y(), 3.f);
  EXPECT_FLOAT_EQ(moved.returns[0].position.y(), 3.f);
}

TEST(VoxelFilter, CollapsesPointsInOneCell) {
  PointCloud cloud({RangefinderPoint{Eigen::Vector3f(0.1f, 0.1f, 0.1f)},
                    RangefinderPoint{Eigen::Vector3f(0.2f, 0.1f, 0.1f)},
                    RangefinderPoint{Eigen::Vector3f(2.f, 0.f, 0.f)}});
  const PointCloud filtered = VoxelFilter(cloud, 1.f);
  EXPECT_EQ(filtered.size(), 2u);
}

TEST(VoxelFilter, AdaptiveDropsFarPoints) {
  PointCloud cloud({RangefinderPoint{Eigen::Vector3f(0.f, 0.f, 0.f)},
                    RangefinderPoint{Eigen::Vector3f(100.f, 0.f, 0.f)}});
  AdaptiveVoxelFilterOptions options;
  options.max_range = 10.f;
  options.min_num_points = 1.f;
  options.max_length = 1.f;
  const PointCloud filtered = AdaptiveVoxelFilter(cloud, options);
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_LT(filtered[0].position.norm(), 10.f);
}

}  // namespace
}  // namespace sensor
}  // namespace atlas
