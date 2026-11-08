/*
 * Copyright 2026 The Openbot Authors
 */

#include "frontend/odometry/downsample.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace frontend {
namespace odometry {
namespace {

TEST(Downsample, InitializationScanPreservesPointTimesAndSortsThem) {
  sensor::PointCloudXYZI cloud;
  sensor::PointType late_point;
  late_point.x = 0.1f;
  late_point.curvature = 0.08f;
  sensor::PointType early_point = late_point;
  early_point.x = 0.7f;
  early_point.curvature = 0.02f;
  cloud.push_back(late_point);
  cloud.push_back(early_point);

  DownsampleInitializationScan(&cloud, 0.5);

  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_FLOAT_EQ(cloud[0].curvature, 0.02f);
  EXPECT_FLOAT_EQ(cloud[1].curvature, 0.08f);
}

}  // namespace
}  // namespace odometry
}  // namespace frontend
}  // namespace atlas
