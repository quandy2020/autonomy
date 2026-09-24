/*
 * Copyright 2026 The Openbot Authors
 *
 * Body-frame lidar scan filter.
 */

#include "autonomy/localization/atlas/frontend/lidar/scan_filter.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

TEST(ScanFilterTest, DropsBlindRangeAndStrides) {
    PointCloud scan;
    for (int i = 0; i < 4; ++i) {
        PointXYZI near;
        near.x = 0.1f;
        near.y = 0.f;
        near.z = 0.f;
        scan.push_back(near);
    }
    for (int i = 0; i < 4; ++i) {
        PointXYZI far;
        far.x = 2.f + 0.1f * static_cast<float>(i);
        far.y = 0.f;
        far.z = 0.5f;
        scan.push_back(far);
    }
    FilterScan(&scan, 0.5, 2, -1.0, 2.0);
    ASSERT_EQ(scan.size(), 2u);
    for (const auto& point : scan) {
        EXPECT_GT(point.x, 1.f);
    }
}

}  // namespace
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
