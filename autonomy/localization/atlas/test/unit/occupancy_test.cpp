/*
 * Copyright 2026 The Openbot Authors
 *
 * Raycast occupancy grid.
 */

#include "autonomy/localization/atlas/map/occupancy/occupancy_map.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

TEST(OccupancyMapTest, RayMarksFreeAndHit) {
    OccupancyMap::Options options;
    options.resolution = 0.5;
    options.min_z = 0.0;
    options.max_z = 2.0;
    OccupancyMap map(options);

    PointCloud scan;
    PointXYZI hit;
    hit.x = 2.f;
    hit.y = 0.f;
    hit.z = 0.5f;
    scan.push_back(hit);
    map.Integrate(SE3Identity(), scan);

    automsgs::msgs::map_msgs::OccupancyGrid grid;
    ASSERT_TRUE(map.Fill(&grid, "map"));
    ASSERT_GT(grid.data_size(), 0);
    int occupied = 0;
    int free = 0;
    for (int value : grid.data()) {
        if (value == 100) {
            ++occupied;
        } else if (value == 0) {
            ++free;
        }
    }
    EXPECT_EQ(occupied, 1);
    EXPECT_GT(free, 0);
}

}  // namespace
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
