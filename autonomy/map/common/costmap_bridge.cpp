/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/map/common/costmap_bridge.hpp"

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy::map::common {

void FeedLaserScan(
    const std::shared_ptr<costmap_2d::Costmap2DWrapper>& wrapper,
    const automsgs::msgs::sensor_msgs::LaserScan& scan) {
    if (wrapper) {
        wrapper->feedLaserScan(scan);
    }
}

bool ApplyOccupancyGrid(
    const std::shared_ptr<costmap_2d::Costmap2DWrapper>& wrapper,
    const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    if (!wrapper) {
        return false;
    }
    return wrapper->applyOccupancyGrid(grid);
}

bool SnapshotOccupancyGrid(
    const std::shared_ptr<costmap_2d::Costmap2DWrapper>& wrapper,
    automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    if (!wrapper) {
        return false;
    }
    return wrapper->snapshotOccupancyGrid(grid);
}

}  // namespace autonomy::map::common
