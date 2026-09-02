/*
 * Copyright 2026 The Openbot Authors
 *
 * Thin bridge so exploration code can use Costmap2D without heavy includes.
 */

#pragma once

#include <memory>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>

namespace autonomy::map::costmap_2d {
class Costmap2DWrapper;
}

namespace autonomy::map::common {

void FeedLaserScan(
    const std::shared_ptr<costmap_2d::Costmap2DWrapper>& wrapper,
    const automsgs::msgs::sensor_msgs::LaserScan& scan);

bool ApplyOccupancyGrid(
    const std::shared_ptr<costmap_2d::Costmap2DWrapper>& wrapper,
    const automsgs::msgs::map_msgs::OccupancyGrid& grid);

bool SnapshotOccupancyGrid(
    const std::shared_ptr<costmap_2d::Costmap2DWrapper>& wrapper,
    automsgs::msgs::map_msgs::OccupancyGrid& grid);

}  // namespace autonomy::map::common
