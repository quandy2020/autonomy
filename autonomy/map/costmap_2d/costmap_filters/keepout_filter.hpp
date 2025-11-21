/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <string>
#include <memory>

#include "autonomy/map/costmap_2d/costmap_filters/costmap_filter.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class KeepoutFilter
 * @brief Reads in a keepout mask and marks keepout regions in the map
 * to prevent planning or control in restricted areas
 */
class KeepoutFilter : public CostmapFilter
{
public:
    /**
     * @brief A constructor
     */
    KeepoutFilter();

    /**
     * @brief Initialize the filter and subscribe to the info topic
     */
    void initializeFilter(const std::string& filter_info_topic);

    /**
     * @brief Process the keepout layer at the current pose / bounds / grid
     */
    void process(Costmap2D& master_grid,
        int min_i, int min_j, int max_i, int max_j,
        const commsgs::geometry_msgs::Pose2D& pose);

    /**
     * @brief Reset the costmap filter / topic / info
     */
    void resetFilter();

    /**
     * @brief If this filter is active
     */
    bool isActive();

private:
    /**
     * @brief Callback for the filter information
     */
    void filterInfoCallback(const commsgs::planning_msgs::CostmapFilterInfo::SharedPtr msg);
    
    /**
     * @brief Callback for the filter mask
     */
    void maskCallback(const commsgs::map_msgs::OccupancyGrid::SharedPtr msg);

    // rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

    commsgs::map_msgs::OccupancyGrid::SharedPtr filter_mask_;

    std::string global_frame_;  // Frame of currnet layer (master_grid)
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
