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


#include <memory>
#include <string>

#include "autonomy/map/costmap_2d/costmap_filters/costmap_filter.hpp"

// #include "nav2_msgs/msg/costmap_filter_info.hpp"
// #include "nav2_msgs/msg/speed_limit.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class SpeedFilter
 * @brief Reads in a speed restriction mask and enables a robot to
 * dynamically adjust speed based on pose in map to slow in dangerous
 * areas. Done via absolute speed setting or percentage of maximum speed
 */
class SpeedFilter : public CostmapFilter
{
public:
    /**
     * @brief A constructor
     */
    SpeedFilter();

    /**
     * @brief Initialize the filter and subscribe to the info topic
     */
    void initializeFilter(const std::string& filter_info_topic);

    /**
     * @brief Process the keepout layer at the current pose / bounds / grid
     */
    void process(Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j,
        const commsgs::geometry_msgs::Pose2D & pose);

    /**
     * @brief Reset the costmap filter / topic / info
     */
    void resetFilter();

    /**
     * @brief If this filter is active
     */
    bool isActive();

private:
    // /**
    //  * @brief Callback for the filter information
    //  */
    // void filterInfoCallback(const commsgs::map_msgs::CostmapFilterInfo::SharedPtr msg);

    /**
     * @brief Callback for the filter mask
     */
    void maskCallback(const commsgs::map_msgs::OccupancyGrid::SharedPtr msg);

    // rclcpp::Subscription<commsgs::map_msgs::CostmapFilterInfo>::SharedPtr filter_info_sub_;
    // rclcpp::Subscription<commsgs::map_msgs::OccupancyGrid>::SharedPtr mask_sub_;

    // rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

    commsgs::map_msgs::OccupancyGrid::SharedPtr filter_mask_;

    std::string global_frame_;  // Frame of currnet layer (master_grid)

    double base_, multiplier_;
    bool percentage_;
    double speed_limit_, speed_limit_prev_;
};


}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
