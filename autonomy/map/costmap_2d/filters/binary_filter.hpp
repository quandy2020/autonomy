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

#include <functional>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/costmap.pb.h>
#include "autonomy/map/costmap_2d/filters/costmap_filter.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class BinaryFilter
 * @brief Reads a binary mask and flips a boolean state when the robot enters
 * regions where (base + mask_data * multiplier) exceeds flip_threshold_.
 */
class BinaryFilter : public CostmapFilter
{
public:
    using BinaryStateCallback = std::function<void(bool state)>;

    BinaryFilter();

    void initializeFilter(const std::string& filter_info_topic) override;

    void process(Costmap2D& master_grid, int min_i, int min_j, int max_i,
                 int max_j,
                 const automsgs::msgs::geometry_msgs::Pose2D& pose) override;

    void resetFilter() override;

    bool isActive();

    void handleFilterInfo(
        const std::shared_ptr<automsgs::msgs::nav_msgs::CostmapFilterInfo>& msg);

    void setFilterMask(const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& msg);

    void applyConfiguration(
        const std::shared_ptr<automsgs::msgs::nav_msgs::CostmapFilterInfo>& info,
        const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& mask);

    static std::shared_ptr<automsgs::msgs::nav_msgs::CostmapFilterInfo>
    makeDefaultFilterInfo(const std::string& mask_topic, float base = 0.0f,
                          float multiplier = 1.0f);

    bool hasFilterMask();

    const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& getFilterMask() const {
        return filter_mask_;
    }

    void setBinaryStateCallback(BinaryStateCallback callback);

    bool getBinaryState();

    void setDefaultState(bool default_state);

    bool getDefaultState();

    void setFlipThreshold(double threshold);

    double getFlipThreshold();

    bool isFilterConfigured();

private:
    void filterInfoCallback(
        const std::shared_ptr<automsgs::msgs::nav_msgs::CostmapFilterInfo> msg);

    void maskCallback(const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> msg);

    static bool validateFilterMask(
        const automsgs::msgs::map_msgs::OccupancyGrid& msg);

    std::string resolveMaskFrame() const;

    bool evaluateMaskCell(int8_t mask_data) const;

    void changeState(bool state);

    std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> filter_mask_;

    std::string global_frame_;
    std::string mask_frame_;
    double base_{0.0};
    double multiplier_{1.0};
    double flip_threshold_{50.0};
    bool default_state_{false};
    bool binary_state_{false};
    bool mask_missing_warned_{false};
    bool unknown_cell_warned_{false};
    bool outside_mask_warned_{false};
    bool filter_info_received_{false};
    BinaryStateCallback binary_state_callback_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
