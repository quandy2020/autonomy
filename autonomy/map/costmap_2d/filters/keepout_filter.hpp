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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/costmap_2d/filters/costmap_filter.hpp"

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
    KeepoutFilter();

    void initializeFilter(const std::string& filter_info_topic) override;

    void process(Costmap2D& master_grid, int min_i, int min_j, int max_i,
                 int max_j,
                 const commsgs::geometry_msgs::Pose2D& pose) override;

    void resetFilter() override;

    bool isActive();

    /**
     * @brief Inject filter info (replaces subscription callback).
     */
    void handleFilterInfo(
        const commsgs::map_msgs::CostmapFilterInfo::SharedPtr& msg);

    /**
     * @brief Inject filter mask OccupancyGrid directly.
     */
    void setFilterMask(const commsgs::map_msgs::OccupancyGrid::SharedPtr& msg);

    /**
     * @brief One-shot setup for offline / test pipelines (info then mask).
     */
    void applyConfiguration(
        const commsgs::map_msgs::CostmapFilterInfo::SharedPtr& info,
        const commsgs::map_msgs::OccupancyGrid::SharedPtr& mask);

    /**
     * @brief Build default keepout filter info for a given mask topic.
     */
    static commsgs::map_msgs::CostmapFilterInfo::SharedPtr
    makeDefaultFilterInfo(const std::string& mask_topic);

    bool hasFilterMask();

    const commsgs::map_msgs::OccupancyGrid::SharedPtr& getFilterMask() const {
        return filter_mask_;
    }

    /**
     * @brief Occupancy value (0–100) at or above which a mask cell is keepout.
     */
    void setOccupancyThreshold(int8_t threshold) {
        occupancy_threshold_ = threshold;
    }

    int8_t getOccupancyThreshold() const {
        return occupancy_threshold_;
    }

private:
    void filterInfoCallback(
        const commsgs::map_msgs::CostmapFilterInfo::SharedPtr msg);

    void maskCallback(const commsgs::map_msgs::OccupancyGrid::SharedPtr msg);

    bool lookupGlobalToMaskTransform(const std::string& mask_frame,
                                     transform::tf2::Transform& out) const;

    void computeIterationBounds(Costmap2D& master_grid, int min_i, int min_j,
                                int max_i, int max_j,
                                const std::string& mask_frame, int& mg_min_x,
                                int& mg_min_y, int& mg_max_x,
                                int& mg_max_y) const;

    static bool validateFilterMask(
        const commsgs::map_msgs::OccupancyGrid& msg);

    std::string resolveMaskFrame() const;

    unsigned char keepoutCostFromMaskCell(unsigned int mx, unsigned int my) const;

    commsgs::map_msgs::OccupancyGrid::SharedPtr filter_mask_;

    std::string global_frame_;
    int8_t occupancy_threshold_{50};
    bool mask_missing_warned_{false};
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
