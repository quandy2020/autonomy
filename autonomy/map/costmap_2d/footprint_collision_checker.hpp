/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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
#include <vector>
#include <memory>
#include <algorithm>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

typedef std::vector<commsgs::geometry_msgs::Point> Footprint;

/**
 * @class FootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */
template<typename CostmapT>
class FootprintCollisionChecker
{
public:
    /**
     * @brief A constructor.
     */
    FootprintCollisionChecker();
    
    /**
     * @brief A constructor.
     */
    explicit FootprintCollisionChecker(CostmapT costmap);

    /**
     * @brief Find the footprint cost in oriented footprint
     */
    double footprintCost(const Footprint& footprint);

    /**
     * @brief Find the footprint cost a a post with an unoriented footprint
     */
    double footprintCostAtPose(double x, double y, double theta, const Footprint& footprint);

    /**
     * @brief Get the cost for a line segment
     */
    double lineCost(int x0, int x1, int y0, int y1) const;

    /**
     * @brief Get the map coordinates from a world point
     */
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

    /**
     * @brief Get the cost of a point
     */
    double pointCost(int x, int y) const;

    /**
     * @brief Set the current costmap object to use for collision detection
     */

    void setCostmap(CostmapT costmap);

    /**
     * @brief Get the current costmap object
     */
    CostmapT getCostmap()
    {
        return costmap_;
    }

protected:
    CostmapT costmap_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy