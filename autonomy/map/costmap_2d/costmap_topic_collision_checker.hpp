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
#include <vector>
#include <memory>
#include <algorithm>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class CostmapTopicCollisionChecker
 * @brief Using a costmap via a ros topic, this object is used to
 * find if robot poses are in collision with the costmap environment
 */
class CostmapTopicCollisionChecker
{
public:
    // /**
    //  * @brief A constructor
    //  */
    // CostmapTopicCollisionChecker(
    //     CostmapSubscriber& costmap_sub,
    //     FootprintSubscriber& footprint_sub,
    //     std::string name = "collision_checker");

    /**
     * @brief A destructor
     */
    ~CostmapTopicCollisionChecker() = default;

    /**
     * @brief Returns the obstacle footprint score for a particular pose
     *
     * @param pose Pose to get score at
     * @param fetch_costmap_and_footprint Defaults to true. When checking with multiple poses at once,
     * data should be fetched in the first check but fetching can be skipped in consequent checks for speedup
     */
    double scorePose(const commsgs::geometry_msgs::Pose2D& pose, bool fetch_costmap_and_footprint = true);

    /**
     * @brief Returns if a pose is collision free
     *
     * @param pose Pose to check collision at
     * @param fetch_costmap_and_footprint Defaults to true. When checking with multiple poses at once,
     * data should be fetched in the first check but fetching can be skipped in consequent checks for speedup
     */
    bool isCollisionFree(const commsgs::geometry_msgs::Pose2D& pose, bool fetch_costmap_and_footprint = true);

protected:
    /**
     * @brief Get a footprint at a set pose
     *
     * @param pose Pose to get footprint at
     * @param fetch_latest_footprint Defaults to true. When checking with multiple poses at once,
     * footprint should be fetched in the first check but fetching can be skipped in consequent checks for speedup
     */
    Footprint getFootprint(const commsgs::geometry_msgs::Pose2D& pose, bool fetch_latest_footprint = true);

    // Name used for logging
    std::string name_;
    // CostmapSubscriber & costmap_sub_;
    // FootprintSubscriber & footprint_sub_;
    FootprintCollisionChecker<std::shared_ptr<Costmap2D>> collision_checker_;
    Footprint footprint_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy