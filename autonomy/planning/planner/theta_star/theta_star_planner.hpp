/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/planner/theta_star/theta_star.hpp"
#include "autonomy/planning/proto/theta_star_planner.pb.h"

namespace autonomy {
namespace planning {
namespace planner {
namespace theta_star {

class ThetaStarPlanner : public common::GlobalPlanner
{
public:
    /**
     * @brief Configure the planner
     * @param options The options to configure the planner with
     * @param name The name of the planner
     * @param costmap Pointer to the costmap (optional, can be nullptr if set
     * later)
     * @return True if the planner was successfully configured, false otherwise
     */
    bool Configure(const proto::PlannerOptions& options, const std::string& name,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap = nullptr) override;

    /**
     * @brief Cleanup the planner
     */
    void Cleanup() override;

    /**
     * @brief Activate the planner
     */
    void Activate() override;

    /**
     * @brief Deactivate the planner
     */
    void Deactivate() override;

    /**
     * @brief Creating a plan from start and goal poses
     * @param start Start pose
     * @param goal Goal pose
     * @param plan The plan... filled by the planner
     * @param cancel_checker Function to check if the action has been canceled
     * @return autonomy::commsgs::planning_msgs::Path of the generated path
     */
    uint32_t CreatePlan(const commsgs::geometry_msgs::PoseStamped& start,
                        const commsgs::geometry_msgs::PoseStamped& goal, commsgs::planning_msgs::Path& plan,
                        std::function<bool()> cancel_checker) override;

protected:
    std::string name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    std::unique_ptr<theta_star::ThetaStar> planner_;

    /**
     * @brief the function responsible for calling the algorithm and retrieving
     * a path from it
     * @param cancel_checker is a function to check if the action has been
     * canceled
     * @return global_path is the planned path to be taken
     */
    void getPlan(commsgs::planning_msgs::Path& global_path, std::function<bool()> cancel_checker);

    /**
     * @brief interpolates points between the consecutive waypoints of the path
     * @param raw_path is used to send in the path received from the planner
     * @param dist_bw_points is used to send in the interpolation_resolution
     * (which has been set as the costmap resolution)
     * @return the final path with waypoints at a distance of the value of
     * interpolation_resolution of each other
     */
    static commsgs::planning_msgs::Path linearInterpolation(const std::vector<coordsW>& raw_path,
                                                            const double& dist_bw_points);

    proto::ThetaStarPlanner options_;
};

}  // namespace theta_star
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
