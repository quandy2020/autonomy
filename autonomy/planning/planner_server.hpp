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

// #include "cyber/cyber.h"
#include <string>
#include <unordered_map>

#include "class_loader/class_loader.hpp"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/transform/buffer.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/common/planner_server_interface.hpp"

namespace autonomy {
namespace planning {

class PlannerServer : common::PlannerServerInterface
{
public:
    using TfBuffer = autonomy::common::transform::Buffer;
    using PlannerMap = std::unordered_map<std::string, common::PlannerInterface::SharedPtr>;

    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerServer)

    /**
     * @brief A constructor for autonomy::planning::PlannerServer
     * @param options Additional options to control creation of the node.
     */
    explicit PlannerServer();

    /**
     * @brief
     */
    explicit PlannerServer(map::common::MapInterface::SharedPtr map);

    /**
     * @brief A Destructor for autonomy::planning::PlannerServer
     */
    ~PlannerServer();

    /**
     * @brief Method to get plan from the desired plugin
     * @param start starting pose
     * @param goal goal request
     * @param planner_id The planner to plan with
     * @param cancel_checker A function to check if the action has been canceled
     * @return Path
     */
    commsgs::planning_msgs::Path GetPlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id,
        std::function<bool()> cancel_checker);

protected:
    /**
     * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
     * clearing recovery. Blocks until true without timeout.
     */
    void WaitForCostmap();

    /**
     * @brief Transform start and goal poses into the costmap
     * global frame for path planning plugins to utilize
     * @param start The starting pose to transform
     * @param goal Goal pose to transform
     * @return bool If successful in transforming poses
     */
    bool TransformPosesToGlobalFrame(
        commsgs::geometry_msgs::PoseStamped& curr_start,
        commsgs::geometry_msgs::PoseStamped& curr_goal);

    /**
     * @brief Validate that the path contains a meaningful path
     * @param action_server Action server to terminate if required
     * @param goal Goal Current goal
     * @param path Current path
     * @param planner_id The planner ID used to generate the path
     * @return bool If path is valid
     */
    bool ValidatePath(
        const commsgs::geometry_msgs::PoseStamped& curr_goal,
        const commsgs::planning_msgs::Path& path,
        const std::string& planner_id);
        
    /**
     * @brief The action server callback which calls planner to get the path
     *  ComputePathToPose
     */
    void ComputePlan();

    /**
     * @brief The action server callback which calls planner to get the path
     * ComputePathThroughPoses
     */
    void ComputePlanThroughPoses();

    void ExceptionWarning(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id,
        const std::exception& ex);

    // All planners
    PlannerMap planners_;
    std::unique_ptr<class_loader::ClassLoader> gp_loader_{nullptr};

    std::vector<std::string> default_ids_;
    std::vector<std::string> default_types_;
    std::vector<std::string> planner_ids_;
    std::vector<std::string> planner_types_;
    double max_planner_duration_;
    std::string planner_ids_concat_;

    // TF buffer
    std::shared_ptr<TfBuffer> tf_{nullptr};

    // Global Costmap
    autonomy::map::costmap_2d::Costmap2D* costmap_{nullptr};
};

}  // namespace planning
}  // namespace autonomy