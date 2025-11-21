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

#include <string>
#include <unordered_map>

#include "autonomy/planning/proto/planning_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/class_loader/class_loader.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/plugins/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/plugins/navfn/navfn_planner.hpp"

namespace autonomy {
namespace planning {

class PlannerServer
{
public:

    /**  
     * Define Buffer type
     */
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define ClassLoader type
     */
    using ClassLoader = autonomy::common::class_loader::ClassLoader;

    /**
     * Define PlannerMap type
     */
    using PlannerMap = std::unordered_map<std::string, common::PlannerInterface::SharedPtr>;

    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerServer)

    /**
     * @brief A constructor for autonomy::planning::PlannerServer
     * @param options Additional options to control creation of the node.
     */
    explicit PlannerServer(const proto::PlannerOptions& options, TfBuffer* tf_buffer);

    /**
     * @brief
     */
    explicit PlannerServer(map::common::MapInterface::SharedPtr map);

    /**
     * @brief A Destructor for autonomy::planning::PlannerServer
     */
    ~PlannerServer();

    /**
     * @brief Starts planning tasks
     */
    void Start();

    /**
     * @brief Shutdown planning tasks
     */
    void WaitForShutdown();

    /**
     * @brief Configures the planner server with the given options
     * @return True if the planner server was successfully configured, false otherwise
     */
    bool Configure();

    /**
     * @brief Activates the planner server
     * @return True if the planner server was successfully activated, false otherwise
     */
    bool Activate();

    /**
     * @brief Deactivates the planner server
     * @return True if the planner server was successfully deactivated, false otherwise
     */
    bool Deactivate();

    /**
     * @brief Cleans up the planner server
     * @return True if the planner server was successfully cleaned up, false otherwise
     */
    bool Cleanup();

    /**
     * @brief Shutdown the planner server
     * @return True if the planner server was successfully shut down, false otherwise
     */
    bool Shutdown();

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

    /**
     * @brief Get trajectory_plan
     */
    commsgs::planning_msgs::Path* trajectory_plan() { return trajectory_plan_.get(); }

    /**
     * @brief Get costmap_2d map
     */
    autonomy::map::costmap_2d::Costmap2D* costmap() { return costmap_; }

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
    void ComputePlan(const commsgs::geometry_msgs::PoseStamped& request, commsgs::planning_msgs::Path& response);

    /**
     * @brief The action server callback which calls planner to get the path
     * ComputePathThroughPoses
     */
    void ComputePlanThroughPoses(const commsgs::geometry_msgs::PoseStamped& request, commsgs::planning_msgs::Path& response);

    /**
     * @brief Publish a path for visualization purposes
     * @param path Reference to Global Path
     */
    void PublishPlan(const commsgs::planning_msgs::Path& path);

    /**
     * @brief Print wanning info
     */
    void ExceptionWarning(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id,
        const std::exception& ex);

    // Options planners
    proto::PlannerOptions options_;

    // trajectory_plan
    commsgs::planning_msgs::Path::SharedPtr trajectory_plan_{nullptr};

    // All planners
    PlannerMap planners_;
    std::unique_ptr<ClassLoader> gp_loader_{nullptr};

    std::vector<std::string> default_ids_;
    std::vector<std::string> default_types_;
    std::vector<std::string> planner_ids_;
    std::vector<std::string> planner_types_;
    double max_planner_duration_;
    std::string planner_ids_concat_;

    // TF buffer
    TfBuffer* tf_{nullptr};

    //     compute_path_to_pose_service_{nullptr};


    // Global Costmap
    map::costmap_2d::Costmap2D* costmap_{nullptr};
    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_{nullptr};
    std::unique_ptr<map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>>  collision_checker_{nullptr};

    // Thread
    std::unique_ptr<std::thread> compute_path_to_pose_thread_{nullptr};
    std::unique_ptr<std::thread> compute_path_through_poses_thread_{nullptr};
};

}  // namespace planning
}  // namespace autonomy