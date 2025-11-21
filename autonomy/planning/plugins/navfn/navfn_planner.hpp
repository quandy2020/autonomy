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

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "autonomy/planning/proto/navfn_planner.pb.h"

// #include "class_loader/class_loader.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/planning/plugins/navfn/navfn.hpp"
#include "autonomy/planning/common/planner_interface.hpp"

namespace autonomy {
namespace planning {
namespace plugins {
namespace navfn {

class NavfnPlanner : public common::PlannerInterface
{
public:

    /**
     * @brief Define Buffer type
     */
    using Buffer = autonomy::transform::Buffer;

    /**
     * @brief constructor
     */
    NavfnPlanner();

    /**
     * @brief Configuring plugin
     * @param parent Lifecycle node pointer
     * @param name Name of plugin map
     * @param tf Shared ptr of TF2 buffer
     * @param costmap_ros Costmap2DROS object
     */
    NavfnPlanner(
        std::string name, std::shared_ptr<Buffer> tf,
        std::shared_ptr<map::costmap_2d::Costmap2D> costmap);
    
    /**
     * @brief destructor
     */
    ~NavfnPlanner();

    /**
     * @brief Creating a plan from start and goal poses
     * @param start Start pose
     * @param goal Goal pose
     * @param cancel_checker Function to check if the task has been canceled
     * @return planning_msgs::Path of the generated path
     */
    uint32 MakePlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal, double tolerance,
        commsgs::planning_msgs::Path& plan, double& cost,
        std::string& message) override;

    /**
     * @brief Cancel planning for robot A->B path find
     */
    bool Cancel() override;

    /**
     * @brief Configures the planner with the given options
     * @param options The options to configure the planner with
     * @param name The name of the planner
     * @param tf_buffer The transform buffer to use for pose transformations
     * @param costmap_wrapper The costmap wrapper to use for obstacle avoidance
     * @return True if the planner was successfully configured, false otherwise
     */
    bool Configure(const proto::PlannerOptions& options, 
        const std::string& name,
        TfBuffer* tf_buffer, 
        map::costmap_2d::Costmap2DWrapper* costmap_wrapper) override;

    /**
     * @brief Activates the planner
     * @return True if the planner was successfully activated, false otherwise
     */
     bool Activate() override;

    /**
     * @brief Deactivates the planner
     * @return True if the planner was successfully deactivated, false otherwise
     */
    bool Deactivate() override;

    /**
     * @brief Cleans up the planner
     * @return True if the planner was successfully cleaned up, false otherwise
     */
    bool Cleanup() override;

    /**
     * @brief Shutdown the planner
     * @return True if the planner was successfully shut down, false otherwise
     */
    bool Shutdown() override;

protected:
    /**
     * @brief Compute a plan given start and goal poses, provided in global world frame.
     * @param start Start pose
     * @param goal Goal pose
     * @param tolerance Relaxation constraint in x and y
     * @param cancel_checker Function to check if the task has been canceled
     * @param plan Path to be computed
     * @return true if can find the path
     */
    bool makePlan(
        const commsgs::geometry_msgs::Pose& start,
        const commsgs::geometry_msgs::Pose& goal, 
        double tolerance,
        std::function<bool()> cancel_checker,
        commsgs::planning_msgs::Path& plan);

    /**
     * @brief Compute the navigation function given a seed point in the world to start from
     * @param world_point Point in world coordinate frame
     * @return true if can compute
     */
    bool computePotential(const commsgs::geometry_msgs::Point& world_point);

    /**
     * @brief Compute a plan to a goal from a potential - must call computePotential first
     * @param goal Goal pose
     * @param plan Path to be computed
     * @return true if can compute a plan path
     */
    bool getPlanFromPotential(
        const commsgs::geometry_msgs::Pose& goal,
        commsgs::planning_msgs::Path& plan);

    /**
     * @brief Remove artifacts at the end of the path - originated from planning on a discretized world
     * @param goal Goal pose
     * @param plan Computed path
     */
    void smoothApproachToGoal(
        const commsgs::geometry_msgs::Pose& goal,
        commsgs::planning_msgs::Path& plan);

    /**
     * @brief Compute the potential, or navigation cost, at a given point in the world
     *        must call computePotential first
     * @param world_point Point in world coordinate frame
     * @return double point potential (navigation cost)
     */
    double getPointPotential(const commsgs::geometry_msgs::Point& world_point);

    // Check for a valid potential value at a given point in the world
    // - must call computePotential first
    // - currently unused
    // bool validPointPotential(const geometry_msgs::msg::Point & world_point);
    // bool validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

    /**
     * @brief Compute the squared distance between two points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return double squared distance between two points
     */
    inline double squared_distance(const commsgs::geometry_msgs::Pose& p1,
        const commsgs::geometry_msgs::Pose& p2)
    {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;
        return dx * dx + dy * dy;
    }

    /**
     * @brief Transform a point from world to map frame
     * @param wx double of world X coordinate
     * @param wy double of world Y coordinate
     * @param mx int of map X coordinate
     * @param my int of map Y coordinate
     * @return true if can transform
     */
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

    /**
     * @brief Transform a point from map to world frame
     * @param mx double of map X coordinate
     * @param my double of map Y coordinate
     * @param wx double of world X coordinate
     * @param wy double of world Y coordinate
     */
    void mapToWorld(double mx, double my, double& wx, double& wy);

    /**
     * @brief Set the corresponding cell cost to be free space
     * @param mx int of map X coordinate
     * @param my int of map Y coordinate
     */
    void clearRobotCell(unsigned int mx, unsigned int my);

    /**
     * @brief Determine if a new planner object should be made
     * @return true if planner object is out of date
     */
    bool isPlannerOutOfDate();

    // Tf
    Buffer* tf_{nullptr};

    // Planner based on ROS1 NavFn algorithm
    std::unique_ptr<NavFn> planner_{nullptr};

    // Global Costmap
    map::costmap_2d::Costmap2D* costmap_{nullptr};

    // The global frame of the costmap
    std::string global_frame_, name_;

    // Whether or not the planner should be allowed to plan through unknown space
    bool allow_unknown_, use_final_approach_orientation_;

    // If the goal is obstructed, the tolerance specifies how many meters the planner
    // can relax the constraint in x and y before failing
    double tolerance_;

    // Whether to use the astar planner or default dijkstras
    bool use_astar_;
};

proto::NavFnPlanner CreateNavFnPlannerOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace navfn
}  // namespace plugins
}  // namespace planning
}  // namespace autonomy