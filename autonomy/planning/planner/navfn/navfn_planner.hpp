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
#include <memory>
#include <string>
#include <vector>

#include "autonomy/planning/proto/navfn_planner.pb.h"

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/planner/navfn/navfn.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace navfn {

class NavfnPlanner : public common::GlobalPlanner
{
public:
    /**
     * @brief Define Buffer type
     */
    using Buffer = autonomy::transform::Buffer;

    NavfnPlanner() = default;

    NavfnPlanner(const proto::PlannerOptions& options, const std::string& name,
                 std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap);

    ~NavfnPlanner() override;

    /**
     * @brief Creating a plan from start and goal poses
     * @param start Start pose
     * @param goal Goal pose
     * @param plan Path to be computed
     * @param cancel_checker Function to check if the task has been canceled
     * @return Result code from PlannerResultCode enum
     */
    uint32 CreatePlan(const automsgs::msgs::geometry_msgs::PoseStamped& start,
                      const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                      automsgs::msgs::planning_msgs::Path& plan,
                      std::function<bool()> cancel_checker) override;

protected:
    /**
     * @brief Compute a plan given start and goal poses, provided in global
     * world frame.
     * @param start Start pose
     * @param goal Goal pose
     * @param tolerance Relaxation constraint in x and y
     * @param cancel_checker Function to check if the task has been canceled
     * @param plan Path to be computed
     * @return true if can find the path
     */
    bool makePlan(const automsgs::msgs::geometry_msgs::Pose& start,
                  const automsgs::msgs::geometry_msgs::Pose& goal, double tolerance,
                  std::function<bool()> cancel_checker,
                  automsgs::msgs::planning_msgs::Path& plan);

    /**
     * @brief Compute the navigation function given a seed point in the world to
     * start from
     * @param world_point Point in world coordinate frame
     * @return true if can compute
     */
    bool computePotential(const automsgs::msgs::geometry_msgs::Point& world_point);

    /**
     * @brief Compute a plan to a goal from a potential - must call
     * computePotential first
     * @param goal Goal pose
     * @param plan Path to be computed
     * @return true if can compute a plan path
     */
    bool getPlanFromPotential(const automsgs::msgs::geometry_msgs::Pose& goal,
                              automsgs::msgs::planning_msgs::Path& plan);

    /**
     * @brief Remove artifacts at the end of the path - originated from planning
     * on a discretized world
     * @param goal Goal pose
     * @param plan Computed path
     */
    void smoothApproachToGoal(const automsgs::msgs::geometry_msgs::Pose& goal,
                              automsgs::msgs::planning_msgs::Path& plan);

    /**
     * @brief Compute the potential, or navigation cost, at a given point in the
     * world must call computePotential first
     * @param world_point Point in world coordinate frame
     * @return double point potential (navigation cost)
     */
    double getPointPotential(const automsgs::msgs::geometry_msgs::Point& world_point);

    // Check for a valid potential value at a given point in the world
    // - must call computePotential first
    // - currently unused
    // bool validPointPotential(const geometry_msgs::msg::Point & world_point);
    // bool validPointPotential(const geometry_msgs::msg::Point & world_point,
    // double tolerance);

    /**
     * @brief Compute the squared distance between two points
     * @param p1 Point 1
     * @param p2 Point 2
     * @return double squared distance between two points
     */
    inline double squared_distance(const automsgs::msgs::geometry_msgs::Pose& p1,
                                   const automsgs::msgs::geometry_msgs::Pose& p2) {
        double dx = p1.position().x() - p2.position().x();
        double dy = p1.position().y() - p2.position().y();
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
     * @brief Force A* on/off (used by DijkstraPlanner plugin).
     */
    void SetUseAstar(bool use_astar) {
        use_astar_ = use_astar;
    }

    void SetTolerance(double tolerance) {
        tolerance_ = tolerance;
    }

    void SetAllowUnknown(bool allow_unknown) {
        allow_unknown_ = allow_unknown;
    }

    void SetUseFinalApproachOrientation(bool value) {
        use_final_approach_orientation_ = value;
    }

    /**
     * @brief Determine if a new planner object should be made
     * @return true if planner object is out of date
     */
    bool isPlannerOutOfDate();

    void InitFromOptions();

    // Planner based on ROS1 NavFn algorithm
    std::unique_ptr<NavFn> planner_{nullptr};

    // The global frame of the costmap
    std::string global_frame_;

    // Whether or not the planner should be allowed to plan through unknown
    // space
    bool allow_unknown_, use_final_approach_orientation_;

    // If the goal is obstructed, the tolerance specifies how many meters the
    // planner can relax the constraint in x and y before failing
    double tolerance_;

    // Whether to use the astar planner or default dijkstras
    bool use_astar_;

    // Local copy of the costmap used during planning (avoids mutating global map)
    std::vector<unsigned char> planning_costmap_copy_;
};

}  // namespace navfn
}  // namespace planner
}  // namespace planning
}  // namespace autonomy