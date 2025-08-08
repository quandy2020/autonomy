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

#include "autonomy/planning/plugins/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/map/utils/geometry_utils.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using std::placeholders::_1;

namespace autonomy {
namespace planning {
namespace plugins {

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::NavfnPlanner(
    std::string name, std::shared_ptr<Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2D> costmap)
: tf_(tf.get()), costmap_(costmap.get())
{
    name_ = name;
    // global_frame_ = costmap_->getGlobalFrameID();
    LOG(INFO) << absl::StrCat("Configuring plugin ", name, "  of type NavfnPlanner");
}

NavfnPlanner::~NavfnPlanner()
{
    LOG(INFO) << absl::StrCat("Destroying plugin ", name_, " of type NavfnPlanner."); 
}

uint32 NavfnPlanner::MakePlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal, double tolerance,
        std::vector<commsgs::geometry_msgs::PoseStamped>& plan, double& cost,
        std::string& message) 
{
    return 0;
}

bool NavfnPlanner::Cancel()
{
    return true;
}

bool NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

bool NavfnPlanner::makePlan(
  const commsgs::geometry_msgs::Pose & start,
  const commsgs::geometry_msgs::Pose & goal, double tolerance,
  std::function<bool()> cancel_checker,
  commsgs::planning_msgs::Path& plan)
{
  // clear the plan, just in case
    plan.poses.clear();

    // plan.header.stamp = clock_->now();
    plan.header.frame_id = global_frame_;

    double wx = start.position.x;
    double wy = start.position.y;

    //   RCLCPP_DEBUG(
    //     logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    //     start.position.x, start.position.y, goal.position.x, goal.position.y);

    unsigned int mx, my;
    worldToMap(wx, wy, mx, my);

    // clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(mx, my);

    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    lock.unlock();

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.position.x;
    wy = goal.position.y;

    worldToMap(wx, wy, mx, my);
    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);
    if (use_astar_) {
        planner_->calcNavFnAstar(cancel_checker);
    } else {
        planner_->calcNavFnDijkstra(cancel_checker, true);
    }

    double resolution = costmap_->getResolution();
    commsgs::geometry_msgs::Pose p, best_pose;

    bool found_legal = false;

    p = goal;
    double potential = getPointPotential(p.position);
    if (potential < POT_HIGH) {
        // Goal is reachable by itself
        best_pose = p;
        found_legal = true;
    } else {
        // Goal is not reachable. Trying to find nearest to the goal
        // reachable point within its tolerance region
        double best_sdist = std::numeric_limits<double>::max();

        p.position.y = goal.position.y - tolerance;
        while (p.position.y <= goal.position.y + tolerance) 
        {
            p.position.x = goal.position.x - tolerance;
            while (p.position.x <= goal.position.x + tolerance) 
            {
                potential = getPointPotential(p.position);
                double sdist = squared_distance(p, goal);
                if (potential < POT_HIGH && sdist < best_sdist) {
                    best_sdist = sdist;
                    best_pose = p;
                    found_legal = true;
                }
                p.position.x += resolution;
            }
            p.position.y += resolution;
        }
    }

    if (found_legal) {
        // extract the plan
        if (getPlanFromPotential(best_pose, plan)) 
        {
            smoothApproachToGoal(best_pose, plan);

            // If use_final_approach_orientation=true, interpolate the last pose orientation from the
            // previous pose to set the orientation to the 'final approach' orientation of the robot so
            // it does not rotate.
            // And deal with corner case of plan of length 1
            if (use_final_approach_orientation_) 
            {
                size_t plan_size = plan.poses.size();
                if (plan_size == 1) {
                    plan.poses.back().pose.orientation = start.orientation;
                } 
                else if (plan_size > 1) 
                {
                    double dx, dy, theta;
                    auto last_pose = plan.poses.back().pose.position;
                    auto approach_pose = plan.poses[plan_size - 2].pose.position;
                    // Deal with the case of NavFn producing a path with two equal last poses
                    if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
                        std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
                    {
                        approach_pose = plan.poses[plan_size - 3].pose.position;
                    }
                    dx = last_pose.x - approach_pose.x;
                    dy = last_pose.y - approach_pose.y;
                    theta = atan2(dy, dx);
                    plan.poses.back().pose.orientation = map::utils::OrientationAroundZAxis(theta);
                }
        }
        } else {
        // RCLCPP_ERROR(
        //     logger_,
        //     "Failed to create a plan from potential when a legal"
        //     " potential was found. This shouldn't happen.");
        }
    }

    return !plan.poses.empty();
}

void NavfnPlanner::smoothApproachToGoal(const commsgs::geometry_msgs::Pose& goal, commsgs::planning_msgs::Path& plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  commsgs::geometry_msgs::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}

bool NavfnPlanner::getPlanFromPotential(const commsgs::geometry_msgs::Pose& goal, commsgs::planning_msgs::Path& plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
//   RCLCPP_DEBUG(
//     logger_,
//     "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    commsgs::geometry_msgs::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double NavfnPlanner::getPointPotential(const commsgs::geometry_msgs::Point& world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
    if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
        return false;
    }

    mx = static_cast<int>(std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
    my = static_cast<int>(std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
        return true;
    }

    //   RCLCPP_ERROR(
    //     logger_,
    //     "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    //     costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

    return false;
}

void NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
    // TODO(orduno): check usage of this function, might instead be a request to
    //               world_model / map server
    costmap_->setCost(mx, my, map::costmap_2d::FREE_SPACE);
}

}  // namespace plugins
}  // namespace planning
}  // namespace autonomy

// // Plugins
// CLASS_LOADER_REGISTER_CLASS(
//   autonomy::planning::plugins::NavfnPlanner, 
//   autonomy::planning::common::PlannerInterface)
