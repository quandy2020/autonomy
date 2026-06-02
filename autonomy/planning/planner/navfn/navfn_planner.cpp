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

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

#include "autonomy/planning/planner/navfn/navfn_planner.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/string_util.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using std::placeholders::_1;

namespace autonomy {
namespace planning {
namespace planner {
namespace navfn {

NavfnPlanner::NavfnPlanner(const proto::PlannerOptions& options,
                           const std::string& name,
                           std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                               costmap)
    : GlobalPlanner(options, name, std::move(costmap)) {
    InitFromOptions();
}

NavfnPlanner::~NavfnPlanner() {
    AINFO << ::autonomy::common::StrCat("Destroying plugin ", name_,
                                        " of type NavfnPlanner.");
    planner_.reset();
    costmap_.reset();
}

void NavfnPlanner::InitFromOptions() {
    const auto& navfn_opts = options_.navfn();
    tolerance_ = navfn_opts.tolerance();
    allow_unknown_ = navfn_opts.allow_unknown();
    use_astar_ = navfn_opts.use_astar();
    use_final_approach_orientation_ =
        navfn_opts.use_final_approach_orientation();

    if (costmap_) {
        auto* costmap_ptr = costmap_->getCostmap();
        if (costmap_ptr) {
            planner_ = std::make_unique<NavFn>(
                static_cast<int>(costmap_ptr->getSizeInCellsX()),
                static_cast<int>(costmap_ptr->getSizeInCellsY()));
            AINFO << "NavfnPlanner costmap set: "
                  << costmap_ptr->getSizeInCellsX() << "x"
                  << costmap_ptr->getSizeInCellsY();
        }
    } else {
        AINFO << "NavfnPlanner configured without costmap (will be set later)";
    }
}

uint32 NavfnPlanner::CreatePlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    commsgs::planning_msgs::Path& plan, std::function<bool()> cancel_checker) {
    if (!costmap_) {
        AERROR << "Costmap is not set for planner " << name_;
        return static_cast<uint32>(
            proto::PlannerResultCode::PLANNER_NO_PATH_FOUND);
    }

    // Set global_frame_ from start pose if not already set
    if (global_frame_.empty() && !start.header.frame_id.empty()) {
        global_frame_ = start.header.frame_id;
    }

    auto* costmap_ptr = costmap_->getCostmap();
    if (!costmap_ptr) {
        AERROR << "Failed to get costmap from wrapper";
        return static_cast<uint32>(
            proto::PlannerResultCode::PLANNER_NO_PATH_FOUND);
    }

    if (!planner_) {
        // Initialize planner if not already initialized
        planner_ = std::make_unique<NavFn>(
            static_cast<int>(costmap_ptr->getSizeInCellsX()),
            static_cast<int>(costmap_ptr->getSizeInCellsY()));
    }

    // Check if planner needs to be recreated
    if (isPlannerOutOfDate()) {
        planner_ = std::make_unique<NavFn>(
            static_cast<int>(costmap_ptr->getSizeInCellsX()),
            static_cast<int>(costmap_ptr->getSizeInCellsY()));
    }

    if (makePlan(start.pose, goal.pose, tolerance_, cancel_checker, plan)) {
        return static_cast<uint32>(proto::PlannerResultCode::PLANNER_SUCCESS);
    }
    if (planner_ && planner_->wasPropagationCancelled()) {
        return static_cast<uint32>(proto::PlannerResultCode::PLANNER_CANCELED);
    }
    return static_cast<uint32>(proto::PlannerResultCode::PLANNER_NO_PATH_FOUND);
}

bool NavfnPlanner::isPlannerOutOfDate() {
    auto* costmap_ptr = costmap_->getCostmap();
    if (!costmap_ptr) {
        return true;
    }

    if (!planner_ ||
        planner_->nx != static_cast<int>(costmap_ptr->getSizeInCellsX()) ||
        planner_->ny != static_cast<int>(costmap_ptr->getSizeInCellsY())) {
        return true;
    }
    return false;
}

bool NavfnPlanner::makePlan(const commsgs::geometry_msgs::Pose& start,
                            const commsgs::geometry_msgs::Pose& goal,
                            double tolerance,
                            std::function<bool()> cancel_checker,
                            commsgs::planning_msgs::Path& plan) {
    // clear the plan, just in case
    plan.poses.clear();

    // plan.header.stamp = clock_->now();
    // Use default frame_id "map" if not set, or get from start/goal pose
    plan.header.frame_id = global_frame_.empty() ? "map" : global_frame_;

    double wx = start.position.x;
    double wy = start.position.y;

    unsigned int mx, my;
    if (!worldToMap(wx, wy, mx, my)) {
        AERROR << "Start position (" << wx << ", " << wy
               << ") is outside map bounds";
        return false;
    }

    auto* costmap_ptr = costmap_->getCostmap();
    if (!costmap_ptr) {
        AERROR << "Failed to get costmap from wrapper";
        return false;
    }

    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap_ptr->getMutex()));

    const unsigned int size_x = costmap_ptr->getSizeInCellsX();
    const unsigned int size_y = costmap_ptr->getSizeInCellsY();
    const size_t map_size = static_cast<size_t>(size_x) * size_y;
    planning_costmap_copy_.resize(map_size);
    std::memcpy(planning_costmap_copy_.data(), costmap_ptr->getCharMap(),
                map_size);
    planning_costmap_copy_[static_cast<size_t>(my) * size_x + mx] =
        map::costmap_2d::FREE_SPACE;

    planner_->setNavArr(size_x, size_y);
    planner_->setCostmap(planning_costmap_copy_.data(), true, allow_unknown_);

    lock.unlock();

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.position.x;
    wy = goal.position.y;

    if (!worldToMap(wx, wy, mx, my)) {
        AERROR << "Goal position (" << wx << ", " << wy
               << ") is outside map bounds";
        return false;
    }
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

    auto* costmap = costmap_->getCostmap();
    if (!costmap) {
        return false;
    }
    double resolution = costmap->getResolution();
    commsgs::geometry_msgs::Pose p, best_pose;

    bool found_legal = false;

    p = goal;
    double potential = getPointPotential(p.position);
    if (potential < POT_HIGH) {
        // Goal is reachable by itself
        best_pose = p;
        found_legal = true;
    } else if (potential == std::numeric_limits<double>::max()) {
        // Point is outside map bounds
        AERROR << "Goal position (" << goal.position.x << ", "
               << goal.position.y << ") is outside map bounds";
    } else {
        // Goal is not reachable. Trying to find nearest to the goal
        // reachable point within its tolerance region
        double best_sdist = std::numeric_limits<double>::max();

        p.position.y = goal.position.y - tolerance;
        while (p.position.y <= goal.position.y + tolerance) {
            p.position.x = goal.position.x - tolerance;
            while (p.position.x <= goal.position.x + tolerance) {
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
        if (getPlanFromPotential(best_pose, plan)) {
            smoothApproachToGoal(best_pose, plan);

            // If use_final_approach_orientation=true, interpolate the last pose
            // orientation from the previous pose to set the orientation to the
            // 'final approach' orientation of the robot so it does not rotate.
            // And deal with corner case of plan of length 1
            if (use_final_approach_orientation_) {
                size_t plan_size = plan.poses.size();
                if (plan_size == 1) {
                    plan.poses.back().pose.orientation = start.orientation;
                } else if (plan_size > 1) {
                    double dx, dy, theta;
                    auto last_pose = plan.poses.back().pose.position;
                    auto approach_pose =
                        plan.poses[plan_size - 2].pose.position;
                    // Deal with the case of NavFn producing a path with two
                    // equal last poses
                    if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
                        std::abs(last_pose.y - approach_pose.y) < 0.0001 &&
                        plan_size > 2) {
                        approach_pose = plan.poses[plan_size - 3].pose.position;
                    }
                    dx = last_pose.x - approach_pose.x;
                    dy = last_pose.y - approach_pose.y;
                    theta = atan2(dy, dx);
                    plan.poses.back().pose.orientation =
                        map::costmap_2d::utils::OrientationAroundZAxis(theta);
                }
            }
        } else {
            AERROR << "Failed to create a plan from potential when a legal"
                      " potential was found. This shouldn't happen.";
        }
    }

    return !plan.poses.empty();
}

void NavfnPlanner::smoothApproachToGoal(
    const commsgs::geometry_msgs::Pose& goal,
    commsgs::planning_msgs::Path& plan) {
    // Replace the last pose of the computed path if it's actually further away
    // to the second to last pose than the goal pose.
    if (plan.poses.size() >= 2) {
        auto second_to_last_pose = plan.poses.end()[-2];
        auto last_pose = plan.poses.back();
        if (squared_distance(last_pose.pose, second_to_last_pose.pose) >
            squared_distance(goal, second_to_last_pose.pose)) {
            plan.poses.back().pose = goal;
            return;
        }
    }
    commsgs::geometry_msgs::PoseStamped goal_copy;
    goal_copy.header.frame_id = plan.header.frame_id;
    goal_copy.pose = goal;
    plan.poses.push_back(goal_copy);
}

bool NavfnPlanner::getPlanFromPotential(
    const commsgs::geometry_msgs::Pose& goal,
    commsgs::planning_msgs::Path& plan) {
    // clear the plan, just in case
    plan.poses.clear();

    // Goal should be in global frame
    double wx = goal.position.x;
    double wy = goal.position.y;

    // the potential has already been computed, so we won't update our copy of
    // the costmap
    unsigned int mx, my;
    if (!worldToMap(wx, wy, mx, my)) {
        AERROR << "Goal position (" << wx << ", " << wy
               << ") is outside map bounds in getPlanFromPotential";
        return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    auto* costmap_ptr = costmap_->getCostmap();
    if (!costmap_ptr) {
        AERROR << "Failed to get costmap from wrapper in getPlanFromPotential";
        return false;
    }

    const int& max_cycles =
        (costmap_ptr->getSizeInCellsX() >= costmap_ptr->getSizeInCellsY())
            ? (costmap_ptr->getSizeInCellsX() * 4)
            : (costmap_ptr->getSizeInCellsY() * 4);

    int path_len = planner_->calcPath(max_cycles);
    if (path_len == 0) {
        return false;
    }

    auto cost = planner_->getLastPathCost();

    // extract the plan
    float* x = planner_->getPathX();
    float* y = planner_->getPathY();
    int len = planner_->getPathLen();

    for (int i = len - 1; i >= 0; --i) {
        // convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(x[i], y[i], world_x, world_y);

        commsgs::geometry_msgs::PoseStamped pose;
        pose.header.frame_id = plan.header.frame_id;
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

double NavfnPlanner::getPointPotential(
    const commsgs::geometry_msgs::Point& world_point) {
    unsigned int mx, my;
    if (!worldToMap(world_point.x, world_point.y, mx, my)) {
        return std::numeric_limits<double>::max();
    }

    if (!planner_) {
        return std::numeric_limits<double>::max();
    }

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
}

bool NavfnPlanner::worldToMap(double wx, double wy, unsigned int& mx,
                              unsigned int& my) {
    auto* costmap = costmap_->getCostmap();
    if (!costmap) {
        return false;
    }

    // Use the same conversion as Costmap2D::worldToMap for consistency
    if (wx < costmap->getOriginX() || wy < costmap->getOriginY()) {
        return false;
    }

    mx = static_cast<unsigned int>((wx - costmap->getOriginX()) /
                                   costmap->getResolution());
    my = static_cast<unsigned int>((wy - costmap->getOriginY()) /
                                   costmap->getResolution());

    if (mx < costmap->getSizeInCellsX() && my < costmap->getSizeInCellsY()) {
        return true;
    }

    AERROR << ::autonomy::common::StrCat("worldToMap failed: mx,my: ", mx, ",", my,
                           ", size_x,size_y: ", costmap->getSizeInCellsX(), ",",
                           costmap->getSizeInCellsY());

    return false;
}

void NavfnPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    auto* costmap = costmap_->getCostmap();
    if (!costmap) {
        return;
    }

    wx = costmap->getOriginX() + mx * costmap->getResolution();
    wy = costmap->getOriginY() + my * costmap->getResolution();
}

}  // namespace navfn
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

using autonomy::planning::common::GlobalPlanner;
using autonomy::planning::planner::navfn::NavfnPlanner;

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(NavfnPlanner, GlobalPlanner);