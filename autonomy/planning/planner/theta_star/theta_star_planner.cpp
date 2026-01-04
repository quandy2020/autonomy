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

#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy {
namespace planning {
namespace planner {
namespace theta_star {

bool ThetaStarPlanner::Configure(
    const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    planner_ = std::make_unique<ThetaStar>();
    name_ = name;
    costmap_ = costmap_wrapper;

    if (costmap_) {
        planner_->costmap_ = costmap_->getCostmap();
    }

    // Load parameters from proto options
    options_ = options.theta_star();

    // Set how_many_corners with validation (must be 4 or 8)
    planner_->how_many_corners_ = options_.how_many_corners();
    if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4) {
        planner_->how_many_corners_ = 8;
        AWARN
            << "Your value for how_many_corners was overridden, and is now set "
               "to 8";
    }

    // Set other parameters with default values if not set in proto
    planner_->allow_unknown_ = options_.allow_unknown();
    planner_->w_euc_cost_ =
        options_.w_euc_cost() > 0.0 ? options_.w_euc_cost() : 2.0;
    planner_->w_traversal_cost_ =
        options_.w_traversal_cost() > 0.0 ? options_.w_traversal_cost() : 1.0;
    planner_->w_heuristic_cost_ =
        planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;
    planner_->terminal_checking_interval_ =
        options_.terminal_checking_interval() > 0
            ? options_.terminal_checking_interval()
            : 5000;

    AINFO << "ThetaStarPlanner configured: " << name_;
    return true;
}

void ThetaStarPlanner::Cleanup() {
    AINFO << "CleaningUp plugin " << name_ << " of type theta_star_planner";
    planner_.reset();
}

void ThetaStarPlanner::Activate() {
    AINFO << "Activating plugin " << name_ << " of type theta_star_planner";
}

void ThetaStarPlanner::Deactivate() {
    AINFO << "Deactivating plugin " << name_ << " of type theta_star_planner";
}

uint32_t ThetaStarPlanner::CreatePlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    commsgs::planning_msgs::Path& plan, std::function<bool()> cancel_checker) {
    if (!costmap_ || !planner_->costmap_) {
        AERROR << "Costmap is not set for planner " << name_;
        return static_cast<uint32_t>(
            proto::PlannerResultCode::PLANNER_NOT_INITIALIZED);
    }

    auto* costmap_ptr = planner_->costmap_;

    // Set frame_id from start pose
    plan.header.frame_id =
        start.header.frame_id.empty() ? "map" : start.header.frame_id;
    plan.header.stamp = commsgs::builtin_interfaces::Time::Now();

    // Check if start and goal are the same
    unsigned int mx_start, my_start, mx_goal, my_goal;
    if (!costmap_ptr->worldToMap(start.pose.position.x, start.pose.position.y,
                                 mx_start, my_start)) {
        AERROR << "Start coordinates (" << start.pose.position.x << ", "
               << start.pose.position.y << ") are outside map bounds";
        return static_cast<uint32_t>(
            proto::PlannerResultCode::PLANNER_INVALID_START);
    }
    if (!costmap_ptr->worldToMap(goal.pose.position.x, goal.pose.position.y,
                                 mx_goal, my_goal)) {
        AERROR << "Goal coordinates (" << goal.pose.position.x << ", "
               << goal.pose.position.y << ") are outside map bounds";
        return static_cast<uint32_t>(
            proto::PlannerResultCode::PLANNER_INVALID_GOAL);
    }

    if (costmap_ptr->getCost(mx_goal, my_goal) ==
        map::costmap_2d::LETHAL_OBSTACLE) {
        AERROR << "Goal coordinates (" << goal.pose.position.x << ", "
               << goal.pose.position.y << ") are in lethal cost";
        return static_cast<uint32_t>(
            proto::PlannerResultCode::PLANNER_BLOCKED_GOAL);
    }

    if (mx_start == mx_goal && my_start == my_goal) {
        plan.poses.clear();
        commsgs::geometry_msgs::PoseStamped pose;
        pose.header = plan.header;
        pose.pose = start.pose;
        pose.pose.position.z = 0.0;
        plan.poses.push_back(pose);
        return static_cast<uint32_t>(proto::PlannerResultCode::PLANNER_SUCCESS);
    }

    auto start_time = std::chrono::steady_clock::now();
    planner_->clearStart();
    planner_->setStartAndGoal(start, goal);

    AINFO << "ThetaStar: Got start (" << planner_->src_.x << ", "
          << planner_->src_.y << ") and goal (" << planner_->dst_.x << ", "
          << planner_->dst_.y << ")";

    getPlan(plan, cancel_checker);

    // Check if a plan is generated
    size_t plan_size = plan.poses.size();
    if (plan_size == 0) {
        AERROR << "No path found";
        return static_cast<uint32_t>(
            proto::PlannerResultCode::PLANNER_NO_PATH_FOUND);
    }

    // Set goal orientation on last pose
    plan.poses.back().pose.orientation = goal.pose.orientation;

    auto stop_time = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(
        stop_time - start_time);
    AINFO << "ThetaStar plan generated in " << dur.count()
          << " microseconds, nodes_opened: " << planner_->nodes_opened;

    return static_cast<uint32_t>(proto::PlannerResultCode::PLANNER_SUCCESS);
}

void ThetaStarPlanner::getPlan(commsgs::planning_msgs::Path& plan,
                               std::function<bool()> cancel_checker) {
    if (!planner_->costmap_) {
        plan.poses.clear();
        return;
    }

    std::vector<coordsW> path;
    if (planner_->isUnsafeToPlan()) {
        plan.poses.clear();
        AWARN << "Either of the start or goal pose are an obstacle!";
        return;
    }

    if (planner_->generatePath(path, cancel_checker)) {
        plan = linearInterpolation(path, planner_->costmap_->getResolution());
    } else {
        plan.poses.clear();
        AWARN << "Could not generate path between the given poses";
    }
}

commsgs::planning_msgs::Path ThetaStarPlanner::linearInterpolation(
    const std::vector<coordsW>& raw_path, const double& dist_bw_points) {
    commsgs::planning_msgs::Path plan;
    commsgs::geometry_msgs::PoseStamped p1;
    p1.pose.position.z = 0.0;
    p1.pose.orientation.w = 1.0;

    for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
        coordsW pt1 = raw_path[j];
        p1.pose.position.x = pt1.x;
        p1.pose.position.y = pt1.y;
        plan.poses.push_back(p1);

        coordsW pt2 = raw_path[j + 1];
        double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
        if (distance < dist_bw_points) {
            continue;
        }
        int loops = static_cast<int>(distance / dist_bw_points);
        double sin_alpha = (pt2.y - pt1.y) / distance;
        double cos_alpha = (pt2.x - pt1.x) / distance;
        for (int k = 1; k < loops; k++) {
            p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
            p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
            plan.poses.push_back(p1);
        }
    }

    // Add the last point
    if (!raw_path.empty()) {
        coordsW last_pt = raw_path.back();
        p1.pose.position.x = last_pt.x;
        p1.pose.position.y = last_pt.y;
        plan.poses.push_back(p1);
    }

    return plan;
}

}  // namespace theta_star
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(
    autonomy::planning::planner::theta_star::ThetaStarPlanner,
    autonomy::planning::common::GlobalPlanner)