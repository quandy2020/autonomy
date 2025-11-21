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

#include "autonomy/planning/planner_server.hpp"

#include <absl/strings/str_cat.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/planning/proto/server_action.pb.h"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"

namespace autonomy {
namespace planning {
    
PlannerServer::PlannerServer(const proto::PlannerOptions& options, TfBuffer* tf_buffer)
    : options_{options},
      tf_{tf_buffer}
{

    if(!Configure()) {
        LOG(FATAL) << "Failed to configure planner server.";
        return;
    } else {
        LOG(INFO) << "Planner Server configured.";
    }

    if(!Activate()) {
        LOG(FATAL) << "Failed to activate planner server.";
        return;
    } else {
        LOG(INFO) << "Planner Server activated.";
    }

    LOG(INFO) << "Planning server init successfully.";
}

PlannerServer::~PlannerServer()
{
    /*
     * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
     * never called.
     */
    planners_.clear();
    // costmap_thread_.reset();
}

void PlannerServer::Start()
{
    LOG(INFO) << "start";
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PlannerServer::WaitForShutdown()
{

}

bool PlannerServer::Configure()
{
    LOG(INFO) << "Configuring";

    costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(options_.costmap(), kCostmapTopicName);
    if (costmap_wrapper_  == nullptr) {
        LOG(FATAL) << "Failed to configure costmap wrapper.  costmap_wrapper_ is nullptr";
        return false;
    }

    costmap_ = costmap_wrapper_->getCostmap();
    if (!costmap_wrapper_->getUseRadius()) {
        collision_checker_ = std::make_unique<map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>>(costmap_);
    }

    LOG(INFO) << "Costmap size: %d,%d", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY();
    planner_types_.resize(planner_ids_.size());

    for (size_t i = 0; i != planner_ids_.size(); i++) {
        try {
            auto planner = gp_loader_->createSharedInstance<common::PlannerInterface>(planner_types_[i]);
            LOG(INFO) << "Created global planner plugin " << planner_ids_[i] << " of type " << planner_types_[i];
            planner->Configure(options_, planner_ids_[i], tf_, costmap_wrapper_.get());
            planners_.insert({planner_ids_[i], planner});
        } catch (const std::exception & ex) {
            LOG(FATAL) << "Failed to create global planner. Exception: " << ex.what();
            Cleanup();
            return false;
        }
    }

    for (size_t i = 0; i != planner_ids_.size(); i++) {
        planner_ids_concat_ += planner_ids_[i] + std::string(" ");
    }
    LOG(INFO) << "Planner Server has planners available: " << planner_ids_concat_;

    if (options_.expected_planner_frequency() > 0) {
        max_planner_duration_ = 1 / options_.expected_planner_frequency();
    } else {
        LOG(WARNING) << "The expected planner frequency parameter is " 
                     << options_.expected_planner_frequency() 
                     <<  "Hz. The value should to be greater than 0.0 to turn on duration overrrun warning messages";
        max_planner_duration_ = 0.0;
    }

    //     commsgs::planning_msgs::Path>>(kPlanTopicName);

    // compute_path_to_pose_service_ = std::make_shared<
    //         commsgs::geometry_msgs::PoseStamped, 
    //         commsgs::planning_msgs::Path>>(kComputePathToPoseServiceName,
    //         [this](const commsgs::geometry_msgs::PoseStamped& request, commsgs::planning_msgs::Path& response) {
    //             ComputePlan(request, response);
    //         });

    return true;
}

bool PlannerServer::Activate()
{
    LOG(INFO) << "Activating";
    // TODO(duyongquan): Add activation logic
    return true;
}

bool PlannerServer::Deactivate()
{
    LOG(INFO) << "Deactivating";
    // TODO(duyongquan): Add deactivation logic
    return true;
}
bool PlannerServer::Cleanup()
{
    LOG(INFO) << "Cleaning up";
    // TODO(duyongquan): Add cleanup logic
    return true;
}
bool PlannerServer::Shutdown()
{
    LOG(INFO) << "Shutting down";
    // TODO(duyongquan): Add shutdown logic
    return true;
}

commsgs::planning_msgs::Path PlannerServer::GetPlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id,
    std::function<bool()> cancel_checker)
{
    commsgs::planning_msgs::Path path;
    LOG(INFO) << "Planning algorithm " << planner_id << " is trying to find a path from (" 
              << start.pose.position.x  << ", " << start.pose.position.y << ")"
              << " to " << "(" 
              << goal.pose.position.x << "," <<  goal.pose.position.y << ")";

    std::string message;
    double cost = 0.0;
    double tolerance = 0.2;

    uint32_t return_code = 0;
    if (planners_.find(planner_id) != planners_.end()) {
        return_code = planners_[planner_id]->MakePlan(start, goal, tolerance, path, cost, message);
    } else {
        if (planners_.size() == 1 && planner_id.empty()) {
            LOG(WARNING) << "No planners specified in action call. Server will use only plugin " 
                         << planner_ids_concat_ << " in server. This warning will appear once.";
            return_code = planners_[planners_.begin()->first]->MakePlan(start, goal, tolerance, path, cost, message);
        } else {
            LOG(ERROR) << "planner " << planner_id << " is not a valid planner. "
                       << "Planner names are: " << planner_ids_concat_;
            throw common::InvalidPlanner("Planner id " + planner_id + " is invalid");
        }
    }

    if (return_code != 0) {
        LOG(ERROR) << "planner " << planner_id << " failed to find a path. "
                   << "Return code: " << return_code << ". "
                   << "Message: " << message;
        throw common::InvalidPlanner("Planner id " + planner_id + " failed to find a path");
    }

    return path;
}

void PlannerServer::WaitForCostmap()
{
    // // Don't compute a plan until costmap is valid (after clear costmap)
    // rclcpp::Rate r(100);
    // auto waiting_start = now();
    // while (!costmap_ros_->isCurrent()) {
    //     if (now() - waiting_start > costmap_update_timeout_) {
    //         throw nav2_core::PlannerTimedOut("Costmap timed out waiting for update");
    //     }
    //     r.sleep();
    // }
}

bool PlannerServer::TransformPosesToGlobalFrame(
    commsgs::geometry_msgs::PoseStamped& curr_start,
    commsgs::geometry_msgs::PoseStamped& curr_goal)
{
    // if (!costmap_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    //     !costmap_->transformPoseToGlobalFrame(curr_goal, curr_goal))
    // {
    //     return false;
    // }

    return true;
}

bool PlannerServer::ValidatePath(
    const commsgs::geometry_msgs::PoseStamped& curr_goal,
    const commsgs::planning_msgs::Path& path,
    const std::string& planner_id)
{
    if (path.poses.empty()) {
        LOG(WARNING) << "Planning algorithm " << planner_id << " failed to generate a valid path to ("
                     << curr_goal.pose.position.x << ", " << curr_goal.pose.position.y << ")";
        return false;
    }

    LOG(INFO) << "Found valid path of size " << path.poses.size() << " to ("
              << curr_goal.pose.position.x << ", " << curr_goal.pose.position.y << ")";
    return true;
}

void PlannerServer::ComputePlan(const commsgs::geometry_msgs::PoseStamped& request, commsgs::planning_msgs::Path& response)
{
    auto start_time = Time::Now();
    WaitForCostmap();

    commsgs::geometry_msgs::PoseStamped start_pose;
    commsgs::geometry_msgs::PoseStamped goal_pose;

    auto cancel_checker = [this]() {
        // return action_server_pose_->is_cancel_requested();
        return true;
    };

    try {
        GetPlan(start_pose, goal_pose, "", cancel_checker);
    } 
    catch (common::InvalidPlanner& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::StartOccupied& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::GoalOccupied& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::NoValidPathCouldBeFound & ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::PlannerTimedOut& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::StartOutsideMapBounds & ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::GoalOutsideMapBounds & ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::PlannerTFError& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (common::PlannerCancelled& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    } catch (std::exception& ex) {
        ExceptionWarning(start_pose, goal_pose, "", ex);
    }

}

void PlannerServer::ComputePlanThroughPoses(const commsgs::geometry_msgs::PoseStamped& request, commsgs::planning_msgs::Path& response)
{
    // TODO: Implement compute plan through poses
}

void PlannerServer::PublishPlan(const commsgs::planning_msgs::Path& path)
{
    LOG(INFO) << "PublishPlan (publisher disabled)";
}

void PlannerServer::ExceptionWarning(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string & planner_id,
    const std::exception & ex)
{
    LOG(WARNING) << absl::StrCat(planner_id, " plugin failed to plan from (", 
        start.pose.position.x, start.pose.position.y, ") to (", 
        goal.pose.position.x, goal.pose.position.y, "): ", ex.what());
}

}  // namespace planning
}  // namespace autonomy