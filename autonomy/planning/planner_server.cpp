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
#include "autonomy/planning/common/planner_exceptions.hpp"

namespace autonomy {
namespace planning {

// proto::PlannerOptions CreatePlannerOptions(
//     ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
// {
//     proto::PlannerOption options;
//     return options;
// }

proto::PlannerOptions CreatePlannerOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::PlannerOptions options;
    return options;
}
    
PlannerServer::PlannerServer()
{

}

PlannerServer::~PlannerServer()
{

}

commsgs::planning_msgs::Path PlannerServer::GetPlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id,
    std::function<bool()> cancel_checker)
{
    commsgs::planning_msgs::Path path;

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
    return true;
}

void PlannerServer::ComputePlan()
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
    } catch (common::PlannerCancelled&) {
        
    } catch (std::exception& ex) {
        
    }

}

void PlannerServer::ComputePlanThroughPoses()
{

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