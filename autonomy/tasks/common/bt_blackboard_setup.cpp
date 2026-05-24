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

#include "autonomy/tasks/common/bt_blackboard_setup.hpp"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace common {
namespace {

void SetupCommonBlackboardKeys(
    const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    const auto bt_loop_ms = options.bt_loop_duration() > 0
                                ? std::chrono::milliseconds(
                                      options.bt_loop_duration())
                                : std::chrono::milliseconds(10);
    const auto server_timeout_ms =
        options.default_server_timeout() > 0
            ? std::chrono::milliseconds(options.default_server_timeout())
            : std::chrono::milliseconds(20);
    const auto wait_for_service_ms =
        options.wait_for_service_timeout() > 0
            ? std::chrono::milliseconds(options.wait_for_service_timeout())
            : std::chrono::milliseconds(1000);

    blackboard->set("bt_loop_duration", bt_loop_ms);              // NOLINT
    blackboard->set("server_timeout", server_timeout_ms);       // NOLINT
    blackboard->set("default_server_timeout", server_timeout_ms);  // NOLINT
    blackboard->set("wait_for_service_timeout", wait_for_service_ms);  // NOLINT

    blackboard->set("global_frame", feedback.global_frame);     // NOLINT
    blackboard->set("robot_base_frame", feedback.robot_frame);  // NOLINT
    blackboard->set("local_survival_timeout",
                    feedback.local_survival_timeout);  // NOLINT

    const std::string controller_id =
        !options.default_controller_id().empty()
            ? options.default_controller_id()
            : (task_context && !task_context->selected_controller_id.empty()
                   ? task_context->selected_controller_id
                   : "FollowPath");
    const std::string planner_id =
        !options.default_planner_id().empty()
            ? options.default_planner_id()
            : (task_context && !task_context->selected_planner_id.empty()
                   ? task_context->selected_planner_id
                   : "navfn_planner");

    blackboard->set("selected_controller", controller_id);  // NOLINT
    blackboard->set("selected_planner", planner_id);        // NOLINT

    if (options.goal_reached_tolerance() > 0.0) {
        blackboard->set("goal_reached_tol",  // NOLINT
                        options.goal_reached_tolerance());
    }

    commsgs::planning_msgs::Path empty_path;
    blackboard->set("path", empty_path);  // NOLINT

    blackboard->set("compute_path_error_code", int32_t{0});  // NOLINT
    blackboard->set("compute_path_error_msg", std::string{});  // NOLINT
    blackboard->set("follow_path_error_code", int32_t{0});     // NOLINT
    blackboard->set("follow_path_error_msg", std::string{});   // NOLINT
    blackboard->set("initial_pose_received", false);  // NOLINT
    blackboard->set("number_recoveries", 0);          // NOLINT
}

}  // namespace

void SetupNavigateToPoseBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    if (!blackboard) {
        return;
    }
    SetupCommonBlackboardKeys(blackboard, task_context, options, feedback);
    blackboard->set("goal", commsgs::geometry_msgs::PoseStamped{});  // NOLINT
}

void SetupNavigatorBlackboard(
    const std::string& navigator_id, const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    if (!blackboard) {
        return;
    }

    if (navigator_id == "navigate_to_pose") {
        SetupNavigateToPoseBlackboard(blackboard, task_context, options,
                                      feedback);
        return;
    }

    SetupCommonBlackboardKeys(blackboard, task_context, options, feedback);

    if (navigator_id == "navigate_through_poses") {
        blackboard->set("goals", commsgs::planning_msgs::Goals{});  // NOLINT
        blackboard->set("compute_path_through_poses_error_code",
                        int32_t{0});  // NOLINT
        return;
    }

    if (navigator_id == "track_to_target") {
        blackboard->set("target_pose",
                        commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        return;
    }

    if (navigator_id == "explore_to_anywhere") {
        blackboard->set("explore_goal",
                        commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        blackboard->set("goal", commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        return;
    }

    if (navigator_id == "navigate_to_docking") {
        blackboard->set("dock_id", std::string{});  // NOLINT
        blackboard->set("dock_pose",
                        commsgs::geometry_msgs::PoseStamped{});  // NOLINT
        return;
    }

    if (navigator_id == "teleop_drive") {
        double max_linear = 0.5;
        double max_angular = 1.5;
        double stale_timeout = 0.5;
        double projection_time = 1.5;
        double simulation_step = 0.1;
        if (options.has_teleop_drive_options()) {
            const auto& opts = options.teleop_drive_options();
            max_linear = opts.default_max_linear_vel();
            max_angular = opts.default_max_angular_vel();
            stale_timeout = opts.cmd_stale_timeout_sec();
            if (opts.projection_time_sec() > 0.0) {
                projection_time = opts.projection_time_sec();
            }
            if (opts.simulation_step_sec() > 0.0) {
                simulation_step = opts.simulation_step_sec();
            }
        }
        blackboard->set("teleop_mode",
                        static_cast<int>(behavior_tree::proto::TELEOP_MOTION_VELOCITY));  // NOLINT
        blackboard->set("teleop_time_allowance", 0.0);  // NOLINT
        blackboard->set("teleop_max_linear_vel", max_linear);  // NOLINT
        blackboard->set("teleop_max_angular_vel", max_angular);  // NOLINT
        blackboard->set("teleop_cmd_stale_timeout_sec", stale_timeout);  // NOLINT
        blackboard->set("teleop_linear_distance", 0.0);  // NOLINT
        blackboard->set("teleop_linear_signed", 0.0);  // NOLINT
        blackboard->set("teleop_linear_speed", max_linear);  // NOLINT
        blackboard->set("teleop_rotation_angle", 0.0);  // NOLINT
        blackboard->set("teleop_angular_speed", max_angular);  // NOLINT
        blackboard->set("teleop_disable_collision_checks", false);  // NOLINT
        blackboard->set("teleop_projection_time_sec", projection_time);  // NOLINT
        blackboard->set("teleop_simulation_step_sec", simulation_step);  // NOLINT
        return;
    }

    SetupNavigateToPoseBlackboard(blackboard, task_context, options, feedback);
}

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
