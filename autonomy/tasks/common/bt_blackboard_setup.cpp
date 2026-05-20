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

namespace autonomy {
namespace tasks {
namespace common {

void SetupNavigateToPoseBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback) {
    if (!blackboard) {
        return;
    }

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

    commsgs::planning_msgs::Path empty_path;
    blackboard->set("path", empty_path);  // NOLINT
    blackboard->set("goal", commsgs::geometry_msgs::PoseStamped{});  // NOLINT

    const std::string controller_id =
        task_context && !task_context->selected_controller_id.empty()
            ? task_context->selected_controller_id
            : "FollowPath";
    const std::string planner_id =
        task_context && !task_context->selected_planner_id.empty()
            ? task_context->selected_planner_id
            : "navfn_planner";

    blackboard->set("selected_controller", controller_id);  // NOLINT
    blackboard->set("selected_planner", planner_id);        // NOLINT

    blackboard->set("compute_path_error_code", int32_t{0});  // NOLINT
    blackboard->set("compute_path_error_msg", std::string{});  // NOLINT
    blackboard->set("follow_path_error_code", int32_t{0});     // NOLINT
    blackboard->set("follow_path_error_msg", std::string{});   // NOLINT

    blackboard->set("initial_pose_received", false);  // NOLINT
    blackboard->set("number_recoveries", 0);          // NOLINT
}

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
