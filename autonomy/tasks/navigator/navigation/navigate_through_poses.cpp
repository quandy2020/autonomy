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

#include "autonomy/tasks/navigator/navigation/navigate_through_poses.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace navigation {

NavigateThroughPosesNavigator::NavigateThroughPosesNavigator(
    const proto::TaskOptions& options,
    const std::shared_ptr<common::TaskContext>& task_context,
    const std::vector<std::string>& plugin_lib_names,
    const common::FeedbackUtils& feedback_utils,
    const std::shared_ptr<common::NavigatorMuxer>& muxer,
    std::shared_ptr<common::OdomSmoother> odom_smoother)
    : BtNavigator<ActionT>("navigate_through_poses",
                                     "navigate_through_poses.xml", options,
                                     task_context, plugin_lib_names,
                                     feedback_utils, muxer, odom_smoother),
      odom_smoother_(odom_smoother) {
    if (options.has_navigate_through_poses_options()) {
        const auto& nav_opts = options.navigate_through_poses_options();
        if (!nav_opts.goals_blackboard_key().empty()) {
            goals_blackboard_id_ = nav_opts.goals_blackboard_key();
        }
        if (!nav_opts.path_blackboard_key().empty()) {
            path_blackboard_id_ = nav_opts.path_blackboard_key();
        }
    }
}

bool NavigateThroughPosesNavigator::GoalReceived(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!goal) {
        return false;
    }
    std::string bt_xml = goal->behavior_tree().empty()
                             ? GetDefaultBTFilename()
                             : goal->behavior_tree();
    bt_xml = common::ResolveBehaviorTreeFile(bt_xml, feedback_utils_);
    if (!LoadBehaviorTree(bt_xml)) {
        SetInternalError(
            static_cast<uint16_t>(proto::
                                      NAVIGATE_THROUGH_POSES_ERROR_FAILED_TO_LOAD_BEHAVIOR_TREE),
            "Error loading XML file: " + bt_xml);
        return false;
    }
    return InitializeGoalPoses(goal);
}

void NavigateThroughPosesNavigator::GoalCompleted(
    std::shared_ptr<typename ActionT::Result> result,
    const common::BtStatus /*final_bt_status*/) {
    if (result && result->error_code() !=
                      static_cast<int>(proto::
                                           NAVIGATE_THROUGH_POSES_ERROR_NONE)) {
        AWARN << "NavigateThroughPoses completed with error "
              << result->error_code() << ": " << result->error_msg();
    }
}

void NavigateThroughPosesNavigator::OnLoop() {
    auto feedback_msg = std::make_shared<typename ActionT::Feedback>();
    commsgs::geometry_msgs::PoseStamped current_pose;
    if (feedback_utils_.tf &&
        autonomy::tasks::utils::getCurrentPose(
            current_pose, feedback_utils_.tf, feedback_utils_.global_frame,
            feedback_utils_.robot_frame,
            static_cast<float>(feedback_utils_.transform_tolerance))) {
        *feedback_msg->mutable_current_pose() =
            commsgs::geometry_msgs::ToProto(current_pose);
    }
    auto now = std::chrono::steady_clock::now();
    *feedback_msg->mutable_navigation_time() =
        commsgs::builtin_interfaces::ToProto(
            commsgs::builtin_interfaces::Duration::FromSeconds(
                std::chrono::duration<double>(now - start_time_).count()));
    PublishFeedback(feedback_msg);
}

void NavigateThroughPosesNavigator::OnPreempt(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (goal && InitializeGoalPoses(goal)) {
        AcceptPendingGoal();
    } else {
        TerminatePendingGoal();
    }
}

bool NavigateThroughPosesNavigator::InitializeGoalPoses(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!goal || goal->poses().goals().empty()) {
        SetInternalError(
            static_cast<uint16_t>(proto::
                                      NAVIGATE_THROUGH_POSES_ERROR_INVALID_GOALS),
            "NavigateThroughPoses requires at least one pose.");
        return false;
    }

    commsgs::planning_msgs::Goals goals;
    for (const auto& pose : goal->poses().goals()) {
        goals.goals.push_back(commsgs::geometry_msgs::FromProto(pose));
    }

    start_time_ = std::chrono::steady_clock::now();
    auto blackboard = GetBlackboard();
    if (blackboard) {
        blackboard->set("number_recoveries", 0);  // NOLINT
        blackboard->set(goals_blackboard_id_, goals);  // NOLINT
        blackboard->set("goals", goals);               // NOLINT
        commsgs::planning_msgs::Path empty_path;
        blackboard->set(path_blackboard_id_, empty_path);  // NOLINT
        blackboard->set("path", empty_path);             // NOLINT
        blackboard->set("initial_pose_received", true);   // NOLINT
    }
    return true;
}

}  // namespace navigation
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
