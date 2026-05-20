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
 *
 * Design aligned with nav2_bt_navigator/navigators/navigate_to_pose.cpp.
 */

#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"

#include <limits>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace navigation {

namespace {

std::string ResolveBehaviorTreeFile(
    const std::string& bt_file,
    const autonomy::tasks::common::FeedbackUtils& feedback) {
    if (bt_file.empty()) {
        return bt_file;
    }
    if (bt_file.find('/') != std::string::npos) {
        return bt_file;
    }
    if (feedback.bt_xml_path_resolver) {
        return feedback.bt_xml_path_resolver(bt_file);
    }
    return bt_file;
}

}  // namespace

std::shared_ptr<OdomSmoother> NavigateToPoseNavigator::ActiveOdomSmoother()
    const {
    if (odom_smoother_) {
        return odom_smoother_;
    }
    if (!bt_) {
        return nullptr;
    }
    const auto blackboard = bt_->GetBlackboard();
    if (!blackboard) {
        return nullptr;
    }
    std::shared_ptr<autonomy::tasks::common::TaskContext> ctx;
    if (blackboard->get("task_context", ctx) && ctx) {
        return ctx->odom_smoother;
    }
    return blackboard->get<std::shared_ptr<OdomSmoother>>("odom_smoother");
}

NavigateToPoseNavigator::NavigateToPoseNavigator(
    const autonomy::tasks::proto::TaskOptions& options,
    const std::shared_ptr<autonomy::tasks::common::TaskContext>& task_context,
    const std::vector<std::string>& plugin_lib_names,
    const autonomy::tasks::common::FeedbackUtils& feedback_utils,
    const std::shared_ptr<autonomy::tasks::common::NavigatorMuxer>& muxer,
    std::shared_ptr<OdomSmoother> odom_smoother)
    : BehaviorTreeNavigator<ActionT>("navigate_to_pose", "navigate_to_pose.xml",
                                     options, task_context, plugin_lib_names,
                                     feedback_utils, muxer, odom_smoother),
      odom_smoother_(odom_smoother) {
    goal_blackboard_id_ = "goal";
    path_blackboard_id_ = "path";
    if (options.has_navigate_to_pose_options()) {
        const auto& nav_opts = options.navigate_to_pose_options();
        if (!nav_opts.goal_blackboard_key().empty()) {
            goal_blackboard_id_ = nav_opts.goal_blackboard_key();
        }
        if (!nav_opts.path_blackboard_key().empty()) {
            path_blackboard_id_ = nav_opts.path_blackboard_key();
        }
    }
}

bool NavigateToPoseNavigator::GoalReceived(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!goal) {
        AERROR << "NavigateToPoseNavigator: received null goal";
        return false;
    }

    std::string bt_xml = goal->behavior_tree().empty()
                             ? bt_->GetDefaultBTFilename()
                             : goal->behavior_tree();
    bt_xml = ResolveBehaviorTreeFile(bt_xml, feedback_utils_);
    if (!bt_->LoadBehaviorTree(bt_xml)) {
        bt_->SetInternalError(
            static_cast<uint16_t>(
                behavior_tree::proto::
                    NAVIGATE_TO_POSE_ERROR_FAILED_TO_LOAD_BEHAVIOR_TREE),
            "Error loading XML file: " + bt_xml + ". Navigation canceled.");
        return false;
    }
    return InitializeGoalPose(goal);
}

void NavigateToPoseNavigator::GoalCompleted(
    std::shared_ptr<typename ActionT::Result> result,
    const BtStatus /*final_bt_status*/) {
    if (!result)
        return;
    if (result->error_code() ==
        static_cast<int>(behavior_tree::proto::NAVIGATE_TO_POSE_ERROR_NONE)) {
        if (bt_->PopulateInternalError(result)) {
            AWARN << "NavigateToPoseNavigator::GoalCompleted: internal error "
                  << result->error_code() << ":" << result->error_msg();
        }
    } else {
        AWARN << "NavigateToPoseNavigator::GoalCompleted error "
              << result->error_code() << ":" << result->error_msg();
    }
}

void NavigateToPoseNavigator::OnLoop() {
    auto feedback_msg = std::make_shared<typename ActionT::Feedback>();

    commsgs::geometry_msgs::PoseStamped current_pose;
    auto blackboard = bt_->GetBlackboard();
    if (!feedback_utils_.tf ||
        !autonomy::tasks::utils::getGlobalRobotPose(
            current_pose, feedback_utils_.tf, ActiveOdomSmoother(),
            feedback_utils_.global_frame, feedback_utils_.robot_frame,
            static_cast<float>(feedback_utils_.transform_tolerance))) {
        if (blackboard) {
            blackboard->set("initial_pose_received", false);  // NOLINT
        }
        return;
    }
    if (blackboard) {
        blackboard->set("initial_pose_received", true);  // NOLINT
    }

    commsgs::planning_msgs::Path current_path;
    double distance_remaining = 0.0;
    if (blackboard && blackboard->get(path_blackboard_id_, current_path) &&
        !current_path.poses.empty()) {
        auto find_closest = [&current_pose, &current_path]() {
            size_t idx = 0;
            double min_d = std::numeric_limits<double>::max();
            for (size_t i = 0; i < current_path.poses.size(); ++i) {
                double d = map::costmap_2d::utils::euclidean_distance(
                    current_pose, current_path.poses[i]);
                if (d < min_d) {
                    min_d = d;
                    idx = i;
                }
            }
            return idx;
        };
        distance_remaining = map::costmap_2d::utils::calculate_path_length(
            current_path, find_closest());
        commsgs::builtin_interfaces::Duration estimated_remaining =
            commsgs::builtin_interfaces::Duration::FromSeconds(0.0);
        if (odom_smoother_) {
            auto twist = odom_smoother_->getTwist();
            double speed = std::hypot(twist.linear.x, twist.linear.y);
            if (std::abs(speed) > 0.01 && distance_remaining > 0.1) {
                estimated_remaining =
                    commsgs::builtin_interfaces::Duration::FromSeconds(
                        distance_remaining / std::abs(speed));
            }
        }
        feedback_msg->set_distance_remaining(
            static_cast<float>(distance_remaining));
        *feedback_msg->mutable_estimated_time_remaining() =
            commsgs::builtin_interfaces::ToProto(estimated_remaining);
    }

    int recovery_count = 0;
    if (blackboard && !blackboard->get("number_recoveries", recovery_count)) {
        recovery_count = 0;
    }
    feedback_msg->set_number_of_recoveries(recovery_count);
    *feedback_msg->mutable_current_pose() =
        autonomy::commsgs::geometry_msgs::ToProto(current_pose);

    auto now = std::chrono::steady_clock::now();
    auto nav_duration =
        std::chrono::duration<double>(now - start_time_).count();
    *feedback_msg->mutable_navigation_time() =
        commsgs::builtin_interfaces::ToProto(
            commsgs::builtin_interfaces::Duration::FromSeconds(nav_duration));

    bt_->PublishFeedback(feedback_msg);
}

void NavigateToPoseNavigator::OnPreempt(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    AUTONOMY_UNUSED(goal);
    AINFO << "NavigateToPoseNavigator: preempt requested";
    const std::string current_bt = bt_->GetCurrentBTFilename();
    const std::string default_bt = bt_->GetDefaultBTFilename();
    bool same_bt = (goal && !goal->behavior_tree().empty())
                       ? (goal->behavior_tree() == current_bt)
                       : (current_bt == default_bt);
    if (same_bt) {
        auto pending = bt_->AcceptPendingGoal();
        if (!pending || !InitializeGoalPose(pending)) {
            AWARN << "Preemption: could not transform goal pose, continuing "
                     "previous goal.";
            bt_->TerminatePendingGoal();
        }
    } else {
        AWARN << "Preemption rejected (different BT requested). Cancel current "
                 "goal and send new "
                 "request to use another BT.";
        bt_->TerminatePendingGoal();
    }
}

bool NavigateToPoseNavigator::InitializeGoalPose(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!goal || !feedback_utils_.tf)
        return false;

    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!autonomy::tasks::utils::getGlobalRobotPose(
            current_pose, feedback_utils_.tf, ActiveOdomSmoother(),
            feedback_utils_.global_frame, feedback_utils_.robot_frame,
            static_cast<float>(feedback_utils_.transform_tolerance))) {
        bt_->SetInternalError(
            static_cast<uint16_t>(
                behavior_tree::proto::NAVIGATE_TO_POSE_ERROR_TF_ERROR),
            "Initial robot pose is not available.");
        return false;
    }

    commsgs::geometry_msgs::PoseStamped goal_pose;
    commsgs::geometry_msgs::PoseStamped goal_in =
        autonomy::commsgs::geometry_msgs::FromProto(goal->pose());
    if (!autonomy::tasks::utils::transformPoseInTargetFrame(
            goal_in, goal_pose, feedback_utils_.tf,
            feedback_utils_.global_frame,
            static_cast<float>(feedback_utils_.transform_tolerance))) {
        bt_->SetInternalError(
            static_cast<uint16_t>(
                behavior_tree::proto::NAVIGATE_TO_POSE_ERROR_TF_ERROR),
            "Failed to transform goal from frame '" +
                goal->pose().header().frame_id() + "' to global frame '" +
                feedback_utils_.global_frame + "'.");
        return false;
    }

    AINFO << "NavigateToPose: from (" << current_pose.pose.position.x << ", "
          << current_pose.pose.position.y << ") to ("
          << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
          << ")";

    start_time_ = std::chrono::steady_clock::now();
    auto blackboard = bt_->GetBlackboard();
    if (blackboard) {
        blackboard->set("number_recoveries", 0);  // NOLINT
        blackboard->set(goal_blackboard_id_, goal_pose);  // NOLINT
        blackboard->set("goal", goal_pose);  // NOLINT navigate_to_pose.xml
        commsgs::planning_msgs::Path empty_path;
        blackboard->set(path_blackboard_id_, empty_path);  // NOLINT
        blackboard->set("path", empty_path);  // NOLINT
        blackboard->set("initial_pose_received", true);  // NOLINT
    }
    return true;
}

}  // namespace navigation
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy