/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigators/navigate_to_pose.hpp"

#include <chrono>
#include <cmath>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {

namespace task_proto = behavior_tree::task_proto;

namespace {

commsgs::builtin_interfaces::Duration ElapsedDuration(
    std::chrono::steady_clock::time_point start) {
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)
                        .count();
    return commsgs::builtin_interfaces::Duration::FromNanoseconds(ns);
}

float Distance2D(const commsgs::geometry_msgs::PoseStamped& a,
                 const commsgs::geometry_msgs::PoseStamped& b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return static_cast<float>(std::hypot(dx, dy));
}

}  // namespace

NavigateToPoseNavigator::NavigateToPoseNavigator(
    std::shared_ptr<autolink::Node> node,
    std::shared_ptr<behavior_tree::BtEngine> engine,
    std::shared_ptr<behavior_tree::BtContext> context, NavigatorMuxer* muxer,
    const std::string& default_tree_xml)
    : BehaviorTreeNavigator("navigate_to_pose", std::move(node), std::move(engine),
                            std::move(context), muxer, default_tree_xml) {}

bool NavigateToPoseNavigator::InitializeGoal(GoalPtr goal) {
    if (!goal || !action_server_) {
        return false;
    }
    const std::string bt_xml = goal->behavior_tree();
    if (!bt_xml.empty() && !action_server_->LoadBehaviorTree(bt_xml)) {
        action_server_->SetInternalError(
            task_proto::NAV_TO_POSE_FAILED_TO_LOAD_BEHAVIOR_TREE,
            "Error loading XML file: " + bt_xml);
        return false;
    }
    const commsgs::geometry_msgs::PoseStamped pose =
        goal->has_pose() ? commsgs::geometry_msgs::FromProto(goal->pose())
                         : commsgs::geometry_msgs::PoseStamped{};
    action_server_->GetBlackboard()->set(behavior_tree::kBlackboardGoalKey, pose);
    if (context_) {
        context_->number_recoveries = 0;
        action_server_->GetBlackboard()->set(
            behavior_tree::kBlackboardNumberRecoveriesKey, 0);
    }
    return true;
}

bool NavigateToPoseNavigator::OnGoalReceived(GoalPtr goal) {
    return InitializeGoal(goal);
}

void NavigateToPoseNavigator::OnLoop() {
    if (!context_ || !action_server_) {
        return;
    }
    commsgs::geometry_msgs::PoseStamped pose;
    if (!utils::getGlobalRobotPose(
            pose, context_->tf_buffer, context_->controller->GetOdomSmoother(),
            context_->options.global_frame(),
            context_->options.robot_base_frame())) {
        return;
    }
    action_server_->GetBlackboard()->set("current_pose", pose);

    commsgs::geometry_msgs::PoseStamped goal;
    if (!action_server_->GetBlackboard()->get(behavior_tree::kBlackboardGoalKey,
                                              goal)) {
        return;
    }
    task_proto::NavigateToPoseAction_Feedback fb;
    *fb.mutable_current_pose() = commsgs::geometry_msgs::ToProto(pose);
    *fb.mutable_navigation_time() = commsgs::builtin_interfaces::ToProto(
        ElapsedDuration(context_->navigation_start));
    fb.set_number_of_recoveries(context_->number_recoveries);
    fb.set_distance_remaining(Distance2D(pose, goal));
    auto feedback = std::make_shared<
        behavior_tree::NavigateToPoseActionTraits::Feedback>(std::move(fb));
    action_server_->PublishFeedback(feedback);
}

void NavigateToPoseNavigator::OnPreempt(GoalPtr goal) {
    if (!goal || !action_server_) {
        return;
    }
    const std::string requested_bt = goal->behavior_tree();
    const bool same_bt =
        requested_bt == action_server_->GetCurrentBTFilename() ||
        (requested_bt.empty() &&
         action_server_->GetCurrentBTFilename() ==
             action_server_->GetDefaultBTFilename());
    if (!same_bt) {
        AWARN << "Preemption rejected: different behavior_tree XML.";
        action_server_->TerminatePendingGoal();
        return;
    }
    if (!InitializeGoal(action_server_->AcceptPendingGoal())) {
        AWARN << "Preemption: failed to apply pending goal.";
        action_server_->TerminatePendingGoal();
    }
}

void NavigateToPoseNavigator::OnCompleted(ResultPtr result,
                                          behavior_tree::RunStatus status) {
    if (!result) {
        return;
    }
    if (action_server_->PopulateInternalError(result)) {
        return;
    }
    switch (status) {
        case behavior_tree::RunStatus::SUCCEEDED:
            result->set_error_code(task_proto::NAV_TO_POSE_NONE);
            break;
        case behavior_tree::RunStatus::CANCELED:
            result->set_error_code(task_proto::NAV_TO_POSE_CANCELED);
            break;
        default:
            if (result->error_code() == task_proto::NAV_TO_POSE_NONE) {
                result->set_error_code(task_proto::NAV_TO_POSE_UNKNOWN);
                result->set_error_msg("Behavior tree navigation failed.");
            }
            break;
    }
}

}  // namespace tasks
}  // namespace autonomy
