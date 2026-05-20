/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigator/docking/navigate_to_docking.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/navigator/utils/navigator_utils.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace docking {

NavigateToDockingNavigator::NavigateToDockingNavigator(
    const proto::TaskOptions& options,
    const std::shared_ptr<common::TaskContext>& task_context,
    const std::vector<std::string>& plugin_lib_names,
    const common::FeedbackUtils& feedback_utils,
    const std::shared_ptr<common::NavigatorMuxer>& muxer,
    std::shared_ptr<common::OdomSmoother> odom_smoother)
    : BehaviorTreeNavigator<ActionT>("navigate_to_docking",
                                     "navigate_to_dock.xml", options,
                                     task_context, plugin_lib_names,
                                     feedback_utils, muxer, odom_smoother) {}

bool NavigateToDockingNavigator::GoalReceived(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!goal) {
        return false;
    }
    std::string bt_xml = bt_->GetDefaultBTFilename();
    bt_xml = utils::ResolveBehaviorTreeFile(bt_xml, feedback_utils_);
    if (!bt_->LoadBehaviorTree(bt_xml)) {
        return false;
    }
    start_time_ = std::chrono::steady_clock::now();
    auto blackboard = bt_->GetBlackboard();
    if (blackboard) {
        blackboard->set("number_recoveries", 0);  // NOLINT
        blackboard->set("dock_id", goal->dock_id());  // NOLINT
        blackboard->set(
            "dock_pose",
            commsgs::geometry_msgs::FromProto(goal->dock_pose()));  // NOLINT
        blackboard->set("initial_pose_received", true);  // NOLINT
    }
    return true;
}

void NavigateToDockingNavigator::GoalCompleted(
    std::shared_ptr<typename ActionT::Result> result,
    const common::BtStatus /*final_bt_status*/) {
    if (result && result->error_code() !=
                      static_cast<int>(
                          behavior_tree::proto::DOCK_ROBOT_ERROR_NONE)) {
        AWARN << "NavigateToDocking error " << result->error_code() << ": "
              << result->error_msg();
    }
}

void NavigateToDockingNavigator::OnLoop() {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_);
    *feedback->mutable_docking_time() = commsgs::builtin_interfaces::ToProto(
        commsgs::builtin_interfaces::Duration::FromSeconds(elapsed.count()));
    bt_->PublishFeedback(feedback);
}

void NavigateToDockingNavigator::OnPreempt(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (goal && GoalReceived(goal)) {
        bt_->AcceptPendingGoal();
    } else {
        bt_->TerminatePendingGoal();
    }
}

}  // namespace docking
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
