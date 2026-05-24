/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigator/exploration/explore_to_anywhere.hpp"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/navigator/utils/navigator_utils.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace exploration {

ExploreToAnywhereNavigator::ExploreToAnywhereNavigator(
    const proto::TaskOptions& options,
    const std::shared_ptr<common::TaskContext>& task_context,
    const std::vector<std::string>& plugin_lib_names,
    const common::FeedbackUtils& feedback_utils,
    const std::shared_ptr<common::NavigatorMuxer>& muxer,
    std::shared_ptr<common::OdomSmoother> odom_smoother)
    : BehaviorTreeNavigator<ActionT>("explore_to_anywhere",
                                     "explore_to_anywhere.xml", options,
                                     task_context, plugin_lib_names,
                                     feedback_utils, muxer, odom_smoother) {
    if (options.has_explore_to_anywhere_options()) {
        const auto& nav_opts = options.explore_to_anywhere_options();
        if (!nav_opts.explore_goal_blackboard_key().empty()) {
            explore_goal_blackboard_id_ = nav_opts.explore_goal_blackboard_key();
        }
    }
}

bool ExploreToAnywhereNavigator::GoalReceived(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    AUTONOMY_UNUSED(goal);
    std::string bt_xml = bt_->GetDefaultBTFilename();
    bt_xml = utils::ResolveBehaviorTreeFile(bt_xml, feedback_utils_);
    if (!bt_->LoadBehaviorTree(bt_xml)) {
        bt_->SetInternalError(
            static_cast<uint16_t>(
                behavior_tree::proto::EXPLORE_TO_ANYWHERE_ERROR_UNKNOWN),
            "Failed to load explore behavior tree.");
        return false;
    }
    start_time_ = std::chrono::steady_clock::now();
    auto blackboard = bt_->GetBlackboard();
    if (blackboard) {
        blackboard->set("number_recoveries", 0);  // NOLINT
        blackboard->set("initial_pose_received", true);  // NOLINT
    }
    return true;
}

void ExploreToAnywhereNavigator::GoalCompleted(
    std::shared_ptr<typename ActionT::Result> result,
    const common::BtStatus /*final_bt_status*/) {
    if (result && result->error_code() !=
                      static_cast<int>(
                          behavior_tree::proto::EXPLORE_TO_ANYWHERE_ERROR_NONE)) {
        AWARN << "ExploreToAnywhere error " << result->error_code() << ": "
              << result->error_msg();
    }
}

void ExploreToAnywhereNavigator::OnLoop() {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_);
    *feedback->mutable_exploration_time() = commsgs::builtin_interfaces::ToProto(
        commsgs::builtin_interfaces::Duration::FromSeconds(elapsed.count()));
    bt_->PublishFeedback(feedback);
}

void ExploreToAnywhereNavigator::OnPreempt(
    std::shared_ptr<const typename ActionT::Goal> /*goal*/) {
    bt_->TerminatePendingGoal();
}

void ExploreToAnywhereNavigator::UpdateExploreGoal(
    const commsgs::geometry_msgs::PoseStamped& explore_goal) {
    auto blackboard = bt_->GetBlackboard();
    if (!blackboard) {
        return;
    }
    blackboard->set(explore_goal_blackboard_id_, explore_goal);  // NOLINT
    blackboard->set("explore_goal", explore_goal);               // NOLINT
    blackboard->set("goal", explore_goal);                       // NOLINT
}

}  // namespace exploration
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
