/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/navigator/tracking/track_to_target.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/tasks/navigator/utils/navigator_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace tracking {

TrackToTargetNavigator::TrackToTargetNavigator(
    const proto::TaskOptions& options,
    const std::shared_ptr<common::TaskContext>& task_context,
    const std::vector<std::string>& plugin_lib_names,
    const common::FeedbackUtils& feedback_utils,
    const std::shared_ptr<common::NavigatorMuxer>& muxer,
    std::shared_ptr<common::OdomSmoother> odom_smoother)
    : BehaviorTreeNavigator<ActionT>("track_to_target", "track_to_target.xml",
                                     options, task_context, plugin_lib_names,
                                     feedback_utils, muxer, odom_smoother) {
    if (options.has_track_to_target_options()) {
        const auto& nav_opts = options.track_to_target_options();
        if (!nav_opts.target_pose_blackboard_key().empty()) {
            target_pose_blackboard_id_ = nav_opts.target_pose_blackboard_key();
        }
    }
}

bool TrackToTargetNavigator::GoalReceived(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (!goal) {
        return false;
    }
    target_id_ = goal->target_id();
    const std::string bt_xml =
        utils::ResolveBehaviorTreeFile(bt_->GetDefaultBTFilename(),
                                       feedback_utils_);
    if (!bt_->LoadBehaviorTree(bt_xml)) {
        return false;
    }
    auto blackboard = bt_->GetBlackboard();
    if (blackboard) {
        blackboard->set("number_recoveries", 0);  // NOLINT
        blackboard->set("initial_pose_received", true);  // NOLINT
    }
    return true;
}

void TrackToTargetNavigator::GoalCompleted(
    std::shared_ptr<typename ActionT::Result> result,
    const common::BtStatus /*final_bt_status*/) {
    if (result && result->error_code() !=
                      static_cast<int>(
                          behavior_tree::proto::TRACK_TO_TARGET_ERROR_NONE)) {
        AWARN << "TrackToTarget error " << result->error_code() << ": "
              << result->error_msg();
    }
}

void TrackToTargetNavigator::OnLoop() {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    feedback->set_current_target_id(target_id_);
    bt_->PublishFeedback(feedback);
}

void TrackToTargetNavigator::OnPreempt(
    std::shared_ptr<const typename ActionT::Goal> goal) {
    if (goal) {
        target_id_ = goal->target_id();
        bt_->AcceptPendingGoal();
    } else {
        bt_->TerminatePendingGoal();
    }
}

}  // namespace tracking
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
