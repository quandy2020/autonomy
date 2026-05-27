/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/navigate_through_poses_navigator.hpp"

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

bool NavigateThroughPosesNavigator::OnGoalReceived() {
    if (!bt_server_ || goals_.empty()) {
        return false;
    }
    if (ctx_ && ctx_->options.has_navigate_through_poses() &&
        !ctx_->options.navigate_through_poses().goals_blackboard_key().empty()) {
        goals_blackboard_key_ =
            ctx_->options.navigate_through_poses().goals_blackboard_key();
    }
    auto bb = bt_server_->GetBlackboard();
    bb->set(goals_blackboard_key_, goals_);
    bb->set(kBlackboardGoalsKey, goals_);
    if (!goals_.empty()) {
        bb->set(kBlackboardGoalKey, goals_.back());
    }
    return true;
}

void NavigateThroughPosesNavigator::OnLoop() {
    if (!ctx_ || !bt_server_) {
        return;
    }
    commsgs::geometry_msgs::PoseStamped pose;
    if (utils::getGlobalRobotPose(
            pose, ctx_->tf_buffer, ctx_->controller->GetOdomSmoother(),
            ctx_->options.global_frame(), ctx_->options.robot_base_frame())) {
        bt_server_->GetBlackboard()->set("current_pose", pose);
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
