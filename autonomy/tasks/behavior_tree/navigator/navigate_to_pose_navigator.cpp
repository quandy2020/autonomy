/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/navigate_to_pose_navigator.hpp"

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

bool NavigateToPoseNavigator::OnGoalReceived() {
    if (!bt_server_) {
        return false;
    }
    if (ctx_ && ctx_->options.has_navigate_to_pose() &&
        !ctx_->options.navigate_to_pose().goal_blackboard_key().empty()) {
        goal_blackboard_key_ =
            ctx_->options.navigate_to_pose().goal_blackboard_key();
    }
    auto bb = bt_server_->GetBlackboard();
    bb->set(goal_blackboard_key_, goal_);
    bb->set(kBlackboardGoalKey, goal_);
    return true;
}

void NavigateToPoseNavigator::OnLoop() {
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
