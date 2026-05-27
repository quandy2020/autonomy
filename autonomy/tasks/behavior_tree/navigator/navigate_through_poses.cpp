/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/navigator/navigate_through_poses.hpp"

#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

bool NavigateThroughPosesNavigator::OnGoalReceived() {
    if (!action_server_ || goals_.empty()) {
        return false;
    }
    auto bb = action_server_->GetBlackboard();
    bb->set(kBlackboardGoalsKey, goals_);
    bb->set(kBlackboardGoalKey, goals_.back());
    return true;
}

void NavigateThroughPosesNavigator::OnLoop() {
    if (!ctx_ || !action_server_) {
        return;
    }
    commsgs::geometry_msgs::PoseStamped pose;
    if (utils::getGlobalRobotPose(
            pose, ctx_->tf_buffer, ctx_->controller->GetOdomSmoother(),
            ctx_->options.global_frame(), ctx_->options.robot_base_frame())) {
        action_server_->GetBlackboard()->set("current_pose", pose);
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
