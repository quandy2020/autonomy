/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigator.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class NavigateToPoseNavigator : public Navigator
{
public:
    std::string GetName() const override { return "navigate_to_pose"; }

    void SetGoal(const commsgs::geometry_msgs::PoseStamped& goal) {
        goal_ = goal;
    }

protected:
    bool OnGoalReceived() override;
    void OnLoop() override;

    commsgs::geometry_msgs::PoseStamped goal_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
