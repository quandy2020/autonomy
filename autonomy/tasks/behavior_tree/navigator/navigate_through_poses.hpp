/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/navigator/navigator.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class NavigateThroughPosesNavigator : public Navigator
{
public:
    std::string GetName() const override { return "navigate_through_poses"; }

    void SetGoals(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals) {
        goals_ = goals;
    }

protected:
    bool OnGoalReceived() override;
    void OnLoop() override;

    std::vector<commsgs::geometry_msgs::PoseStamped> goals_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
