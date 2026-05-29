/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"

namespace autonomy {
namespace tasks {

/**
 * @brief Multi-waypoint BT navigator
 * (nav2_bt_navigator::NavigateThroughPosesNavigator).
 */
class NavigateThroughPosesNavigator
    : public BehaviorTreeNavigator<behavior_tree::NavigateThroughPosesActionTraits>
{
public:
    NavigateThroughPosesNavigator(
        std::shared_ptr<autolink::Node> node,
        std::shared_ptr<behavior_tree::BtEngine> engine,
        std::shared_ptr<behavior_tree::BtContext> context, NavigatorMuxer* muxer,
        const std::string& default_tree_xml);

protected:
    bool InitializeGoal(GoalPtr goal);
    bool OnGoalReceived(GoalPtr goal) override;
    void OnLoop() override;
    void OnPreempt(GoalPtr goal) override;
    void OnCompleted(ResultPtr result, behavior_tree::RunStatus status) override;
};

}  // namespace tasks
}  // namespace autonomy
