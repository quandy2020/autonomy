/*
 * Copyright 2026 The Openbot Authors
 *
 * NavigationTask — behavior-tree navigator for single-pose and through-poses
 * goals. Mirrors Nav2 BtNavigator: orchestrates planning / control via
 * NavigationClient while TaskServer owns exclusive activation.
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/proto/navigation.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class NavigationTask : public BtTaskApp<
                           ::autonomy::task::proto::NavigationGoal,
                           ::autonomy::task::proto::NavigationFeedback,
                           ::autonomy::task::proto::NavigationResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigationTask);

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

protected:

    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::NavigationGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::NavigationFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::NavigationResult* result) const override;

private:
    ::autonomy::task::proto::NavigationStatus MapStatus() const;
    std::string ResolveTreeForGoal(
        const ::autonomy::task::proto::NavigationGoal& goal) const;
    bool EnsureNavigationClient();

    std::optional<::autonomy::task::proto::NavigationGoal> active_goal_;
    float initial_distance_{-1.f};
};

}  // namespace task
}  // namespace autonomy
