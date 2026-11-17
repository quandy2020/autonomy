/*
 * Copyright 2026 The Openbot Authors
 *
 * NavigationTask — behavior-tree navigator for single-pose and through-poses
 * goals. Mirrors Nav2 BtNavigator: orchestrates planning / control via
 * NavigationClient while TaskServer owns exclusive activation.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include <automsgs/task/navigation.pb.h>
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

    ~NavigationTask() override { Shutdown(); }
    void Shutdown() override {
        shutting_down_.store(true);
        ++goal_epoch_;
        if (navigation()) navigation()->CancelPlanning();
        std::lock_guard<std::mutex> goal_lock(goal_mutex_);
        BtTaskApp::Shutdown();
    }

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
    void ApplyGoalToBlackboard(
        const BT::Blackboard::Ptr& blackboard,
        const ::autonomy::task::proto::NavigationGoal& goal);

    std::optional<::autonomy::task::proto::NavigationGoal> active_goal_;
    uint64_t replan_epoch_{0};
    float initial_distance_{-1.f};
    // Serialize START/CANCEL/preempt — cancel must not race a blocked START.
    mutable std::mutex goal_mutex_;
    // Goal/progress data uses the inherited recursive lifecycle mutex. A
    // separate hook lock would invert GetFeedback's lifecycle -> hook order.
    // The lifecycle mutex is never held across a worker join.
    // Bumped on every START/CANCEL so a stalled START can abandon StartTree.
    std::atomic<uint64_t> goal_epoch_{0};
    // Prevent a dequeued request from reopening navigation after shutdown.
    std::atomic<bool> shutting_down_{false};
};

}  // namespace task
}  // namespace autonomy
