/*
 * Copyright 2026 The Openbot Authors
 *
 * Exploration task: BT orchestration delegating motion to planning/control.
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autonomy/task/apps/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/apps/exploration/exploration_client.hpp"
#include "autonomy/task/proto/exploration.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class ExplorationTask : public BtTaskApp<
                            ::autonomy::task::proto::ExplorationGoal,
                            ::autonomy::task::proto::ExplorationFeedback,
                            ::autonomy::task::proto::ExplorationResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationTask);

    ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetExplorationClient(exploration::ExplorationClient::Ptr client);

protected:
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::ExplorationGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::ExplorationFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::ExplorationResult* result) const override;

private:
    ::autonomy::task::proto::ExplorationStatus MapStatus() const;
    bool EnsureExplorationClient();
    void ApplyGoalParams(const ::autonomy::task::proto::ExplorationGoal& goal);
    std::string ResolveTreeForGoal(
        const ::autonomy::task::proto::ExplorationGoal& goal) const;

    exploration::ExplorationClient::Ptr exploration_client_;
    std::optional<::autonomy::task::proto::ExplorationGoal> active_goal_;
    std::string saved_map_name_;
};

}  // namespace task
}  // namespace autonomy
