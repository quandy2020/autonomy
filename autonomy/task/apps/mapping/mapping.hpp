/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autonomy/task/apps/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/apps/mapping/mapping_client.hpp"
#include "autonomy/task/proto/mapping.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class MappingTask : public BtTaskApp<::autonomy::task::proto::MappingGoal,
                                     ::autonomy::task::proto::MappingFeedback,
                                     ::autonomy::task::proto::MappingResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(MappingTask);

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetMappingClient(mapping::MappingClient::Ptr client);

protected:
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::MappingGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::MappingFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::MappingResult* result) const override;

private:
    ::autonomy::task::proto::MapStatus MapStatus() const;
    bool EnsureMappingClient();
    std::string ResolveTreeForGoal(
        const ::autonomy::task::proto::MappingGoal& goal) const;

    mapping::MappingClient::Ptr mapping_client_;
    std::optional<::autonomy::task::proto::MappingGoal> active_goal_;
};

}  // namespace task
}  // namespace autonomy
