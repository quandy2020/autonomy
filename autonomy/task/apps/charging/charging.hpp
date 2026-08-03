/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <optional>

#include "autonomy/task/apps/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/apps/charging/charging_client.hpp"
#include "autonomy/task/proto/charging.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class ChargingTask : public BtTaskApp<::autonomy::task::proto::ChargingGoal,
                                      ::autonomy::task::proto::ChargingFeedback,
                                      ::autonomy::task::proto::ChargingResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ChargingTask);

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetChargingClient(charging::ChargingClient::Ptr client);

protected:
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::ChargingGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::ChargingFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::ChargingResult* result) const override;

private:
    ::autonomy::task::proto::DockStatus MapStatus() const;
    bool EnsureChargingClient();

    charging::ChargingClient::Ptr charging_client_;
    std::optional<::autonomy::task::proto::ChargingGoal> active_goal_;
};

}  // namespace task
}  // namespace autonomy
