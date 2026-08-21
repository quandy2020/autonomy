/*
 * Copyright 2026 The Openbot Authors
 *
 * Teleop task: BT watchdog loop publishing /cmd_vel.
 */

#pragma once

#include <memory>
#include <optional>

#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/teleop/teleop_client.hpp"
#include "autonomy/task/teleop/teleop_mppi_assist.hpp"
#include "autonomy/task/proto/teleop.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class TeleopTask : public BtTaskApp<::autonomy::task::proto::TeleopGoal,
                                    ::autonomy::task::proto::TeleopFeedback,
                                    ::autonomy::task::proto::TeleopResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopTask);

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetTeleopClient(teleop::TeleopClient::Ptr client);

    void Shutdown() override;

protected:
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::TeleopGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::TeleopFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::TeleopResult* result) const override;

private:
    ::autonomy::task::proto::TeleopStatus MapStatus() const;
    bool EnsureTeleopClient();
    void ApplyGoalParams(const ::autonomy::task::proto::TeleopGoal& goal);
    void StopTeleop();

    teleop::TeleopClient::Ptr teleop_client_;
    std::shared_ptr<teleop::TeleopMppiAssist> teleop_assist_;
    std::optional<::autonomy::task::proto::TeleopGoal> active_goal_;
    bool watchdog_timed_out_{false};
};

}  // namespace task
}  // namespace autonomy
