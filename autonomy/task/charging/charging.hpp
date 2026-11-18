/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file charging.hpp
 * @brief ChargingTask: BT dock/undock around DockingEngine.
 */

#pragma once

#include <memory>
#include <optional>

#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/charging/auto_recharge.hpp"
#include "autonomy/task/charging/charging_client.hpp"
#include <automsgs/task/charging.pb.h>
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

/**
 * @class autonomy::task::ChargingTask
 * @brief TaskServer adapter that runs dock.xml / undock.xml.
 */
class ChargingTask : public BtTaskApp<::autonomy::task::proto::ChargingGoal,
                                      ::autonomy::task::proto::ChargingFeedback,
                                      ::autonomy::task::proto::ChargingResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ChargingTask);

    /**
     * @brief Task type advertised to TaskServer.
     * @return ROBOT_TASK_DOCK.
     */
    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    /**
     * @brief Inject a client (tests); production uses EnsureChargingClient().
     * @param[in] client Shared charging facade.
     */
    void SetChargingClient(charging::ChargingClient::Ptr client);

    /**
     * @brief Start the low-SOC return-to-dock worker (TaskServer Bind).
     */
    void StartAutoRecharge(
        charging::AutoRechargeSupervisor::SubmitFn submit,
        charging::AutoRechargeSupervisor::TeleopFn teleop_active);

    /**
     * @brief Stop the auto-recharge worker.
     */
    void StopAutoRecharge();

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
    charging::AutoRechargeSupervisor auto_recharge_;
    std::optional<::autonomy::task::proto::ChargingGoal> active_goal_;
};

}  // namespace task
}  // namespace autonomy
