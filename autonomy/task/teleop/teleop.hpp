/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <optional>

#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/teleop/client.hpp"
#include "autonomy/task/teleop/mppi_assist.hpp"
#include <automsgs/task/teleop.pb.h>
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

/**
 * @class task::TeleopTask
 * @brief Teleop task server: goal ingress, BT watchdog loop, MPPI assist
 *
 * Handles TELEOP_CMD_START / VELOCITY / STOP goals. The behavior tree runs
 * CommandValid and PerceptionValid conditions and TrackCommand to publish
 * /cmd_vel at a fixed rate while teleop is active.
 */
class TeleopTask : public BtTaskApp<::autonomy::task::proto::TeleopGoal,
                                    ::autonomy::task::proto::TeleopFeedback,
                                    ::autonomy::task::proto::TeleopResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopTask);

    /**
     * @brief Return ROBOT_TASK_TELEOP task type identifier
     */
    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    /**
     * @brief Inject teleop client (tests) or use EnsureClient() default
     * @param client Shared teleop client instance
     */
    void SetTeleopClient(teleop::TeleopClient::Ptr client);

    /**
     * @brief Stop teleop, assist, and behavior tree
     */
    void Shutdown() override;

protected:
    /**
     * @brief Load teleop assist config and wire TeleopClient on tree start
     */
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    /**
     * @brief Expose teleop_client and watchdog params on BT blackboard
     */
    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;

    /**
     * @brief Poll BT state, watchdog, and lifecycle each tick
     */
    void OnTreeTick() override;

    /**
     * @brief Handle START / VELOCITY / STOP teleop goals
     */
    bool OnGoal(const ::autonomy::task::proto::TeleopGoal& goal) override;

    /**
     * @brief Fill teleop status and applied velocity feedback
     */
    void FillFeedback(
        ::autonomy::task::proto::TeleopFeedback* feedback) const override;

    /**
     * @brief Fill final teleop result and status
     */
    void FillResult(
        ::autonomy::task::proto::TeleopResult* result) const override;

private:
    /**
     * @brief Map internal lifecycle to TeleopStatus proto enum
     */
    ::autonomy::task::proto::TeleopStatus MapStatus() const;

    /**
     * @brief Lazily create TeleopClient if not injected
     */
    bool EnsureClient();

    /**
     * @brief Apply velocity limits, watchdog, and stick from TeleopGoal
     */
    void ApplyGoal(const ::autonomy::task::proto::TeleopGoal& goal);

    /**
     * @brief Publish zero velocity and stop behavior tree
     */
    void StopTeleop();

    /**
     * @brief Cancel active navigation motion before teleop starts
     */
    void CancelNav();

    // BT facade: commanded twist, watchdog, /cmd_vel.
    teleop::TeleopClient::Ptr teleop_client_;
    // Optional MPPI assist pipeline.
    std::shared_ptr<teleop::TeleopMppiAssist> teleop_assist_;
    // Last accepted teleop goal (START or VELOCITY).
    std::optional<::autonomy::task::proto::TeleopGoal> active_goal_;
    // Set when watchdog expires; affects MapStatus().
    bool watchdog_timed_out_{false};
};

}  // namespace task
}  // namespace autonomy
