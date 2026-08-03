/*
 * Copyright 2026 The Openbot Authors
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

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"
#include "autonomy/task/apps/charging/charging.hpp"
// #include "autonomy/task/apps/exploration/exploration.hpp"  // temporarily disabled
#include "autonomy/task/apps/localization/localization.hpp"
#include "autonomy/task/apps/mapping/mapping.hpp"
#include "autonomy/task/apps/navigation/navigation.hpp"
#include "autonomy/task/apps/behavior_tree/bt_defaults.hpp"
#include "autonomy/task/apps/teleop/teleop.hpp"
#include "autonomy/task/apps/tracking/tracker.hpp"
#include "autonomy/task/scheduler/scheduler.hpp"
#include "autonomy/task/autolink_tf_listener.hpp"
#include "autonomy/task/teleop_goal_ingress.hpp"
#include "autonomy/task/teleop_feedback_publisher.hpp"

namespace autonomy {
namespace task {

/**
 * @brief 任务服务入口：装配插件、调度器，并向 Bridge / 本地 API 提供统一提交接口。
 */
class TaskServer
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskServer);

    TaskServer();
    ~TaskServer();

    TaskServer(const TaskServer&) = delete;
    TaskServer& operator=(const TaskServer&) = delete;

    bool Configure(const ::autonomy::task::proto::TaskServerOptions& options);
    bool Start();
    void Shutdown();

    bool IsRunning() const;

    TaskScheduler::SharedPtr GetScheduler() const { return scheduler_; }

    NavigationTask::SharedPtr navigation() const { return navigation_; }
    TrackerTask::SharedPtr tracking() const { return tracking_; }
    TeleopTask::SharedPtr teleop() const { return teleop_; }
    // ExplorationTask temporarily disabled during automsgs migration.
    ChargingTask::SharedPtr charging() const { return charging_; }
    MappingTask::SharedPtr mapping() const { return mapping_; }
    LocalizationTask::SharedPtr localization() const { return localization_; }

    std::shared_ptr<autolink::Node> task_node() const { return task_node_; }

    navigation::NavigationClient::Ptr navigation_client() const
    {
        return navigation_client_;
    }

    bool SubmitNavigationGoal(const ::autonomy::task::proto::NavigationGoal& goal);
    bool SubmitTrackerGoal(const ::autonomy::task::proto::TrackerGoal& goal);
    bool SubmitTeleopGoal(const ::autonomy::task::proto::TeleopGoal& goal);
    bool SubmitExplorationGoal(const ::autonomy::task::proto::ExplorationGoal& goal);
    bool SubmitChargingGoal(const ::autonomy::task::proto::ChargingGoal& goal);
    bool SubmitMappingGoal(const ::autonomy::task::proto::MappingGoal& goal);
    bool SubmitLocalizationGoal(const ::autonomy::task::proto::LocalizationGoal& goal);

    std::vector<::autonomy::task::proto::ActiveTaskSnapshot> GetActiveSnapshots() const;

    static ::autonomy::task::proto::TaskServerOptions DefaultOptions();

private:
    template <typename TaskT, typename GoalT>
    bool SubmitGoal(const std::shared_ptr<TaskT>& task, const GoalT& goal)
    {
        if (!task || !scheduler_) {
            return false;
        }

        const int command = static_cast<int>(goal.command());
        const bool is_stop_like = command == 0 || command == 2 || command == 5;
        const bool needs_activation = !task->IsActive() && !is_stop_like;

        if (needs_activation && !scheduler_->RequestActivation(task)) {
            return false;
        }

        if (!task->SubmitGoal(goal)) {
            if (needs_activation) {
                scheduler_->ReleaseActivation(task);
            }
            return false;
        }
        return true;
    }

    void RegisterEnabledPlugins(
        const ::autonomy::task::proto::TaskAppOptions& apps);

    TaskScheduler::SharedPtr scheduler_;
    NavigationTask::SharedPtr navigation_;
    TrackerTask::SharedPtr tracking_;
    TeleopTask::SharedPtr teleop_;
    ChargingTask::SharedPtr charging_;
    MappingTask::SharedPtr mapping_;
    LocalizationTask::SharedPtr localization_;
    std::shared_ptr<autolink::Node> task_node_;
    navigation::NavigationClient::Ptr navigation_client_;
    ::autonomy::task::proto::TaskServerOptions options_;
    TeleopGoalIngress::SharedPtr teleop_goal_ingress_;
    TeleopFeedbackPublisher::SharedPtr teleop_feedback_publisher_;
    AutolinkTfListener::SharedPtr autolink_tf_listener_;
    bool configured_{false};
};

}  // namespace task
}  // namespace autonomy
