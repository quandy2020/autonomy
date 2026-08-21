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

#ifndef AUTONOMY_TASK_TASK_SERVER_HPP_
#define AUTONOMY_TASK_TASK_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/behavior_tree/bt_defaults.hpp"
#include "autonomy/task/charging/charging.hpp"
#include "autonomy/task/common/transform_listener.hpp"
#include "autonomy/task/exploration/exploration.hpp"
#include "autonomy/task/interface/goal_ingress.hpp"
#include "autonomy/task/interface/navigator.hpp"
#include "autonomy/task/localization/localization.hpp"
#include "autonomy/task/mapping/mapping.hpp"
#include "autonomy/task/navigation/navigation.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/proto/charging.pb.h"
#include "autonomy/task/proto/exploration.pb.h"
#include "autonomy/task/proto/localization.pb.h"
#include "autonomy/task/proto/mapping.pb.h"
#include "autonomy/task/proto/navigation.pb.h"
#include "autonomy/task/proto/teleop.pb.h"
#include "autonomy/task/proto/tracker.pb.h"
#include "autonomy/task/register_tasks.hpp"
#include "autonomy/task/scheduler/scheduler.hpp"
#include "autonomy/task/teleop/teleop.hpp"
#include "autonomy/task/tracking/tracking.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

namespace autonomy {
namespace task {

// Composition root: plugins + exclusive slot + Navigator / channel I/O.
class TaskServer
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskServer)

    TaskServer();
    ~TaskServer();

    TaskServer(const TaskServer&) = delete;
    TaskServer& operator=(const TaskServer&) = delete;

    bool Configure(const proto::TaskServerOptions& options);
    bool Start();
    void Shutdown();
    bool IsRunning() const;

    TaskScheduler::SharedPtr scheduler() const { return scheduler_; }
    std::shared_ptr<autolink::Node> node() const { return node_; }
    navigation::NavigationClient::Ptr navigation_client() const {
        return navigation_client_;
    }

    NavigationTask::SharedPtr navigation() const { return navigation_; }
    TrackerTask::SharedPtr tracking() const { return tracking_; }
    TeleopTask::SharedPtr teleop() const { return teleop_; }
    ExplorationTask::SharedPtr exploration() const { return exploration_; }
    ChargingTask::SharedPtr charging() const { return charging_; }
    MappingTask::SharedPtr mapping() const { return mapping_; }
    LocalizationTask::SharedPtr localization() const { return localization_; }

    bool Submit(const proto::NavigationGoal& goal);
    bool Submit(const proto::TrackerGoal& goal);
    bool Submit(const proto::TeleopGoal& goal);
    bool Submit(const proto::ExplorationGoal& goal);
    bool Submit(const proto::ChargingGoal& goal);
    bool Submit(const proto::MappingGoal& goal);
    bool Submit(const proto::LocalizationGoal& goal);

    bool SubmitTeleopGoal(const proto::TeleopGoal& goal) { return Submit(goal); }
    bool SubmitNavigationGoal(const proto::NavigationGoal& goal) {
        return Submit(goal);
    }

    std::vector<proto::ActiveTaskSnapshot> GetActiveSnapshots() const;
    static proto::TaskServerOptions DefaultOptions();

private:
    static bool NeedsSlot(const proto::NavigationGoal& goal) {
        return goal.command() == proto::NAV_CMD_START ||
               goal.command() == proto::NAV_CMD_REPLAN;
    }
    static bool NeedsSlot(const proto::TeleopGoal& goal) {
        return goal.command() == proto::TELEOP_CMD_START;
    }
    static bool NeedsSlot(const proto::TrackerGoal& goal) {
        return goal.command() == proto::TRACKER_CMD_START ||
               goal.command() == proto::TRACKER_CMD_UPDATE_TARGET;
    }
    static bool NeedsSlot(const proto::ChargingGoal& goal) {
        return goal.command() == proto::DOCK_CMD_START ||
               goal.command() == proto::DOCK_CMD_UNDOCK;
    }
    static bool NeedsSlot(const proto::MappingGoal&) { return true; }
    static bool NeedsSlot(const proto::LocalizationGoal& goal) {
        return goal.command() == proto::LOCALIZATION_CMD_START ||
               goal.command() == proto::LOCALIZATION_CMD_SWITCH_ALGORITHM;
    }
    static bool NeedsSlot(const proto::ExplorationGoal& goal) {
        return goal.command() == proto::EXPLORATION_CMD_START;
    }

    template <typename TaskT, typename GoalT>
    bool Dispatch(const std::shared_ptr<TaskT>& task, const GoalT& goal) {
        if (!task || !scheduler_) {
            return false;
        }
        // Drop stale exclusive / session state after SUCCEEDED/FAILED so the
        // next /goal_pose can RequestActivation without fighting a dead hold.
        if (!task->IsActive()) {
            scheduler_->ReleaseActivation(task);
        }
        const bool needs_activation =
            !task->IsActive() && NeedsSlot(goal);
        if (needs_activation && !scheduler_->RequestActivation(task)) {
            AWARN << "TaskServer: RequestActivation failed for task type "
                  << static_cast<int>(task->GetTaskType());
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

    template <typename TaskT>
    void Register(const std::shared_ptr<TaskT>& task) {
        if (!task) {
            return;
        }
        task->SetNode(node_);
        if constexpr (TaskT::kUsesNavigationClient) {
            task->SetNavigationClient(navigation_client_);
        }
        scheduler_->RegisterTask(task);
    }

    template <typename TaskT, typename GoalT, typename FeedbackT,
              typename TerminalFn, typename RejectedFn>
    bool BindDomain(
        bool enabled, const std::shared_ptr<TaskT>& task,
        const char* goal_channel, const char* feedback_channel,
        std::chrono::milliseconds period,
        interface::GoalIngress<GoalT, FeedbackT>* ingress,
        TerminalFn&& is_terminal, RejectedFn&& make_rejected) {
        if (!enabled || !task || ingress == nullptr) {
            return false;
        }
        return ingress->Start(
            node_, goal_channel, feedback_channel, period,
            [this, task](const GoalT& goal) { return Dispatch(task, goal); },
            [task](FeedbackT* feedback) {
                return task && task->GetFeedback(feedback);
            },
            [task]() { return task && task->IsActive(); },
            std::forward<TerminalFn>(is_terminal),
            std::forward<RejectedFn>(make_rejected));
    }

    void AddApps(const proto::TaskAppOptions& apps);
    void Bind();
    void Unbind();

    TaskScheduler::SharedPtr scheduler_;
    NavigationTask::SharedPtr navigation_;
    TrackerTask::SharedPtr tracking_;
    TeleopTask::SharedPtr teleop_;
    ExplorationTask::SharedPtr exploration_;
    ChargingTask::SharedPtr charging_;
    MappingTask::SharedPtr mapping_;
    LocalizationTask::SharedPtr localization_;
    std::shared_ptr<autolink::Node> node_;
    navigation::NavigationClient::Ptr navigation_client_;
    proto::TaskServerOptions options_;

    std::shared_ptr<
        autolink::Reader<::automsgs::msgs::geometry_msgs::PoseStamped>>
        goal_pose_reader_;
    std::chrono::steady_clock::time_point last_goal_pose_time_{};
    double last_goal_pose_x_{0.0};
    double last_goal_pose_y_{0.0};
    // Coalesce rapid Autoviz 2D Goal clicks onto the latest pose only.
    std::mutex goal_pose_mutex_;
    std::optional<::automsgs::msgs::geometry_msgs::PoseStamped>
        pending_goal_pose_;
    std::atomic<bool> goal_pose_worker_busy_{false};
    interface::Navigator::SharedPtr navigator_;

    interface::GoalIngress<proto::TeleopGoal, proto::TeleopFeedback>
        teleop_ingress_;
    interface::GoalIngress<proto::TrackerGoal, proto::TrackerFeedback>
        tracking_ingress_;
    interface::GoalIngress<proto::ExplorationGoal, proto::ExplorationFeedback>
        exploration_ingress_;
    interface::GoalIngress<proto::ChargingGoal, proto::ChargingFeedback>
        charging_ingress_;
    interface::GoalIngress<proto::MappingGoal, proto::MappingFeedback>
        mapping_ingress_;
    interface::GoalIngress<proto::LocalizationGoal, proto::LocalizationFeedback>
        localization_ingress_;

    TransformListener::SharedPtr transform_listener_;
    bool configured_ = false;
};

}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_TASK_SERVER_HPP_
