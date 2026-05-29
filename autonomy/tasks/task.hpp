/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#ifndef AUTONOMY_TASKS_TASK_H_
#define AUTONOMY_TASKS_TASK_H_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/common/interface.hpp"
#include "autonomy/tasks/navigators/bt_navigator.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autolink {
class Node;
}

namespace autonomy {
namespace control {
class ControllerServer;
}
namespace planning {
class PlannerServer;
class SmootherServer;
}
namespace transform {
class Buffer;
}
namespace tasks {

/**
 * @brief Behavior-tree navigation task manager for single-goal and multi-waypoint goals.
 *
 * Owns a BtNavigator and enforces at most one active navigation session. Thread-safe
 * for concurrent API calls via an internal mutex.
 */
class Task : public TaskInterface
{
public:
    /**
     * @brief Smart pointer definitions for Task.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Task)

    /**
     * @brief Constructs the task and wires planner, smoother, controller, and TF into BT.
     *
     * @param options Task and BT configuration (frames, default trees, autolink flags).
     * @param planner Global planner server (non-null).
     * @param smoother Path smoother server (non-null).
     * @param controller Controller server (non-null).
     * @param tf_buffer Transform buffer for navigation frames (non-null).
     * @param autolink_node Optional autolink node; used when
     *        options.enable_autolink_action_servers is true.
     *
     * @note If any server pointer is null, or BtNavigator initialization fails,
     *       IsConfigured() returns false.
     */
    Task(const proto::TaskOptions& options,
         std::shared_ptr<planning::PlannerServer> planner,
         std::shared_ptr<planning::SmootherServer> smoother,
         std::shared_ptr<control::ControllerServer> controller,
         std::shared_ptr<transform::Buffer> tf_buffer,
         std::shared_ptr<autolink::Node> autolink_node = nullptr);

    /**
     * @return True if BtNavigator was created successfully in the constructor.
     */
    bool IsConfigured() const { return configured_; }

    /**
     * @brief Starts navigate_to_pose using the given goal.
     *
     * @param goal Target pose in the planning frame.
     * @param behavior_tree_file BT XML file name; empty uses TaskOptions default.
     * @return True if the navigator accepted the goal.
     *
     * @pre IsConfigured() and no other navigation session is RUNNING.
     */
    bool StartNavigateToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& behavior_tree_file = "");

    /**
     * @brief Starts navigate_through_poses for an ordered waypoint list.
     *
     * @param goals Non-empty sequence of poses.
     * @param behavior_tree_file BT XML file name; empty uses TaskOptions default.
     * @return True if the navigator accepted the goal.
     *
     * @pre IsConfigured(), goals non-empty, and no other navigation session is RUNNING.
     */
    bool StartNavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& behavior_tree_file = "");

    /**
     * @return Active navigation mode, or NONE when idle.
     */
    NavigationMode GetNavigationMode() const;

    /**
     * @return True while lifecycle state is RUNNING.
     */
    bool IsNavigating() const;

    /**
     * @return True while the underlying BtNavigator reports an active BT session.
     */
    bool IsNavigatorActive() const;

    /**
     * @brief Registers a callback invoked when the controller publishes an updated path.
     *
     * @param callback Called with the latest path; may be empty to clear.
     *
     * @pre IsConfigured().
     */
    void SetPathCallback(
        std::function<void(const commsgs::planning_msgs::Path&)> callback);

    /**
     * @return True if the last completed BT run ended in SUCCEEDED.
     */
    bool LastNavigationSucceeded() const;

    /**
     * @brief Updates lifecycle state after Autonomy finishes waiting on the navigator.
     *
     * @param succeeded Whether navigation completed successfully.
     */
    void FinalizeNavigation(bool succeeded);

    /**
     * @brief Cancels the active navigation session.
     *
     * @return True if a RUNNING session was canceled.
     */
    bool Cancel() override;

    /**
     * @brief Tears down the task: cancels any active navigation and marks SHUTDOWN.
     */
    void Shutdown() override;

    /** 
     * @return Current lifecycle state.
     */
    TaskState GetState() const override;

private:
    std::string DefaultBtFileForMode(NavigationMode mode) const;
    void OnNavigationStarted(NavigationMode mode);
    void OnNavigationCanceled();

    proto::TaskOptions options_;
    std::shared_ptr<BtNavigator> navigator_;

    mutable std::mutex mutex_;
    std::atomic<TaskState> state_{TaskState::kIdle};
    std::atomic<NavigationMode> mode_{NavigationMode::NONE};
    bool configured_{false};
};

}  // namespace tasks
}  // namespace autonomy

#endif  // AUTONOMY_TASKS_TASK_H_
