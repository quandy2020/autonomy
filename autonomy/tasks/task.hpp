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

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/common/navigation_engine.hpp"
#include "autonomy/tasks/common/task_interface.hpp"
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
 * @brief Task manager: single-point / multi-waypoint navigation and lifecycle.
 *
 * Delegates BT execution to common::NavigationEngine (BtNavigator when available).
 * Only one navigation session is active at a time.
 */
class Task : public common::TaskInterface
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Task)

    Task();
    explicit Task(const proto::TaskOptions& options);

    /** Inject or replace the navigation backend (e.g. BtNavigator wrapper). */
    void SetNavigationEngine(common::NavigationEngine::SharedPtr engine);

    /**
     * @brief Wire planner / smoother / controller / TF and configure the engine.
     */
    bool Configure(
        const proto::TaskOptions& options,
        std::shared_ptr<planning::PlannerServer> planner,
        std::shared_ptr<planning::SmootherServer> smoother,
        std::shared_ptr<control::ControllerServer> controller,
        std::shared_ptr<transform::Buffer> tf_buffer);

    bool Configure(
        std::shared_ptr<planning::PlannerServer> planner,
        std::shared_ptr<planning::SmootherServer> smoother,
        std::shared_ptr<control::ControllerServer> controller,
        std::shared_ptr<transform::Buffer> tf_buffer);

    /**
     * @brief Same as Configure above, and register autolink navigation actions.
     */
    bool Configure(
        const proto::TaskOptions& options,
        std::shared_ptr<planning::PlannerServer> planner,
        std::shared_ptr<planning::SmootherServer> smoother,
        std::shared_ptr<control::ControllerServer> controller,
        std::shared_ptr<transform::Buffer> tf_buffer,
        std::shared_ptr<autolink::Node> autolink_node);

    /** Attach autolink action servers after Configure (if node was not passed). */
    bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node);

    // --- Single-goal navigation (navigate_to_pose) ---

    bool StartNavigateToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& behavior_tree_file = "");

    bool CancelNavigateToPose();

    // --- Multi-waypoint navigation (navigate_through_poses) ---

    bool StartNavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& behavior_tree_file = "");

    bool CancelNavigateThroughPoses();

    // --- Pause / resume (active navigation only) ---

    bool PauseNavigation();
    bool ResumeNavigation();

    common::NavigationMode GetNavigationMode() const;
    bool IsNavigating() const;

    /** True while the navigation engine BT/direct session is still running. */
    bool IsNavigationEngineActive() const;

    /** After the engine finishes, whether the last session succeeded (BT only). */
    bool LastNavigationSucceeded() const;

    /** Called by AutonomyNode when the engine reports completion. */
    void FinalizeNavigation(bool succeeded);

    const proto::TaskOptions& GetOptions() const { return options_; }

    // --- TaskInterface ---

    bool Resume() override;
    bool Cancel() override;
    bool Stop() override;
    void Shutdown() override;
    TaskState GetState() const override;
    std::string GetName() const override;

protected:
    bool StartImpl(std::vector<std::any>&& args) override;

private:
    std::string DefaultBtFileForMode(common::NavigationMode mode) const;

    bool EnsureConfigured();
    bool EnsureIdleForStart();
    void OnNavigationStarted(common::NavigationMode mode);
    void OnNavigationCanceled();

    /** Caller must hold mutex_. */
    bool StartNavigateToPoseLocked(
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& behavior_tree_file);
    bool StartNavigateThroughPosesLocked(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& behavior_tree_file);

    proto::TaskOptions options_;
    common::NavigationEngine::SharedPtr engine_;
    std::shared_ptr<planning::PlannerServer> planner_;
    std::shared_ptr<planning::SmootherServer> smoother_;
    std::shared_ptr<control::ControllerServer> controller_;
    std::shared_ptr<transform::Buffer> tf_buffer_;

    mutable std::mutex mutex_;
    std::atomic<TaskState> state_{TaskState::IDLE};
    std::atomic<common::NavigationMode> navigation_mode_{
        common::NavigationMode::NONE};
    bool configured_{false};
};

}  // namespace tasks
}  // namespace autonomy
