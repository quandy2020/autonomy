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
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace scheduler {

/**
 * @brief Single-process task scheduler: owns planner/controller and BT navigators.
 */
class TaskScheduler
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskScheduler)

    TaskScheduler() = default;

    void Initialize(const std::string& configuration_directory);

    void Shutdown();

    behavior_tree::BtStatus NavigateToPose(
        std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal>
            goal);

    void RequestCancel();

    std::shared_ptr<common::TaskContext> TaskContext() const {
        return task_context_;
    }

private:
    void SetupNavigators();

    std::string configuration_directory_;
    proto::TaskOptions task_options_;
    std::shared_ptr<common::TaskContext> task_context_;
    std::shared_ptr<planning::PlannerServer> planner_;
    std::shared_ptr<control::ControllerServer> controller_;
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother_;
    std::shared_ptr<navigator::navigation::NavigateToPoseNavigator>
        navigate_to_pose_;
    common::NavigatorMuxer muxer_;
    std::atomic<bool> cancel_requested_{false};
    bool initialized_{false};
};

}  // namespace scheduler
}  // namespace tasks
}  // namespace autonomy
