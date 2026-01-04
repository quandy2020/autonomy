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

#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/thread_pool.hpp"
#include "autonomy/tasks/common/task_interface.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {

class TaskManager
{
public:
    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskManager)

    /**
     * @brief A constructor for autonomy::tasks::TaskManager
     * @param options Additional options to control creation of the node.
     */
    explicit TaskManager(const proto::TaskOptions& options);

    /**
     * @brief A Destructor for autonomy::tasks::TaskManager
     */
    ~TaskManager();

    /**
     * @brief Start task manager
     */
    void Start();

    /**
     * @brief Shutdown task manager
     */
    void Shutdown();

private:
    // Configuration for task options
    proto::TaskOptions options_;

    // Navigate to pose navigator
    navigator::navigation::NavigateToPoseNavigator::SharedPtr
        navigate_to_pose_navigator_{nullptr};

    // // Track_to_target navigator
    // navigator::navigation::TrackToTargetNavigator::SharedPtr
    // track_to_target_navigator_{nullptr};

    // // Explore_to_anywhere navigator
    // navigator::exploration::ExploreToAnywhereNavigator::SharedPtr
    // explore_to_anywhere_navigator_{nullptr};

    // Thread pool
    autonomy::common::ThreadPool thread_pool_;
};

}  // namespace tasks
}  // namespace autonomy