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

#include "autonomy/tasks/task_manager.hpp"

#include "autonomy/common/json_util.hpp"
#include "autonomy/tasks/constants.hpp"

namespace autonomy {
namespace tasks {

TaskManager::TaskManager(const proto::TaskOptions& options)
    : options_(options), thread_pool_(std::thread::hardware_concurrency()) {
    navigate_to_pose_navigator_ =
        std::make_shared<navigator::navigation::NavigateToPoseNavigator>();
}

TaskManager::~TaskManager() {}

void TaskManager::Start() {}

void TaskManager::Shutdown() {
    // if (navigate_to_pose_navigator_) {
    //     navigate_to_pose_navigator_->Shutdown();
    // }
    // if (track_to_target_navigator_) {
    //     track_to_target_navigator_->Shutdown();
    // }
    // if (explore_to_anywhere_navigator_) {
    //     explore_to_anywhere_navigator_->Shutdown();
    // }
}

}  // namespace tasks
}  // namespace autonomy