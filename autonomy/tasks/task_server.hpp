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

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/task_bridge.hpp"

namespace autonomy {
namespace tasks { 

class TaskServer
{
public:
    /**
     * Define TaskServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskServer)

    /**
     * @brief A constructor for TaskServer
     */
    explicit TaskServer();

    /**
     * @brief A Destructor for TaskServer
     */
    ~TaskServer() = default;

    /**
     * @brief Starts autonomy tasks
     */
    void Start();

    /**
     * @brief Shutdown autonomy
     */
    void Shutdown();

private:
    /// @brief  bridge all tasks
    TaskBridge::SharedPtr task_bridge_{nullptr};
};


}   // namespace tasks
}   // namespace autonomy