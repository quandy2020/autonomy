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

#include <vector>
#include <unordered_map>

#include "autonomy/tasks/proto/task_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/thread_pool.hpp"
#include "autonomy/tasks/common/task_interface.hpp"
#include "autonomy/tasks/multi_ordered_cmd_queue.hpp"

namespace autonomy {
namespace tasks { 

class TaskBridge
{
public:
    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskBridge)

    /**
     * @brief A constructor for autonomy::tasks::TaskBridge
     * @param options Additional options to control creation of the node.
     */
    explicit TaskBridge();

    /**
     * @brief A Destructor for autonomy::tasks::MapServer
     */
    ~TaskBridge();

    /**
     * @brief Start task bridge
     */
    void Start();

    /**
     * @brief Shutdown task bridge
     */
    void Shutdown();

private:

    /**
     * @brief Run tasks
     */
    void RunTasks();

    // tasks options
    const proto::TaskOptions options_;

    // Task map(key & value)
    std::unordered_map<std::string, common::TaskInterface::SharedPtr> tasks_;

    // Task command queue
    OrderedMultiCommandQueue cmd_queue_;

    // Thread pool
    autonomy::common::ThreadPool thread_pool_;
};

}   // namespace tasks
}   // namespace autonomy