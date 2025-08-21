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
#include "autonomy/tasks/proto/extend_command.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/common/task_interface.hpp"

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
     * @brief Shutdown 
     */
    void Shutdown();

    /**
     * @brief Handle user navigation task commands
     * 
     * @return True or false
     */
    bool HandleCommandMessageCallback(const proto::ExtendCommand& msgs);

private:

    /**
     * @brief Get task type name
     */
    std::string name(const proto::ExtendCommand::Type& msgs);

    // tasks options
    const proto::TaskOptions options_;

    // Task map(key & value)
    std::unordered_map<std::string, common::TaskInterface::SharedPtr> tasks_;
};

}   // namespace tasks
}   // namespace autonomy