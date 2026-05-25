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

#include <any>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace common {

class TaskInterface
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskInterface)

    TaskInterface() = default;
    virtual ~TaskInterface() = default;

    enum class TaskState {
        IDLE,
        RUNNING,
        PAUSED,
        COMPLETED,
        FAILED,
        CANCELED,
        STOPPED,
        SHUTDOWN
    };

    template <typename... Args>
    bool Start(Args&&... args) {
        std::vector<std::any> captured_args;
        (captured_args.emplace_back(std::forward<Args>(args)), ...);
        return StartImpl(std::move(captured_args));
    }

    virtual bool Resume() = 0;
    virtual bool Cancel() = 0;
    virtual bool Stop() = 0;
    virtual void Shutdown() = 0;
    virtual TaskState GetState() const = 0;
    virtual std::string GetName() const = 0;

protected:
    virtual bool StartImpl(std::vector<std::any>&& args) = 0;
};

proto::TaskOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
