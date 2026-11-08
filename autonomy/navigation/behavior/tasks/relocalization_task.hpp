/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file relocalization_task.hpp
 * @brief Requests relocalization and waits for the session to become ready.
 */

#pragma once

#include <cstdint>

#include "navigation/behavior/tasks/task_primitive.hpp"

namespace navigation
{
namespace behavior
{

class RelocalizationTask final : public TaskPrimitive
{
public:
    /** @brief Construct with a relocalization timeout. */
    explicit RelocalizationTask(std::int64_t timeout_ns = 10000000000LL);

    /** @brief Begin the task. */
    void Start(TaskContext& context) override;

    /** @brief Advance one tick. */
    TaskStatus Tick(TaskContext& context) override;

    /** @brief Reset local state. */
    void Reset() override;

    /** @brief Stable name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Relocalization timeout in nanoseconds. */
    std::int64_t timeout_ns_{10000000000LL};
    /** @brief Start time in nanoseconds. */
    std::int64_t start_ns_{0};
    /** @brief Primitive lifecycle state. */
    TaskStatus status_{TaskStatus::kIdle};
};

}  // namespace behavior
}  // namespace navigation
