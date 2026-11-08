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
 * @brief Planning-side relocalization trigger.
 */

#pragma once

#include "navigation/planning/interface/task_base.hpp"

namespace navigation
{
namespace planning
{

class RelocalizationTask final : public TaskBase
{
public:
    /** @brief Trigger relocalization and zero the command. */
    common::Status Execute(NavigationFrame& frame) override;

    /** @brief Reset task state. */
    void Reset() override;

    /** @brief Task name. */
    [[nodiscard]] std::string_view Name() const override;
};

}  // namespace planning
}  // namespace navigation
