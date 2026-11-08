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
 * @file stage_base.hpp
 * @brief One processing stage inside a scenario.
 */

#pragma once

#include <string_view>

#include "navigation/common/status.hpp"
#include "navigation/planning/base/navigation_frame.hpp"

namespace navigation
{
namespace planning
{

class StageBase
{
public:
    virtual ~StageBase() = default;

    /**
     * @brief Process one frame in place.
     *
     * @param[in,out] frame Command/intent state.
     */
    virtual common::Status Process(NavigationFrame& frame) = 0;

    /** @brief Reset stage-local history. */
    virtual void Reset() = 0;

    /** @brief Stage name used in diagnostics. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

}  // namespace planning
}  // namespace navigation
