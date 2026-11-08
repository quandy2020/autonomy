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
 * @file rule_base.hpp
 * @brief Final velocity shrink rules (never expand).
 */

#pragma once

#include <string_view>

#include "navigation/common/status.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/base/navigation_frame.hpp"

namespace navigation
{
namespace planning
{

class RuleBase
{
public:
    virtual ~RuleBase() = default;

    /**
     * @brief Shrink a candidate command.
     *
     * @param[in] frame Context (pose, platform, reflex).
     * @param[in,out] command Candidate command to clamp in place.
     * @return kOk, or a failure that the manager converts to zero speed.
     */
    virtual common::Status Apply(NavigationFrame& frame, common::Twist2D* command) = 0;

    /** @brief Reset rule-local state. */
    virtual void Reset() = 0;

    /** @brief Rule name used in configuration. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

}  // namespace planning
}  // namespace navigation
