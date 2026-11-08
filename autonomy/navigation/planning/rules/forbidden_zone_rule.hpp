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
 * @file forbidden_zone_rule.hpp
 * @brief Zero-speed guard for polygonal keep-out zones.
 */

#pragma once

#include <vector>

#include "navigation/planning/interface/rule_base.hpp"

namespace navigation
{
namespace planning
{

class ForbiddenZoneRule final : public RuleBase
{
public:
    /** @brief Replace the keep-out polygon list. */
    /**
     * @brief Replace the keep-out polygons.
     *
     * @param[in] zones navigation::Region messages (automsgs Point rings).
     */
    void SetZones(std::vector<Region> zones);

    /**
     * @brief Apply the rule.
     *
     * @param[in] frame Tick context.
     * @param[in,out] command Candidate command clamped in place.
     */
    common::Status Apply(NavigationFrame& frame, common::Twist2D* command) override;

    /** @brief Reset rule-local state. */
    void Reset() override;

    /** @brief Rule name used in configuration. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Point-in-polygon test (odd-even rule). */
    static bool Contains(const Region& region, double x, double y);

    /** @brief Keep-out regions. */
    std::vector<Region> zones_;
};

}  // namespace planning
}  // namespace navigation
