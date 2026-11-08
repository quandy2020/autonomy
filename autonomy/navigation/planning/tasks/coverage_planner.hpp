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
 * @file coverage_planner.hpp
 * @brief Boustrophedon decomposition for TYPE_REGION goals.
 */

#pragma once

#include <vector>

#include "navigation/planning/base/waypoint.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{

struct CoverageOptions
{
    double spacing{0.5};
    double tolerance{0.15};
};

class CoveragePlanner
{
public:
    /** @brief Construct with spacing/tolerance. */
    explicit CoveragePlanner(CoverageOptions options = {});

    /**
     * @brief Decompose a region into boustrophedon waypoints.
     *
     * @param[in] region Closed polygon.
     * @param[out] waypoints Generated path.
     * @return false for an empty/invalid polygon.
     */
    bool Plan(const Region& region, std::vector<Waypoint>* waypoints) const;

    /** @brief Reset planner state (currently stateless). */
    void Reset();

private:
    /** @brief Spacing/tolerance options. */
    CoverageOptions options_;
};

}  // namespace planning
}  // namespace navigation
