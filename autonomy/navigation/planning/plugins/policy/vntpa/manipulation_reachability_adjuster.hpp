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
 * @file manipulation_reachability_adjuster.hpp
 * @brief Optional docking adjustment for mobile manipulators.
 */

#pragma once

#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

class ManipulationReachabilityAdjuster
{
public:
    ManipulationReachabilityAdjuster(double reach_min, double reach_max);

    bool Adjust(const common::Pose2D& current, const common::Pose2D& goal,
                common::Twist2D* command) const;

private:
    double reach_min_{0.35};
    double reach_max_{0.75};
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
