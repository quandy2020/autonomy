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

#include "navigation/planning/plugins/policy/vntpa/manipulation_reachability_adjuster.hpp"

#include <algorithm>


namespace navigation
{
namespace planning
{
namespace vntpa
{

ManipulationReachabilityAdjuster::ManipulationReachabilityAdjuster(double reach_min,
                                                                   double reach_max)
    : reach_min_(std::max(0.0, reach_min)), reach_max_(std::max(reach_min_, reach_max))
{
}

bool ManipulationReachabilityAdjuster::Adjust(const common::Pose2D& current,
                                              const common::Pose2D& goal,
                                              common::Twist2D* command) const
{
    if (command == nullptr)
    {
        return false;
    }
    const double distance = common::Distance(current, goal);
    if (distance < reach_min_)
    {
        // Back off slightly so the manipulator keeps a valid reach annulus.
        command->set_x(-std::min(0.1, (reach_min_ - distance) / 1.0));
    }
    else if (distance > reach_max_)
    {
        command->set_x(std::min(0.2, (distance - reach_max_) * 0.5));
    }
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
