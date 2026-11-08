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

#include "navigation/planning/scenarios/coverage/coverage_stage.hpp"

namespace navigation
{
namespace planning
{
namespace scenarios
{

bool CoverageStage::SetRegion(const Region& region)
{
    index_ = 0;
    waypoints_.clear();
    return planner_.Plan(region, &waypoints_);
}

bool CoverageStage::Plan(const common::Pose2D& pose, common::Twist2D* command)
{
    if (command == nullptr || index_ >= waypoints_.size())
    {
        return false;
    }
    if (follower_.Reached(pose, waypoints_[index_]))
    {
        ++index_;
        if (index_ >= waypoints_.size())
        {
            return false;
        }
    }
    return follower_.Compute(pose, waypoints_[index_], command);
}

void CoverageStage::Reset()
{
    planner_.Reset();
    waypoints_.clear();
    index_ = 0;
}

std::size_t CoverageStage::RemainingWaypoints() const
{
    return waypoints_.size() - std::min(index_, waypoints_.size());
}

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
