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

#include "navigation/behavior/tasks/coverage_task.hpp"

namespace navigation
{
namespace behavior
{

bool CoverageTask::SetRegion(const Region& region)
{
    std::vector<planning::Waypoint> waypoints;
    if (!planner_.Plan(region, &waypoints) || waypoints.empty())
    {
        has_region_ = false;
        return false;
    }
    sequence_.SetWaypoints(std::move(waypoints));
    has_region_ = true;
    return true;
}

void CoverageTask::Start(TaskContext& context)
{
    if (!has_region_)
    {
        sequence_.Reset();
        return;
    }
    sequence_.Start(context);
}

TaskStatus CoverageTask::Tick(TaskContext& context)
{
    return sequence_.Tick(context);
}

void CoverageTask::Reset()
{
    planner_.Reset();
    sequence_.Reset();
    has_region_ = false;
}

std::string_view CoverageTask::Name() const
{
    return "coverage";
}

}  // namespace behavior
}  // namespace navigation
