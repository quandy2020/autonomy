/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/exploration/planner/exploration_planner.hpp"

#include "autonomy/exploration/exploration_options.hpp"

namespace autonomy {
namespace exploration {
namespace planner {
namespace {

commsgs::geometry_msgs::Polygon MakeDefaultArea(double half_extent)
{
    const ::autonomy::common::math::Polygon2d poly(
        std::vector<::autonomy::common::math::Vec2d>{
            {-half_extent, -half_extent},
            {half_extent, -half_extent},
            {half_extent, half_extent},
            {-half_extent, half_extent},
        });
    commsgs::geometry_msgs::Polygon area;
    for (const auto& v : poly.points()) {
        commsgs::geometry_msgs::Point32 p;
        p.x = static_cast<float>(v.x());
        p.y = static_cast<float>(v.y());
        p.z = 0.f;
        area.points.push_back(p);
    }
    return area;
}

}  // namespace

ExplorationPlanner::ExplorationPlanner()
    : common::ExplorerInterface(DefaultOptions(), "rgbd_hierarchical"),
      planner_(GetOptions())
{
    UseDefaultExplorationArea();
}

ExplorationPlanner::ExplorationPlanner(
    const proto::ExplorationOptions& options, const std::string& name)
    : common::ExplorerInterface(options, name),
      planner_(options)
{
    UseDefaultExplorationArea();
}

void ExplorationPlanner::Configure(const proto::ExplorationOptions& options,
                                   const std::string& name)
{
    common::ExplorerInterface::Configure(options, name);
    planner_.SetOptions(options);
    UseDefaultExplorationArea();
}

void ExplorationPlanner::SetExplorationArea(
    const commsgs::geometry_msgs::Polygon& area)
{
    planner_.SetExplorationArea(area);
}

void ExplorationPlanner::UseDefaultExplorationArea()
{
    SetExplorationArea(
        MakeDefaultArea(GetOptions().grid_world().exploration_area_half_extent()));
}

void ExplorationPlanner::UpdateOdometry(
    const commsgs::planning_msgs::Odometry& odom)
{
    planner_.UpdateOdometry(odom);
}

void ExplorationPlanner::UpdateDepth(
    const commsgs::sensor_msgs::Image& depth,
    const commsgs::sensor_msgs::CameraInfo& info,
    const commsgs::geometry_msgs::Transform& map_t_camera)
{
    planner_.UpdateDepth(depth, info, map_t_camera);
}

bool ExplorationPlanner::ExecutePlanningCycle()
{
    cycle_ran_ = planner_.ExecutePlanningCycle();
    return cycle_ran_;
}

bool ExplorationPlanner::HasExplorationTarget() const
{
    return planner_.HasTarget();
}

bool ExplorationPlanner::GetNextWaypoint(
    commsgs::geometry_msgs::PoseStamped& out)
{
    if (!planner_.HasTarget()) {
        if (!planner_.ExecutePlanningCycle()) {
            return false;
        }
    }
    out = planner_.GetLookahead();
    return true;
}

void ExplorationPlanner::MarkWaypointReached()
{
    planner_.env().UpdateCoverageFromTf();
    planner_.AdvanceWaypointIndex();
}

bool ExplorationPlanner::IsFinished() const
{
    return planner_.IsFinished();
}

float ExplorationPlanner::Progress() const
{
    return planner_.Progress();
}

float ExplorationPlanner::ExploredAreaM2() const
{
    return planner_.ExploredAreaM2();
}

commsgs::planning_msgs::Path ExplorationPlanner::GetExplorationPath() const
{
    return planner_.GetExplorationPath();
}

commsgs::map_msgs::OccupancyGrid ExplorationPlanner::GetOccupancyGrid(
    const std::string& frame_id) const
{
    return planner_.GetOccupancyGrid(frame_id);
}

}  // namespace planner
}  // namespace exploration
}  // namespace autonomy
