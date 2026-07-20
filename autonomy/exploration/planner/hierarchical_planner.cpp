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

#include "autonomy/exploration/planner/hierarchical_planner.hpp"

#include <algorithm>
#include <cmath>

#include "Eigen/Geometry"

#include "autonomy/common/math/vec2d.hpp"
#include "autonomy/common/transform/rigid_transform.hpp"
#include "autonomy/common/transform/transform.hpp"
#include "autonomy/exploration/planner/los_checker.hpp"
#include "autonomy/exploration/planner/tsp_solver.hpp"

namespace autonomy {
namespace exploration {

HierarchicalPlanner::HierarchicalPlanner(const proto::ExplorationOptions& options)
    : options_(options),
      env_(options),
      viewpoints_(options),
      grid_world_(options),
      keypose_graph_(options),
      local_planner_(options)
{
}

void HierarchicalPlanner::SetOptions(const proto::ExplorationOptions& options)
{
    options_ = options;
    env_.SetOptions(options);
    viewpoints_.SetOptions(options);
    grid_world_.SetOptions(options);
    keypose_graph_.SetOptions(options);
    local_planner_.SetOptions(options);
}

void HierarchicalPlanner::SetExplorationArea(
    const commsgs::geometry_msgs::Polygon& area)
{
    env_.SetExplorationArea(area);
}

void HierarchicalPlanner::UpdateOdometry(
    const commsgs::planning_msgs::Odometry& odom)
{
    env_.UpdateOdometry(odom);
    has_odom_ = true;
    // Keep planning outputs in map; do not inherit odom.header.frame_id.
    frame_id_ = "map";
    if (!has_initial_) {
        initial_x_ = env_.robot_x();
        initial_y_ = env_.robot_y();
        has_initial_ = true;
    }
    viewpoints_.UpdateRobotPosition(env_.robot_x(), env_.robot_y(),
                                    env_.robot_z(), env_.robot_yaw());
    keypose_graph_.UpdateRobotPose(env_.robot_x(), env_.robot_y(),
                                   env_.robot_z(), env_);
}

void HierarchicalPlanner::UpdateDepth(
    const commsgs::sensor_msgs::Image& depth,
    const commsgs::sensor_msgs::CameraInfo& info,
    const commsgs::geometry_msgs::Transform& map_t_camera)
{
    env_.UpdateDepth(depth, info, map_t_camera);
}

commsgs::geometry_msgs::PoseStamped HierarchicalPlanner::MakePose(
    double x, double y, double z, double yaw) const
{
    const Eigen::Quaterniond q =
        ::autonomy::common::transform::RollPitchYaw(0.0, 0.0, yaw);
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    return pose;
}

std::vector<int> HierarchicalPlanner::SolveGlobalCellOrder() const
{
    auto cells = grid_world_.GetExploringCells(
        env_.robot_x(), env_.robot_y(),
        options_.grid_world().nearby_cell_radius());
    if (cells.empty()) {
        return {};
    }

    // Pick depot as the exploring cell nearest the robot.
    int depot_local = 0;
    double best_d = 1e9;
    for (size_t i = 0; i < cells.size(); ++i) {
        const auto c = grid_world_.CellCenter(cells[i]);
        const double d =
            ::autonomy::common::math::Vec2d(env_.robot_x(), env_.robot_y())
                .DistanceTo(::autonomy::common::math::Vec2d(c.x, c.y));
        if (d < best_d) {
            best_d = d;
            depot_local = static_cast<int>(i);
        }
    }

    const int n = static_cast<int>(cells.size());
    TspDataModel data;
    data.depot = depot_local;
    data.distance_matrix.assign(static_cast<size_t>(n),
                                std::vector<int>(static_cast<size_t>(n), 0));
    for (int i = 0; i < n; ++i) {
        const auto a = grid_world_.CellCenter(cells[static_cast<size_t>(i)]);
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                continue;
            }
            const auto b = grid_world_.CellCenter(cells[static_cast<size_t>(j)]);
            const auto geom = keypose_graph_.ShortestPathPoints(
                a.x, a.y, a.z, b.x, b.y, b.z);
            double len = PathLengthXy(geom);
            if (len < 1e-6) {
                len = ::autonomy::common::math::Vec2d(a.x, a.y)
                          .DistanceTo(::autonomy::common::math::Vec2d(b.x, b.y));
            }
            // Bias with graph cost from robot toward j when leaving depot.
            const double via =
                keypose_graph_.ShortestPathCostToPoint(b.x, b.y, b.z);
            data.distance_matrix[static_cast<size_t>(i)]
                                [static_cast<size_t>(j)] =
                TspSolver::MetersToCost(len + 0.1 * via);
        }
    }

    TspSolver solver(std::move(data), options_.tsp_max_exact_n());
    solver.Solve();
    std::vector<int> local_order;
    solver.GetSolutionNodeIndex(&local_order, false);

    std::vector<int> order;
    order.reserve(local_order.size());
    for (int idx : local_order) {
        if (idx < 0 || idx >= n) {
            continue;
        }
        order.push_back(cells[static_cast<size_t>(idx)]);
    }
    return order;
}

commsgs::geometry_msgs::PoseStamped HierarchicalPlanner::ComputeLookahead(
    const commsgs::planning_msgs::Path& path) const
{
    if (path.poses.size() < 2) {
        return MakePose(env_.robot_x(), env_.robot_y(), env_.robot_z(),
                        env_.robot_yaw());
    }

    const double robot_x = env_.robot_x();
    const double robot_y = env_.robot_y();
    const bool stop_at_unknown = options_.los_stop_at_unknown();
    const double lookahead = options_.lookahead_distance();
    const double min_ahead = std::max(0.6, 0.35 * lookahead);

    // Start from the path pose nearest the robot (avoid walking backward
    // through old keyposes near the origin).
    size_t start_i = 0;
    double nearest = 1e9;
    for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto& p = path.poses[i].pose.position;
        const double d =
            ::autonomy::common::math::Vec2d(robot_x, robot_y)
                .DistanceTo(::autonomy::common::math::Vec2d(p.x, p.y));
        if (d < nearest) {
            nearest = d;
            start_i = i;
        }
    }

    size_t best_i = start_i;
    double accum = 0.0;
    for (size_t i = start_i + 1; i < path.poses.size(); ++i) {
        const auto& a = path.poses[i - 1].pose.position;
        const auto& b = path.poses[i].pose.position;
        accum += ::autonomy::common::math::Vec2d(a.x, a.y)
                     .DistanceTo(::autonomy::common::math::Vec2d(b.x, b.y));

        if (env_.IsOccupied(b.x, b.y)) {
            break;
        }
        if (!HasLineOfSight(env_, robot_x, robot_y, b.x, b.y,
                            stop_at_unknown)) {
            break;
        }
        const double dist_robot =
            ::autonomy::common::math::Vec2d(robot_x, robot_y)
                .DistanceTo(::autonomy::common::math::Vec2d(b.x, b.y));
        if (dist_robot < min_ahead && i + 1 < path.poses.size()) {
            continue;
        }
        best_i = i;
        if (accum >= lookahead && dist_robot >= min_ahead) {
            break;
        }
    }

    if (best_i == start_i) {
        for (size_t i = start_i + 1; i < path.poses.size(); ++i) {
            const auto& p = path.poses[i].pose.position;
            if (env_.IsOccupied(p.x, p.y)) {
                continue;
            }
            const double dist_robot =
                ::autonomy::common::math::Vec2d(robot_x, robot_y)
                    .DistanceTo(::autonomy::common::math::Vec2d(p.x, p.y));
            if (dist_robot < min_ahead && i + 1 < path.poses.size()) {
                continue;
            }
            auto out = path.poses[i];
            out.header.frame_id = frame_id_;
            return out;
        }
        // Last resort: final pose even if close.
        auto out = path.poses.back();
        out.header.frame_id = frame_id_;
        return out;
    }

    auto out = path.poses[best_i];
    out.header.frame_id = frame_id_;
    return out;
}

bool HierarchicalPlanner::ExecutePlanningCycle()
{
    if (!has_odom_) {
        return false;
    }

    grid_world_.UpdateCellStatus(viewpoints_, env_);
    viewpoints_.UpdateFromEnv(env_, grid_world_.cell_status(),
                              grid_world_.cols(), grid_world_.cell_size(),
                              grid_world_.origin_x(), grid_world_.origin_y());

    global_cell_order_ = SolveGlobalCellOrder();
    path_ = local_planner_.Solve(env_, viewpoints_, keypose_graph_,
                                 global_cell_order_, grid_world_);
    path_.header.frame_id = frame_id_;
    lookahead_ = ComputeLookahead(path_);
    waypoint_index_ = 0;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        const auto& p = path_.poses[i].pose.position;
        const auto& q = lookahead_.pose.position;
        if (std::hypot(p.x - q.x, p.y - q.y) < 1e-3) {
            waypoint_index_ = i;
            break;
        }
    }
    has_target_ = path_.poses.size() > 1;

    if (!has_target_ && global_cell_order_.empty() &&
        local_planner_.IsLocalCoverageComplete()) {
        finished_ = true;
        has_target_ = false;
    }
    return has_target_;
}

float HierarchicalPlanner::Progress() const
{
    const float grid_p = grid_world_.CoverageProgress();
    const float local_p = local_planner_.IsLocalCoverageComplete() ? 1.f : 0.5f;
    return std::min(1.f, 0.5f * grid_p + 0.5f * local_p);
}

float HierarchicalPlanner::ExploredAreaM2() const
{
    size_t covered = 0;
    for (const auto st : grid_world_.cell_status()) {
        if (st == CellStatus::kCovered) {
            ++covered;
        }
    }
    const double cell_area =
        grid_world_.cell_size() * grid_world_.cell_size();
    return static_cast<float>(covered * cell_area);
}

commsgs::map_msgs::OccupancyGrid HierarchicalPlanner::GetOccupancyGrid(
    const std::string& frame_id) const
{
    return env_.GetOccupancyGrid(frame_id);
}

void HierarchicalPlanner::AdvanceWaypointIndex()
{
    if (path_.poses.size() < 2 ||
        waypoint_index_ + 1 >= path_.poses.size()) {
        has_target_ = false;
        return;
    }

    // Drop poses up to the reached lookahead, then re-pick min-ahead.
    if (waypoint_index_ > 0) {
        path_.poses.erase(path_.poses.begin(),
                          path_.poses.begin() +
                              static_cast<std::ptrdiff_t>(waypoint_index_));
    } else if (!path_.poses.empty()) {
        path_.poses.erase(path_.poses.begin());
    }
    if (path_.poses.size() < 2) {
        has_target_ = false;
        return;
    }

    lookahead_ = ComputeLookahead(path_);
    waypoint_index_ = 0;
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        const auto& p = path_.poses[i].pose.position;
        const auto& q = lookahead_.pose.position;
        if (std::hypot(p.x - q.x, p.y - q.y) < 1e-3) {
            waypoint_index_ = i;
            break;
        }
    }
    if (waypoint_index_ == 0 && path_.poses.size() > 1) {
        // Still underfoot: force next discrete pose.
        waypoint_index_ = 1;
        lookahead_ = path_.poses[1];
        lookahead_.header.frame_id = frame_id_;
    }
}

}  // namespace exploration
}  // namespace autonomy
