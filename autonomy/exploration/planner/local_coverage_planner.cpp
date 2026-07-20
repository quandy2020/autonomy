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

#include "autonomy/exploration/planner/local_coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Core"

#include "autonomy/exploration/planner/los_checker.hpp"
#include "autonomy/exploration/planner/tsp_solver.hpp"

namespace autonomy {
namespace exploration {
namespace {

double Euclidean3(double x0, double y0, double z0, double x1, double y1,
                  double z1)
{
    return (Eigen::Vector3d(x0, y0, z0) - Eigen::Vector3d(x1, y1, z1)).norm();
}

}  // namespace

LocalCoveragePlanner::LocalCoveragePlanner(
    const proto::ExplorationOptions& options)
    : options_(options)
{
}

void LocalCoveragePlanner::SetOptions(const proto::ExplorationOptions& options)
{
    options_ = options;
}

void LocalCoveragePlanner::AppendGeometricSegment(
    const KeyposeGraph& keypose_graph, double from_x, double from_y,
    double from_z, const Viewpoint& to, const std::string& frame_id,
    commsgs::planning_msgs::Path* path) const
{
    if (!path) {
        return;
    }
    const auto points = keypose_graph.ShortestPathPoints(
        from_x, from_y, from_z, to.x, to.y, to.z);
    for (size_t i = 0; i < points.size(); ++i) {
        Viewpoint mid;
        mid.x = points[i].x;
        mid.y = points[i].y;
        mid.z = points[i].z;
        if (i + 1 < points.size()) {
            mid.yaw = std::atan2(points[i + 1].y - points[i].y,
                                 points[i + 1].x - points[i].x);
        } else {
            mid.yaw = to.yaw;
        }
        // Skip duplicating the segment start if it matches the last pose.
        if (!path->poses.empty()) {
            const auto& last = path->poses.back().pose.position;
            if (Euclidean3(last.x, last.y, last.z, mid.x, mid.y, mid.z) <
                1e-3) {
                continue;
            }
        }
        path->poses.push_back(mid.ToPoseStamped(frame_id));
    }
}

commsgs::planning_msgs::Path LocalCoveragePlanner::Solve(
    const PlanningEnv& env, ViewpointManager& viewpoints,
    const KeyposeGraph& keypose_graph,
    const std::vector<int>& global_cell_order, const GridWorld& grid_world)
{
    local_complete_ = false;
    commsgs::planning_msgs::Path path;
    path.header.frame_id = "map";

    path.poses.push_back(
        viewpoints.GetRobotViewpoint().ToPoseStamped(path.header.frame_id));

    auto candidates = viewpoints.GetCandidates(options_.viewpoint().min_gain());
    const int max_nodes = std::max(1, options_.local_tsp_max_nodes());
    if (static_cast<int>(candidates.size()) > max_nodes) {
        candidates.resize(static_cast<size_t>(max_nodes));
    }

    if (candidates.empty()) {
        local_complete_ = true;
        Viewpoint goal;
        bool have_goal = false;
        // Prefer a frontier/target that is ahead of the robot — not underfoot.
        constexpr double kMinGoalDist = 1.0;
        constexpr double kMaxGoalDist = 8.0;
        auto pick_goal =
            [&](const std::vector<commsgs::geometry_msgs::Point>& pts) {
                int best = -1;
                double best_score = -1.0;
                int farthest = -1;
                double farthest_d = -1.0;
                for (size_t i = 0; i < pts.size(); ++i) {
                    const auto& p = pts[i];
                    const double d = Euclidean3(
                        env.robot_x(), env.robot_y(), env.robot_z(), p.x, p.y,
                        p.z);
                    if (d > farthest_d) {
                        farthest_d = d;
                        farthest = static_cast<int>(i);
                    }
                    if (d < kMinGoalDist || d > kMaxGoalDist) {
                        continue;
                    }
                    // Prefer farther goals within the local horizon.
                    if (d > best_score) {
                        best_score = d;
                        best = static_cast<int>(i);
                    }
                }
                const int pick = best >= 0 ? best : farthest;
                if (pick < 0) {
                    return;
                }
                const auto& p = pts[static_cast<size_t>(pick)];
                goal.x = p.x;
                goal.y = p.y;
                goal.z = env.robot_z();
                goal.yaw =
                    std::atan2(p.y - env.robot_y(), p.x - env.robot_x());
                have_goal = true;
            };
        if (!env.targets().empty()) {
            pick_goal(env.targets());
        } else if (!env.frontiers().empty()) {
            pick_goal(env.frontiers());
        } else if (!global_cell_order.empty()) {
            const int cell = global_cell_order.front();
            const auto center = grid_world.CellCenter(cell);
            goal.x = center.x;
            goal.y = center.y;
            goal.z = env.robot_z();
            goal.yaw =
                std::atan2(center.y - env.robot_y(), center.x - env.robot_x());
            have_goal = true;
        }
        if (have_goal) {
            AppendGeometricSegment(keypose_graph, env.robot_x(), env.robot_y(),
                                   env.robot_z(), goal, path.header.frame_id,
                                   &path);
            // Ensure at least a 2-pose path for RViz Path display.
            if (path.poses.size() < 2) {
                path.poses.push_back(goal.ToPoseStamped(path.header.frame_id));
            }
        }
        return path;
    }

    // Nodes: 0 = robot depot, 1..K = candidates, K+1 = optional dummy (open).
    const int k = static_cast<int>(candidates.size());
    const bool use_dummy = !global_cell_order.empty();
    const int n = use_dummy ? k + 2 : k + 1;
    const int dummy = use_dummy ? n - 1 : -1;

    TspDataModel data;
    data.depot = 0;
    data.distance_matrix.assign(static_cast<size_t>(n),
                                std::vector<int>(static_cast<size_t>(n), 0));

    auto node_xyz = [&](int idx, double* x, double* y, double* z) {
        if (idx == 0) {
            *x = env.robot_x();
            *y = env.robot_y();
            *z = env.robot_z();
            return;
        }
        if (use_dummy && idx == dummy) {
            const auto center = grid_world.CellCenter(global_cell_order.front());
            *x = center.x;
            *y = center.y;
            *z = env.robot_z();
            return;
        }
        const auto& vp = candidates[static_cast<size_t>(idx - 1)];
        *x = vp.x;
        *y = vp.y;
        *z = vp.z;
    };

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                data.distance_matrix[static_cast<size_t>(i)]
                                    [static_cast<size_t>(j)] = 0;
                continue;
            }
            if (use_dummy) {
                // Open tour: free arcs involving dummy / force exit via dummy.
                if (i == dummy || j == dummy) {
                    if ((i == 0 && j == dummy) || (i == dummy && j == 0)) {
                        data.distance_matrix[static_cast<size_t>(i)]
                                            [static_cast<size_t>(j)] = 9999;
                    } else if (j == dummy || i == dummy) {
                        data.distance_matrix[static_cast<size_t>(i)]
                                            [static_cast<size_t>(j)] = 0;
                    }
                    continue;
                }
            }
            double x0 = 0.0;
            double y0 = 0.0;
            double z0 = 0.0;
            double x1 = 0.0;
            double y1 = 0.0;
            double z1 = 0.0;
            node_xyz(i, &x0, &y0, &z0);
            node_xyz(j, &x1, &y1, &z1);
            const auto geom =
                keypose_graph.ShortestPathPoints(x0, y0, z0, x1, y1, z1);
            double len = PathLengthXy(geom);
            if (len < 1e-6) {
                len = Euclidean3(x0, y0, z0, x1, y1, z1);
            }
            data.distance_matrix[static_cast<size_t>(i)]
                                [static_cast<size_t>(j)] =
                TspSolver::MetersToCost(len);
        }
    }

    TspSolver solver(std::move(data), options_.tsp_max_exact_n());
    solver.Solve();
    std::vector<int> order_idx;
    solver.GetSolutionNodeIndex(&order_idx, use_dummy);

    double cur_x = env.robot_x();
    double cur_y = env.robot_y();
    double cur_z = env.robot_z();
    for (int node : order_idx) {
        if (node <= 0) {
            continue;
        }
        if (use_dummy && node == dummy) {
            continue;
        }
        if (node - 1 >= static_cast<int>(candidates.size())) {
            continue;
        }
        auto& vp = candidates[static_cast<size_t>(node - 1)];
        vp.selected = true;
        AppendGeometricSegment(keypose_graph, cur_x, cur_y, cur_z, vp,
                               path.header.frame_id, &path);
        viewpoints.MarkVisited(vp.id);
        cur_x = vp.x;
        cur_y = vp.y;
        cur_z = vp.z;
    }

    if (path.poses.size() <= 1) {
        local_complete_ = true;
    }

    if (!global_cell_order.empty()) {
        const int cell = global_cell_order.front();
        const auto center = grid_world.CellCenter(cell);
        Viewpoint global_vp;
        global_vp.x = center.x;
        global_vp.y = center.y;
        global_vp.z = env.robot_z();
        global_vp.yaw =
            std::atan2(center.y - cur_y, center.x - cur_x);
        AppendGeometricSegment(keypose_graph, cur_x, cur_y, cur_z, global_vp,
                               path.header.frame_id, &path);
    }

    return path;
}

}  // namespace exploration
}  // namespace autonomy
