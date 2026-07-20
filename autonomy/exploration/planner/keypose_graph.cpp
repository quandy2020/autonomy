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

#include "autonomy/exploration/planner/keypose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "Eigen/Core"

#include "autonomy/exploration/planner/los_checker.hpp"

namespace autonomy {
namespace exploration {
namespace {

double Distance3(double x1, double y1, double z1, double x2, double y2,
                 double z2)
{
    return (Eigen::Vector3d(x1, y1, z1) - Eigen::Vector3d(x2, y2, z2)).norm();
}

}  // namespace

KeyposeGraph::KeyposeGraph(const proto::ExplorationOptions& options)
    : options_(options)
{
}

void KeyposeGraph::SetOptions(const proto::ExplorationOptions& options)
{
    options_ = options;
}

commsgs::geometry_msgs::Point KeyposeGraph::ToPoint(const KeyposeNode& node)
{
    commsgs::geometry_msgs::Point p;
    p.x = node.x;
    p.y = node.y;
    p.z = node.z;
    return p;
}

int KeyposeGraph::AddOrGetNode(double x, double y, double z)
{
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (Distance3(nodes_[i].x, nodes_[i].y, nodes_[i].z, x, y, z) <
            options_.keypose_min_dist() * 0.5) {
            return static_cast<int>(i);
        }
    }
    KeyposeNode node;
    node.id = static_cast<int>(nodes_.size());
    node.x = x;
    node.y = y;
    node.z = z;
    nodes_.push_back(node);
    adj_.emplace_back();
    return node.id;
}

void KeyposeGraph::ConnectToNeighbors(int node_id, const PlanningEnv& env)
{
    const auto& node = nodes_[static_cast<size_t>(node_id)];
    const bool stop_at_unknown = options_.los_stop_at_unknown();
    const double detour_ratio =
        options_.keypose_astar_detour_ratio() > 0.0
            ? options_.keypose_astar_detour_ratio()
            : 2.5;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (static_cast<int>(i) == node_id) {
            continue;
        }
        const double dist = Distance3(node.x, node.y, node.z, nodes_[i].x,
                                      nodes_[i].y, nodes_[i].z);
        if (dist > options_.keypose_min_dist() * 4.0) {
            continue;
        }
        if (env.IsOccupied(nodes_[i].x, nodes_[i].y)) {
            continue;
        }
        if (HasLineOfSight(env, node.x, node.y, nodes_[i].x, nodes_[i].y,
                           stop_at_unknown)) {
            AddEdge(node_id, static_cast<int>(i), dist);
            continue;
        }
        // No LOS: try inflated-grid A* detour within ratio limit.
        std::vector<commsgs::geometry_msgs::Point> path;
        const double path_len =
            env.PlanPathAStar(node.x, node.y, nodes_[i].x, nodes_[i].y, &path);
        if (!std::isfinite(path_len) || path_len > detour_ratio * dist) {
            continue;
        }
        AddEdge(node_id, static_cast<int>(i), path_len);
    }
}

void KeyposeGraph::UpdateRobotPose(double x, double y, double z,
                                   const PlanningEnv& env)
{
    const int id = AddOrGetNode(x, y, z);
    ConnectToNeighbors(id, env);
    last_robot_node_ = id;
}

void KeyposeGraph::AddEdge(int from, int to, double cost)
{
    if (from < 0 || to < 0 || from >= static_cast<int>(adj_.size()) ||
        to >= static_cast<int>(adj_.size())) {
        return;
    }
    auto add_one = [&](int a, int b, double c) {
        auto& edges = adj_[static_cast<size_t>(a)];
        for (const auto& e : edges) {
            if (e.first == b) {
                return;
            }
        }
        edges.emplace_back(b, c);
    };
    add_one(from, to, cost);
    add_one(to, from, cost);
}

bool KeyposeGraph::Dijkstra(int from, std::vector<double>* dist,
                            std::vector<int>* parent) const
{
    if (!dist || !parent || nodes_.empty() || from < 0 ||
        from >= static_cast<int>(nodes_.size())) {
        return false;
    }
    const double inf = std::numeric_limits<double>::infinity();
    dist->assign(nodes_.size(), inf);
    parent->assign(nodes_.size(), -1);
    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    (*dist)[static_cast<size_t>(from)] = 0.0;
    pq.emplace(0.0, from);
    while (!pq.empty()) {
        const auto [d, u] = pq.top();
        pq.pop();
        if (d > (*dist)[static_cast<size_t>(u)]) {
            continue;
        }
        for (const auto& [v, w] : adj_[static_cast<size_t>(u)]) {
            const double nd = d + w;
            if (nd < (*dist)[static_cast<size_t>(v)]) {
                (*dist)[static_cast<size_t>(v)] = nd;
                (*parent)[static_cast<size_t>(v)] = u;
                pq.emplace(nd, v);
            }
        }
    }
    return true;
}

double KeyposeGraph::ShortestPathCost(int from, int to) const
{
    if (from == to && from >= 0) {
        return 0.0;
    }
    std::vector<double> dist;
    std::vector<int> parent;
    if (!Dijkstra(from, &dist, &parent)) {
        return 1e9;
    }
    if (to < 0 || to >= static_cast<int>(dist.size()) ||
        !std::isfinite(dist[static_cast<size_t>(to)])) {
        if (from >= 0 && from < static_cast<int>(nodes_.size()) && to >= 0 &&
            to < static_cast<int>(nodes_.size())) {
            return Distance3(nodes_[static_cast<size_t>(from)].x,
                             nodes_[static_cast<size_t>(from)].y,
                             nodes_[static_cast<size_t>(from)].z,
                             nodes_[static_cast<size_t>(to)].x,
                             nodes_[static_cast<size_t>(to)].y,
                             nodes_[static_cast<size_t>(to)].z);
        }
        return 1e9;
    }
    return dist[static_cast<size_t>(to)];
}

std::vector<int> KeyposeGraph::ShortestPathNodeIds(int from, int to) const
{
    std::vector<int> ids;
    if (from == to && from >= 0) {
        ids.push_back(from);
        return ids;
    }
    std::vector<double> dist;
    std::vector<int> parent;
    if (!Dijkstra(from, &dist, &parent) || to < 0 ||
        to >= static_cast<int>(parent.size()) ||
        !std::isfinite(dist[static_cast<size_t>(to)])) {
        return ids;
    }
    for (int cur = to; cur >= 0; cur = parent[static_cast<size_t>(cur)]) {
        ids.push_back(cur);
        if (cur == from) {
            break;
        }
    }
    std::reverse(ids.begin(), ids.end());
    if (ids.empty() || ids.front() != from) {
        ids.clear();
    }
    return ids;
}

int KeyposeGraph::NearestNode(double x, double y, double z) const
{
    if (nodes_.empty()) {
        return -1;
    }
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < nodes_.size(); ++i) {
        const double d =
            Distance3(nodes_[i].x, nodes_[i].y, nodes_[i].z, x, y, z);
        if (d < best_d) {
            best_d = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

double KeyposeGraph::ShortestPathCostToPoint(double x, double y,
                                             double z) const
{
    const int nearest = NearestNode(x, y, z);
    if (nearest < 0 || last_robot_node_ < 0) {
        return 0.0;
    }
    return ShortestPathCost(last_robot_node_, nearest);
}

std::vector<commsgs::geometry_msgs::Point> KeyposeGraph::ShortestPathPoints(
    double from_x, double from_y, double from_z, double to_x, double to_y,
    double to_z) const
{
    std::vector<commsgs::geometry_msgs::Point> path;
    commsgs::geometry_msgs::Point start;
    start.x = from_x;
    start.y = from_y;
    start.z = from_z;
    commsgs::geometry_msgs::Point goal;
    goal.x = to_x;
    goal.y = to_y;
    goal.z = to_z;

    auto direct = [&]() {
        std::vector<commsgs::geometry_msgs::Point> out;
        out.push_back(start);
        if (Distance3(from_x, from_y, from_z, to_x, to_y, to_z) > 1e-3) {
            out.push_back(goal);
        }
        return out;
    };

    const double eucl = Distance3(from_x, from_y, from_z, to_x, to_y, to_z);
    if (eucl < 1e-3) {
        return direct();
    }

    const int from_id = NearestNode(from_x, from_y, from_z);
    const int to_id = NearestNode(to_x, to_y, to_z);
    if (from_id < 0 || to_id < 0) {
        return direct();
    }

    // Only trust roadmap snaps that are actually near the query points.
    // Otherwise NearestNode pulls short local goals through the origin keypose.
    const double snap = std::max(0.5, options_.keypose_min_dist());
    const auto& fn = nodes_[static_cast<size_t>(from_id)];
    const auto& tn = nodes_[static_cast<size_t>(to_id)];
    if (Distance3(fn.x, fn.y, fn.z, from_x, from_y, from_z) > snap ||
        Distance3(tn.x, tn.y, tn.z, to_x, to_y, to_z) > snap ||
        from_id == to_id) {
        return direct();
    }

    const auto node_ids = ShortestPathNodeIds(from_id, to_id);
    if (node_ids.empty()) {
        return direct();
    }

    path.push_back(start);
    for (int id : node_ids) {
        const auto p = ToPoint(nodes_[static_cast<size_t>(id)]);
        if (path.empty() ||
            Distance3(path.back().x, path.back().y, path.back().z, p.x, p.y,
                      p.z) > 1e-3) {
            path.push_back(p);
        }
    }
    if (Distance3(path.back().x, path.back().y, path.back().z, goal.x, goal.y,
                  goal.z) > 1e-3) {
        path.push_back(goal);
    }

    const double graph_len = PathLengthXy(path);
    const double detour =
        options_.keypose_astar_detour_ratio() > 0.0
            ? options_.keypose_astar_detour_ratio()
            : 2.5;
    if (!std::isfinite(graph_len) || graph_len > detour * eucl) {
        return direct();
    }
    return path;
}

}  // namespace exploration
}  // namespace autonomy
