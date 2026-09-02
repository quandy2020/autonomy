/*
 * Copyright 2026 The Openbot Authors
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "autonomy/perception/exploration/tare/keypose_graph.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"

namespace autonomy::perception::exploration {
namespace {

void AppendSegment(
    const std::vector<automsgs::msgs::geometry_msgs::Point>& segment,
    std::vector<automsgs::msgs::geometry_msgs::Point>* path) {
  if (path == nullptr || segment.empty()) {
    return;
  }
  if (path->empty()) {
    path->insert(path->end(), segment.begin(), segment.end());
    return;
  }
  size_t start = 0;
  const auto& last = path->back();
  if (std::hypot(segment.front().x() - last.x(),
                 segment.front().y() - last.y()) < 1e-3) {
    start = 1;
  }
  for (size_t i = start; i < segment.size(); ++i) {
    path->push_back(segment[i]);
  }
}

double PolylineLength(
    const std::vector<automsgs::msgs::geometry_msgs::Point>& path) {
  if (path.size() < 2) {
    return 0.0;
  }
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x() - path[i - 1].x(),
                      path[i].y() - path[i - 1].y());
  }
  return len;
}

}  // namespace

KeyposeGraph::KeyposeGraph(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void KeyposeGraph::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void KeyposeGraph::UpdateRobotPose(double x, double y, double yaw,
                                   const PlanningEnv& env) {
  if (!has_last_keypose_) {
    TryAddKeypose(x, y, yaw, env);
    return;
  }
  const double dist =
      std::hypot(x - last_keypose_x_, y - last_keypose_y_);
  if (dist >= options_.keypose_spacing_m()) {
    TryAddKeypose(x, y, yaw, env);
  }
}

void KeyposeGraph::TryAddKeypose(double x, double y, double yaw,
                                 const PlanningEnv& env) {
  if (env.IsOccupied(x, y)) {
    return;
  }
  Node node;
  node.x = x;
  node.y = y;
  node.yaw = yaw;
  nodes_.push_back(node);
  adjacency_.emplace_back();
  last_keypose_x_ = x;
  last_keypose_y_ = y;
  has_last_keypose_ = true;
  RebuildEdges(env);
}

void KeyposeGraph::RebuildEdges(const PlanningEnv& env) {
  const double max_dist = options_.keypose_connect_dist_m();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    adjacency_[i].clear();
    for (size_t j = 0; j < nodes_.size(); ++j) {
      if (i == j) {
        continue;
      }
      const double dist = std::hypot(nodes_[i].x - nodes_[j].x,
                                     nodes_[i].y - nodes_[j].y);
      if (dist > max_dist) {
        continue;
      }
      std::vector<automsgs::msgs::geometry_msgs::Point> path;
      const double len = PathPlanner::Plan(env, nodes_[i].x, nodes_[i].y,
                                                nodes_[j].x, nodes_[j].y, &path);
      if (std::isfinite(len)) {
        adjacency_[i].push_back(static_cast<int>(j));
      }
    }
  }
}

int KeyposeGraph::ClosestNodeIndex(double x, double y) const {
  if (nodes_.empty()) {
    return -1;
  }
  int best = 0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const double dist = std::hypot(nodes_[i].x - x, nodes_[i].y - y);
    if (dist < best_dist) {
      best_dist = dist;
      best = static_cast<int>(i);
    }
  }
  return best;
}

bool KeyposeGraph::RunDijkstra(int from, int to, std::vector<int>* order,
                               double* length) const {
  if (order == nullptr || length == nullptr || from < 0 || to < 0 ||
      static_cast<size_t>(from) >= nodes_.size() ||
      static_cast<size_t>(to) >= nodes_.size()) {
    return false;
  }
  const size_t n = nodes_.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> parent(n, -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  dist[static_cast<size_t>(from)] = 0.0;
  open.push({0.0, from});
  while (!open.empty()) {
    const Entry current = open.top();
    open.pop();
    if (current.first > dist[static_cast<size_t>(current.second)]) {
      continue;
    }
    if (current.second == to) {
      break;
    }
    for (int next : adjacency_[static_cast<size_t>(current.second)]) {
      const double edge = std::hypot(
          nodes_[static_cast<size_t>(current.second)].x - nodes_[next].x,
          nodes_[static_cast<size_t>(current.second)].y - nodes_[next].y);
      const double tentative = dist[static_cast<size_t>(current.second)] + edge;
      if (tentative >= dist[static_cast<size_t>(next)]) {
        continue;
      }
      dist[static_cast<size_t>(next)] = tentative;
      parent[static_cast<size_t>(next)] = current.second;
      open.push({tentative, next});
    }
  }
  if (!std::isfinite(dist[static_cast<size_t>(to)])) {
    return false;
  }
  order->clear();
  for (int cur = to; cur != -1; cur = parent[static_cast<size_t>(cur)]) {
    order->push_back(cur);
  }
  std::reverse(order->begin(), order->end());
  *length = dist[static_cast<size_t>(to)];
  return !order->empty();
}

bool KeyposeGraph::QueryPath(
    const PlanningEnv& env, double from_x, double from_y, double to_x,
    double to_y, std::vector<automsgs::msgs::geometry_msgs::Point>* path) const {
  if (path == nullptr) {
    return false;
  }
  path->clear();
  if (nodes_.empty()) {
    return PathPlanner::Plan(env, from_x, from_y, to_x, to_y, path) >= 0.0 &&
           !path->empty();
  }

  const int from_node = ClosestNodeIndex(from_x, from_y);
  const int to_node = ClosestNodeIndex(to_x, to_y);
  if (from_node < 0 || to_node < 0) {
    return false;
  }

  std::vector<int> graph_order;
  double graph_len = 0.0;
  if (!RunDijkstra(from_node, to_node, &graph_order, &graph_len)) {
    return PathPlanner::Plan(env, from_x, from_y, to_x, to_y, path) >= 0.0 &&
           !path->empty();
  }

  std::vector<automsgs::msgs::geometry_msgs::Point> prefix;
  if (std::hypot(nodes_[static_cast<size_t>(from_node)].x - from_x,
                 nodes_[static_cast<size_t>(from_node)].y - from_y) > 0.05) {
  PathPlanner::Plan(env, from_x, from_y, nodes_[static_cast<size_t>(from_node)].x,
                    nodes_[static_cast<size_t>(from_node)].y, &prefix);
    AppendSegment(prefix, path);
  } else {
    automsgs::msgs::geometry_msgs::Point pt;
    pt.set_x(from_x);
    pt.set_y(from_y);
    path->push_back(pt);
  }

  for (int idx : graph_order) {
    automsgs::msgs::geometry_msgs::Point pt;
    pt.set_x(nodes_[static_cast<size_t>(idx)].x);
    pt.set_y(nodes_[static_cast<size_t>(idx)].y);
    AppendSegment({pt}, path);
  }

  std::vector<automsgs::msgs::geometry_msgs::Point> suffix;
  if (std::hypot(nodes_[static_cast<size_t>(to_node)].x - to_x,
                 nodes_[static_cast<size_t>(to_node)].y - to_y) > 0.05) {
    PathPlanner::Plan(env, nodes_[static_cast<size_t>(to_node)].x,
                      nodes_[static_cast<size_t>(to_node)].y, to_x, to_y,
                      &suffix);
    AppendSegment(suffix, path);
  } else if (path->empty() ||
             std::hypot(path->back().x() - to_x, path->back().y() - to_y) >
                 0.05) {
    automsgs::msgs::geometry_msgs::Point pt;
    pt.set_x(to_x);
    pt.set_y(to_y);
    path->push_back(pt);
  }
  return !path->empty();
}

double KeyposeGraph::QueryDistance(const PlanningEnv& env, double from_x,
                                   double from_y, double to_x,
                                   double to_y) const {
  std::vector<automsgs::msgs::geometry_msgs::Point> path;
  if (!QueryPath(env, from_x, from_y, to_x, to_y, &path)) {
    return std::numeric_limits<double>::infinity();
  }
  return PolylineLength(path);
}

bool KeyposeGraph::QueryPathToFirstKeypose(
    const PlanningEnv& env, double from_x, double from_y,
    std::vector<automsgs::msgs::geometry_msgs::Point>* path) const {
  if (nodes_.empty()) {
    return false;
  }
  return QueryPath(env, from_x, from_y, nodes_.front().x, nodes_.front().y,
                   path);
}

bool KeyposeGraph::HasPath(const PlanningEnv& env, double from_x, double from_y,
                           double to_x, double to_y) const {
  std::vector<automsgs::msgs::geometry_msgs::Point> path;
  return QueryPath(env, from_x, from_y, to_x, to_y, &path);
}

void KeyposeGraph::Reset() {
  nodes_.clear();
  adjacency_.clear();
  has_last_keypose_ = false;
  last_keypose_x_ = 0.0;
  last_keypose_y_ = 0.0;
}

}  // namespace autonomy::perception::exploration
