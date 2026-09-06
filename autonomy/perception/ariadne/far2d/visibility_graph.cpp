/*
 * Copyright 2026 The Openbot Authors
 */
#include "autonomy/perception/exploration/far2d/visibility_graph.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autonomy::perception::exploration {

namespace {

using map::costmap_2d::FREE_SPACE;
using map::costmap_2d::LETHAL_OBSTACLE;
using map::costmap_2d::NO_INFORMATION;
using automsgs::msgs::visualization_msgs::Marker;

int64_t CellKey(int gx, int gy) {
  return (static_cast<int64_t>(gx) << 32) ^
         static_cast<int64_t>(gy & 0xffffffff);
}

}  // namespace

VisibilityGraph::VisibilityGraph(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void VisibilityGraph::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void VisibilityGraph::Reset() {
  nodes_.clear();
  adjacency_.clear();
  edge_votes_.clear();
  robot_node_id_ = -1;
  next_id_ = 0;
}

void VisibilityGraph::Rebuild(const PlanningEnv& env) {
  Reset();
  Update(env, nullptr);
}

int VisibilityGraph::AddNode(double x, double y, bool is_robot,
                             bool is_frontier, int frontier_cluster_size) {
  VisibilityNode node;
  node.id = next_id_++;
  node.x = x;
  node.y = y;
  node.is_robot = is_robot;
  node.is_frontier = is_frontier;
  node.frontier_cluster_size = frontier_cluster_size;
  nodes_.push_back(node);
  adjacency_.emplace_back();
  if (is_robot) {
    robot_node_id_ = node.id;
  }
  return node.id;
}

bool VisibilityGraph::IsNearExisting(double x, double y) const {
  const double margin = options_.far3d_position_filter_m() > 0
                            ? options_.far3d_position_filter_m()
                            : 0.5;
  for (const auto& node : nodes_) {
    if (node.is_merged) {
      continue;
    }
    if (std::hypot(node.x - x, node.y - y) < margin) {
      return true;
    }
  }
  return false;
}

int VisibilityGraph::FindFrontierClusterSize(const PlanningEnv& env, double x,
                                             double y) const {
  return exploration::FindFrontierClusterSize(env, x, y);
}

void VisibilityGraph::AddContourNodes(const PlanningEnv& env) {
  const double robot_x = env.robot_x();
  const double robot_y = env.robot_y();
  const double range = options_.sensor_range_m();
  const auto& map = env.inflated_costmap();
  const int stride = std::max(options_.far_contour_stride_cells(), 1);
  const unsigned int w = map.getSizeInCellsX();
  const unsigned int h = map.getSizeInCellsY();

  std::unordered_map<int64_t, bool> used;
  for (unsigned int y = 1; y + 1 < h; y += static_cast<unsigned int>(stride)) {
    for (unsigned int x = 1; x + 1 < w; x += static_cast<unsigned int>(stride)) {
      if (map.getCost(x, y) != LETHAL_OBSTACLE) {
        continue;
      }
      bool on_boundary = false;
      for (int dy = -1; dy <= 1 && !on_boundary; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const unsigned char c =
              map.getCost(x + static_cast<unsigned int>(dx),
                          y + static_cast<unsigned int>(dy));
          if (c == FREE_SPACE || c == NO_INFORMATION) {
            on_boundary = true;
            break;
          }
        }
      }
      if (!on_boundary) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      map.mapToWorld(x, y, wx, wy);
      if (std::hypot(wx - robot_x, wy - robot_y) > range) {
        continue;
      }
      const int64_t k = CellKey(static_cast<int>(x), static_cast<int>(y));
      if (used.count(k) != 0) {
        continue;
      }
      used[k] = true;
      if (IsNearExisting(wx, wy)) {
        continue;
      }
      AddNode(wx, wy, false, false);
    }
  }
}

void VisibilityGraph::AddFrontierNodes(const PlanningEnv& env) {
  const double robot_x = env.robot_x();
  const double robot_y = env.robot_y();
  const double range = options_.sensor_range_m();
  const auto& frontiers = env.frontiers();
  const auto& sizes = env.frontier_cluster_sizes();
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const auto& frontier = frontiers[i];
    if (std::hypot(frontier.x() - robot_x, frontier.y() - robot_y) > range) {
      continue;
    }
    if (env.IsOccupied(frontier.x(), frontier.y())) {
      continue;
    }
    if (IsNearExisting(frontier.x(), frontier.y())) {
      continue;
    }
    const int cluster_size =
        i < sizes.size() ? sizes[static_cast<size_t>(i)] : 1;
    AddNode(frontier.x(), frontier.y(), false, true, cluster_size);
  }
}

void VisibilityGraph::AddTrajectoryNodes(
    const std::vector<std::pair<double, double>>& trajectory) {
  if (!options_.far_use_trajectory_edges()) {
    return;
  }
  for (const auto& pt : trajectory) {
    if (IsNearExisting(pt.first, pt.second)) {
      continue;
    }
    const int id = AddNode(pt.first, pt.second, false, false);
    nodes_[static_cast<size_t>(id)].is_trajectory = true;
    nodes_[static_cast<size_t>(id)].is_static = true;
  }
}

void VisibilityGraph::AddBoundaryNodes(const PlanningEnv& env) {
  if (!env.HasExplorationBoundary()) {
    return;
  }
  const auto boundary = env.GetExplorationBoundary();
  for (const auto& pt : boundary.points()) {
    if (IsNearExisting(pt.x(), pt.y())) {
      continue;
    }
    const int id = AddNode(pt.x(), pt.y(), false, false);
    nodes_[static_cast<size_t>(id)].is_boundary = true;
    nodes_[static_cast<size_t>(id)].is_static = true;
  }
}

void VisibilityGraph::PruneNodes(const PlanningEnv& env) {
  const double robot_x = env.robot_x();
  const double robot_y = env.robot_y();
  const double range = options_.sensor_range_m();
  const int pool = std::max(options_.far2d_graph_pool_size(), 64);
  const int dumper_thred = std::max(options_.far2d_dumper_thred(), 5);

  std::vector<int> active_ids;
  active_ids.reserve(nodes_.size());
  for (const auto& node : nodes_) {
    if (!node.is_merged && node.is_active) {
      active_ids.push_back(node.id);
    }
  }
  if (static_cast<int>(active_ids.size()) > pool) {
    std::sort(active_ids.begin(), active_ids.end(),
              [&](int a, int b) {
                const auto& na = nodes_[static_cast<size_t>(a)];
                const auto& nb = nodes_[static_cast<size_t>(b)];
                if (na.is_robot) {
                  return false;
                }
                if (nb.is_robot) {
                  return true;
                }
                const double da = std::hypot(na.x - robot_x, na.y - robot_y);
                const double db = std::hypot(nb.x - robot_x, nb.y - robot_y);
                return da > db;
              });
    for (size_t i = static_cast<size_t>(pool); i < active_ids.size(); ++i) {
      nodes_[static_cast<size_t>(active_ids[i])].is_merged = true;
    }
  }

  for (auto& node : nodes_) {
    if (node.is_robot || node.is_static) {
      continue;
    }
    if (std::hypot(node.x - robot_x, node.y - robot_y) > range) {
      node.clear_dumper_count++;
    } else {
      node.clear_dumper_count = std::max(0, node.clear_dumper_count - 1);
    }
    if (node.clear_dumper_count > dumper_thred) {
      node.is_merged = true;
    }
  }
  (void)env;
}

void VisibilityGraph::Update(
    const PlanningEnv& env,
    const std::vector<std::pair<double, double>>* trajectory) {
  const double robot_x = env.robot_x();
  const double robot_y = env.robot_y();

  if (robot_node_id_ < 0 ||
      robot_node_id_ >= static_cast<int>(nodes_.size()) ||
      nodes_[static_cast<size_t>(robot_node_id_)].is_merged) {
    robot_node_id_ = AddNode(robot_x, robot_y, true, false);
  } else {
    auto& robot = nodes_[static_cast<size_t>(robot_node_id_)];
    robot.x = robot_x;
    robot.y = robot_y;
    robot.is_merged = false;
    robot.is_active = true;
  }

  AddContourNodes(env);
  AddFrontierNodes(env);
  AddBoundaryNodes(env);
  if (trajectory != nullptr) {
    AddTrajectoryNodes(*trajectory);
  }
  PruneNodes(env);
  BuildEdges(env);
  ValidateEdges(env);
}

void VisibilityGraph::BuildEdges(const PlanningEnv& env) {
  const double max_dist = options_.far_connect_dist_m() > 0
                              ? options_.far_connect_dist_m()
                              : options_.sensor_range_m();
  const int n = static_cast<int>(nodes_.size());
  for (int i = 0; i < n; ++i) {
    if (nodes_[static_cast<size_t>(i)].is_merged) {
      continue;
    }
    for (int j = i + 1; j < n; ++j) {
      if (nodes_[static_cast<size_t>(j)].is_merged) {
        continue;
      }
      bool connected = false;
      for (const auto& edge : adjacency_[static_cast<size_t>(i)]) {
        if (edge.first == j) {
          connected = true;
          break;
        }
      }
      if (connected) {
        continue;
      }
      const double dist = std::hypot(nodes_[static_cast<size_t>(i)].x -
                                         nodes_[static_cast<size_t>(j)].x,
                                     nodes_[static_cast<size_t>(i)].y -
                                         nodes_[static_cast<size_t>(j)].y);
      if (dist > max_dist) {
        continue;
      }
      if (!HasLineOfSight(env, i, j)) {
        continue;
      }
      adjacency_[static_cast<size_t>(i)].emplace_back(j, dist);
      adjacency_[static_cast<size_t>(j)].emplace_back(i, dist);
      edge_votes_[i][j] = 0;
      edge_votes_[j][i] = 0;
    }
  }
}

void VisibilityGraph::ValidateEdges(const PlanningEnv& env) {
  const int finalize_thred = std::max(options_.far3d_edge_finalize_thred(), 2);
  const int n = static_cast<int>(nodes_.size());
  for (int i = 0; i < n; ++i) {
    if (nodes_[static_cast<size_t>(i)].is_merged) {
      adjacency_[static_cast<size_t>(i)].clear();
      continue;
    }
    std::vector<std::pair<int, double>> keep;
    for (const auto& edge : adjacency_[static_cast<size_t>(i)]) {
      const int j = edge.first;
      if (j < 0 || j >= n || nodes_[static_cast<size_t>(j)].is_merged) {
        continue;
      }
      if (HasLineOfSight(env, i, j)) {
        edge_votes_[i][j] = 0;
        keep.push_back(edge);
      } else {
        edge_votes_[i][j]++;
        if (edge_votes_[i][j] < finalize_thred) {
          keep.push_back(edge);
        }
      }
    }
    adjacency_[static_cast<size_t>(i)] = std::move(keep);
  }
}

bool VisibilityGraph::HasLineOfSight(const PlanningEnv& env, int from_id,
                                     int to_id) const {
  if (from_id < 0 || to_id < 0 ||
      from_id >= static_cast<int>(nodes_.size()) ||
      to_id >= static_cast<int>(nodes_.size())) {
    return false;
  }
  const auto& a = nodes_[static_cast<size_t>(from_id)];
  const auto& b = nodes_[static_cast<size_t>(to_id)];
  return LineOfSightChecker::HasLineOfSight(env, a.x, a.y, b.x, b.y);
}

automsgs::msgs::nav_msgs::Path VisibilityGraph::ToDebugPath(
    const std::string& frame_id) const {
  automsgs::msgs::nav_msgs::Path path;
  path.mutable_header()->set_frame_id(frame_id);
  for (size_t i = 0; i < adjacency_.size(); ++i) {
    if (nodes_[i].is_merged) {
      continue;
    }
    for (const auto& edge : adjacency_[i]) {
      if (static_cast<int>(i) >= edge.first) {
        continue;
      }
      if (nodes_[static_cast<size_t>(edge.first)].is_merged) {
        continue;
      }
      const auto& a = nodes_[i];
      const auto& b = nodes_[static_cast<size_t>(edge.first)];
      automsgs::msgs::geometry_msgs::PoseStamped pose_a;
      pose_a.mutable_header()->set_frame_id(frame_id);
      pose_a.mutable_pose()->mutable_position()->set_x(a.x);
      pose_a.mutable_pose()->mutable_position()->set_y(a.y);
      *path.add_poses() = pose_a;
      automsgs::msgs::geometry_msgs::PoseStamped pose_b;
      pose_b.mutable_header()->set_frame_id(frame_id);
      pose_b.mutable_pose()->mutable_position()->set_x(b.x);
      pose_b.mutable_pose()->mutable_position()->set_y(b.y);
      *path.add_poses() = pose_b;
    }
  }
  return path;
}

automsgs::msgs::visualization_msgs::MarkerArray VisibilityGraph::ToMarkerArray(
    const std::string& frame_id) const {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  Marker edge_marker;
  edge_marker.mutable_header()->set_frame_id(frame_id);
  edge_marker.set_ns("exploration_vg_edges");
  edge_marker.set_id(0);
  edge_marker.set_type(Marker::LINE_LIST);
  edge_marker.set_action(Marker::ADD);
  edge_marker.mutable_scale()->set_x(0.05);
  edge_marker.mutable_color()->set_r(0.2f);
  edge_marker.mutable_color()->set_g(0.8f);
  edge_marker.mutable_color()->set_b(0.3f);
  edge_marker.mutable_color()->set_a(0.9f);

  int node_id = 0;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const auto& node = nodes_[i];
    if (node.is_merged) {
      continue;
    }
    Marker sphere;
    sphere.mutable_header()->set_frame_id(frame_id);
    sphere.set_ns("exploration_vg_nodes");
    sphere.set_id(node_id++);
    sphere.set_type(Marker::SPHERE);
    sphere.set_action(Marker::ADD);
    sphere.mutable_pose()->mutable_position()->set_x(node.x);
    sphere.mutable_pose()->mutable_position()->set_y(node.y);
    sphere.mutable_pose()->mutable_position()->set_z(0.1);
    sphere.mutable_pose()->mutable_orientation()->set_w(1.0);
    sphere.mutable_scale()->set_x(0.25);
    sphere.mutable_scale()->set_y(0.25);
    sphere.mutable_scale()->set_z(0.25);
    if (node.is_robot) {
      sphere.mutable_color()->set_r(0.1f);
      sphere.mutable_color()->set_g(0.4f);
      sphere.mutable_color()->set_b(1.0f);
    } else if (node.is_frontier) {
      sphere.mutable_color()->set_r(1.0f);
      sphere.mutable_color()->set_g(0.5f);
      sphere.mutable_color()->set_b(0.0f);
    } else if (node.is_trajectory) {
      sphere.mutable_color()->set_r(0.2f);
      sphere.mutable_color()->set_g(0.9f);
      sphere.mutable_color()->set_b(0.9f);
    } else if (node.is_boundary) {
      sphere.mutable_color()->set_r(0.6f);
      sphere.mutable_color()->set_g(0.2f);
      sphere.mutable_color()->set_b(0.9f);
    } else {
      sphere.mutable_color()->set_r(0.9f);
      sphere.mutable_color()->set_g(0.9f);
      sphere.mutable_color()->set_b(0.2f);
    }
    sphere.mutable_color()->set_a(0.95f);
    *array.add_markers() = sphere;

    for (const auto& edge : adjacency_[i]) {
      if (static_cast<int>(i) >= edge.first) {
        continue;
      }
      const auto& nb = nodes_[static_cast<size_t>(edge.first)];
      if (nb.is_merged) {
        continue;
      }
      auto* pa = edge_marker.add_points();
      pa->set_x(node.x);
      pa->set_y(node.y);
      pa->set_z(0.05);
      auto* pb = edge_marker.add_points();
      pb->set_x(nb.x);
      pb->set_y(nb.y);
      pb->set_z(0.05);
    }
  }
  if (edge_marker.points_size() > 0) {
    *array.add_markers() = edge_marker;
  }
  return array;
}


bool VisibilityPlanner::Plan(const VisibilityGraph& graph, int from_id,
                             int to_id,
                             std::vector<automsgs::msgs::geometry_msgs::Point>*
                                 path) {
  if (path == nullptr || from_id < 0 || to_id < 0) {
    return false;
  }
  path->clear();

  const auto& nodes = graph.nodes();
  const auto& adjacency = graph.adjacency();
  if (from_id >= static_cast<int>(nodes.size()) ||
      to_id >= static_cast<int>(nodes.size())) {
    return false;
  }
  if (nodes[static_cast<size_t>(from_id)].is_merged ||
      nodes[static_cast<size_t>(to_id)].is_merged) {
    return false;
  }

  struct State {
    int id{0};
    double cost{0.0};
  };
  struct Cmp {
    bool operator()(const State& a, const State& b) const {
      return a.cost > b.cost;
    }
  };

  std::priority_queue<State, std::vector<State>, Cmp> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, int> parent;

  g_score[from_id] = 0.0;
  open.push({from_id, 0.0});

  while (!open.empty()) {
    const State current = open.top();
    open.pop();
    if (current.id == to_id) {
      break;
    }
    if (current.cost > g_score[current.id]) {
      continue;
    }
    if (current.id >= static_cast<int>(adjacency.size())) {
      continue;
    }
    for (const auto& edge : adjacency[static_cast<size_t>(current.id)]) {
      if (edge.first < 0 ||
          edge.first >= static_cast<int>(nodes.size()) ||
          nodes[static_cast<size_t>(edge.first)].is_merged) {
        continue;
      }
      const double tentative = current.cost + edge.second;
      auto it = g_score.find(edge.first);
      if (it != g_score.end() && tentative >= it->second) {
        continue;
      }
      g_score[edge.first] = tentative;
      parent[edge.first] = current.id;
      open.push({edge.first, tentative});
    }
  }

  if (g_score.find(to_id) == g_score.end()) {
    return false;
  }

  std::vector<int> order;
  for (int cur = to_id; cur != from_id;) {
    order.push_back(cur);
    auto it = parent.find(cur);
    if (it == parent.end()) {
      return false;
    }
    cur = it->second;
  }
  order.push_back(from_id);

  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    const auto& node = nodes[static_cast<size_t>(*it)];
    automsgs::msgs::geometry_msgs::Point pt;
    pt.set_x(node.x);
    pt.set_y(node.y);
    path->push_back(pt);
  }
  return !path->empty();
}

}  // namespace autonomy::perception::exploration
