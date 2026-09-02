/*
 * Copyright 2026 The Openbot Authors
 *
 * FAR 2D visibility graph and shortest-path planner.
 */

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/exploration/common/types.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

class VisibilityGraph {
 public:
  explicit VisibilityGraph(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void Reset();
  void Update(const PlanningEnv& env,
              const std::vector<std::pair<double, double>>* trajectory = nullptr);
  void Rebuild(const PlanningEnv& env);
  void AddTrajectoryNodes(
      const std::vector<std::pair<double, double>>& trajectory);
  void AddBoundaryNodes(const PlanningEnv& env);

  const std::vector<VisibilityNode>& nodes() const { return nodes_; }
  const std::vector<std::vector<std::pair<int, double>>>& adjacency() const {
    return adjacency_;
  }
  int robot_node_id() const { return robot_node_id_; }

  automsgs::msgs::nav_msgs::Path ToDebugPath(
      const std::string& frame_id) const;
  automsgs::msgs::visualization_msgs::MarkerArray ToMarkerArray(
      const std::string& frame_id) const;

 private:
  int AddNode(double x, double y, bool is_robot, bool is_frontier,
              int frontier_cluster_size = 0);
  void AddContourNodes(const PlanningEnv& env);
  void AddFrontierNodes(const PlanningEnv& env);
  bool IsNearExisting(double x, double y) const;
  void PruneNodes(const PlanningEnv& env);
  void BuildEdges(const PlanningEnv& env);
  void ValidateEdges(const PlanningEnv& env);
  bool HasLineOfSight(const PlanningEnv& env, int from_id, int to_id) const;
  int FindFrontierClusterSize(const PlanningEnv& env, double x,
                              double y) const;

  proto::ExplorationOptions options_;
  std::vector<VisibilityNode> nodes_;
  std::vector<std::vector<std::pair<int, double>>> adjacency_;
  std::unordered_map<int, std::unordered_map<int, int>> edge_votes_;
  int robot_node_id_{-1};
  int next_id_{0};
};

class VisibilityPlanner {
 public:
  static bool Plan(const VisibilityGraph& graph, int from_id, int to_id,
                   std::vector<automsgs::msgs::geometry_msgs::Point>* path);
};

}  // namespace autonomy::perception::exploration
