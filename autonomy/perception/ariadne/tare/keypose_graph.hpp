/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>

#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

// Sparse keypose connectivity graph (TARE KeyposeGraph, 2D).
class KeyposeGraph {
 public:
  struct Node {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  explicit KeyposeGraph(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void UpdateRobotPose(double x, double y, double yaw, const PlanningEnv& env);

  bool empty() const { return nodes_.empty(); }
  std::size_t size() const { return nodes_.size(); }

  bool HasPath(const PlanningEnv& env, double from_x, double from_y, double to_x,
               double to_y) const;

  // Shortest path length via keypose graph + NavFn connectors; falls back to NavFn.
  double QueryDistance(const PlanningEnv& env, double from_x, double from_y,
                       double to_x, double to_y) const;

  bool QueryPath(const PlanningEnv& env, double from_x, double from_y, double to_x,
                 double to_y,
                 std::vector<automsgs::msgs::geometry_msgs::Point>* path) const;

  bool QueryPathToFirstKeypose(
      const PlanningEnv& env, double from_x, double from_y,
      std::vector<automsgs::msgs::geometry_msgs::Point>* path) const;

  void Reset();

  const std::vector<Node>& nodes() const { return nodes_; }
  const std::vector<std::vector<int>>& adjacency() const { return adjacency_; }

 private:
  void TryAddKeypose(double x, double y, double yaw, const PlanningEnv& env);
  void RebuildEdges(const PlanningEnv& env);
  int ClosestNodeIndex(double x, double y) const;
  bool RunDijkstra(int from, int to, std::vector<int>* order,
                   double* length) const;

  proto::ExplorationOptions options_;
  std::vector<Node> nodes_;
  std::vector<std::vector<int>> adjacency_;
  double last_keypose_x_{0.0};
  double last_keypose_y_{0.0};
  bool has_last_keypose_{false};
};

}  // namespace autonomy::perception::exploration
