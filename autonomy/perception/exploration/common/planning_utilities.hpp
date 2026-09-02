/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared planning helpers: boundary, LOS, path, momentum, goals, typed paths.
 */

#pragma once

#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

class BoundaryLoader {
 public:
  static bool LoadPolygonFromPly(
      const std::string& path,
      automsgs::msgs::geometry_msgs::Polygon* polygon);

  static bool LoadPolygonFromConfigPath(
      const std::string& relative_or_absolute_path,
      const std::vector<std::string>& config_directories,
      automsgs::msgs::geometry_msgs::Polygon* polygon);
};

class LineOfSightChecker {
 public:
  static bool HasLineOfSight(const PlanningEnv& env, double x0, double y0,
                             double x1, double y1);
  static bool HasLineOfSightOnFreeSpace(const PlanningEnv& env, double x0,
                                        double y0, double x1, double y1);
};

class PathPlanner {
 public:
  static double Plan(const PlanningEnv& env, double from_x, double from_y,
                     double to_x, double to_y,
                     std::vector<automsgs::msgs::geometry_msgs::Point>* path);
};

class MomentumPlanner {
 public:
  explicit MomentumPlanner(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void Reset();

  bool ShouldKeepPreviousPath(const automsgs::msgs::nav_msgs::Path& previous,
                              const automsgs::msgs::nav_msgs::Path& candidate,
                              double robot_x, double robot_y,
                              double robot_yaw) const;

  automsgs::msgs::geometry_msgs::PoseStamped SmoothWaypoint(
      const automsgs::msgs::geometry_msgs::PoseStamped& candidate,
      const automsgs::msgs::geometry_msgs::PoseStamped& previous,
      double robot_x, double robot_y, double robot_yaw) const;

  void OnPathAccepted();

 private:
  proto::ExplorationOptions options_;
  int path_momentum_counter_{0};
  automsgs::msgs::nav_msgs::Path recorded_path_;
};

class ExplorationPath {
 public:
  enum class NodeType {
    kRobot,
    kGlobalViewpoint,
    kGlobalViaPoint,
    kLocalViewpoint,
    kLocalViaPoint,
    kLookahead,
    kHome,
  };

  struct Node {
    NodeType type{NodeType::kLocalViaPoint};
    automsgs::msgs::geometry_msgs::Point position;
    double yaw{0.0};
  };

  void Clear();
  void Append(const Node& node);
  void AppendPath(const automsgs::msgs::nav_msgs::Path& path, NodeType type);
  double Length() const;
  automsgs::msgs::nav_msgs::Path ToNavPath(const std::string& frame_id) const;

  const std::vector<Node>& nodes() const { return nodes_; }

 private:
  std::vector<Node> nodes_;
};

double ScoreFarGoal(const PlanningEnv& env,
                    const proto::ExplorationOptions& options, double goal_x,
                    double goal_y, bool is_frontier, int frontier_cluster_size,
                    double robot_x, double robot_y);

int FindFrontierClusterSize(const PlanningEnv& env, double x, double y);

}  // namespace autonomy::perception::exploration
