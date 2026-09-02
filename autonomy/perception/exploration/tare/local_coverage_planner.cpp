/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/local_coverage_planner.hpp"

#include <cmath>
#include <limits>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/tare/tsp_solver.hpp"

namespace autonomy::perception::exploration {
namespace {

struct Waypoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

int DistanceCost(const PlanningEnv& env, const Waypoint& from,
                 const Waypoint& to) {
  std::vector<automsgs::msgs::geometry_msgs::Point> segment;
  const double len =
      PathPlanner::Plan(env, from.x, from.y, to.x, to.y, &segment);
  if (!std::isfinite(len)) {
    return TspSolver::kInfCost;
  }
  return static_cast<int>(std::lround(len * 100.0));
}

automsgs::msgs::geometry_msgs::PoseStamped MakePose(
    const std::string& frame_id, double x, double y, double yaw) {
  automsgs::msgs::geometry_msgs::PoseStamped pose;
  pose.mutable_header()->set_frame_id(frame_id);
  pose.mutable_pose()->mutable_position()->set_x(x);
  pose.mutable_pose()->mutable_position()->set_y(y);
  pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  return pose;
}

}  // namespace

LocalCoveragePlanner::LocalCoveragePlanner(
    const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void LocalCoveragePlanner::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

automsgs::msgs::nav_msgs::Path LocalCoveragePlanner::Solve(
    const PlanningEnv& env, const ViewpointManager& viewpoints, double robot_x,
    double robot_y,
    const std::vector<automsgs::msgs::geometry_msgs::Point>& global_targets) {
  automsgs::msgs::nav_msgs::Path best_path;
  double best_len = std::numeric_limits<double>::infinity();
  const int iters = std::max(options_.local_path_optimization_iters(), 1);

  for (int iter = 0; iter < iters; ++iter) {
    automsgs::msgs::nav_msgs::Path path;
    path.mutable_header()->set_frame_id(options_.map_frame());

  std::vector<Waypoint> nodes;
  nodes.push_back({robot_x, robot_y, env.robot_yaw()});
  for (const auto& vp : viewpoints.selected()) {
    nodes.push_back({vp.x, vp.y, vp.yaw});
  }
  if (options_.use_frontier_points()) {
    for (const auto& frontier : env.frontiers()) {
      nodes.push_back({frontier.x(), frontier.y(), 0.0});
    }
  }
  for (const auto& target : global_targets) {
    nodes.push_back({target.x(), target.y(), 0.0});
  }
  if (nodes.size() <= 1) {
    return best_path;
  }

  const int n = static_cast<int>(nodes.size());
  TspSolver::DataModel model;
  model.depot = 0;
  model.distance_matrix.assign(static_cast<size_t>(n),
                               std::vector<int>(static_cast<size_t>(n), 0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i == j) {
        continue;
      }
      model.distance_matrix[static_cast<size_t>(i)]
                           [static_cast<size_t>(j)] =
          DistanceCost(env, nodes[static_cast<size_t>(i)],
                       nodes[static_cast<size_t>(j)]);
    }
  }

  TspSolver solver(std::move(model), options_.tsp_exact_max_nodes());
  TspSolver::SolveOptions tsp_opts;
  tsp_opts.max_exact_nodes = options_.tsp_exact_max_nodes();
  tsp_opts.use_two_opt = options_.tsp_use_two_opt();
  tsp_opts.use_ortools = options_.tsp_use_ortools();
  const std::vector<int> order = solver.Solve(tsp_opts);
  if (order.empty()) {
    continue;
  }

  for (size_t i = 0; i + 1 < order.size(); ++i) {
    const auto& from = nodes[static_cast<size_t>(order[i])];
    const auto& to = nodes[static_cast<size_t>(order[i + 1])];
    std::vector<automsgs::msgs::geometry_msgs::Point> segment;
    PathPlanner::Plan(env, from.x, from.y, to.x, to.y, &segment);
    for (size_t j = 0; j < segment.size(); ++j) {
      if (i > 0 && j == 0) {
        continue;
      }
      const double yaw = std::atan2(to.y - from.y, to.x - from.x);
      *path.add_poses() =
          MakePose(options_.map_frame(), segment[j].x(), segment[j].y(), yaw);
    }
  }
  if (path.poses_size() == 0 && !order.empty()) {
    const auto& last = nodes[static_cast<size_t>(order.back())];
    *path.add_poses() =
        MakePose(options_.map_frame(), last.x, last.y, last.yaw);
  }
    double len = 0.0;
    for (int i = 1; i < path.poses_size(); ++i) {
      const auto& a = path.poses(i - 1).pose().position();
      const auto& b = path.poses(i).pose().position();
      len += std::hypot(b.x() - a.x(), b.y() - a.y());
    }
    if (len < best_len) {
      best_len = len;
      best_path = path;
    }
  }
  return best_path;
}

}  // namespace autonomy::perception::exploration
