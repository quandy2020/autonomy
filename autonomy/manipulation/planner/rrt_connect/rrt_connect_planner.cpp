/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/rrt_connect/rrt_connect_planner.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

using Vec = std::vector<double>;

double Dist(const Vec& a, const Vec& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = a[i] - b[i];
    s += d * d;
  }
  return std::sqrt(s);
}

Vec Steer(const Vec& from, const Vec& to, double step) {
  const double d = Dist(from, to);
  if (d < step || d < 1e-9) {
    return to;
  }
  Vec out = from;
  for (std::size_t i = 0; i < from.size(); ++i) {
    out[i] = from[i] + step * (to[i] - from[i]) / d;
  }
  return out;
}

bool Valid(const MotionPlanRequest& request, const Vec& q,
           const std::vector<std::string>& names) {
  core::JointState state;
  SetJointState(&state, names, q);
  if (request.scene) {
    return !request.scene->CheckCollision(state);
  }
  return true;
}

struct Node {
  Vec q;
  int parent = -1;
};

int Nearest(const std::vector<Node>& tree, const Vec& q) {
  int best = 0;
  double best_d = Dist(tree[0].q, q);
  for (int i = 1; i < static_cast<int>(tree.size()); ++i) {
    const double d = Dist(tree[i].q, q);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

}  // namespace

bool RrtConnectPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse RrtConnectPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (request.start_state.position_size() == 0 ||
      request.goal_state.position_size() == 0 ||
      request.start_state.position_size() !=
          request.goal_state.position_size()) {
    response.error = "invalid start/goal";
    response.error_code = ErrorCode::kInvalidRobotState;
    return response;
  }

  std::vector<std::string> names;
  if (request.goal_state.name_size() > 0) {
    names.assign(request.goal_state.name().begin(),
                 request.goal_state.name().end());
  } else {
    names.assign(request.start_state.name().begin(),
                 request.start_state.name().end());
  }
  const Vec start(request.start_state.position().begin(),
                  request.start_state.position().end());
  const Vec goal(request.goal_state.position().begin(),
                 request.goal_state.position().end());
  if (!Valid(request, start, names) || !Valid(request, goal, names)) {
    response.error = "start or goal in collision";
    response.error_code = ErrorCode::kStartStateInCollision;
    return response;
  }

  // Direct connect first (fast path).
  {
    const double d = Dist(start, goal);
    const int n = std::max(2, static_cast<int>(d / 0.1) + 1);
    core::RobotTrajectory traj;
    bool ok = true;
    for (int i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n - 1);
      Vec q(start.size());
      for (std::size_t j = 0; j < start.size(); ++j) {
        q[j] = start[j] + t * (goal[j] - start[j]);
      }
      if (!Valid(request, q, names)) {
        ok = false;
        break;
      }
      core::JointState js;
      SetJointState(&js, names, q);
      AddTrajectoryPoint(&traj, js, 0.05 * i);
    }
    if (ok) {
      response.trajectory = std::move(traj);
      response.success = true;
      response.error_code = ErrorCode::kSuccess;
      return response;
    }
  }

  std::vector<Node> tree_a{{start, -1}};
  std::vector<Node> tree_b{{goal, -1}};
  std::mt19937 rng{42};
  std::vector<std::uniform_real_distribution<double>> dists;
  for (std::size_t i = 0; i < start.size(); ++i) {
    double lo = -3.14;
    double hi = 3.14;
    if (request.model) {
      if (i < names.size()) {
        if (const auto* lim = request.model->GetJointLimits(names[i])) {
          lo = lim->min_position;
          hi = lim->max_position;
        }
      }
    }
    dists.emplace_back(lo, hi);
  }

  const double step = 0.15;
  int connect_a = -1;
  int connect_b = -1;
  for (int iter = 0; iter < max_iters_; ++iter) {
    Vec q_rand(start.size());
    for (std::size_t i = 0; i < start.size(); ++i) {
      q_rand[i] = dists[i](rng);
    }
    auto extend = [&](std::vector<Node>& tree, const Vec& target) -> int {
      const int nearest = Nearest(tree, target);
      const Vec q_new = Steer(tree[nearest].q, target, step);
      if (!Valid(request, q_new, names)) {
        return -1;
      }
      tree.push_back(Node{q_new, nearest});
      return static_cast<int>(tree.size()) - 1;
    };

    const int na = extend(tree_a, q_rand);
    if (na < 0) {
      continue;
    }
    const int nb = extend(tree_b, tree_a[na].q);
    if (nb >= 0 && Dist(tree_a[na].q, tree_b[nb].q) < step) {
      connect_a = na;
      connect_b = nb;
      break;
    }
    std::swap(tree_a, tree_b);
  }

  if (connect_a < 0) {
    response.error = "RRT-Connect failed";
    response.error_code = ErrorCode::kPlanningFailed;
    return response;
  }

  auto walk = [](const std::vector<Node>& tree, int idx) {
    std::vector<Vec> path;
    for (int i = idx; i >= 0; i = tree[i].parent) {
      path.push_back(tree[i].q);
    }
    std::reverse(path.begin(), path.end());
    return path;
  };

  auto* start_tree = &tree_a;
  auto* goal_tree = &tree_b;
  if (Dist(tree_a[0].q, start) > 1e-6) {
    start_tree = &tree_b;
    goal_tree = &tree_a;
    std::swap(connect_a, connect_b);
  }

  auto path_a = walk(*start_tree, connect_a);
  auto path_b = walk(*goal_tree, connect_b);
  std::reverse(path_b.begin(), path_b.end());

  core::RobotTrajectory traj;
  int k = 0;
  auto append = [&](const Vec& q) {
    core::JointState js;
    SetJointState(&js, names, q);
    AddTrajectoryPoint(&traj, js, 0.05 * k++);
  };
  for (const auto& q : path_a) {
    append(q);
  }
  for (std::size_t i = 1; i < path_b.size(); ++i) {
    append(path_b[i]);
  }

  response.trajectory = std::move(traj);
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(RrtConnectPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
