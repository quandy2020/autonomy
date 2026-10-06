/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

/**
 * @file sando_planner.cpp
 * @brief SANDO cycle: horizon goal, factor window, committed prefix, yaw filter, and hover latch.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/sando_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "Eigen/Eigenvalues"

#include "autonomy/common/math/math.hpp"

#include "autonomy/control/controller/sando_controller/safe_corridor.hpp"
#include "autonomy/control/controller/sando_controller/hover_monitor.hpp"
#include "autonomy/control/controller/sando_controller/sando_defaults.hpp"
#include "autonomy/control/controller/sando_controller/geometric_path.hpp"
#include "autonomy/control/controller/sando_controller/path_searcher.hpp"
#include "autonomy/control/controller/sando_controller/trajectory_optimizer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {
namespace {

/**
 * @brief Smallest positive root of a t^2 + b t + c = 0.
 *
 * A vanishing leading coefficient reduces the equation to linear. A negative
 * discriminant, or no positive root, returns 0 so the caller can ignore that
 * axis when it takes a maximum over roots.
 *
 * @param a Quadratic coefficient.
 * @param b Linear coefficient.
 * @param c Constant term.
 * @return Smallest t > 0, or 0 when none exists.
 */
double FindPositiveQuadraticRoot(double a, double b, double c) {
  if (std::abs(a) < 1e-12) {
    if (std::abs(b) < 1e-12) {
      return 0.0;
    }
    const double t = -c / b;
    return t > 0.0 ? t : 0.0;
  }
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return 0.0;
  }
  const double root = std::sqrt(disc);
  double best = 0.0;
  const double ts[2] = {(-b + root) / (2.0 * a), (-b - root) / (2.0 * a)};
  for (double t : ts) {
    if (t > 0.0 && (best == 0.0 || t < best)) {
      best = t;
    }
  }
  return best;
}

/**
 * @brief Smallest positive real root of a t^3 + b t^2 + c t + d = 0.
 *
 * A vanishing leading coefficient delegates to FindPositiveQuadraticRoot.
 * Otherwise the roots are the eigenvalues of the companion matrix. Imaginary
 * parts larger than 1e-6 are discarded.
 *
 * @param a Cubic coefficient.
 * @param b Quadratic coefficient.
 * @param c Linear coefficient.
 * @param d Constant term.
 * @return Smallest t > 0, or 0 when none exists.
 */
double FindPositiveCubicRoot(double a, double b, double c, double d) {
  if (std::abs(a) < 1e-9) {
    return FindPositiveQuadraticRoot(b, c, d);
  }
  const double A = b / a;
  const double B = c / a;
  const double C = d / a;
  Eigen::Matrix3d companion;
  companion << 0.0, 0.0, -C, 1.0, 0.0, -B, 0.0, 1.0, -A;
  const Eigen::EigenSolver<Eigen::Matrix3d> solver(companion, false);
  double best = 0.0;
  for (int i = 0; i < 3; ++i) {
    const auto value = solver.eigenvalues()(i);
    if (std::abs(value.imag()) > 1e-6 || value.real() <= 0.0) {
      continue;
    }
    if (best == 0.0 || value.real() < best) {
      best = value.real();
    }
  }
  return best;
}

/**
 * @brief Lower bound on the total duration, divided by the segment count.
 *
 * Each axis contributes the maximum of |xf - x0| / maximum_velocity, the positive
 * quadratic acceleration root, and the positive cubic jerk root. The jerk
 * and acceleration signs point toward the goal. A bound above 10000 seconds
 * is treated as degenerate and returns 0, which lets the factor window fall
 * back to 2 * dc.
 *
 * @param start State at the beginning of the local trajectory.
 * @param goal Horizon goal, world meters.
 * @param options Speed, acceleration, and jerk limits.
 * @param segments Number of polynomial pieces. The bound is divided by this count.
 * @return Per-piece duration lower bound, seconds.
 */
double ComputeBangCoastDuration(const State& start, const Eigen::Vector2d& goal,
                   const proto::SandoControllerOptions& options, int segments) {
  const double x0[2] = {start.pose().position().x(), start.pose().position().y()};
  const double xf[2] = {goal.x(), goal.y()};
  const double v0[2] = {start.velocity().linear().x(), start.velocity().linear().y()};
  const double a0[2] = {start.acceleration().linear().x(), start.acceleration().linear().y()};
  const double maximum_velocity = std::max(options.max_linear_vel(), 1e-3);
  const double maximum_acceleration = std::max(options.max_linear_accel(), 1e-3);
  const double maximum_jerk = std::max(options.j_max(), 1e-3);
  double horizon = 0.0;
  for (int axis = 0; axis < 2; ++axis) {
    horizon = std::max(horizon, std::abs(xf[axis] - x0[axis]) / maximum_velocity);
    const double sign = std::copysign(1.0, xf[axis] - x0[axis]);
    horizon = std::max(horizon, FindPositiveCubicRoot(sign * maximum_jerk / 6.0, a0[axis] / 2.0, v0[axis],
                                                 x0[axis] - xf[axis]));
    horizon = std::max(horizon, FindPositiveQuadraticRoot(0.5 * sign * maximum_acceleration, v0[axis],
                                                     x0[axis] - xf[axis]));
  }
  double initial = horizon / std::max(1, segments);
  if (initial > 10000.0) {
    initial = 0.0;
  }
  return initial;
}

}  // namespace

void SandoPlanner::Configure(const proto::SandoControllerOptions& options) {
  options_ = options;
  SandoDefaults().Apply(&options_);
  last_factor_ = std::min(options_.factor_final(), std::max(options_.factor_initial(), 1.5));
  Reset();
}

void SandoPlanner::Reset() {
  plan_.clear();
  global_path_.clear();
  obstacle_tracker_.ClearObstacles();
  map_ready_ = false;
  goal_ready_ = false;
  state_ready_ = false;
  status_ = options_.skip_initial_yawing() ? Status::kTraveling : Status::kYawing;
  yaw_start_ = std::chrono::steady_clock::now();
  goal_distance_ = 1.0e9;
  goal_yaw_error_ = 0.0;
  replan_count_ = 0;
  computation_times_.clear();
  failure_count_ = 0;
  adapt_commit_length_ = false;
  estimated_computation_time_ = options_.dc();
  committed_sample_count_ = 0;
  last_command_time_ = 0.0;
}

void SandoPlanner::SetTerminalGoal(double x, double y, double yaw) {
  const bool changed = !goal_ready_ || std::hypot(goal_.pose().position().x() - x, goal_.pose().position().y() - y) > 0.05;
  goal_.mutable_pose()->mutable_position()->set_x(x);
  goal_.mutable_pose()->mutable_position()->set_y(y);
  SetYaw(&goal_, yaw);
  terminal_ = goal_;
  goal_ready_ = true;
  if (changed) {
    status_ = options_.skip_initial_yawing() ? Status::kTraveling : Status::kYawing;
    yaw_start_ = std::chrono::steady_clock::now();
    yaw_start_x_ = robot_.pose().position().x();
    yaw_start_y_ = robot_.pose().position().y();
    plan_.clear();
    failure_count_ = 0;
  }
}

void SandoPlanner::IngestCostmap(const map::costmap_2d::Costmap2D& costmap, double now, double robot_x,
                             double robot_y) {
  grid_.LoadFromCostmap(costmap, options_.inflation(), robot_x, robot_y,
             options_.horizon() + options_.map_buffer() + options_.inflation());
  obstacle_tracker_.TrackObstacles(grid_, options_, now, robot_x, robot_y);
  const double stamp_r = std::max(0.15, options_.robot_radius());
  for (const auto& obs : obstacle_tracker_.GetObstacles()) {
    const double age = std::max(0.0, now - obs.last_seen());
    const double px = obs.motion().pose().position().x() + obs.motion().velocity().linear().x() * age;
    const double py = obs.motion().pose().position().y() + obs.motion().velocity().linear().y() * age;
    if (options_.dynamic_as_occupied_current() && std::hypot(obs.motion().velocity().linear().x(), obs.motion().velocity().linear().y()) >= options_.velocity_threshold()) {
      grid_.MarkDisk(px, py, stamp_r, Cell::kOccupied);
    }
    if (options_.dynamic_as_occupied_future()) {
      const int samples = 4;
      for (int s = 1; s <= samples; ++s) {
        const double t = options_.prediction_horizon() * s / samples;
        grid_.MarkDisk(px + obs.motion().velocity().linear().x() * t, py + obs.motion().velocity().linear().y() * t,
                       stamp_r, Cell::kOccupied);
      }
    }
  }
  grid_.FreeDisk(robot_x, robot_y, std::max(options_.robot_radius() * 2.0, options_.inflation()));
  if (options_.dynamic_heat() || options_.static_heat_alpha() > 0.0) {
    grid_.BuildHeat(options_.static_heat_alpha(), options_.static_heat_rmax(), 50.0, obstacle_tracker_.GetObstacles(), now,
                    options_.prediction_horizon(), 1.0, 2.0, 0.0, options_.obst_max_vel(), robot_x, robot_y,
                    options_.horizon() + options_.map_buffer());
  }
  map_ready_ = !grid_.IsEmpty();
}

void SandoPlanner::AddDynamicObstacle(double x, double y, double velocity_x, double velocity_y, double radius) {
  obstacle_tracker_.AddObstacle(x, y, velocity_x, velocity_y, radius, options_.robot_radius());
}

void SandoPlanner::ProjectHorizonGoal(const State& from, double* goal_x, double* goal_y) const {
  const double dx = goal_.pose().position().x() - from.pose().position().x();
  const double dy = goal_.pose().position().y() - from.pose().position().y();
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-3 || dist <= options_.horizon()) {
    *goal_x = goal_.pose().position().x();
    *goal_y = goal_.pose().position().y();
    return;
  }
  *goal_x = from.pose().position().x() + options_.horizon() * dx / dist;
  *goal_y = from.pose().position().y() + options_.horizon() * dy / dist;
}

bool SandoPlanner::NeedsNewPlan(const State& robot) const {
  const double speed = std::hypot(robot.velocity().linear().x(), robot.velocity().linear().y());
  if (options_.hover_avoidance() &&
      (status_ == Status::kGoalReached || status_ == Status::kHoverAvoiding)) {
    return true;
  }
  if (goal_distance_ < options_.goal_dist_tol() && speed < 0.1 && status_ != Status::kYawing) {
    return false;
  }
  if (status_ == Status::kYawing || status_ == Status::kGoalReached) {
    return false;
  }
  if (status_ == Status::kGoalSeen && !plan_.empty()) {
    const State& tail = plan_.back();
    if (std::hypot(tail.pose().position().x() - terminal_.pose().position().x(), tail.pose().position().y() - terminal_.pose().position().y()) < options_.goal_dist_tol()) {
      return false;
    }
  }
  (void)robot;
  return true;
}

bool SandoPlanner::ReplanTrajectory(const State& robot, double now) {
  const auto t_begin = std::chrono::steady_clock::now();
  const auto timed_out = [&]() {
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();
    return ms > static_cast<double>(options_.hgp_timeout_ms());
  };

  State start = robot;
  int keep = 0;
  const bool drifted =
      plan_.empty() || std::hypot(plan_.front().pose().position().x() - robot.pose().position().x(), plan_.front().pose().position().y() - robot.pose().position().y()) > 0.45;
  if (!drifted && static_cast<int>(plan_.size()) > 1) {
    const int plan_size = static_cast<int>(plan_.size());
    int commit = options_.default_k();
    if (adapt_commit_length_) {
      commit = std::max(2, static_cast<int>(options_.k_value_factor() * estimated_computation_time_ / options_.dc()));
    }
    keep = std::max(1, std::min(plan_size, commit));
    start = plan_[static_cast<size_t>(keep - 1)];
  }
  grid_.FreeDisk(start.pose().position().x(), start.pose().position().y(), std::max(options_.robot_radius(), grid_.GetResolution()));

  double goal_x = 0.0;
  double goal_y = 0.0;
  ProjectHorizonGoal(start, &goal_x, &goal_y);
  int goal_column = 0;
  int goal_row = 0;
  if (!grid_.ConvertWorldToCell(goal_x, goal_y, &goal_column, &goal_row)) {
    return false;
  }
  if (grid_.GetCell(goal_column, goal_row) == Cell::kOccupied) {
    const double dx = goal_x - start.pose().position().x();
    const double dy = goal_y - start.pose().position().y();
    const double dist = std::max(1e-3, std::hypot(dx, dy));
    bool snapped = false;
    for (double s = dist; s > 0.3; s -= grid_.GetResolution()) {
      const double x = start.pose().position().x() + dx / dist * s;
      const double y = start.pose().position().y() + dy / dist * s;
      int column_index = 0;
      int row_index = 0;
      if (grid_.ConvertWorldToCell(x, y, &column_index, &row_index) && grid_.GetCell(column_index, row_index) != Cell::kOccupied) {
        goal_x = x;
        goal_y = y;
        snapped = true;
        break;
      }
    }
    if (!snapped) {
      return false;
    }
  }

  SearchWeights weights;
  weights.heuristic_weight = options_.heuristic_weight();
  weights.heat_weight = options_.heat_weight();
  weights.unknown_cell_cost = options_.w_unknown();
  weights.soft_occupied_cost = options_.obstacle_soft_cost();
  weights.allow_soft_occupied = options_.use_soft_cost();
  weights.max_expansions = options_.max_expansions();
  weights.timeout_milliseconds = options_.hgp_timeout_ms();
  weights.alignment_weight = options_.w_align();
  weights.alignment_decay_cell_count = options_.decay_len_cells();
  weights.crossing_weight = options_.w_side();
  if (global_path_.size() >= 2) {
    const Eigen::Vector2d seg = global_path_[1] - global_path_[0];
    const double seg_n = seg.norm();
    if (seg_n > 1e-8) {
      weights.reference_direction_x = seg.x() / seg_n;
      weights.reference_direction_y = seg.y() / seg_n;
    }
  } else {
    const double gd = std::hypot(goal_x - start.pose().position().x(), goal_y - start.pose().position().y());
    if (gd > 1e-3) {
      weights.reference_direction_x = (goal_x - start.pose().position().x()) / gd;
      weights.reference_direction_y = (goal_y - start.pose().position().y()) / gd;
    }
  }
  const double buf = options_.map_buffer();
  weights.window_min_x = std::min(start.pose().position().x(), goal_x) - buf;
  weights.window_min_y = std::min(start.pose().position().y(), goal_y) - buf;
  weights.window_max_x = std::max(start.pose().position().x(), goal_x) + buf;
  weights.window_max_y = std::max(start.pose().position().y(), goal_y) + buf;

  auto crosses_heat = [&](const std::vector<Eigen::Vector2d>& pts) {
    for (const auto& q : pts) {
      int column_index = 0;
      int row_index = 0;
      if (grid_.ConvertWorldToCell(q.x(), q.y(), &column_index, &row_index) && grid_.GetCellHeat(column_index, row_index) > 0.5f) {
        return true;
      }
    }
    return false;
  };

  std::vector<Eigen::Vector2d> raw;
  bool found = false;
  const std::string planner = options_.global_planner();
  if (planner == "sjps") {
    SearchWeights plain = weights;
    plain.heat = 0.0;
    found = path_searcher_.Jump(grid_, start.pose().position().x(), start.pose().position().y(), goal_x, goal_y, plain, &raw) && !crosses_heat(raw);
  } else if (planner == "sastar") {
    SearchWeights plain = weights;
    plain.heat = 0.0;
    found = path_searcher_.Search(grid_, start.pose().position().x(), start.pose().position().y(), goal_x, goal_y, plain, &raw);
  }
  if (!found) {
    found = path_searcher_.Search(grid_, start.pose().position().x(), start.pose().position().y(), goal_x, goal_y, weights, &raw);
  }
  if (!found) {
    SearchWeights wide = weights;
    wide.window_min_x = -1.0e9;
    wide.window_min_y = -1.0e9;
    wide.window_max_x = 1.0e9;
    wide.window_max_y = 1.0e9;
    found = path_searcher_.Search(grid_, start.pose().position().x(), start.pose().position().y(), goal_x, goal_y, wide, &raw);
  }
  if (!found) {
    if (std::hypot(start.pose().position().x() - goal_x, start.pose().position().y() - goal_y) > std::max(0.3, grid_.GetResolution() * 3.0)) {
      ++failure_count_;
      return false;
    }
    raw = {Eigen::Vector2d(start.pose().position().x(), start.pose().position().y()), Eigen::Vector2d(goal_x, goal_y)};
    if ((raw[1] - raw[0]).norm() < 1e-3) {
      raw[1].x() += 0.05;
    }
  }
  path_searcher_.Shortcut(grid_, &raw, options_.los_cells(), options_.min_len(), options_.min_turn_deg());
  const double heat_w =
      (options_.dynamic_heat() || options_.static_heat_alpha() > 0.0) ? options_.heat_weight() : 0.0;
  path_searcher_.Prune(grid_, &raw, heat_w);
  auto spatial = geometric_path_.Resample(raw, options_.max_dist_vertexes(), options_.num_polytopes() + 1);
  geometric_path_.Truncate(grid_, options_, &spatial);
  if (spatial.size() < 2) {
    ++failure_count_;
    return false;
  }
  while (spatial.size() == 2) {
    spatial.insert(spatial.begin() + 1, 0.5 * (spatial.front() + spatial.back()));
  }
  auto path = geometric_path_.Resample(spatial, options_.max_dist_vertexes(), options_.num_segments() + 1);
  while (path.size() < 3) {
    path.insert(path.begin() + 1, 0.5 * (path.front() + path.back()));
  }
  path.front() = Eigen::Vector2d(start.pose().position().x(), start.pose().position().y());

  const bool stop = status_ == Status::kGoalSeen || status_ == Status::kHoverAvoiding ||
                    std::hypot(terminal_.pose().position().x() - goal_x, terminal_.pose().position().y() - goal_y) < options_.goal_seen_radius();

  TrajRequest request;
  request.path = path;
  request.initial = start;
  request.maximum_velocity = options_.max_linear_vel();
  request.maximum_acceleration = options_.max_linear_accel();
  request.maximum_jerk = options_.j_max();
  request.jerk_weight = options_.jerk_weight();
  request.dynamic_constraint = options_.dynamic_constraint();
  request.stop_at_end = stop;

  std::vector<Piece> pieces;
  std::vector<Piece> fallback;
  bool have_fallback = false;
  bool solved = false;
  const auto try_factor = [&](double factor) {
    if (timed_out()) {
      return false;
    }
    request.factor = factor;
    const int segments = static_cast<int>(path.size()) - 1;
    const double initial_dt = ComputeBangCoastDuration(start, path.back(), options_, segments);
    const double piece_dt = factor * std::max(initial_dt, 2.0 * options_.dc());
    std::vector<std::vector<std::vector<HalfPlane>>> layers;
    if (!safe_corridor_.Build(grid_, options_, obstacle_tracker_.GetObstacles(), spatial, segments, piece_dt, &layers)) {
      return false;
    }
    request.time_layered = std::move(layers);
    request.uniform_piece_duration = piece_dt;
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();
    request.time_limit_seconds =
        std::max(0.02, static_cast<double>(options_.hgp_timeout_ms()) / 1000.0 - elapsed);
    std::vector<Piece> trial;
    if (!trajectory_optimizer_.Optimize(request, &trial)) {
      return false;
    }
    if (!have_fallback) {
      fallback = trial;
      have_fallback = true;
    }
    if (trajectory_optimizer_.IsWithinLimits(trial, options_.max_linear_vel(), options_.max_linear_accel(), options_.j_max(),
                               options_.dynamic_constraint())) {
      pieces = std::move(trial);
      return true;
    }
    return false;
  };
  // Window around the last successful factor. Smaller factors are tried first.
  constexpr double kFactorRadius = 0.4;
  const double step = std::max(options_.factor_step(), 1e-3);
  std::vector<double> factors;
  for (double factor = last_factor_ - kFactorRadius; factor <= last_factor_ + kFactorRadius + 1e-9;
       factor += step) {
    if (factor + 1e-9 < options_.factor_initial() || factor > options_.factor_final() + 1e-9) {
      continue;
    }
    factors.push_back(factor);
  }
  if (factors.empty()) {
    factors.push_back(std::min(options_.factor_final(), std::max(options_.factor_initial(), last_factor_)));
  }
  for (double factor : factors) {
    if (try_factor(factor)) {
      last_factor_ = factor;
      solved = true;
      break;
    }
    if (timed_out()) {
      break;
    }
  }
  if (!solved) {
    const double shifted = last_factor_ + step;
    if (shifted > options_.factor_final()) {
      last_factor_ = std::min(options_.factor_final(), std::max(options_.factor_initial(), 1.5));
    } else {
      last_factor_ = shifted;
    }
  }
  if (!solved && have_fallback) {
    pieces = std::move(fallback);
    solved = true;
  }
  if (!solved) {
    ++failure_count_;
    return false;
  }
  global_path_ = spatial;

  std::vector<State> samples;
  trajectory_optimizer_.Sample(pieces, options_.dc(), now, &samples);
  if (samples.empty()) {
    ++failure_count_;
    return false;
  }

  committed_sample_count_ = keep;
  if (keep > 0 && keep <= static_cast<int>(plan_.size())) {
    plan_.erase(plan_.begin() + keep, plan_.end());
  } else {
    plan_.clear();
  }
  if (!samples.empty()) {
    samples.erase(samples.begin());
  }
  plan_.insert(plan_.end(), samples.begin(), samples.end());
  failure_count_ = 0;

  const double comp = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();
  ++replan_count_;
  if (!adapt_commit_length_) {
    if (replan_count_ > 1) {
      computation_times_.push_back(comp);
    }
    if (computation_times_.size() >= 10) {
      double sum = 0.0;
      for (double sample : computation_times_) {
        sum += sample;
      }
      estimated_computation_time_ = sum / static_cast<double>(computation_times_.size());
      adapt_commit_length_ = true;
    }
  } else {
    estimated_computation_time_ = 0.9 * comp + 0.1 * estimated_computation_time_;
  }
  return true;
}

void SandoPlanner::ComputeDesiredYaw(const State& robot, State* command) {
  if (failure_count_ > options_.yaw_spinning_threshold() && status_ != Status::kHoverAvoiding) {
    command->mutable_velocity()->mutable_angular()->set_z(options_.yaw_spinning_dyaw());
    SetYaw(command, previous_yaw_ + command->velocity().angular().z() * options_.dc());
    previous_yaw_ = Yaw(*command);
    return;
  }
  auto step_toward = [&](double desired, double rate, bool filtered) {
    const double diff = common::NormalizeAngleDifference(desired - previous_yaw_);
    const double max_step = rate * options_.dc();
    double step = filtered ? (1.0 - options_.alpha_filter_dyaw()) * diff : diff;
    step = std::max(-max_step, std::min(max_step, step));
    SetYaw(command, previous_yaw_ + step);
    command->mutable_velocity()->mutable_angular()->set_z(options_.dc() > 1e-4 ? step / options_.dc() : 0.0);
    previous_yaw_ = Yaw(*command);
  };
  if (status_ == Status::kGoalReached) {
    SetYaw(command, previous_yaw_);
    command->mutable_velocity()->mutable_angular()->set_z(0.0);
    return;
  }
  if (status_ == Status::kYawing) {
    const double desired = std::atan2(goal_.pose().position().y() - yaw_start_y_, goal_.pose().position().x() - yaw_start_x_);
    const double err = common::NormalizeAngleDifference(desired - Yaw(robot));
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - yaw_start_).count();
    if (std::abs(err) < 0.3 || (elapsed > 10.0 && std::abs(err) < 1.0)) {
      status_ = Status::kTraveling;
    }
    step_toward(desired, options_.w_max_yawing(), false);
    return;
  }
  if (status_ == Status::kHoverAvoiding) {
    const double dx = hover_x_ - command->pose().position().x();
    const double dy = hover_y_ - command->pose().position().y();
    if (std::hypot(dx, dy) < 0.3) {
      SetYaw(command, previous_yaw_);
      command->mutable_velocity()->mutable_angular()->set_z(0.0);
      return;
    }
    step_toward(std::atan2(dy, dx), options_.w_max_yawing(), false);
    return;
  }
  if (std::hypot(command->velocity().linear().x(), command->velocity().linear().y()) < 0.01) {
    SetYaw(command, previous_yaw_);
    command->mutable_velocity()->mutable_angular()->set_z(0.0);
    return;
  }
  step_toward(std::atan2(command->velocity().linear().y(), command->velocity().linear().x()), options_.max_angular_vel(), true);
}

bool SandoPlanner::ComputeCommand(const State& robot, double now, State* command, std::string* message) {
  robot_ = robot;
  if (!state_ready_) {
    previous_yaw_ = Yaw(robot);
    yaw_start_x_ = robot.pose().position().x();
    yaw_start_y_ = robot.pose().position().y();
    state_ready_ = true;
    plan_.push_back(robot);
  }
  goal_distance_ = std::hypot(goal_.pose().position().x() - robot.pose().position().x(), goal_.pose().position().y() - robot.pose().position().y());
  goal_yaw_error_ = common::NormalizeAngleDifference(Yaw(goal_) - Yaw(robot));
  const double speed = std::hypot(robot.velocity().linear().x(), robot.velocity().linear().y());

  if (!goal_ready_) {
    *message = "SANDO: no terminal goal";
    return false;
  }
  if (goal_distance_ < options_.goal_dist_tol() && speed < 0.1 && status_ != Status::kYawing) {
    if (options_.hover_avoidance()) {
      if (status_ != Status::kHoverAvoiding) {
        hover_x_ = terminal_.pose().position().x();
        hover_y_ = terminal_.pose().position().y();
        status_ = Status::kHoverAvoiding;
      }
    } else {
      status_ = Status::kGoalReached;
    }
  } else if (goal_distance_ < options_.goal_seen_radius() && status_ == Status::kTraveling) {
    status_ = Status::kGoalSeen;
  }

  if (options_.hover_avoidance() && status_ == Status::kHoverAvoiding && map_ready_) {
    hover_monitor_.Evade(grid_, options_, obstacle_tracker_.GetObstacles(), robot, hover_x_, hover_y_, &goal_);
  }

  if (status_ != Status::kYawing && map_ready_ && NeedsNewPlan(robot)) {
    if (!ReplanTrajectory(robot, now)) {
      *message = "SANDO: replan failed";
    } else {
      message->clear();
      last_replan_time_ = now;
    }
  }

  if (plan_.empty()) {
    *command = robot;
    command->mutable_velocity()->mutable_linear()->set_x(0.0);
    command->mutable_velocity()->mutable_linear()->set_y(0.0);
    command->mutable_velocity()->mutable_angular()->set_z(0.0);
    ComputeDesiredYaw(robot, command);
    if (message->empty()) {
      *message = "SANDO: empty plan";
    }
    return status_ == Status::kYawing;
  }

  *command = plan_.front();
  const int plan_n = static_cast<int>(plan_.size());
  if (status_ != Status::kYawing) {
    const double sample_period = std::max(options_.dc(), 1e-3);
    int steps = 1;
    if (last_command_time_ <= 0.0) {
      last_command_time_ = now;
    } else {
      steps = static_cast<int>(std::floor((now - last_command_time_) / sample_period));
      if (steps > 0) {
        last_command_time_ += static_cast<double>(steps) * sample_period;
      }
    }
    while (steps > 0 && plan_.size() > 1) {
      plan_.pop_front();
      --steps;
    }
  }
  if (failure_count_ > options_.yaw_spinning_threshold() && status_ != Status::kHoverAvoiding) {
    ComputeDesiredYaw(robot, command);
  } else if (plan_n < 5 && status_ != Status::kYawing && status_ != Status::kHoverAvoiding) {
    SetYaw(command, previous_yaw_);
    command->mutable_velocity()->mutable_linear()->set_x(0.0);
    command->mutable_velocity()->mutable_linear()->set_y(0.0);
    command->mutable_velocity()->mutable_angular()->set_z(0.0);
  } else {
    ComputeDesiredYaw(robot, command);
  }
  if (status_ == Status::kYawing || status_ == Status::kGoalReached) {
    command->mutable_velocity()->mutable_linear()->set_x(0.0);
    command->mutable_velocity()->mutable_linear()->set_y(0.0);
    if (status_ == Status::kGoalReached) {
      command->mutable_velocity()->mutable_angular()->set_z(0.0);
    }
  }
  if (options_.stop_when_occupied() && map_ready_) {
    int column_index = 0;
    int row_index = 0;
    if (grid_.ConvertWorldToCell(robot.pose().position().x(), robot.pose().position().y(), &column_index, &row_index) && grid_.GetCell(column_index, row_index) == Cell::kOccupied) {
      command->mutable_velocity()->mutable_linear()->set_x(0.0);
      command->mutable_velocity()->mutable_linear()->set_y(0.0);
      command->mutable_velocity()->mutable_angular()->set_z(0.0);
      *message = "SANDO: occupied, stop-avoid";
    }
  }
  return true;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
