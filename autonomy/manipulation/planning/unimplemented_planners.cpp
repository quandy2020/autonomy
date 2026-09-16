/*
 * Copyright 2026 The Openbot Authors
 *
 * CHOMP / STOMP / Hybrid — MoveIt-aligned industrial-lite optimizers.
 */

#include "autonomy/manipulation/planning/planner_base.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/plugins.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

constexpr int kDiffRuleLength = 7;
// Acceleration-focused finite-difference stencil (MoveIt DIFF_RULES[1]).
constexpr double kAccRule[kDiffRuleLength] = {
    0.0, 1.0 / 12.0, -1.5 / 12.0, 2.0 / 12.0, -1.5 / 12.0, 1.0 / 12.0, 0.0};

struct ChompParams {
  int max_iterations = 50;
  int max_iterations_after_collision_free = 5;
  int num_timesteps = 30;
  double learning_rate = 0.1;
  double smoothness_cost_weight = 0.1;
  double obstacle_cost_weight = 1.0;
  double ridge_factor = 1e-4;
  double joint_update_limit = 0.1;
};

struct StompParams {
  int num_iterations = 10;
  int num_iterations_after_valid = 2;
  int num_timesteps = 20;
  int num_rollouts = 10;
  double noise_stddev = 0.05;
  double collision_penalty = 10.0;
  double control_cost_weight = 0.1;
  double exponentiated_cost_sensitivity = 10.0;
};

core::RobotTrajectory SeedInterpolate(const MotionPlanRequest& request,
                                      int waypoints) {
  core::RobotTrajectory traj;
  const std::size_t dof = request.start_state.positions.size();
  const auto names = request.goal_state.names.empty()
                         ? request.start_state.names
                         : request.goal_state.names;
  const int n = std::max(2, waypoints);
  traj.waypoints.resize(static_cast<std::size_t>(n));
  traj.time_from_start.resize(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    traj.waypoints[static_cast<std::size_t>(i)].names = names;
    traj.waypoints[static_cast<std::size_t>(i)].positions.resize(dof);
    for (std::size_t j = 0; j < dof; ++j) {
      const double a = request.start_state.positions[j];
      const double b = request.goal_state.positions[j];
      traj.waypoints[static_cast<std::size_t>(i)].positions[j] = a + t * (b - a);
    }
    traj.time_from_start[static_cast<std::size_t>(i)] = t;
  }
  return traj;
}

void ClampTrajectory(const MotionPlanRequest& request,
                     core::RobotTrajectory* traj) {
  if (!traj || !request.model) {
    return;
  }
  for (auto& wp : traj->waypoints) {
    for (std::size_t i = 0; i < wp.positions.size() && i < wp.names.size();
         ++i) {
      const auto* lim = request.model->GetJointLimits(wp.names[i]);
      if (!lim || !lim->has_position_limits) {
        continue;
      }
      wp.positions[i] =
          std::clamp(wp.positions[i], lim->min_position, lim->max_position);
    }
  }
}

bool StateCollides(const MotionPlanRequest& request,
                   const core::JointState& state) {
  if (!request.scene) {
    return false;
  }
  return request.scene->CheckCollision(state) ||
         !request.scene->IsStateValid(state);
}

double ObstacleCostWaypoint(const MotionPlanRequest& request,
                            const core::JointState& state) {
  // Soft potential without SDF (MoveIt getPotential analogue on binary probes).
  constexpr double kEps = 0.05;
  constexpr double kStep = 0.02;
  if (!request.scene) {
    return 0.0;
  }
  if (StateCollides(request, state)) {
    return 1.0 + 0.5 * kEps;  // d < 0 → -d + ½ε with unit penetration proxy
  }
  // Soft band: if a neighbor along ±e_j collides, treat as 0 ≤ d < ε.
  int hits = 0;
  int probes = 0;
  for (std::size_t j = 0; j < state.positions.size(); ++j) {
    for (const double s : {-kStep, kStep}) {
      core::JointState probe = state;
      probe.positions[j] += s;
      ++probes;
      if (StateCollides(request, probe)) {
        ++hits;
      }
    }
  }
  if (hits == 0 || probes == 0) {
    return 0.0;
  }
  const double d = kEps * (1.0 - static_cast<double>(hits) / probes);
  return 0.5 * (d - kEps) * (d - kEps) / kEps;
}

double SmoothnessCost(const core::RobotTrajectory& traj) {
  if (traj.waypoints.size() < 3) {
    return 0.0;
  }
  double cost = 0.0;
  const std::size_t dof = traj.waypoints.front().positions.size();
  for (std::size_t i = 1; i + 1 < traj.waypoints.size(); ++i) {
    for (std::size_t j = 0; j < dof; ++j) {
      const double acc = traj.waypoints[i - 1].positions[j] -
                         2.0 * traj.waypoints[i].positions[j] +
                         traj.waypoints[i + 1].positions[j];
      cost += acc * acc;
    }
  }
  return cost;
}

double CollisionCost(const MotionPlanRequest& request,
                     const core::RobotTrajectory& traj) {
  double cost = 0.0;
  for (const auto& wp : traj.waypoints) {
    cost += ObstacleCostWaypoint(request, wp);
  }
  // Midpoint interpolation checks (STOMP-style densification).
  for (std::size_t i = 0; i + 1 < traj.waypoints.size(); ++i) {
    core::JointState mid = traj.waypoints[i];
    for (std::size_t j = 0; j < mid.positions.size(); ++j) {
      mid.positions[j] =
          0.5 * (traj.waypoints[i].positions[j] +
                 traj.waypoints[i + 1].positions[j]);
    }
    cost += ObstacleCostWaypoint(request, mid);
  }
  return cost;
}

/** Dense (AᵀA + ridge I)^{-1} for free waypoints; falls back to diagonal. */
bool InvertDense(std::vector<std::vector<double>>* mat,
                 std::vector<std::vector<double>>* inv) {
  const int n = static_cast<int>(mat->size());
  if (n <= 0) {
    return false;
  }
  inv->assign(static_cast<std::size_t>(n),
              std::vector<double>(static_cast<std::size_t>(n), 0.0));
  for (int i = 0; i < n; ++i) {
    (*inv)[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = 1.0;
  }
  for (int col = 0; col < n; ++col) {
    int pivot = col;
    double best = std::abs((*mat)[static_cast<std::size_t>(col)]
                                 [static_cast<std::size_t>(col)]);
    for (int r = col + 1; r < n; ++r) {
      const double v = std::abs((*mat)[static_cast<std::size_t>(r)]
                                     [static_cast<std::size_t>(col)]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-12) {
      return false;
    }
    if (pivot != col) {
      std::swap((*mat)[static_cast<std::size_t>(col)],
                (*mat)[static_cast<std::size_t>(pivot)]);
      std::swap((*inv)[static_cast<std::size_t>(col)],
                (*inv)[static_cast<std::size_t>(pivot)]);
    }
    const double diag =
        (*mat)[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)];
    for (int c = 0; c < n; ++c) {
      (*mat)[static_cast<std::size_t>(col)][static_cast<std::size_t>(c)] /=
          diag;
      (*inv)[static_cast<std::size_t>(col)][static_cast<std::size_t>(c)] /=
          diag;
    }
    for (int r = 0; r < n; ++r) {
      if (r == col) {
        continue;
      }
      const double f =
          (*mat)[static_cast<std::size_t>(r)][static_cast<std::size_t>(col)];
      for (int c = 0; c < n; ++c) {
        (*mat)[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] -=
            f * (*mat)[static_cast<std::size_t>(col)]
                      [static_cast<std::size_t>(c)];
        (*inv)[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] -=
            f * (*inv)[static_cast<std::size_t>(col)]
                      [static_cast<std::size_t>(c)];
      }
    }
  }
  return true;
}

/** Build metric inverse of acceleration AᵀA + ridge for free interior points. */
std::vector<std::vector<double>> MetricInverse(int free_points, double ridge) {
  const int n = std::max(0, free_points);
  std::vector<std::vector<double>> ata(
      static_cast<std::size_t>(n),
      std::vector<double>(static_cast<std::size_t>(n), 0.0));
  // Discrete accel: q[i-1] - 2 q[i] + q[i+1] on free indices mapped 0..n-1
  // corresponding to trajectory indices 1..T-2.
  for (int i = 0; i < n; ++i) {
    // Contribution from accel at free point i (traj i+1): coeffs on
    // neighbors via second difference energy.
    ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += 6.0;
    if (i + 1 < n) {
      ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i + 1)] -= 4.0;
      ata[static_cast<std::size_t>(i + 1)][static_cast<std::size_t>(i)] -= 4.0;
    }
    if (i + 2 < n) {
      ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i + 2)] += 1.0;
      ata[static_cast<std::size_t>(i + 2)][static_cast<std::size_t>(i)] += 1.0;
    }
    ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += ridge;
  }
  std::vector<std::vector<double>> inv;
  auto work = ata;
  if (!InvertDense(&work, &inv)) {
    inv.assign(static_cast<std::size_t>(n),
               std::vector<double>(static_cast<std::size_t>(n), 0.0));
    const double d = 6.0 + ridge;
    for (int i = 0; i < n; ++i) {
      inv[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = 1.0 / d;
    }
  }
  return inv;
}

core::RobotTrajectory ChompOptimize(const MotionPlanRequest& request,
                                    core::RobotTrajectory traj,
                                    const ChompParams& params) {
  if (traj.waypoints.size() < 3) {
    return traj;
  }
  ClampTrajectory(request, &traj);
  const std::size_t T = traj.waypoints.size();
  const std::size_t dof = traj.waypoints.front().positions.size();
  const int free = static_cast<int>(T) - 2;
  if (free <= 0) {
    return traj;
  }
  const auto minv = MetricInverse(free, params.ridge_factor);
  int valid_streak = 0;

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    // Gradients on free waypoints [1..T-2], per dof.
    std::vector<std::vector<double>> grad(
        static_cast<std::size_t>(free), std::vector<double>(dof, 0.0));

    for (int fi = 0; fi < free; ++fi) {
      const std::size_t i = static_cast<std::size_t>(fi + 1);
      for (std::size_t j = 0; j < dof; ++j) {
        double g_s = 0.0;
        for (int k = 0; k < kDiffRuleLength; ++k) {
          const int idx =
              static_cast<int>(i) + k - kDiffRuleLength / 2;
          if (idx < 0 || idx >= static_cast<int>(T)) {
            continue;
          }
          g_s += kAccRule[k] *
                 traj.waypoints[static_cast<std::size_t>(idx)].positions[j];
        }
        constexpr double kStep = 1e-3;
        core::JointState plus = traj.waypoints[i];
        core::JointState minus = traj.waypoints[i];
        plus.positions[j] += kStep;
        minus.positions[j] -= kStep;
        const double g_o =
            (ObstacleCostWaypoint(request, plus) -
             ObstacleCostWaypoint(request, minus)) /
            (2.0 * kStep);
        grad[static_cast<std::size_t>(fi)][j] =
            params.smoothness_cost_weight * g_s +
            params.obstacle_cost_weight * g_o;
      }
    }

    // Δq = -η M^{-1} g  (per dof, free×free)
    for (std::size_t j = 0; j < dof; ++j) {
      std::vector<double> gcol(static_cast<std::size_t>(free), 0.0);
      for (int fi = 0; fi < free; ++fi) {
        gcol[static_cast<std::size_t>(fi)] =
            grad[static_cast<std::size_t>(fi)][j];
      }
      for (int fi = 0; fi < free; ++fi) {
        double dq = 0.0;
        for (int k = 0; k < free; ++k) {
          dq += minv[static_cast<std::size_t>(fi)][static_cast<std::size_t>(k)] *
                gcol[static_cast<std::size_t>(k)];
        }
        dq = -params.learning_rate * dq;
        dq = std::clamp(dq, -params.joint_update_limit,
                        params.joint_update_limit);
        traj.waypoints[static_cast<std::size_t>(fi + 1)].positions[j] += dq;
      }
    }

    traj.waypoints.back().positions = request.goal_state.positions;
    traj.waypoints.front().positions = request.start_state.positions;
    ClampTrajectory(request, &traj);

    const bool valid =
        !request.scene || request.scene->IsPathValid(traj);
    if (valid) {
      ++valid_streak;
      if (valid_streak >= params.max_iterations_after_collision_free) {
        break;
      }
    } else {
      valid_streak = 0;
    }
  }
  return traj;
}

core::RobotTrajectory ChompPolish(const MotionPlanRequest& request,
                                 core::RobotTrajectory traj, int iters) {
  ChompParams params;
  params.max_iterations = std::max(1, iters);
  return ChompOptimize(request, std::move(traj), params);
}

std::vector<double> SmoothKernel(const std::vector<double>& x) {
  static const double k[5] = {1, 2, 3, 2, 1};
  static const double ksum = 9.0;
  std::vector<double> y(x.size(), 0.0);
  for (std::size_t i = 0; i < x.size(); ++i) {
    double acc = 0.0;
    for (int k = -2; k <= 2; ++k) {
      const int j = static_cast<int>(i) + k;
      if (j < 0 || j >= static_cast<int>(x.size())) {
        continue;
      }
      acc += k[k + 2] * x[static_cast<std::size_t>(j)];
    }
    y[i] = acc / ksum;
  }
  return y;
}

core::RobotTrajectory StompOptimize(const MotionPlanRequest& request,
                                    core::RobotTrajectory seed,
                                    const StompParams& params) {
  ClampTrajectory(request, &seed);
  if (seed.waypoints.size() < 3) {
    return seed;
  }
  const std::size_t T = seed.waypoints.size();
  const std::size_t dof = seed.waypoints.front().positions.size();
  core::RobotTrajectory best = seed;
  double best_cost =
      SmoothnessCost(best) * params.control_cost_weight +
      CollisionCost(request, best) * params.collision_penalty;
  int valid_streak = 0;
  std::mt19937 rng(42);
  std::normal_distribution<double> gauss(0.0, 1.0);

  auto traj_cost = [&](const core::RobotTrajectory& traj) {
    return params.control_cost_weight * SmoothnessCost(traj) +
           params.collision_penalty * CollisionCost(request, traj);
  };

  for (int it = 0; it < params.num_iterations; ++it) {
    std::vector<std::vector<std::vector<double>>> noises;
    std::vector<double> costs;
    noises.reserve(static_cast<std::size_t>(params.num_rollouts));
    costs.reserve(static_cast<std::size_t>(params.num_rollouts));

    for (int r = 0; r < params.num_rollouts; ++r) {
      std::vector<std::vector<double>> noise(
          T, std::vector<double>(dof, 0.0));
      for (std::size_t j = 0; j < dof; ++j) {
        std::vector<double> raw(T, 0.0);
        for (std::size_t i = 1; i + 1 < T; ++i) {
          raw[i] = gauss(rng);
        }
        auto sm = SmoothKernel(raw);
        for (std::size_t i = 1; i + 1 < T; ++i) {
          noise[i][j] = params.noise_stddev * sm[i];
        }
      }
      core::RobotTrajectory cand = best;
      for (std::size_t i = 1; i + 1 < T; ++i) {
        for (std::size_t j = 0; j < dof; ++j) {
          cand.waypoints[i].positions[j] += noise[i][j];
        }
      }
      ClampTrajectory(request, &cand);
      costs.push_back(traj_cost(cand));
      noises.push_back(std::move(noise));
    }

    const double cmin = *std::min_element(costs.begin(), costs.end());
    const double cmax = *std::max_element(costs.begin(), costs.end());
    const double span = std::max(1e-9, cmax - cmin);
    std::vector<std::vector<double>> prob(T, std::vector<double>(dof, 0.0));
    std::vector<double> weight_sum(T, 0.0);

    for (std::size_t r = 0; r < costs.size(); ++r) {
      const double norm = (costs[r] - cmin) / span;
      const double w =
          std::exp(-params.exponentiated_cost_sensitivity * norm);
      for (std::size_t i = 1; i + 1 < T; ++i) {
        weight_sum[i] += w;
        for (std::size_t j = 0; j < dof; ++j) {
          prob[i][j] += w * noises[r][i][j];
        }
      }
    }
    for (std::size_t i = 1; i + 1 < T; ++i) {
      if (weight_sum[i] < 1e-12) {
        continue;
      }
      for (std::size_t j = 0; j < dof; ++j) {
        prob[i][j] /= weight_sum[i];
      }
    }
    // Smooth update in time (M approx).
    for (std::size_t j = 0; j < dof; ++j) {
      std::vector<double> col(T, 0.0);
      for (std::size_t i = 0; i < T; ++i) {
        col[i] = prob[i][j];
      }
      col = SmoothKernel(col);
      for (std::size_t i = 1; i + 1 < T; ++i) {
        best.waypoints[i].positions[j] += col[i];
      }
    }
    best.waypoints.front().positions = request.start_state.positions;
    best.waypoints.back().positions = request.goal_state.positions;
    ClampTrajectory(request, &best);

    const double cost = traj_cost(best);
    if (cost < best_cost) {
      best_cost = cost;
    }
    const bool valid = !request.scene || request.scene->IsPathValid(best);
    if (valid) {
      ++valid_streak;
      if (valid_streak >= params.num_iterations_after_valid) {
        break;
      }
    } else {
      valid_streak = 0;
    }
  }
  return best;
}

}  // namespace

/** @brief CHOMP-style covariant gradient optimizer (industrial-lite). */
class ChompPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name (e.g. "chomp").
   * @return true.
   */
  bool Init(const std::string& planner_id) override {
    planner_id_ = planner_id;
    return true;
  }

  /**
   * @brief Optimize a linear seed with smoothness + obstacle metric update.
   * @param[in] request Matching DOF start / goal; optional scene / model.
   * @return Polished trajectory or planning failure.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override {
    MotionPlanResponse response;
    if (request.start_state.positions.size() !=
            request.goal_state.positions.size() ||
        request.start_state.positions.empty()) {
      response.error_code = ErrorCode::kInvalidRobotState;
      response.error = "CHOMP DOF mismatch";
      return response;
    }
    ChompParams params;
    if (request.planning_time > 0) {
      params.max_iterations =
          std::max(10, static_cast<int>(request.planning_time * 20));
    }
    core::RobotTrajectory traj =
        SeedInterpolate(request, params.num_timesteps);
    traj = ChompOptimize(request, std::move(traj), params);
    if (request.scene && !request.scene->IsPathValid(traj)) {
      response.error_code = ErrorCode::kPlanningFailed;
      response.error = "CHOMP failed to clear collision";
      return response;
    }
    response.trajectory = std::move(traj);
    response.success = true;
    response.error_code = ErrorCode::kSuccess;
    return response;
  }

 private:
  std::string planner_id_;
};

/** @brief STOMP-style correlated-noise trajectory optimizer (industrial-lite). */
class StompPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name (e.g. "stomp").
   * @return true.
   */
  bool Init(const std::string& planner_id) override {
    planner_id_ = planner_id;
    return true;
  }

  /**
   * @brief Iterative STOMP: correlated rollouts, exp-cost weighting, M smooth.
   * @param[in] request Matching DOF start / goal; optional scene / model.
   * @return Optimized trajectory or planning failure.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override {
    MotionPlanResponse response;
    if (request.start_state.positions.size() !=
            request.goal_state.positions.size() ||
        request.start_state.positions.empty()) {
      response.error_code = ErrorCode::kInvalidRobotState;
      response.error = "STOMP DOF mismatch";
      return response;
    }
    StompParams params;
    if (request.planning_time > 0) {
      params.num_iterations =
          std::max(4, static_cast<int>(request.planning_time * 4));
    }
    core::RobotTrajectory seed =
        SeedInterpolate(request, params.num_timesteps);
    auto best = StompOptimize(request, std::move(seed), params);
    if (request.scene && !request.scene->IsPathValid(best)) {
      response.error_code = ErrorCode::kPlanningFailed;
      response.error = "STOMP no collision-free trajectory";
      return response;
    }
    response.trajectory = std::move(best);
    response.success = true;
    response.error_code = ErrorCode::kSuccess;
    return response;
  }

 private:
  std::string planner_id_;
};

/** @brief Sampling seed (OMPL or RRT-Connect) + CHOMP polish. */
class HybridPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name (e.g. "hybrid").
   * @return true.
   */
  bool Init(const std::string& planner_id) override {
    planner_id_ = planner_id;
    return true;
  }

  /**
   * @brief Plan with OMPL (fallback RRT-Connect) then CHOMP-polish the path.
   * @param[in] request Forwarded to the seed planner; polish uses scene if set.
   * @return Seed trajectory, optionally polished when collision-free.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override {
    MotionPlanResponse response;
    RegisterManipulationPlugins();

    auto try_seed = [&](const char* id) -> MotionPlanResponse {
      MotionPlanResponse out;
      auto planner = CreatePlugin<PlannerBase>(id);
      if (!planner || !planner->Init(id)) {
        out.error_code = ErrorCode::kFailure;
        out.error = std::string("hybrid: seed init failed: ") + id;
        return out;
      }
      return planner->Plan(request);
    };

    response = try_seed("ompl");
    if (!response.success) {
      response = try_seed("rrt_connect");
    }
    if (!response.success || response.trajectory.waypoints.size() < 3) {
      return response;
    }

    const int iters = std::max(8, static_cast<int>(request.planning_time * 10));
    auto polished = ChompPolish(request, response.trajectory, iters);
    if (!request.scene || request.scene->IsPathValid(polished)) {
      response.trajectory = std::move(polished);
    }
    response.success = true;
    response.error_code = ErrorCode::kSuccess;
    return response;
  }

 private:
  std::string planner_id_;
};

std::shared_ptr<PlannerBase> CreateChompPlanner() {
  return std::make_shared<ChompPlanner>();
}

std::shared_ptr<PlannerBase> CreateStompPlanner() {
  return std::make_shared<StompPlanner>();
}

std::shared_ptr<PlannerBase> CreateHybridPlanner() {
  return std::make_shared<HybridPlanner>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ChompPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(StompPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(HybridPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
