/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/utils/trajectory_optimize.hpp"

#include "autonomy/manipulation/model/joint_state_utilities.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <random>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/planner/chomp/voxel_distance_field.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_helpers.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace utils {
namespace {


constexpr int kDiffRuleLength = 7;
// MoveIt DIFF_RULES[0] velocity / [1] acceleration / [2] jerk (7-point).
constexpr double kVelRule[kDiffRuleLength] = {
    0.0, -1.0 / 12.0, 8.0 / 12.0, 0.0, -8.0 / 12.0, 1.0 / 12.0, 0.0};
constexpr double kAccRule[kDiffRuleLength] = {
    0.0, 1.0 / 12.0, -1.5 / 12.0, 2.0 / 12.0, -1.5 / 12.0, 1.0 / 12.0, 0.0};
constexpr double kJerkRule[kDiffRuleLength] = {
    0.0, -0.5, 1.0, 0.0, -1.0, 0.5, 0.0};

// Thread-local active CHOMP clearance for potential (set in ChompOptimize).
thread_local double g_min_clearance = 0.05;

/** Randomize free waypoints for CHOMP failure recovery. */


bool StateCollides(const MotionPlanRequest& request,
                   const automsgs::msgs::sensor_msgs::JointState& state) {
  if (!request.scene) {
    return false;
  }
  return request.scene->CheckCollision(state) ||
         !request.scene->IsStateValid(state);
}

/** Signed distance to scene primitives (positive outside). Mesh → AABB. */
double PrimitiveSdf(const automsgs::msgs::moveit_msgs::CollisionObject& o, double x, double y,
                    double z) {
  using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
  const auto pose = scene::GetObjectPose(o);
  const double dx = x - pose.position().x();
  const double dy = y - pose.position().y();
  const double dz = z - pose.position().z();
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  scene::GetPrimitiveSizes(o, &sx, &sy, &sz);
  if (scene::GetPrimitiveType(o) == SP::SPHERE) {
    return std::sqrt(dx * dx + dy * dy + dz * dz) - sx;
  }
  if (scene::GetPrimitiveType(o) == SP::CYLINDER) {
    const double radial = std::sqrt(dx * dx + dy * dy) - sx;
    const double axial = std::abs(dz) - 0.5 * sz;
    if (radial > 0.0 && axial > 0.0) {
      return std::sqrt(radial * radial + axial * axial);
    }
    return std::max(radial, axial);
  }
  // box / mesh AABB
  const double qx = std::abs(dx) - 0.5 * sx;
  const double qy = std::abs(dy) - 0.5 * sy;
  const double qz = std::abs(dz) - 0.5 * sz;
  const double outside = std::sqrt(std::max(qx, 0.0) * std::max(qx, 0.0) +
                                   std::max(qy, 0.0) * std::max(qy, 0.0) +
                                   std::max(qz, 0.0) * std::max(qz, 0.0));
  const double inside = std::min({qx, qy, qz});
  return outside + std::min(inside, 0.0);
}

double SceneSdf(const MotionPlanRequest& request, double x, double y,
                double z) {
  if (!request.scene) {
    return 1e3;
  }
  double d = 1e3;
  for (const auto& o : request.scene->GetCollisionObjects()) {
    d = std::min(d, PrimitiveSdf(o, x, y, z));
  }
  const double res = std::max(1e-3, request.scene->OccupancyResolution());
  for (const auto& p : request.scene->OccupiedPoints()) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double dz = z - p.z;
    d = std::min(d, std::sqrt(dx * dx + dy * dy + dz * dz) - 0.5 * res);
  }
  return d;
}

/** Optional voxel DF override for CHOMP (set for duration of optimize). */
const VoxelDistanceField* g_voxel_df = nullptr;

void SceneSdfGradient(const MotionPlanRequest& request, double x, double y,
                      double z, double* gx, double* gy, double* gz);

double QuerySdf(const MotionPlanRequest& request, double x, double y,
                double z) {
  if (g_voxel_df && !g_voxel_df->empty()) {
    return g_voxel_df->Distance(x, y, z);
  }
  return SceneSdf(request, x, y, z);
}

void QuerySdfGradient(const MotionPlanRequest& request, double x, double y,
                      double z, double* gx, double* gy, double* gz) {
  if (g_voxel_df && !g_voxel_df->empty()) {
    g_voxel_df->Gradient(x, y, z, gx, gy, gz);
    return;
  }
  SceneSdfGradient(request, x, y, z, gx, gy, gz);
}

/** Tip / EE proxy from joint sum (planar) when no kinematics — else FK. */
bool EstimateTip(const MotionPlanRequest& request, const automsgs::msgs::sensor_msgs::JointState& state,
                 double* x, double* y, double* z) {
  if (request.kinematics) {
    automsgs::msgs::geometry_msgs::Pose tip;
    if (request.kinematics->GetPositionFK(state, &tip)) {
      *x = tip.position().x();
      *y = tip.position().y();
      *z = tip.position().z();
      return true;
    }
  }
  double yaw = 0.0;
  double px = 0.0;
  double py = 0.0;
  constexpr double kLink = 0.3;
  for (double q : state.position()) {
    yaw += q;
    px += kLink * std::cos(yaw);
    py += kLink * std::sin(yaw);
  }
  *x = px;
  *y = py;
  *z = 0.0;
  return true;
}

/** MoveIt CHOMP potential of signed distance d (ε = g_min_clearance). */
double DistancePotential(double d) {
  const double eps = g_min_clearance;
  if (d > eps) {
    return 0.0;
  }
  if (d >= 0.0) {
    return 0.5 * (d - eps) * (d - eps) / eps;
  }
  return -d + 0.5 * eps;
}

/** ∂V/∂d for DistancePotential. */
double DistancePotentialDeriv(double d) {
  const double eps = g_min_clearance;
  if (d > eps) {
    return 0.0;
  }
  if (d >= 0.0) {
    return (d - eps) / eps;
  }
  return -1.0;
}

/** Workspace gradient of SDF via central differences. */
void SceneSdfGradient(const MotionPlanRequest& request, double x, double y,
                      double z, double* gx, double* gy, double* gz) {
  constexpr double kH = 1e-3;
  *gx = (SceneSdf(request, x + kH, y, z) - SceneSdf(request, x - kH, y, z)) /
        (2.0 * kH);
  *gy = (SceneSdf(request, x, y + kH, z) - SceneSdf(request, x, y - kH, z)) /
        (2.0 * kH);
  *gz = (SceneSdf(request, x, y, z + kH) - SceneSdf(request, x, y, z - kH)) /
        (2.0 * kH);
}

/**
 * Full-chain obstacle cost: sum DistancePotential over all link tips
 * (or EE proxy when no LinkForwardKinematicsTree).
 */
double ObstacleCostWaypoint(const MotionPlanRequest& request,
                            const automsgs::msgs::sensor_msgs::JointState& state) {
  if (!request.scene) {
    return 0.0;
  }
  if (request.link_tree) {
    std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose> poses;
    if (!request.link_tree->Compute(state, &poses) || poses.empty()) {
      return StateCollides(request, state) ? 1.0 : 0.0;
    }
    double cost = 0.0;
    for (const auto& kv : poses) {
      const double d =
          QuerySdf(request, kv.second.x, kv.second.y, kv.second.z);
      cost += DistancePotential(d);
    }
    return cost;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (!EstimateTip(request, state, &x, &y, &z)) {
    return StateCollides(request, state) ? 1.0 : 0.0;
  }
  return DistancePotential(QuerySdf(request, x, y, z));
}

/**
 * Joint-space obstacle gradient via full-chain Jacobian of the distance field:
 * g_q = Σ_links Jᵀ (∂V/∂d · ∇d).
 */
std::vector<double> ObstacleGradientJacobian(const MotionPlanRequest& request,
                                             const automsgs::msgs::sensor_msgs::JointState& state) {
  const std::size_t dof = static_cast<std::size_t>(state.position_size());
  std::vector<double> g(dof, 0.0);
  if (!request.scene || dof == 0) {
    return g;
  }
  constexpr double kQ = 1e-4;

  auto add_point = [&](double x, double y, double z,
                       const std::function<bool(std::size_t, double*, double*,
                                                double*)>& jac_col) {
    const double d = QuerySdf(request, x, y, z);
    const double dVdd = DistancePotentialDeriv(d);
    if (std::abs(dVdd) < 1e-12) {
      return;
    }
    double gx = 0.0, gy = 0.0, gz = 0.0;
    QuerySdfGradient(request, x, y, z, &gx, &gy, &gz);
    const double wx = dVdd * gx;
    const double wy = dVdd * gy;
    const double wz = dVdd * gz;
    for (std::size_t j = 0; j < dof; ++j) {
      double jx = 0.0, jy = 0.0, jz = 0.0;
      if (!jac_col(j, &jx, &jy, &jz)) {
        continue;
      }
      g[j] += jx * wx + jy * wy + jz * wz;
    }
  };

  if (request.link_tree) {
    std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose> poses0;
    if (!request.link_tree->Compute(state, &poses0)) {
      return g;
    }
    for (const auto& kv : poses0) {
      add_point(kv.second.x, kv.second.y, kv.second.z,
                [&](std::size_t j, double* jx, double* jy, double* jz) {
                  automsgs::msgs::sensor_msgs::JointState plus = state;
                  automsgs::msgs::sensor_msgs::JointState minus = state;
                  plus.set_position(static_cast<int>(j), plus.position(static_cast<int>(j)) + kQ);
                  minus.set_position(static_cast<int>(j), minus.position(static_cast<int>(j)) - kQ);
                  std::unordered_map<std::string, automsgs::msgs::geometry_msgs::Pose> pp, pm;
                  if (!request.link_tree->Compute(plus, &pp) ||
                      !request.link_tree->Compute(minus, &pm) ||
                      !pp.count(kv.first) || !pm.count(kv.first)) {
                    return false;
                  }
                  *jx = (pp[kv.first].x - pm[kv.first].x) / (2.0 * kQ);
                  *jy = (pp[kv.first].y - pm[kv.first].y) / (2.0 * kQ);
                  *jz = (pp[kv.first].z - pm[kv.first].z) / (2.0 * kQ);
                  return true;
                });
    }
    return g;
  }

  double x = 0.0, y = 0.0, z = 0.0;
  if (!EstimateTip(request, state, &x, &y, &z)) {
    return g;
  }
  add_point(x, y, z, [&](std::size_t j, double* jx, double* jy, double* jz) {
    automsgs::msgs::sensor_msgs::JointState plus = state;
    automsgs::msgs::sensor_msgs::JointState minus = state;
    plus.set_position(static_cast<int>(j), plus.position(static_cast<int>(j)) + kQ);
    minus.set_position(static_cast<int>(j), minus.position(static_cast<int>(j)) - kQ);
    double xp, yp, zp, xm, ym, zm;
    EstimateTip(request, plus, &xp, &yp, &zp);
    EstimateTip(request, minus, &xm, &ym, &zm);
    *jx = (xp - xm) / (2.0 * kQ);
    *jy = (yp - ym) / (2.0 * kQ);
    *jz = (zp - zm) / (2.0 * kQ);
    return true;
  });
  return g;
}

double SmoothnessCost(const automsgs::msgs::trajectory_msgs::JointTrajectory& traj) {
  if (traj.points_size() < 3) {
    return 0.0;
  }
  double cost = 0.0;
  const int dof = traj.points(0).positions_size();
  for (int i = 1; i + 1 < traj.points_size(); ++i) {
    for (int j = 0; j < dof; ++j) {
      const double acc = traj.points(i - 1).positions(j) -
                         2.0 * traj.points(i).positions(j) +
                         traj.points(i + 1).positions(j);
      cost += acc * acc;
    }
  }
  return cost;
}

double CollisionCost(const MotionPlanRequest& request,
                     const automsgs::msgs::trajectory_msgs::JointTrajectory& traj) {
  double cost = 0.0;
  for (int i = 0; i < traj.points_size(); ++i) {
    cost += ObstacleCostWaypoint(request, MakeJointStateFromPoint(traj, i));
  }
  // Midpoint interpolation checks (STOMP-style densification).
  for (int i = 0; i + 1 < traj.points_size(); ++i) {
    automsgs::msgs::sensor_msgs::JointState mid = MakeJointStateFromPoint(traj, i);
    for (int j = 0; j < mid.position_size(); ++j) {
      mid.set_position(
          j, 0.5 * (traj.points(i).positions(j) +
                    traj.points(i + 1).positions(j)));
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



}  // namespace

automsgs::msgs::trajectory_msgs::JointTrajectory InterpolateSeedTrajectory(const MotionPlanRequest& request,
                                      int waypoints,
                                      const std::string& method) {
  automsgs::msgs::trajectory_msgs::JointTrajectory traj;
  const int dof = GetJointStateDegreesOfFreedom(request.pb.start_state());
  std::vector<std::string> names;
  if (request.pb.goal_state().name_size() > 0) {
    names.assign(request.pb.goal_state().name().begin(),
                 request.pb.goal_state().name().end());
  } else {
    names.assign(request.pb.start_state().name().begin(),
                 request.pb.start_state().name().end());
  }
  const int n = std::max(2, waypoints);
  const bool cubic = (method == "cubic");
  const bool quintic = (method == "quintic");
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    double s = t;
    if (quintic) {
      // 10t³ − 15t⁴ + 6t⁵ (zero vel/acc at ends)
      s = t * t * t * (10.0 + t * (-15.0 + t * 6.0));
    } else if (cubic) {
      // 3t² − 2t³ (zero vel at ends)
      s = t * t * (3.0 - 2.0 * t);
    }
    std::vector<double> positions(static_cast<std::size_t>(dof));
    for (int j = 0; j < dof; ++j) {
      const double a = request.pb.start_state().position(j);
      const double b = request.pb.goal_state().position(j);
      positions[static_cast<std::size_t>(j)] = a + s * (b - a);
    }
    automsgs::msgs::sensor_msgs::JointState wp;
    SetJointState(&wp, names, positions);
    AddTrajectoryPoint(&traj, wp, t);
  }
  return traj;
}

automsgs::msgs::trajectory_msgs::JointTrajectory PerturbSeed(const automsgs::msgs::trajectory_msgs::JointTrajectory& seed,
                                  std::mt19937* rng, double scale) {
  automsgs::msgs::trajectory_msgs::JointTrajectory out = seed;
  if (!rng || out.points_size() < 3) {
    return out;
  }
  std::uniform_real_distribution<double> uni(-scale, scale);
  for (int i = 1; i + 1 < out.points_size(); ++i) {
    auto* pt = out.mutable_points(i);
    for (int j = 0; j < pt->positions_size(); ++j) {
      pt->set_positions(j, pt->positions(j) + uni(*rng));
    }
  }
  return out;
}

void ClampTrajectory(const MotionPlanRequest& request,
                     automsgs::msgs::trajectory_msgs::JointTrajectory* traj) {
  if (!traj || !request.model) {
    return;
  }
  for (int p = 0; p < traj->points_size(); ++p) {
    auto* pt = traj->mutable_points(p);
    for (int i = 0; i < pt->positions_size() && i < traj->joint_names_size();
         ++i) {
      const auto* lim = request.model->GetJointLimits(traj->joint_names(i));
      if (!lim || !lim->has_position_limits) {
        continue;
      }
      pt->set_positions(
          i, std::clamp(pt->positions(i), lim->min_position, lim->max_position));
    }
  }
}

namespace {

void CopyStatePositionsToPoint(const automsgs::msgs::sensor_msgs::JointState& state,
                               automsgs::msgs::trajectory_msgs::JointTrajectoryPoint* pt) {
  if (!pt) {
    return;
  }
  pt->clear_positions();
  for (double q : state.position()) {
    pt->add_positions(q);
  }
}

}  // namespace

automsgs::msgs::trajectory_msgs::JointTrajectory ChompOptimize(const MotionPlanRequest& request,
                                    automsgs::msgs::trajectory_msgs::JointTrajectory traj,
                                    const ChompParams& params) {
  if (traj.points_size() < 3) {
    return traj;
  }
  ClampTrajectory(request, &traj);
  const int T = traj.points_size();
  const std::size_t dof =
      static_cast<std::size_t>(traj.points(0).positions_size());
  const int free = T - 2;
  if (free <= 0) {
    return traj;
  }

  VoxelDistanceField voxel_df;
  voxel_df.Build(request.scene.get(), params.voxel_resolution,
                 params.voxel_padding, params.voxel_margin);
  const VoxelDistanceField* prev_df = g_voxel_df;
  g_voxel_df = voxel_df.empty() ? nullptr : &voxel_df;
  const double prev_eps = g_min_clearance;
  g_min_clearance = params.min_clearance > 1e-6 ? params.min_clearance : 0.05;

  const auto minv = MetricInverse(free, params.ridge_factor);
  int valid_streak = 0;
  // Previous covariant update (momentum / filter warm-start).
  std::vector<std::vector<double>> prev_update(
      static_cast<std::size_t>(free), std::vector<double>(dof, 0.0));
  const auto t0 = std::chrono::steady_clock::now();

  for (int iter = 0; iter < params.max_iterations; ++iter) {
    if (params.planning_time_limit > 0.0) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= params.planning_time_limit) {
        break;
      }
    }
    // Gradients on free waypoints [1..T-2], per dof.
    std::vector<std::vector<double>> grad(
        static_cast<std::size_t>(free), std::vector<double>(dof, 0.0));

    for (int fi = 0; fi < free; ++fi) {
      const int i = fi + 1;
      const auto g_obs =
          ObstacleGradientJacobian(request, MakeJointStateFromPoint(traj, i));
      for (std::size_t j = 0; j < dof; ++j) {
        double g_vel = 0.0;
        double g_acc = 0.0;
        double g_jerk = 0.0;
        for (int k = 0; k < kDiffRuleLength; ++k) {
          const int idx = i + k - kDiffRuleLength / 2;
          if (idx < 0 || idx >= T) {
            continue;
          }
          const double q = traj.points(idx).positions(static_cast<int>(j));
          g_vel += kVelRule[k] * q;
          g_acc += kAccRule[k] * q;
          g_jerk += kJerkRule[k] * q;
        }
        const double g_s =
            params.smoothness_cost_velocity * g_vel +
            params.smoothness_cost_acceleration * g_acc +
            params.smoothness_cost_jerk * g_jerk;
        const double g_o = j < g_obs.size() ? g_obs[j] : 0.0;
        grad[static_cast<std::size_t>(fi)][j] =
            params.smoothness_cost_weight * g_s +
            params.obstacle_cost_weight * g_o;
      }
    }

    // Covariant filter: Δq = -η M^{-1} g, then optional temporal low-pass
    // (MoveIt CHOMP updateFromMatrix + smoothness on the update).
    std::vector<std::vector<double>> update(
        static_cast<std::size_t>(free), std::vector<double>(dof, 0.0));
    for (std::size_t j = 0; j < dof; ++j) {
      std::vector<double> gcol(static_cast<std::size_t>(free), 0.0);
      for (int fi = 0; fi < free; ++fi) {
        gcol[static_cast<std::size_t>(fi)] =
            grad[static_cast<std::size_t>(fi)][j];
      }
      std::vector<double> dqcol(static_cast<std::size_t>(free), 0.0);
      for (int fi = 0; fi < free; ++fi) {
        double dq = 0.0;
        for (int k = 0; k < free; ++k) {
          dq += minv[static_cast<std::size_t>(fi)][static_cast<std::size_t>(k)] *
                gcol[static_cast<std::size_t>(k)];
        }
        dqcol[static_cast<std::size_t>(fi)] = -params.learning_rate * dq;
      }
      if (params.filter_update && free >= 3) {
        dqcol = SmoothKernel(dqcol);
      }
      for (int fi = 0; fi < free; ++fi) {
        double dq = dqcol[static_cast<std::size_t>(fi)];
        // Light momentum toward previous filtered update.
        dq = 0.85 * dq + 0.15 * prev_update[static_cast<std::size_t>(fi)][j];
        dq = std::clamp(dq, -params.joint_update_limit,
                        params.joint_update_limit);
        update[static_cast<std::size_t>(fi)][j] = dq;
        auto* pt = traj.mutable_points(fi + 1);
        pt->set_positions(static_cast<int>(j),
                          pt->positions(static_cast<int>(j)) + dq);
      }
    }
    prev_update = std::move(update);

    CopyStatePositionsToPoint(request.pb.goal_state(),
                              traj.mutable_points(traj.points_size() - 1));
    CopyStatePositionsToPoint(request.pb.start_state(), traj.mutable_points(0));
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
  g_voxel_df = prev_df;
  g_min_clearance = prev_eps;
  return traj;
}

automsgs::msgs::trajectory_msgs::JointTrajectory ChompPolish(const MotionPlanRequest& request,
                                 automsgs::msgs::trajectory_msgs::JointTrajectory traj, int iters) {
  ChompParams params = DefaultChompParams();
  LoadChompParamsFromShare(&params);
  params.max_iterations = std::max(1, iters);
  return ChompOptimize(request, std::move(traj), params);
}

automsgs::msgs::trajectory_msgs::JointTrajectory StompOptimize(const MotionPlanRequest& request,
                                    automsgs::msgs::trajectory_msgs::JointTrajectory seed,
                                    const StompParams& params) {
  ClampTrajectory(request, &seed);
  if (seed.points_size() < 3) {
    return seed;
  }
  const std::size_t T = static_cast<std::size_t>(seed.points_size());
  const std::size_t dof =
      static_cast<std::size_t>(seed.points(0).positions_size());
  automsgs::msgs::trajectory_msgs::JointTrajectory best = seed;
  double best_cost =
      SmoothnessCost(best) * params.control_cost_weight +
      CollisionCost(request, best) * params.collision_penalty;
  int valid_streak = 0;
  std::mt19937 rng(42);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const auto t0 = std::chrono::steady_clock::now();

  auto traj_cost = [&](const automsgs::msgs::trajectory_msgs::JointTrajectory& traj) {
    return params.control_cost_weight * SmoothnessCost(traj) +
           params.collision_penalty * CollisionCost(request, traj);
  };

  for (int it = 0; it < params.num_iterations; ++it) {
    if (params.planning_time_limit > 0.0) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= params.planning_time_limit) {
        break;
      }
    }
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
      automsgs::msgs::trajectory_msgs::JointTrajectory cand = best;
      for (std::size_t i = 1; i + 1 < T; ++i) {
        auto* pt = cand.mutable_points(static_cast<int>(i));
        for (std::size_t j = 0; j < dof; ++j) {
          pt->set_positions(static_cast<int>(j),
                            pt->positions(static_cast<int>(j)) + noise[i][j]);
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
        auto* pt = best.mutable_points(static_cast<int>(i));
        pt->set_positions(static_cast<int>(j),
                          pt->positions(static_cast<int>(j)) + col[i]);
      }
    }
    CopyStatePositionsToPoint(request.pb.start_state(), best.mutable_points(0));
    CopyStatePositionsToPoint(request.pb.goal_state(),
                              best.mutable_points(best.points_size() - 1));
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

}  // namespace utils
}  // namespace manipulation
}  // namespace autonomy
