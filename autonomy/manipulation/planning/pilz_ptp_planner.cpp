/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planning/pilz_ptp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/planning/constraint_samplers.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

using kinematics::Pose;

constexpr double kPi = 3.141592653589793;

Pose LerpPose(const Pose& a, const Pose& b, double t) {
  Pose p;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.z = a.z + t * (b.z - a.z);
  p.qx = a.qx + t * (b.qx - a.qx);
  p.qy = a.qy + t * (b.qy - a.qy);
  p.qz = a.qz + t * (b.qz - a.qz);
  p.qw = a.qw + t * (b.qw - a.qw);
  const double n =
      std::sqrt(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw);
  if (n > 1e-9) {
    p.qx /= n;
    p.qy /= n;
    p.qz /= n;
    p.qw /= n;
  }
  return p;
}

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 Sub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 Add(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 Scale(const Vec3& a, double s) {
  return {a.x * s, a.y * s, a.z * s};
}
double Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
double Norm(const Vec3& a) {
  return std::sqrt(Dot(a, a));
}
Vec3 Normalize(const Vec3& a) {
  const double n = Norm(a);
  return n > 1e-12 ? Scale(a, 1.0 / n) : Vec3{1.0, 0.0, 0.0};
}

bool CircCenter(const Vec3& a, const Vec3& b, const Vec3& c, Vec3* center,
                double* radius, Vec3* normal) {
  const Vec3 ab = Sub(b, a);
  const Vec3 ac = Sub(c, a);
  *normal = Normalize(Cross(ab, ac));
  const Vec3 n = *normal;
  if (Norm(Cross(ab, ac)) < 1e-9) {
    return false;
  }
  // Circumcenter in plane of a,b,c
  const Vec3 ab_x_ac = Cross(ab, ac);
  const double denom = 2.0 * Dot(ab_x_ac, ab_x_ac);
  if (std::abs(denom) < 1e-12) {
    return false;
  }
  const Vec3 term1 = Scale(Cross(ab_x_ac, ab), Dot(ac, ac));
  const Vec3 term2 = Scale(Cross(ac, ab_x_ac), Dot(ab, ab));
  *center = Add(a, Scale(Add(term1, term2), 1.0 / denom));
  *radius = Norm(Sub(a, *center));
  (void)n;
  return *radius > 1e-6;
}

MotionPlanResponse SampleCartesianPath(
    const MotionPlanRequest& request, const std::vector<Pose>& poses,
    double duration_hint) {
  MotionPlanResponse response;
  if (!request.kinematics || poses.size() < 2) {
    response.error_code = ErrorCode::kInvalidGoalConstraints;
    response.error = "cartesian sample needs kinematics and >=2 poses";
    return response;
  }
  const int n = std::max(2, static_cast<int>(duration_hint / 0.05) + 1);
  core::JointState seed = request.start_state;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    const double s = 0.5 * (1.0 - std::cos(kPi * t));
    // Map s onto piecewise linear pose polyline
    const double u = s * static_cast<double>(poses.size() - 1);
    const int seg = std::min(static_cast<int>(poses.size()) - 2,
                             static_cast<int>(std::floor(u)));
    const double local = u - static_cast<double>(seg);
    const Pose pose = LerpPose(poses[static_cast<std::size_t>(seg)],
                               poses[static_cast<std::size_t>(seg) + 1], local);
    core::JointState sol;
    kinematics::IkOptions opts;
    opts.max_attempts = 6;
    if (request.kinematics->GetPositionIK(pose, seed, opts, &sol) !=
        ErrorCode::kSuccess) {
      response.error_code = ErrorCode::kNoIkSolution;
      response.error = "Pilz cartesian IK failed";
      return response;
    }
    if (request.scene && request.scene->CheckCollision(sol)) {
      response.error_code = ErrorCode::kInvalidMotionPlan;
      response.error = "Pilz path in collision";
      return response;
    }
    seed = sol;
    response.trajectory.waypoints.push_back(sol);
    response.trajectory.time_from_start.push_back(duration_hint * t);
  }
  if (!response.trajectory.waypoints.empty()) {
    response.trajectory.waypoints.front() = request.start_state;
    if (!request.goal_state.positions.empty() &&
        request.goal_state.positions.size() ==
            response.trajectory.waypoints.back().positions.size()) {
      // Keep IK tip accuracy; do not overwrite joint goal for LIN/CIRC.
    }
  }
  if (!constraint_samplers::SatisfiesPathConstraints(request,
                                                    response.trajectory)) {
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.error = "Pilz path violates constraints";
    response.trajectory = {};
    response.success = false;
    return response;
  }
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

}  // namespace

bool PilzPtpPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse PilzPtpPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (request.start_state.positions.size() !=
          request.goal_state.positions.size() ||
      request.start_state.positions.empty()) {
    response.error_code = ErrorCode::kInvalidRobotState;
    response.error = "PTP DOF mismatch";
    return response;
  }

  double max_delta = 0.0;
  const std::size_t dof = request.start_state.positions.size();
  for (std::size_t i = 0; i < dof; ++i) {
    max_delta = std::max(
        max_delta, std::abs(request.goal_state.positions[i] -
                            request.start_state.positions[i]));
  }
  const double vmax =
      std::max(1e-3, request.max_velocity * std::max(1e-3, request.velocity_scale));
  const double duration = max_delta / vmax;
  const int n = std::max(2, static_cast<int>(duration / 0.05) + 1);

  const auto names = request.goal_state.names.empty()
                         ? request.start_state.names
                         : request.goal_state.names;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    const double s = 0.5 * (1.0 - std::cos(kPi * t));
    core::JointState wp;
    wp.names = names;
    wp.positions.resize(dof);
    for (std::size_t j = 0; j < dof; ++j) {
      wp.positions[j] = request.start_state.positions[j] +
                        s * (request.goal_state.positions[j] -
                             request.start_state.positions[j]);
    }
    response.trajectory.waypoints.push_back(std::move(wp));
    response.trajectory.time_from_start.push_back(duration * t);
  }

  if (request.scene && !request.scene->IsPathValid(response.trajectory)) {
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.error = "PTP path in collision";
    response.trajectory = {};
    return response;
  }
  if (!response.trajectory.waypoints.empty()) {
    response.trajectory.waypoints.front().positions =
        request.start_state.positions;
    response.trajectory.waypoints.back().positions =
        request.goal_state.positions;
  }
  if (!constraint_samplers::SatisfiesPathConstraints(request,
                                                    response.trajectory)) {
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.error = "PTP path violates constraints";
    response.trajectory = {};
    return response;
  }
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

bool PilzLinPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse PilzLinPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (!request.kinematics) {
    response.error = "LIN requires kinematics";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  Pose start_pose;
  if (!request.kinematics->GetPositionFK(request.start_state, &start_pose)) {
    response.error = "LIN FK failed";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  Pose goal = request.goal_pose;
  if (!request.has_goal_pose) {
    if (!request.kinematics->GetPositionFK(request.goal_state, &goal)) {
      response.error = "LIN goal FK failed";
      response.error_code = ErrorCode::kInvalidGoalConstraints;
      return response;
    }
  }
  const double dist = std::sqrt((goal.x - start_pose.x) * (goal.x - start_pose.x) +
                                (goal.y - start_pose.y) * (goal.y - start_pose.y) +
                                (goal.z - start_pose.z) * (goal.z - start_pose.z));
  const double vmax =
      std::max(1e-3, request.max_velocity * std::max(1e-3, request.velocity_scale));
  return SampleCartesianPath(request, {start_pose, goal},
                             std::max(0.1, dist / vmax));
}

bool PilzCircPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse PilzCircPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (!request.kinematics) {
    response.error = "CIRC requires kinematics";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  Pose start_pose;
  if (!request.kinematics->GetPositionFK(request.start_state, &start_pose)) {
    response.error = "CIRC FK failed";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  Pose goal = request.goal_pose;
  if (!request.has_goal_pose) {
    if (!request.kinematics->GetPositionFK(request.goal_state, &goal)) {
      response.error = "CIRC goal FK failed";
      response.error_code = ErrorCode::kInvalidGoalConstraints;
      return response;
    }
  }
  if (request.cartesian_waypoints.empty()) {
    response.error = "CIRC needs interim pose in cartesian_waypoints[0]";
    response.error_code = ErrorCode::kInvalidGoalConstraints;
    return response;
  }
  const Pose& interim = request.cartesian_waypoints.front();
  Vec3 a{start_pose.x, start_pose.y, start_pose.z};
  Vec3 b{interim.x, interim.y, interim.z};
  Vec3 c{goal.x, goal.y, goal.z};
  Vec3 center;
  Vec3 normal;
  double radius = 0.0;
  if (!CircCenter(a, b, c, &center, &radius, &normal)) {
    response.error = "CIRC points are colinear";
    response.error_code = ErrorCode::kInvalidGoalConstraints;
    return response;
  }
  Vec3 u = Normalize(Sub(a, center));
  Vec3 v = Normalize(Cross(normal, u));
  auto angle_of = [&](const Vec3& p) {
    const Vec3 d = Normalize(Sub(p, center));
    return std::atan2(Dot(d, v), Dot(d, u));
  };
  double a0 = angle_of(a);
  double a1 = angle_of(b);
  double a2 = angle_of(c);
  // Unwrap so path passes near interim angle
  while (a1 < a0) {
    a1 += 2.0 * kPi;
  }
  while (a2 < a1) {
    a2 += 2.0 * kPi;
  }
  const int n = 24;
  std::vector<Pose> poses;
  poses.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    const double ang = a0 + t * (a2 - a0);
    Pose p = LerpPose(start_pose, goal, t);
    p.x = center.x + radius * (std::cos(ang) * u.x + std::sin(ang) * v.x);
    p.y = center.y + radius * (std::cos(ang) * u.y + std::sin(ang) * v.y);
    p.z = center.z + radius * (std::cos(ang) * u.z + std::sin(ang) * v.z);
    poses.push_back(p);
  }
  const double arc = radius * std::abs(a2 - a0);
  const double vmax =
      std::max(1e-3, request.max_velocity * std::max(1e-3, request.velocity_scale));
  return SampleCartesianPath(request, poses, std::max(0.1, arc / vmax));
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzPtpPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzLinPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzCircPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
