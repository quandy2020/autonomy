/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/pilz/pilz_ptp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"
#include "autonomy/manipulation/planner/pilz/pilz_blend.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/motion/kinematics/pose_util.hpp"
#include "autonomy/manipulation/planner/pilz/pilz_limits.hpp"
#include "autonomy/manipulation/planner/pipeline/time_parameterization.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

using kinematics::Pose;
using kinematics::InterpolatePose;

constexpr double kPi = 3.141592653589793;

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

void ApplyPilzTiming(const MotionPlanRequest& request,
                     core::RobotTrajectory* traj) {
  if (!traj || traj->points_size() < 2) {
    return;
  }
  trajectory::TimeParamOptions opts;
  opts.max_velocity =
      request.max_velocity * std::max(1e-3, request.velocity_scale);
  opts.max_acceleration = request.max_acceleration;
  opts.path_tolerance = std::max(1e-3, request.blend_radius);
  if (request.model) {
    opts.max_velocity_vector.resize(
        static_cast<std::size_t>(traj->joint_names_size()));
    opts.max_acceleration_vector.resize(
        static_cast<std::size_t>(traj->joint_names_size()));
    for (int i = 0; i < traj->joint_names_size(); ++i) {
      const auto* lim = request.model->GetJointLimits(traj->joint_names(i));
      opts.max_velocity_vector[static_cast<std::size_t>(i)] =
          (lim && lim->max_velocity > 0 ? lim->max_velocity
                                        : request.max_velocity) *
          std::max(1e-3, request.velocity_scale);
      opts.max_acceleration_vector[static_cast<std::size_t>(i)] =
          lim && lim->max_acceleration > 0 ? lim->max_acceleration
                                           : request.max_acceleration;
    }
  }
  trajectory::ApplyTotg(traj, opts);
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
    const Pose pose = InterpolatePose(poses[static_cast<std::size_t>(seg)],
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
    AddTrajectoryPoint(&response.trajectory, sol, duration_hint * t);
  }
  if (response.trajectory.points_size() > 0) {
    // Keep start exact; do not overwrite joint goal for LIN/CIRC tip accuracy.
    auto* front = response.trajectory.mutable_points(0);
    front->clear_positions();
    for (double q : request.start_state.position()) {
      front->add_positions(q);
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
  BlendJointTrajectory(&response.trajectory, request.blend_radius);
  ApplyPilzTiming(request, &response.trajectory);
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
  if (request.start_state.position_size() !=
          request.goal_state.position_size() ||
      request.start_state.position_size() == 0) {
    response.error_code = ErrorCode::kInvalidRobotState;
    response.error = "PTP DOF mismatch";
    return response;
  }

  const std::size_t dof =
      static_cast<std::size_t>(request.start_state.position_size());
  const double v_scale = std::max(1e-3, request.velocity_scale);
  const double a_scale = std::max(1e-3, request.acceleration_scale);
  std::vector<std::string> names;
  if (request.goal_state.name_size() > 0) {
    names.assign(request.goal_state.name().begin(),
                 request.goal_state.name().end());
  } else {
    names.assign(request.start_state.name().begin(),
                 request.start_state.name().end());
  }

  // MoveIt PTP: per-joint fastest ATRAP → leading axis → full sync.
  std::vector<AtrapProfile> profiles(dof);
  std::size_t leading = 0;
  double max_duration = -1.0;
  for (std::size_t i = 0; i < dof; ++i) {
    double vmax = request.max_velocity * v_scale;
    double amax = request.max_acceleration * a_scale;
    double dmax = amax;
    if (request.model && i < names.size()) {
      const auto* lim = request.model->GetJointLimits(names[i]);
      if (lim) {
        if (lim->max_velocity > 0) {
          vmax = lim->max_velocity * v_scale;
        }
        if (lim->max_acceleration > 0) {
          amax = lim->max_acceleration * a_scale;
          dmax = amax;
        }
      }
    }
    profiles[i].SetProfile(request.start_state.position(static_cast<int>(i)),
                           request.goal_state.position(static_cast<int>(i)),
                           vmax, amax, dmax);
    if (profiles[i].Duration() > max_duration) {
      max_duration = profiles[i].Duration();
      leading = i;
    }
  }
  if (max_duration < 1e-9) {
    // Already at goal.
    response.trajectory.Clear();
    AddTrajectoryPoint(&response.trajectory, request.start_state, 0.0);
    AddTrajectoryPoint(&response.trajectory, request.goal_state, 0.05);
    response.success = true;
    response.error_code = ErrorCode::kSuccess;
    return response;
  }

  const double t_acc = profiles[leading].t_a;
  const double t_cru = profiles[leading].t_b;
  const double t_dec = profiles[leading].t_c;
  for (std::size_t i = 0; i < dof; ++i) {
    if (i == leading) {
      continue;
    }
    if (!profiles[i].SetProfileAllDurations(
            request.start_state.position(static_cast<int>(i)),
            request.goal_state.position(static_cast<int>(i)), t_acc, t_cru,
            t_dec)) {
      // Keep fastest profile if sync impossible (limits mismatch).
    }
  }
  max_duration = profiles[leading].Duration();
  const double dt = 0.05;
  const int n = std::max(2, static_cast<int>(std::ceil(max_duration / dt)) + 1);

  for (int k = 0; k < n; ++k) {
    const double t =
        (k + 1 == n) ? max_duration
                     : std::min(max_duration, dt * static_cast<double>(k));
    std::vector<double> positions(dof);
    for (std::size_t j = 0; j < dof; ++j) {
      positions[j] = profiles[j].Pos(t);
    }
    core::JointState wp;
    SetJointState(&wp, names, positions);
    for (std::size_t j = 0; j < dof; ++j) {
      wp.add_velocity(0.0);
    }
    AddTrajectoryPoint(&response.trajectory, wp, t);
  }
  if (response.trajectory.points_size() > 0) {
    auto* front = response.trajectory.mutable_points(0);
    front->clear_positions();
    for (double q : request.start_state.position()) {
      front->add_positions(q);
    }
    auto* back =
        response.trajectory.mutable_points(response.trajectory.points_size() - 1);
    back->clear_positions();
    for (double q : request.goal_state.position()) {
      back->add_positions(q);
    }
    // Zero terminal velocity (MoveIt PTP).
    back->clear_velocities();
    for (std::size_t j = 0; j < dof; ++j) {
      back->add_velocities(0.0);
    }
  }

  if (request.scene && !request.scene->IsPathValid(response.trajectory)) {
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.error = "PTP path in collision";
    response.trajectory = {};
    return response;
  }
  if (!constraint_samplers::SatisfiesPathConstraints(request,
                                                    response.trajectory)) {
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.error = "PTP path violates constraints";
    response.trajectory = {};
    return response;
  }
  ApplyPilzTiming(request, &response.trajectory);
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
  const double dx = goal.position().x() - start_pose.position().x();
  const double dy = goal.position().y() - start_pose.position().y();
  const double dz = goal.position().z() - start_pose.position().z();
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double dqw = goal.orientation().w() * start_pose.orientation().w() +
                     goal.orientation().x() * start_pose.orientation().x() +
                     goal.orientation().y() * start_pose.orientation().y() +
                     goal.orientation().z() * start_pose.orientation().z();
  const double rot =
      2.0 * std::acos(std::min(1.0, std::max(0.0, std::abs(dqw))));
  const double duration = CartesianStrokeDuration(request, dist, rot);
  return SampleCartesianPath(request, {start_pose, goal}, duration);
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
  Vec3 a{start_pose.position().x(), start_pose.position().y(),
         start_pose.position().z()};
  Vec3 b{interim.position().x(), interim.position().y(),
         interim.position().z()};
  Vec3 c{goal.position().x(), goal.position().y(), goal.position().z()};
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
    Pose p = InterpolatePose(start_pose, goal, t);
    p.mutable_position()->set_x(center.x + radius * (std::cos(ang) * u.x + std::sin(ang) * v.x));
    p.mutable_position()->set_y(center.y + radius * (std::cos(ang) * u.y + std::sin(ang) * v.y));
    p.mutable_position()->set_z(center.z + radius * (std::cos(ang) * u.z + std::sin(ang) * v.z));
    poses.push_back(p);
  }
  const double arc = radius * std::abs(a2 - a0);
  const double dqw = goal.orientation().w() * start_pose.orientation().w() +
                     goal.orientation().x() * start_pose.orientation().x() +
                     goal.orientation().y() * start_pose.orientation().y() +
                     goal.orientation().z() * start_pose.orientation().z();
  const double rot =
      2.0 * std::acos(std::min(1.0, std::max(0.0, std::abs(dqw))));
  return SampleCartesianPath(
      request, poses, CartesianStrokeDuration(request, arc, rot));
}

bool PilzSequencePlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id.empty() ? "pilz_sequence" : planner_id;
  return true;
}

MotionPlanResponse PilzSequencePlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  std::vector<SequenceItem> items = request.sequence;

  // Fallback: multi-waypoint LIN chain.
  if (items.empty() && request.cartesian_waypoints.size() >= 1 &&
      request.kinematics) {
    Pose start_pose;
    if (!request.kinematics->GetPositionFK(request.start_state, &start_pose)) {
      response.error = "Sequence FK failed";
      response.error_code = ErrorCode::kFailure;
      return response;
    }
    std::vector<Pose> corners;
    corners.push_back(start_pose);
    for (const auto& p : request.cartesian_waypoints) {
      corners.push_back(p);
    }
    if (request.has_goal_pose) {
      corners.push_back(request.goal_pose);
    }
    for (std::size_t i = 1; i < corners.size(); ++i) {
      SequenceItem item;
      item.type = "LIN";
      item.goal_pose = corners[i];
      item.has_goal_pose = true;
      item.blend_radius =
          (i + 1 < corners.size()) ? request.blend_radius : 0.0;
      item.velocity_scale = request.velocity_scale;
      items.push_back(item);
    }
  }

  if (items.empty()) {
    // Single PTP if only joint goal.
    SequenceItem item;
    item.type = "PTP";
    item.goal_state = request.goal_state;
    item.velocity_scale = request.velocity_scale;
    items.push_back(item);
  }

  PilzPtpPlanner ptp;
  PilzLinPlanner lin;
  PilzCircPlanner circ;
  ptp.Init("PTP");
  lin.Init("LIN");
  circ.Init("CIRC");

  core::JointState cursor = request.start_state;
  core::RobotTrajectory merged;
  double t_offset = 0.0;

  for (std::size_t i = 0; i < items.size(); ++i) {
    const SequenceItem& item = items[i];
    MotionPlanRequest sub = request;
    sub.sequence.clear();
    sub.start_state = cursor;
    sub.velocity_scale =
        item.velocity_scale > 0.0 ? item.velocity_scale : request.velocity_scale;
    sub.blend_radius = 0.0;  // blend across segment seams below
    sub.goal_state = item.goal_state;
    sub.goal_pose = item.goal_pose;
    sub.has_goal_pose = item.has_goal_pose;
    sub.cartesian_waypoints.clear();
    if (item.has_interim) {
      sub.cartesian_waypoints.push_back(item.interim_pose);
    }

    std::string typ = item.type;
    for (char& c : typ) {
      if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - 'a' + 'A');
      }
    }
    if (typ.rfind("PILZ_", 0) == 0) {
      typ = typ.substr(5);
    }

    MotionPlanResponse part;
    if (typ == "LIN") {
      if (!sub.has_goal_pose && sub.goal_state.position_size() > 0 &&
          request.kinematics) {
        Pose g;
        if (request.kinematics->GetPositionFK(sub.goal_state, &g)) {
          sub.goal_pose = g;
          sub.has_goal_pose = true;
        }
      }
      part = lin.Plan(sub);
    } else if (typ == "CIRC") {
      part = circ.Plan(sub);
    } else {
      if (sub.goal_state.position_size() == 0 && sub.has_goal_pose &&
          request.kinematics) {
        core::JointState seed = cursor;
        if (!request.kinematics->GetPositionIK(sub.goal_pose, seed,
                                               &sub.goal_state)) {
          response.error = "Sequence PTP IK failed at item " + std::to_string(i);
          response.error_code = ErrorCode::kNoIkSolution;
          return response;
        }
      }
      part = ptp.Plan(sub);
    }
    if (!part.success || part.trajectory.points_size() == 0) {
      response.error = part.error.empty()
                           ? ("Sequence item " + std::to_string(i) + " failed")
                           : part.error;
      response.error_code = part.error_code;
      return response;
    }

    if (merged.points_size() == 0) {
      merged = std::move(part.trajectory);
    } else if (i > 0 && items[i - 1].blend_radius > 1e-9) {
      // Pilz: blend_radius on item i-1 blends into item i (transition window).
      // Prefer Cartesian sphere + IK when kinematics available.
      core::RobotTrajectory blended;
      const bool ok =
          request.kinematics
              ? BlendTransitionWindowCartesian(
                    merged, part.trajectory, items[i - 1].blend_radius,
                    request.kinematics.get(), &blended)
              : BlendTransitionWindow(merged, part.trajectory,
                                      items[i - 1].blend_radius, &blended);
      if (ok) {
        merged = std::move(blended);
      } else {
        // Radius too large / seam mismatch → hard-stop concatenate.
        for (int k = 1; k < part.trajectory.points_size(); ++k) {
          AddTrajectoryPoint(&merged, MakeJointStateFromPoint(part.trajectory, k),
                             t_offset + GetTrajectoryTime(part.trajectory, k));
        }
      }
    } else {
      for (int k = 1; k < part.trajectory.points_size(); ++k) {
        AddTrajectoryPoint(&merged, MakeJointStateFromPoint(part.trajectory, k),
                           t_offset + GetTrajectoryTime(part.trajectory, k));
      }
    }
    t_offset = merged.points_size() == 0
                   ? t_offset
                   : GetTrajectoryTime(merged, merged.points_size() - 1);
    cursor = MakeJointStateFromPoint(merged, merged.points_size() - 1);
  }

  if (merged.points_size() < 2) {
    response.error = "Sequence produced empty trajectory";
    response.error_code = ErrorCode::kFailure;
    return response;
  }
  if (request.scene && !request.scene->IsPathValid(merged)) {
    response.error = "Sequence path in collision";
    response.error_code = ErrorCode::kInvalidMotionPlan;
    return response;
  }
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  response.trajectory = std::move(merged);
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzPtpPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzLinPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzCircPlanner, PlannerBase);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PilzSequencePlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
