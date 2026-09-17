/*
 * Copyright 2026 The Openbot Authors
 *
 * Pilz-style trajectory blending (joint Bezier + transition-window).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {
namespace {

inline double JointDist(const automsgs::msgs::sensor_msgs::JointState& a, const automsgs::msgs::sensor_msgs::JointState& b) {
  const int n = std::min(a.position_size(), b.position_size());
  double s = 0.0;
  for (int k = 0; k < n; ++k) {
    const double d = a.position(k) - b.position(k);
    s += d * d;
  }
  return std::sqrt(s);
}

/** MoveIt Pilz smoothstep: α = 6s⁵ − 15s⁴ + 10s³. */
inline double QuinticAlpha(double s) {
  s = std::clamp(s, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;
  return 6.0 * s3 * s2 - 15.0 * s2 * s2 + 10.0 * s3;
}

inline double TipDist(const automsgs::msgs::geometry_msgs::Pose& a, const automsgs::msgs::geometry_msgs::Pose& b) {
  const double dx = a.position().x() - b.position().x();
  const double dy = a.position().y() - b.position().y();
  const double dz = a.position().z() - b.position().z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline automsgs::msgs::geometry_msgs::Pose BlendPose(const automsgs::msgs::geometry_msgs::Pose& a,
                                  const automsgs::msgs::geometry_msgs::Pose& b, double alpha) {
  automsgs::msgs::geometry_msgs::Pose p;
  const double s =
      (a.orientation().w() * b.orientation().w() +
       a.orientation().x() * b.orientation().x() +
       a.orientation().y() * b.orientation().y() +
       a.orientation().z() * b.orientation().z()) < 0.0
          ? -1.0
          : 1.0;
  const double omu = 1.0 - alpha;
  double qw = omu * a.orientation().w() + alpha * s * b.orientation().w();
  double qx = omu * a.orientation().x() + alpha * s * b.orientation().x();
  double qy = omu * a.orientation().y() + alpha * s * b.orientation().y();
  double qz = omu * a.orientation().z() + alpha * s * b.orientation().z();
  const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n > 1e-12) {
    qw /= n;
    qx /= n;
    qy /= n;
    qz /= n;
  }
  SetPose(&p,
          a.position().x() + alpha * (b.position().x() - a.position().x()),
          a.position().y() + alpha * (b.position().y() - a.position().y()),
          a.position().z() + alpha * (b.position().z() - a.position().z()),
          qx, qy, qz, qw);
  return p;
}

inline void AppendTimedPoint(automsgs::msgs::trajectory_msgs::JointTrajectory* traj,
                             const automsgs::msgs::sensor_msgs::JointState& js, double t) {
  AddTrajectoryPoint(traj, js, t);
}

inline void AppendTimedPointFromTraj(automsgs::msgs::trajectory_msgs::JointTrajectory* dest,
                                     const automsgs::msgs::trajectory_msgs::JointTrajectory& src,
                                     int index, double t) {
  AppendTimedPoint(dest, MakeJointStateFromPoint(src, index), t);
}

}  // namespace

/**
 * @brief Smooth sharp corners in a joint trajectory (Pilz blender lite).
 */
inline bool BlendJointTrajectory(automsgs::msgs::trajectory_msgs::JointTrajectory* traj,
                                 double blend_radius) {
  if (!traj || traj->points_size() < 3 || blend_radius <= 1e-9) {
    return true;
  }
  const int n = traj->points_size();
  const int dof = traj->points(0).positions_size();
  if (dof == 0) {
    return true;
  }

  auto dist = [&](int i, int j) {
    return JointDist(MakeJointStateFromPoint(*traj, i),
                     MakeJointStateFromPoint(*traj, j));
  };

  auto lerp = [&](int i, int j, double t) {
    automsgs::msgs::sensor_msgs::JointState out = MakeJointStateFromPoint(*traj, i);
    ResizeJointState(&out, dof);
    for (int k = 0; k < dof; ++k) {
      out.set_position(
          k, traj->points(i).positions(k) +
                 t * (traj->points(j).positions(k) - traj->points(i).positions(k)));
    }
    return out;
  };

  automsgs::msgs::trajectory_msgs::JointTrajectory out;
  AppendTimedPointFromTraj(&out, *traj, 0, 0.0);

  for (int i = 1; i + 1 < n; ++i) {
    const double d_in = dist(i - 1, i);
    const double d_out = dist(i, i + 1);
    const double r_in = std::min(blend_radius, 0.45 * d_in);
    const double r_out = std::min(blend_radius, 0.45 * d_out);
    if (r_in < 1e-9 || r_out < 1e-9) {
      AppendTimedPointFromTraj(&out, *traj, i, 0.0);
      continue;
    }
    const double t_in = 1.0 - r_in / std::max(1e-9, d_in);
    const double t_out = r_out / std::max(1e-9, d_out);
    const automsgs::msgs::sensor_msgs::JointState p0 = lerp(i - 1, i, t_in);
    const automsgs::msgs::sensor_msgs::JointState p1 = MakeJointStateFromPoint(*traj, i);
    const automsgs::msgs::sensor_msgs::JointState p2 = lerp(i, i + 1, t_out);
    constexpr int kSamples = 5;
    for (int s = 0; s <= kSamples; ++s) {
      const double u = static_cast<double>(s) / static_cast<double>(kSamples);
      const double omu = 1.0 - u;
      automsgs::msgs::sensor_msgs::JointState b = p0;
      ResizeJointState(&b, dof);
      for (int k = 0; k < dof; ++k) {
        b.set_position(k, omu * omu * p0.position(k) +
                              2.0 * omu * u * p1.position(k) +
                              u * u * p2.position(k));
      }
      AppendTimedPoint(&out, b, 0.0);
    }
  }
  AppendTimedPointFromTraj(&out, *traj, n - 1, 0.0);

  double total = 0.0;
  std::vector<double> times(static_cast<std::size_t>(out.points_size()), 0.0);
  for (int i = 1; i < out.points_size(); ++i) {
    total += JointDist(MakeJointStateFromPoint(out, i - 1),
                       MakeJointStateFromPoint(out, i));
    times[static_cast<std::size_t>(i)] = total;
  }
  const double old_t = traj->points_size() == 0
                           ? total
                           : std::max(1e-6, GetTrajectoryPointTimeSeconds(*traj, traj->points_size() - 1));
  if (total > 1e-9) {
    for (int i = 0; i < out.points_size(); ++i) {
      SetDurationSeconds(old_t * (times[static_cast<std::size_t>(i)] / total),
                         out.mutable_points(i)->mutable_time_from_start());
    }
  }
  *traj = std::move(out);
  return true;
}

/**
 * @brief Blend two timed trajectories at a shared seam (Pilz transition window).
 */
inline bool BlendTransitionWindow(const automsgs::msgs::trajectory_msgs::JointTrajectory& first,
                                  const automsgs::msgs::trajectory_msgs::JointTrajectory& second,
                                  double blend_radius,
                                  automsgs::msgs::trajectory_msgs::JointTrajectory* out) {
  if (!out || first.points_size() < 2 || second.points_size() < 2 ||
      blend_radius <= 1e-9) {
    return false;
  }
  const automsgs::msgs::sensor_msgs::JointState seam =
      MakeJointStateFromPoint(first, first.points_size() - 1);
  if (JointDist(seam, MakeJointStateFromPoint(second, 0)) > 1e-3) {
    return false;  // seam mismatch
  }

  auto time_at = [](const automsgs::msgs::trajectory_msgs::JointTrajectory& tr, int i) {
    return GetTrajectoryPointTimeSeconds(tr, i);
  };

  int i1 = first.points_size() - 1;
  bool found1 = false;
  for (int k = first.points_size() - 1; k >= 0; --k) {
    if (JointDist(MakeJointStateFromPoint(first, k), seam) >=
        blend_radius - 1e-9) {
      i1 = k;
      found1 = true;
      break;
    }
  }
  int i2 = 0;
  bool found2 = false;
  for (int k = 0; k < second.points_size(); ++k) {
    if (JointDist(MakeJointStateFromPoint(second, k), seam) >=
        blend_radius - 1e-9) {
      i2 = k;
      found2 = true;
      break;
    }
  }
  if (!found1 || !found2 || i1 + 1 >= first.points_size() || i2 == 0) {
    return false;  // radius swallows a whole segment
  }

  const int n1 = first.points_size() - i1;
  const int n2 = i2 + 1;
  const int n_blend = std::max(n1, n2);
  const double dt1 =
      (time_at(first, first.points_size() - 1) - time_at(first, i1)) /
      std::max(1.0, static_cast<double>(n1 - 1));
  const double dt2 =
      (time_at(second, i2) - time_at(second, 0)) /
      std::max(1.0, static_cast<double>(n2 - 1));
  const double dt = std::max(1e-3, 0.5 * (dt1 + dt2));

  automsgs::msgs::trajectory_msgs::JointTrajectory merged;
  for (int i = 0; i <= i1; ++i) {
    AppendTimedPointFromTraj(&merged, first, i, time_at(first, i));
  }
  double t = GetTrajectoryPointTimeSeconds(merged, merged.points_size() - 1);

  const int dof = GetJointStateDegreesOfFreedom(seam);
  for (int s = 1; s <= n_blend; ++s) {
    const double u = static_cast<double>(s) / static_cast<double>(n_blend);
    const double alpha = QuinticAlpha(u);
    const int ia = std::min(i1 + s, first.points_size() - 1);
    const int ib = std::min(s, i2);
    const automsgs::msgs::sensor_msgs::JointState qa = MakeJointStateFromPoint(first, ia);
    const automsgs::msgs::sensor_msgs::JointState qb = MakeJointStateFromPoint(second, ib);
    automsgs::msgs::sensor_msgs::JointState wp = qa;
    ResizeJointState(&wp, dof);
    for (int j = 0; j < dof; ++j) {
      const double a =
          j < qa.position_size() ? qa.position(j) : seam.position(j);
      const double b =
          j < qb.position_size() ? qb.position(j) : seam.position(j);
      wp.set_position(j, a + alpha * (b - a));
    }
    t += dt;
    AppendTimedPoint(&merged, wp, t);
  }

  const double t2_at_i2 = time_at(second, i2);
  for (int k = i2 + 1; k < second.points_size(); ++k) {
    const double tk = time_at(second, k);
    AppendTimedPointFromTraj(&merged, second, k, t + (tk - t2_at_i2));
  }
  for (int i = 1; i < merged.points_size(); ++i) {
    const double prev = GetTrajectoryPointTimeSeconds(merged, i - 1);
    const double cur = GetTrajectoryPointTimeSeconds(merged, i);
    if (cur < prev + 1e-6) {
      SetDurationSeconds(prev + 1e-6,
                         merged.mutable_points(i)->mutable_time_from_start());
    }
  }
  *out = std::move(merged);
  return true;
}

/**
 * @brief Cartesian transition-window blend (MoveIt Pilz blender analogue).
 */
inline bool BlendTransitionWindowCartesian(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& first, const automsgs::msgs::trajectory_msgs::JointTrajectory& second,
    double blend_radius, const common::KinematicsInterface* kinematics,
    automsgs::msgs::trajectory_msgs::JointTrajectory* out) {
  if (!kinematics) {
    return BlendTransitionWindow(first, second, blend_radius, out);
  }
  if (!out || first.points_size() < 2 || second.points_size() < 2 ||
      blend_radius <= 1e-9) {
    return false;
  }
  const automsgs::msgs::sensor_msgs::JointState seam =
      MakeJointStateFromPoint(first, first.points_size() - 1);
  if (JointDist(seam, MakeJointStateFromPoint(second, 0)) > 1e-3) {
    return false;
  }
  automsgs::msgs::geometry_msgs::Pose seam_tip;
  if (!kinematics->GetPositionFK(seam, &seam_tip)) {
    return BlendTransitionWindow(first, second, blend_radius, out);
  }

  auto tip_of = [&](const automsgs::msgs::sensor_msgs::JointState& js, automsgs::msgs::geometry_msgs::Pose* tip) {
    return kinematics->GetPositionFK(js, tip);
  };

  auto time_at = [](const automsgs::msgs::trajectory_msgs::JointTrajectory& tr, int i) {
    return GetTrajectoryPointTimeSeconds(tr, i);
  };

  int i1 = first.points_size() - 1;
  bool found1 = false;
  for (int k = first.points_size() - 1; k >= 0; --k) {
    automsgs::msgs::geometry_msgs::Pose tip;
    if (!tip_of(MakeJointStateFromPoint(first, k), &tip)) {
      continue;
    }
    if (TipDist(tip, seam_tip) >= blend_radius - 1e-9) {
      i1 = k;
      found1 = true;
      break;
    }
  }
  int i2 = 0;
  bool found2 = false;
  for (int k = 0; k < second.points_size(); ++k) {
    automsgs::msgs::geometry_msgs::Pose tip;
    if (!tip_of(MakeJointStateFromPoint(second, k), &tip)) {
      continue;
    }
    if (TipDist(tip, seam_tip) >= blend_radius - 1e-9) {
      i2 = k;
      found2 = true;
      break;
    }
  }
  if (!found1 || !found2 || i1 + 1 >= first.points_size() || i2 == 0) {
    return BlendTransitionWindow(first, second, blend_radius, out);
  }

  const int n1 = first.points_size() - i1;
  const int n2 = i2 + 1;
  const int n_blend = std::max(n1, n2);
  const double dt1 =
      (time_at(first, first.points_size() - 1) - time_at(first, i1)) /
      std::max(1.0, static_cast<double>(n1 - 1));
  const double dt2 =
      (time_at(second, i2) - time_at(second, 0)) /
      std::max(1.0, static_cast<double>(n2 - 1));
  const double dt = std::max(1e-3, 0.5 * (dt1 + dt2));

  automsgs::msgs::trajectory_msgs::JointTrajectory merged;
  for (int i = 0; i <= i1; ++i) {
    AppendTimedPointFromTraj(&merged, first, i, time_at(first, i));
  }
  double t = GetTrajectoryPointTimeSeconds(merged, merged.points_size() - 1);
  automsgs::msgs::sensor_msgs::JointState seed = MakeJointStateFromPoint(first, i1);

  for (int s = 1; s <= n_blend; ++s) {
    const double u = static_cast<double>(s) / static_cast<double>(n_blend);
    const double alpha = QuinticAlpha(u);
    const int ia = std::min(i1 + s, first.points_size() - 1);
    const int ib = std::min(s, i2);
    automsgs::msgs::geometry_msgs::Pose pa;
    automsgs::msgs::geometry_msgs::Pose pb;
    if (!tip_of(MakeJointStateFromPoint(first, ia), &pa) ||
        !tip_of(MakeJointStateFromPoint(second, ib), &pb)) {
      return BlendTransitionWindow(first, second, blend_radius, out);
    }
    const automsgs::msgs::geometry_msgs::Pose target = BlendPose(pa, pb, alpha);
    automsgs::msgs::sensor_msgs::JointState sol;
    common::InverseKinematicsOptions opts;
    opts.set_max_attempts(6);
    opts.set_timeout(0.02);
    if (kinematics->GetPositionIK(target, seed, opts, &sol) !=
        ErrorCode::SUCCESS) {
      return BlendTransitionWindow(first, second, blend_radius, out);
    }
    seed = sol;
    t += dt;
    AppendTimedPoint(&merged, sol, t);
  }

  const double t2_at_i2 = time_at(second, i2);
  for (int k = i2 + 1; k < second.points_size(); ++k) {
    const double tk = time_at(second, k);
    AppendTimedPointFromTraj(&merged, second, k, t + (tk - t2_at_i2));
  }
  for (int i = 1; i < merged.points_size(); ++i) {
    const double prev = GetTrajectoryPointTimeSeconds(merged, i - 1);
    const double cur = GetTrajectoryPointTimeSeconds(merged, i);
    if (cur < prev + 1e-6) {
      SetDurationSeconds(prev + 1e-6,
                         merged.mutable_points(i)->mutable_time_from_start());
    }
  }
  *out = std::move(merged);
  return true;
}

/**
 * @brief Asymmetric trapezoidal duration for a 1-D stroke (Pilz ATRAP lite).
 *
 * @param[in] distance Absolute stroke length.
 * @param[in] vmax Peak velocity (>0).
 * @param[in] amax Acceleration (>0).
 * @param[in] dmax Deceleration (>0); defaults to @p amax when ≤0.
 * @return Minimal duration from rest to rest.
 */
inline double AtrapDuration(double distance, double vmax, double amax,
                            double dmax = 0.0) {
  const double L = std::abs(distance);
  if (L < 1e-12) {
    return 0.0;
  }
  vmax = std::max(1e-6, vmax);
  amax = std::max(1e-6, amax);
  dmax = dmax > 1e-9 ? dmax : amax;
  // Distance to reach vmax under amax then stop under dmax.
  const double d_acc = 0.5 * vmax * vmax / amax;
  const double d_dec = 0.5 * vmax * vmax / dmax;
  if (d_acc + d_dec <= L) {
    const double t_acc = vmax / amax;
    const double t_dec = vmax / dmax;
    const double t_cruise = (L - d_acc - d_dec) / vmax;
    return t_acc + t_cruise + t_dec;
  }
  // Triangular: peak v* < vmax.
  // L = 0.5 v*^2 / a + 0.5 v*^2 / d  →  v* = sqrt(2 L / (1/a+1/d))
  const double v_peak = std::sqrt(2.0 * L / (1.0 / amax + 1.0 / dmax));
  return v_peak / amax + v_peak / dmax;
}

/**
 * @brief Sample normalized ATRAP position law s(τ)∈[0,1], τ∈[0,1].
 *
 * Rest-to-rest asymmetric trap mapped onto unit interval.
 */
inline double AtrapPositionLaw(double tau, double distance, double vmax,
                               double amax, double dmax = 0.0) {
  tau = std::clamp(tau, 0.0, 1.0);
  const double T = AtrapDuration(distance, vmax, amax, dmax);
  if (T < 1e-12 || std::abs(distance) < 1e-12) {
    return tau;
  }
  vmax = std::max(1e-6, vmax);
  amax = std::max(1e-6, amax);
  dmax = dmax > 1e-9 ? dmax : amax;
  const double t = tau * T;
  const double L = std::abs(distance);
  const double d_acc = 0.5 * vmax * vmax / amax;
  const double d_dec = 0.5 * vmax * vmax / dmax;
  double pos = 0.0;
  if (d_acc + d_dec <= L) {
    const double t_acc = vmax / amax;
    const double t_dec = vmax / dmax;
    const double t_cruise = (L - d_acc - d_dec) / vmax;
    if (t <= t_acc) {
      pos = 0.5 * amax * t * t;
    } else if (t <= t_acc + t_cruise) {
      pos = d_acc + vmax * (t - t_acc);
    } else {
      const double td = t - (t_acc + t_cruise);
      pos = L - 0.5 * dmax * (t_dec - td) * (t_dec - td);
    }
  } else {
    const double v_peak = std::sqrt(2.0 * L / (1.0 / amax + 1.0 / dmax));
    const double t_acc = v_peak / amax;
    if (t <= t_acc) {
      pos = 0.5 * amax * t * t;
    } else {
      const double td = t - t_acc;
      const double t_dec = v_peak / dmax;
      pos = L - 0.5 * dmax * (t_dec - td) * (t_dec - td);
    }
  }
  return std::clamp(pos / L, 0.0, 1.0);
}

/**
 * @brief Full ATRAP profile with Pos(t) and duration-phase sync (MoveIt ATrap).
 *
 * Used by Pilz PTP for leading-axis sync: slower joints stretch to match the
 * leading axis's (t_acc, t_cruise, t_dec) via @ref SetProfileAllDurations.
 */
struct AtrapProfile {
  double start = 0.0;
  double end = 0.0;
  double vmax = 1.0;
  double amax = 1.0;
  double dmax = 1.0;
  double t_a = 0.0;
  double t_b = 0.0;
  double t_c = 0.0;
  double a1 = 0.0, a2 = 0.0, a3 = 0.0;
  double b1 = 0.0, b2 = 0.0, b3 = 0.0;
  double c1 = 0.0, c2 = 0.0, c3 = 0.0;

  double Duration() const { return t_a + t_b + t_c; }

  void SetEmpty() {
    a1 = b1 = c1 = end;
    a2 = a3 = b2 = b3 = c2 = c3 = 0.0;
    t_a = t_b = t_c = 0.0;
  }

  void SetProfile(double pos1, double pos2, double max_vel, double max_acc,
                  double max_dec) {
    start = pos1;
    end = pos2;
    vmax = std::max(1e-9, std::abs(max_vel));
    amax = std::max(1e-9, std::abs(max_acc));
    dmax = std::max(1e-9, std::abs(max_dec));
    if (std::abs(end - start) < 1e-12) {
      SetEmpty();
      return;
    }
    const double s = (end > start) ? 1.0 : -1.0;
    const double dis = std::abs(end - start);
    const double min_dis =
        0.5 * vmax * vmax / amax + 0.5 * vmax * vmax / dmax;
    if (dis > min_dis) {
      a1 = start;
      a2 = 0.0;
      a3 = s * amax / 2.0;
      t_a = vmax / amax;
      b1 = a1 + a3 * t_a * t_a;
      b2 = s * vmax;
      b3 = 0.0;
      t_b = (dis - min_dis) / vmax;
      c1 = b1 + b2 * t_b;
      c2 = s * vmax;
      c3 = -s * dmax / 2.0;
      t_c = vmax / dmax;
    } else {
      const double v_peak = std::sqrt(2.0 * dis / (1.0 / amax + 1.0 / dmax));
      a1 = start;
      a2 = 0.0;
      a3 = s * amax / 2.0;
      t_a = v_peak / amax;
      b1 = a1 + a3 * t_a * t_a;
      b2 = s * v_peak;
      b3 = 0.0;
      t_b = 0.0;
      c1 = b1;
      c2 = s * v_peak;
      c3 = -s * dmax / 2.0;
      t_c = v_peak / dmax;
    }
  }

  /** Stretch to prescribed phase durations (MoveIt setProfileAllDurations). */
  bool SetProfileAllDurations(double pos1, double pos2, double d1, double d2,
                              double d3) {
    SetProfile(pos1, pos2, vmax, amax, dmax);
    if (d1 <= 1e-12 || d3 <= 1e-12) {
      return false;
    }
    const double T = d1 + d2 + d3;
    if (Duration() - T > 1e-9) {
      return false;  // cannot be faster than fastest profile
    }
    if (std::abs(pos2 - pos1) < 1e-12) {
      start = pos1;
      end = pos2;
      SetEmpty();
      return true;
    }
    const double s = (pos2 > pos1) ? 1.0 : -1.0;
    const double dis = std::abs(pos2 - pos1);
    const double new_vel = s * dis / (d2 + d1 / 2.0 + d3 / 2.0);
    const double new_acc = new_vel / d1;
    const double new_dec = -new_vel / d3;
    if (std::abs(new_vel) - vmax > 1e-9 || std::abs(new_acc) - amax > 1e-9 ||
        std::abs(new_dec) - dmax > 1e-9) {
      return false;
    }
    start = pos1;
    end = pos2;
    t_a = d1;
    t_b = d2;
    t_c = d3;
    a1 = start;
    a2 = 0.0;
    a3 = new_acc / 2.0;
    b1 = a1 + a3 * t_a * t_a;
    b2 = new_vel;
    b3 = 0.0;
    c1 = b1 + b2 * t_b;
    c2 = new_vel;
    c3 = new_dec / 2.0;
    return true;
  }

  double Pos(double time) const {
    if (time <= 0.0) {
      return start;
    }
    if (time < t_a) {
      return a1 + time * (a2 + a3 * time);
    }
    if (time < t_a + t_b) {
      const double tau = time - t_a;
      return b1 + tau * (b2 + b3 * tau);
    }
    if (time <= t_a + t_b + t_c + 1e-12) {
      const double tau = time - t_a - t_b;
      return c1 + tau * (c2 + c3 * tau);
    }
    return end;
  }
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
