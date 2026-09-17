/*
 * Copyright 2026 The Openbot Authors
 *
 * Kunz–Stilman TOTG with PathSegment curvature (linear + circular blends).
 */

#include "autonomy/manipulation/planner/pipeline/time_parameterization.hpp"

#include "autonomy/manipulation/common/joint_state_util.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "autonomy/manipulation/planner/pipeline/kunz_path.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace {

double JointVmax(const TimeParamOptions& options, std::size_t j) {
  if (j < options.max_velocity_vector.size() &&
      options.max_velocity_vector[j] > 0.0) {
    return options.max_velocity_vector[j];
  }
  return std::max(1e-6, options.max_velocity);
}

double JointAmax(const TimeParamOptions& options, std::size_t j) {
  if (j < options.max_acceleration_vector.size() &&
      options.max_acceleration_vector[j] > 0.0) {
    return options.max_acceleration_vector[j];
  }
  return std::max(1e-6, options.max_acceleration);
}

void WriteTimes(core::RobotTrajectory* traj, const std::vector<double>& times) {
  for (int i = 0; i < traj->points_size() &&
                   static_cast<std::size_t>(i) < times.size();
       ++i) {
    SetDurationSeconds(times[static_cast<std::size_t>(i)],
                       traj->mutable_points(i)->mutable_time_from_start());
  }
}

bool ApplyConstantVmax(core::RobotTrajectory* traj,
                       const TimeParamOptions& options) {
  const double vmax = std::max(1e-6, options.max_velocity);
  std::vector<double> times(static_cast<std::size_t>(traj->points_size()), 0.0);
  double t = 0.0;
  for (int i = 1; i < traj->points_size(); ++i) {
    const auto& a = traj->points(i - 1);
    const auto& b = traj->points(i);
    const int n = std::min(a.positions_size(), b.positions_size());
    double max_dq = 0.0;
    for (int j = 0; j < n; ++j) {
      max_dq = std::max(max_dq, std::abs(b.positions(j) - a.positions(j)));
    }
    t += max_dq / vmax;
    times[static_cast<std::size_t>(i)] = t;
  }
  WriteTimes(traj, times);
  return true;
}

/**
 * Discrete Kunz parameterization on a blended Path:
 * sample s, apply vmax/amax with curvature, forward/backward integrate.
 */
bool ApplyTotgPathSegment(core::RobotTrajectory* traj,
                          const TimeParamOptions& options) {
  const std::size_t n_wp = static_cast<std::size_t>(traj->points_size());
  const std::size_t dof =
      static_cast<std::size_t>(traj->points(0).positions_size());
  std::vector<kunz::Config> waypoints(n_wp);
  for (std::size_t i = 0; i < n_wp; ++i) {
    const auto& pt = traj->points(static_cast<int>(i));
    waypoints[i].assign(pt.positions().begin(), pt.positions().end());
    waypoints[i].resize(dof, 0.0);
  }

  const double max_dev =
      options.path_tolerance > 0.0 ? options.path_tolerance : 0.1;
  kunz::Path path = kunz::Path::Create(waypoints, max_dev);
  if (path.GetLength() < 1e-12) {
    WriteTimes(traj, std::vector<double>(n_wp, 0.0));
    return true;
  }

  std::vector<double> vmax(dof), amax(dof);
  for (std::size_t j = 0; j < dof; ++j) {
    vmax[j] = JointVmax(options, j);
    amax[j] = JointAmax(options, j);
  }

  // Prefer continuous Kunz switching-point integration.
  std::vector<kunz::Trajectory::Step> steps;
  const double dt = options.integration_time_step > 0.0
                        ? options.integration_time_step
                        : 0.001;
  if (kunz::Trajectory::Create(path, vmax, amax, dt, &steps) &&
      steps.size() >= 2) {
    std::vector<double> wp_s(n_wp, 0.0);
    for (std::size_t i = 0; i < n_wp; ++i) {
      double best_s = 0.0;
      double best_d = std::numeric_limits<double>::max();
      for (const auto& st : steps) {
        const kunz::Config q = path.GetConfig(st.path_pos);
        double d2 = 0.0;
        for (std::size_t j = 0; j < dof; ++j) {
          const double dq = q[j] - waypoints[i][j];
          d2 += dq * dq;
        }
        if (d2 < best_d) {
          best_d = d2;
          best_s = st.path_pos;
        }
      }
      wp_s[i] = best_s;
    }
    wp_s.front() = 0.0;
    wp_s.back() = path.GetLength();
    for (std::size_t i = 1; i < n_wp; ++i) {
      wp_s[i] = std::max(wp_s[i], wp_s[i - 1]);
    }
    std::vector<double> times(n_wp, 0.0);
    for (std::size_t i = 0; i < n_wp; ++i) {
      times[i] = kunz::Trajectory::TimeAtPathPos(steps, wp_s[i]);
    }
    WriteTimes(traj, times);
    return true;
  }

  // Discrete fallback (same curvature limits).
  constexpr double kDs = 0.01;
  std::vector<double> s_samples;
  s_samples.push_back(0.0);
  for (double s = kDs; s < path.GetLength(); s += kDs) {
    s_samples.push_back(s);
  }
  for (const auto& sp : path.SwitchingPoints()) {
    s_samples.push_back(sp.first);
  }
  s_samples.push_back(path.GetLength());
  std::sort(s_samples.begin(), s_samples.end());
  s_samples.erase(std::unique(s_samples.begin(), s_samples.end(),
                              [](double a, double b) {
                                return std::abs(a - b) < 1e-9;
                              }),
                  s_samples.end());

  const std::size_t n = s_samples.size();
  std::vector<double> v_lim(n, 0.0);
  std::vector<double> a_fwd_cap(n, options.max_acceleration);
  for (std::size_t i = 0; i < n; ++i) {
    const double s = s_samples[i];
    const kunz::Config tang = path.GetTangent(s);
    const kunz::Config curv = path.GetCurvature(s);
    const double vv = kunz::VelocityMaxPathVelocity(tang, vmax);
    const double va = kunz::AccelerationMaxPathVelocity(tang, curv, amax);
    v_lim[i] = std::min(vv, va);
    if (!std::isfinite(v_lim[i])) {
      v_lim[i] = vv;
    }
  }
  v_lim.front() = 0.0;
  v_lim.back() = 0.0;

  // Forward: v²[i+1] ≤ v²[i] + 2 a_path ds  with curvature-aware a_path.
  std::vector<double> v_fwd(n, 0.0);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = s_samples[i + 1] - s_samples[i];
    if (ds < 1e-12) {
      v_fwd[i + 1] = std::min(v_lim[i + 1], v_fwd[i]);
      continue;
    }
    const double s_mid = 0.5 * (s_samples[i] + s_samples[i + 1]);
    const kunz::Config tang = path.GetTangent(s_mid);
    const kunz::Config curv = path.GetCurvature(s_mid);
    double a_path = kunz::MinMaxPathAcceleration(tang, curv, v_fwd[i], amax,
                                                 true);
    if (!std::isfinite(a_path) || a_path < 1e-9) {
      a_path = options.max_acceleration;
    }
    const double v2 = v_fwd[i] * v_fwd[i] + 2.0 * a_path * ds;
    v_fwd[i + 1] = std::min(v_lim[i + 1], std::sqrt(std::max(0.0, v2)));
  }

  std::vector<double> v_bwd(n, 0.0);
  for (std::size_t k = n - 1; k > 0; --k) {
    const std::size_t i = k - 1;
    const double ds = s_samples[i + 1] - s_samples[i];
    if (ds < 1e-12) {
      v_bwd[i] = std::min(v_lim[i], v_bwd[i + 1]);
      continue;
    }
    const double s_mid = 0.5 * (s_samples[i] + s_samples[i + 1]);
    const kunz::Config tang = path.GetTangent(s_mid);
    const kunz::Config curv = path.GetCurvature(s_mid);
    double a_path = std::abs(
        kunz::MinMaxPathAcceleration(tang, curv, v_bwd[i + 1], amax, false));
    if (!std::isfinite(a_path) || a_path < 1e-9) {
      a_path = options.max_acceleration;
    }
    const double v2 = v_bwd[i + 1] * v_bwd[i + 1] + 2.0 * a_path * ds;
    v_bwd[i] = std::min(v_lim[i], std::sqrt(std::max(0.0, v2)));
  }

  std::vector<double> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = std::min({v_lim[i], v_fwd[i], v_bwd[i]});
  }

  // Map original waypoints onto path by nearest config, then interpolate time.
  std::vector<double> wp_s(n_wp, 0.0);
  for (std::size_t i = 0; i < n_wp; ++i) {
    double best_s = 0.0;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t k = 0; k < n; ++k) {
      const kunz::Config q = path.GetConfig(s_samples[k]);
      double d2 = 0.0;
      for (std::size_t j = 0; j < dof; ++j) {
        const double dq = q[j] - waypoints[i][j];
        d2 += dq * dq;
      }
      if (d2 < best_d) {
        best_d = d2;
        best_s = s_samples[k];
      }
    }
    wp_s[i] = best_s;
  }
  wp_s.front() = 0.0;
  wp_s.back() = path.GetLength();
  for (std::size_t i = 1; i < n_wp; ++i) {
    wp_s[i] = std::max(wp_s[i], wp_s[i - 1]);
  }

  // Build time(s) from samples.
  std::vector<double> t_s(n, 0.0);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double ds = s_samples[i + 1] - s_samples[i];
    const double v_avg = 0.5 * (v[i] + v[i + 1]);
    if (ds < 1e-12) {
      t_s[i + 1] = t_s[i];
      continue;
    }
    if (v_avg < 1e-9) {
      return ApplyConstantVmax(traj, options);
    }
    t_s[i + 1] = t_s[i] + ds / v_avg;
  }

  auto time_at_s = [&](double s_query) {
    if (s_query <= s_samples.front()) {
      return 0.0;
    }
    if (s_query >= s_samples.back()) {
      return t_s.back();
    }
    for (std::size_t i = 0; i + 1 < n; ++i) {
      if (s_query <= s_samples[i + 1]) {
        const double u =
            (s_query - s_samples[i]) /
            std::max(1e-12, s_samples[i + 1] - s_samples[i]);
        return t_s[i] + u * (t_s[i + 1] - t_s[i]);
      }
    }
    return t_s.back();
  };

  std::vector<double> times(n_wp, 0.0);
  for (std::size_t i = 0; i < n_wp; ++i) {
    times[i] = time_at_s(wp_s[i]);
  }
  WriteTimes(traj, times);
  return true;
}

}  // namespace

bool ApplyTotg(core::RobotTrajectory* traj, const TimeParamOptions& options) {
  if (!traj || traj->points_size() < 2) {
    return false;
  }
  if (traj->points(0).positions_size() == 0) {
    return false;
  }

  TimeParamOptions opts = options;
  opts.max_velocity = std::max(1e-6, opts.max_velocity);
  opts.max_acceleration = std::max(1e-6, opts.max_acceleration);
  for (double& v : opts.max_velocity_vector) {
    v = std::max(1e-6, v);
  }
  for (double& a : opts.max_acceleration_vector) {
    a = std::max(1e-6, a);
  }
  if (opts.path_tolerance <= 0.0) {
    opts.path_tolerance = 0.1;
  }
  if (opts.integration_time_step <= 0.0) {
    opts.integration_time_step = 0.001;
  }

  // Collapse near-duplicate consecutive waypoints (zero-length edges break TOTG).
  {
    core::RobotTrajectory compact;
    AddTrajectoryPoint(&compact, MakeJointStateFromPoint(*traj, 0), 0.0);
    for (int i = 1; i < traj->points_size(); ++i) {
      double d2 = 0.0;
      const auto& a = compact.points(compact.points_size() - 1);
      const auto& b = traj->points(i);
      const int n = std::min(a.positions_size(), b.positions_size());
      for (int j = 0; j < n; ++j) {
        const double dq = a.positions(j) - b.positions(j);
        d2 += dq * dq;
      }
      if (d2 > 1e-16) {
        AddTrajectoryPoint(&compact, MakeJointStateFromPoint(*traj, i), 0.0);
      }
    }
    if (compact.points_size() < 2) {
      WriteTimes(traj, std::vector<double>(
                           static_cast<std::size_t>(traj->points_size()), 0.0));
      return true;
    }
    if (compact.points_size() != traj->points_size()) {
      *traj = std::move(compact);
    }
  }

  bool ok = ApplyTotgPathSegment(traj, opts);
  if (!ok) {
    ok = ApplyConstantVmax(traj, opts);
  }
  if (!ok) {
    return false;
  }
  // Strictly non-decreasing timestamps (controllers reject regressions).
  std::vector<double> times(static_cast<std::size_t>(traj->points_size()), 0.0);
  for (int i = 0; i < traj->points_size(); ++i) {
    times[static_cast<std::size_t>(i)] = GetTrajectoryTime(*traj, i);
  }
  times.front() = 0.0;
  for (std::size_t i = 1; i < times.size(); ++i) {
    if (!std::isfinite(times[i]) || times[i] < times[i - 1]) {
      times[i] = times[i - 1] + 1e-4;
    } else if (times[i] <= times[i - 1]) {
      times[i] = times[i - 1] + 1e-6;
    }
  }
  WriteTimes(traj, times);
  return true;
}

}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
