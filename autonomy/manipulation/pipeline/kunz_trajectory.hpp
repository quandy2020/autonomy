/*
 * Copyright 2026 The Openbot Authors
 *
 * Continuous Kunz–Stilman trajectory on a Path (MoveIt Trajectory lite).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "autonomy/manipulation/pipeline/kunz_joint_configuration.hpp"
#include "autonomy/manipulation/pipeline/kunz_path.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace kunz {

/** Path velocity limit from joint maximum velocity / |q'(s)|. */
inline double VelocityMaxPathVelocity(const JointConfiguration& tangent,
                                      const std::vector<double>& max_velocities) {
  double lim = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tangent.size() && i < max_velocities.size(); ++i) {
    const double t = std::abs(tangent[i]);
    if (t > 1e-12) {
      lim = std::min(lim, max_velocities[i] / t);
    }
  }
  return lim;
}

/**
 * Acceleration-limited path velocity (curvature term), MoveIt analogue of
 * getAccelerationMaxPathVelocity.
 */
inline double AccelerationMaxPathVelocity(const JointConfiguration& tangent,
                                          const JointConfiguration& curvature,
                                          const std::vector<double>& max_accelerations) {
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const std::size_t n = tangent.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (std::abs(tangent[i]) > 1e-12) {
      for (std::size_t j = i + 1; j < n; ++j) {
        if (std::abs(tangent[j]) > 1e-12) {
          const double a_ij =
              curvature[i] / tangent[i] - curvature[j] / tangent[j];
          if (std::abs(a_ij) > 1e-12) {
            max_path_velocity = std::min(
                max_path_velocity,
                std::sqrt((max_accelerations[i] / std::abs(tangent[i]) +
                           max_accelerations[j] / std::abs(tangent[j])) /
                          std::abs(a_ij)));
          }
        }
      }
    } else if (std::abs(curvature[i]) > 1e-12) {
      max_path_velocity =
          std::min(max_path_velocity, std::sqrt(max_accelerations[i] / std::abs(curvature[i])));
    }
  }
  return max_path_velocity;
}

/**
 * Min/max path acceleration with curvature: a = ±(a_max/|q'| − q'' v² / q').
 */
inline double MinMaxPathAcceleration(const JointConfiguration& tangent,
                                     const JointConfiguration& curvature, double path_vel,
                                     const std::vector<double>& max_accelerations,
                                     bool max) {
  const double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tangent.size() && i < max_accelerations.size(); ++i) {
    if (std::abs(tangent[i]) > 1e-12) {
      max_path_acceleration = std::min(
          max_path_acceleration,
          max_accelerations[i] / std::abs(tangent[i]) -
              factor * curvature[i] * path_vel * path_vel / tangent[i]);
    }
  }
  return factor * max_path_acceleration;
}

/**
 * Continuous Kunz–Stilman trajectory on a Path (MoveIt Trajectory lite).
 *
 * Integrates path velocity with switching-point aware forward/backward passes.
 */
class Trajectory {
 public:
  struct Step {
    double path_pos = 0.0;
    double path_vel = 0.0;
    double time = 0.0;
  };

  static bool Create(const Path& path, const std::vector<double>& max_velocities,
                     const std::vector<double>& max_accelerations, double time_step,
                     std::vector<Step>* out) {
    if (!out || path.GetLength() < 1e-12 || time_step <= 0.0) {
      return false;
    }
    Trajectory trajectory(path, max_velocities, max_accelerations, time_step);
    trajectory.steps_.push_back({0.0, 0.0, 0.0});
    double after_acc = trajectory.MinMaxPathAcceleration(0.0, 0.0, true);
    int guard = 0;
    while (!trajectory.IntegrateForward(after_acc) && trajectory.valid_ && guard++ < 10000) {
      double before_acc = 0.0;
      Step sp;
      if (trajectory.NextSwitchingPoint(trajectory.steps_.back().path_pos, &sp, &before_acc,
                                  &after_acc)) {
        break;
      }
      trajectory.IntegrateBackward(sp.path_pos, sp.path_vel, before_acc);
    }
    if (!trajectory.valid_) {
      return false;
    }
    const double before_end =
        trajectory.MinMaxPathAcceleration(path.GetLength(), 0.0, false);
    trajectory.IntegrateBackward(path.GetLength(), 0.0, before_end);
    if (!trajectory.valid_ || trajectory.steps_.size() < 2) {
      return false;
    }
    // Assign times.
    trajectory.steps_.front().time = 0.0;
    for (std::size_t i = 1; i < trajectory.steps_.size(); ++i) {
      const double ds =
          trajectory.steps_[i].path_pos - trajectory.steps_[i - 1].path_pos;
      const double vavg =
          0.5 * (trajectory.steps_[i].path_vel + trajectory.steps_[i - 1].path_vel);
      trajectory.steps_[i].time =
          trajectory.steps_[i - 1].time +
          (vavg > 1e-12 ? ds / vavg : 0.0);
    }
    *out = std::move(trajectory.steps_);
    return true;
  }

  static double TimeAtPathPos(const std::vector<Step>& steps, double s) {
    if (steps.empty()) {
      return 0.0;
    }
    if (s <= steps.front().path_pos) {
      return steps.front().time;
    }
    if (s >= steps.back().path_pos) {
      return steps.back().time;
    }
    for (std::size_t i = 0; i + 1 < steps.size(); ++i) {
      if (s <= steps[i + 1].path_pos) {
        const double u =
            (s - steps[i].path_pos) /
            std::max(1e-12, steps[i + 1].path_pos - steps[i].path_pos);
        return steps[i].time + u * (steps[i + 1].time - steps[i].time);
      }
    }
    return steps.back().time;
  }

 private:
  Trajectory(const Path& path, const std::vector<double>& max_velocities,
             const std::vector<double>& max_accelerations, double time_step)
      : path_(path), max_velocities_(max_velocities), max_accelerations_(max_accelerations), time_step_(time_step) {}

  double VelocityMax(double s) const {
    return VelocityMaxPathVelocity(path_.GetTangent(s), max_velocities_);
  }
  double AccelerationMax(double s) const {
    return AccelerationMaxPathVelocity(path_.GetTangent(s),
                                       path_.GetCurvature(s), max_accelerations_);
  }
  double MinMaxPathAcceleration(double s, double v, bool max) const {
    return kunz::MinMaxPathAcceleration(path_.GetTangent(s),
                                        path_.GetCurvature(s), v, max_accelerations_, max);
  }
  double PathVelocityLimit(double s) const {
    return std::min(VelocityMax(s), AccelerationMax(s));
  }

  bool NextSwitchingPoint(double path_pos, Step* next, double* before_acc,
                          double* after_acc) const {
    // Scan path switching points for acceleration discontinuity.
    for (const auto& sp : path_.SwitchingPoints()) {
      if (sp.first <= path_pos + 1e-9) {
        continue;
      }
      if (sp.first >= path_.GetLength() - 1e-9) {
        return true;
      }
      const double s = sp.first;
      const double vlim = PathVelocityLimit(s);
      *before_acc = MinMaxPathAcceleration(s - 1e-6, vlim, false);
      *after_acc = MinMaxPathAcceleration(s + 1e-6, vlim, true);
      next->path_pos = s;
      next->path_vel = vlim;
      return false;
    }
    return true;
  }

  bool IntegrateForward(double acceleration) {
    double path_pos = steps_.back().path_pos;
    double path_vel = steps_.back().path_vel;
    const auto& switches = path_.SwitchingPoints();
    std::size_t next_disc = 0;
    while (next_disc < switches.size() &&
           (switches[next_disc].first <= path_pos || !switches[next_disc].second)) {
      ++next_disc;
    }
    for (int iter = 0; iter < 100000; ++iter) {
      const double old_pos = path_pos;
      const double old_vel = path_vel;
      path_vel += time_step_ * acceleration;
      path_pos += time_step_ * 0.5 * (old_vel + path_vel);
      if (next_disc < switches.size() && path_pos > switches[next_disc].first) {
        if (path_pos - switches[next_disc].first < 1e-9) {
          continue;
        }
        path_vel = old_vel +
                   (switches[next_disc].first - old_pos) * (path_vel - old_vel) /
                       std::max(1e-12, path_pos - old_pos);
        path_pos = switches[next_disc].first;
      }
      if (path_pos >= path_.GetLength()) {
        steps_.push_back({path_.GetLength(),
                          std::max(0.0, path_vel -
                                            (path_pos - path_.GetLength()) *
                                                acceleration / std::max(1e-9, path_vel)),
                          0.0});
        return true;
      }
      if (path_vel < 0.0) {
        valid_ = false;
        return true;
      }
      const double vlim = PathVelocityLimit(path_pos);
      if (path_vel > vlim) {
        path_vel = vlim;
      }
      steps_.push_back({path_pos, path_vel, 0.0});
      acceleration = MinMaxPathAcceleration(path_pos, path_vel, true);
      if (path_vel <= 1e-12 && std::abs(acceleration) < 1e-12) {
        valid_ = false;
        return true;
      }
      if (path_vel > PathVelocityLimit(path_pos) + 1e-6) {
        return false;
      }
    }
    valid_ = false;
    return true;
  }

  void IntegrateBackward(double path_pos, double path_vel, double acceleration) {
    std::vector<Step> bwd;
    bwd.push_back({path_pos, path_vel, 0.0});
    for (int iter = 0; iter < 100000; ++iter) {
      const double old_pos = path_pos;
      const double old_vel = path_vel;
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (old_vel + path_vel);
      if (path_vel < 0.0) {
        valid_ = false;
        return;
      }
      if (path_pos <= 0.0) {
        bwd.push_back({0.0, 0.0, 0.0});
        break;
      }
      bwd.push_back({path_pos, path_vel, 0.0});
      acceleration = std::abs(MinMaxPathAcceleration(path_pos, path_vel, false));
      // Intersection with forward trajectory.
      for (std::size_t i = 0; i + 1 < steps_.size(); ++i) {
        if (steps_[i].path_pos <= path_pos &&
            path_pos <= steps_[i + 1].path_pos) {
          // Splice: keep forward up to i, append reversed bwd.
          steps_.resize(i + 1);
          for (auto it = bwd.rbegin(); it != bwd.rend(); ++it) {
            if (it->path_pos > steps_.back().path_pos + 1e-12) {
              steps_.push_back(*it);
            }
          }
          return;
        }
      }
      if (path_pos < steps_.front().path_pos) {
        steps_.clear();
        for (auto it = bwd.rbegin(); it != bwd.rend(); ++it) {
          steps_.push_back(*it);
        }
        return;
      }
    }
  }

  const Path& path_;
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;
  double time_step_ = 0.001;
  std::vector<Step> steps_;
  bool valid_ = true;
};

}  // namespace kunz
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
