/*
 * Copyright 2026 The Openbot Authors
 *
 * Kunz–Stilman PathSegment path (linear + circular blends) for TOTG.
 *
 * Adapted from MoveIt TimeOptimalTrajectoryGeneration Path (BSD).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace kunz {

/** @brief Configuration-space vector (joint positions). */
using Config = std::vector<double>;

inline double Norm(const Config& a) {
  double s = 0.0;
  for (double v : a) {
    s += v * v;
  }
  return std::sqrt(s);
}

inline Config Sub(const Config& a, const Config& b) {
  Config o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] - (i < b.size() ? b[i] : 0.0);
  }
  return o;
}

inline Config Add(const Config& a, const Config& b) {
  Config o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] + (i < b.size() ? b[i] : 0.0);
  }
  return o;
}

inline Config Scale(const Config& a, double s) {
  Config o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] * s;
  }
  return o;
}

inline Config Normalized(const Config& a) {
  const double n = Norm(a);
  return n > 1e-12 ? Scale(a, 1.0 / n) : Config(a.size(), 0.0);
}

inline double Dot(const Config& a, const Config& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
    s += a[i] * b[i];
  }
  return s;
}

inline Config Lerp(const Config& a, const Config& b, double t) {
  Config o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] + t * ((i < b.size() ? b[i] : 0.0) - a[i]);
  }
  return o;
}

class PathSegment {
 public:
  virtual ~PathSegment() = default;
  double GetLength() const { return length_; }
  double position = 0.0;  // absolute s offset on path
  virtual Config GetConfig(double s) const = 0;
  virtual Config GetTangent(double s) const = 0;
  virtual Config GetCurvature(double s) const = 0;
  virtual std::vector<double> GetSwitchingPoints() const = 0;
  virtual std::unique_ptr<PathSegment> Clone() const = 0;

 protected:
  double length_ = 0.0;
};

class LinearPathSegment : public PathSegment {
 public:
  LinearPathSegment(const Config& start, const Config& end) : end_(end) {
    Config diff = Sub(end, start);
    length_ = Norm(diff);
    tangent_ = length_ > 1e-12 ? Scale(diff, 1.0 / length_) : Config(start.size(), 0.0);
    start_ = start;
  }

  Config GetConfig(double s) const override {
    return Add(start_, Scale(tangent_, s));
  }
  Config GetTangent(double /*s*/) const override { return tangent_; }
  Config GetCurvature(double /*s*/) const override {
    return Config(start_.size(), 0.0);
  }
  std::vector<double> GetSwitchingPoints() const override { return {}; }
  std::unique_ptr<PathSegment> Clone() const override {
    return std::make_unique<LinearPathSegment>(*this);
  }

 private:
  Config start_;
  Config end_;
  Config tangent_;
};

class CircularPathSegment : public PathSegment {
 public:
  CircularPathSegment(const Config& start, const Config& intersection,
                      const Config& end, double max_deviation) {
    const Config zero(start.size(), 0.0);
    if (Norm(Sub(intersection, start)) < 1e-6 ||
        Norm(Sub(end, intersection)) < 1e-6) {
      length_ = 0.0;
      radius_ = 1.0;
      center_ = intersection;
      x_ = zero;
      y_ = zero;
      return;
    }
    const Config start_direction = Normalized(Sub(intersection, start));
    const Config end_direction = Normalized(Sub(end, intersection));
    const double start_dot_end = Dot(start_direction, end_direction);
    if (start_dot_end > 0.999999 || start_dot_end < -0.999999) {
      length_ = 0.0;
      radius_ = 1.0;
      center_ = intersection;
      x_ = zero;
      y_ = zero;
      return;
    }
    const double angle = std::acos(std::clamp(start_dot_end, -1.0, 1.0));
    const double start_distance = Norm(Sub(start, intersection));
    const double end_distance = Norm(Sub(end, intersection));
    double distance = std::min(start_distance, end_distance);
    distance = std::min(
        distance, max_deviation * std::sin(0.5 * angle) /
                      std::max(1e-12, 1.0 - std::cos(0.5 * angle)));
    radius_ = distance / std::tan(0.5 * angle);
    length_ = angle * radius_;
    center_ = Add(
        intersection,
        Scale(Normalized(Sub(end_direction, start_direction)),
              radius_ / std::cos(0.5 * angle)));
    x_ = Normalized(
        Sub(Sub(intersection, Scale(start_direction, distance)), center_));
    y_ = start_direction;
  }

  Config GetConfig(double s) const override {
    const double angle = s / radius_;
    return Add(center_,
               Scale(Add(Scale(x_, std::cos(angle)), Scale(y_, std::sin(angle))),
                     radius_));
  }
  Config GetTangent(double s) const override {
    const double angle = s / radius_;
    return Add(Scale(x_, -std::sin(angle)), Scale(y_, std::cos(angle)));
  }
  Config GetCurvature(double s) const override {
    const double angle = s / radius_;
    return Scale(Add(Scale(x_, std::cos(angle)), Scale(y_, std::sin(angle))),
                 -1.0 / radius_);
  }
  std::vector<double> GetSwitchingPoints() const override {
    std::vector<double> switching_points;
    for (std::size_t i = 0; i < x_.size(); ++i) {
      double switching_angle = std::atan2(y_[i], x_[i]);
      if (switching_angle < 0.0) {
        switching_angle += 3.141592653589793;
      }
      const double switching_point = switching_angle * radius_;
      if (switching_point < length_) {
        switching_points.push_back(switching_point);
      }
    }
    std::sort(switching_points.begin(), switching_points.end());
    return switching_points;
  }
  std::unique_ptr<PathSegment> Clone() const override {
    return std::make_unique<CircularPathSegment>(*this);
  }

 private:
  double radius_ = 1.0;
  Config center_;
  Config x_;
  Config y_;
};

/**
 * @brief Blended path: linear segments + circular blends at waypoints.
 */
class Path {
 public:
  static Path Create(const std::vector<Config>& waypoints,
                     double max_deviation) {
    Path path;
    if (waypoints.size() < 2 || max_deviation <= 0.0) {
      return path;
    }
    Config start_config = waypoints.front();
    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
      const Config& w1 = waypoints[i - 1];
      const Config& w2 = waypoints[i];
      const Config& w3 = waypoints[i + 1];
      auto blend = std::make_unique<CircularPathSegment>(
          Lerp(w1, w2, 0.5), w2, Lerp(w2, w3, 0.5), max_deviation);
      Config end_config = blend->GetConfig(0.0);
      if (Norm(Sub(end_config, start_config)) > 1e-6) {
        path.segments_.push_back(
            std::make_unique<LinearPathSegment>(start_config, end_config));
      }
      start_config = blend->GetConfig(blend->GetLength());
      path.segments_.push_back(std::move(blend));
    }
    path.segments_.push_back(std::make_unique<LinearPathSegment>(
        start_config, waypoints.back()));

    for (auto& seg : path.segments_) {
      seg->position = path.length_;
      for (double sp : seg->GetSwitchingPoints()) {
        path.switching_.emplace_back(path.length_ + sp, false);
      }
      path.length_ += seg->GetLength();
      while (!path.switching_.empty() &&
             path.switching_.back().first >= path.length_) {
        path.switching_.pop_back();
      }
      path.switching_.emplace_back(path.length_, true);
    }
    if (!path.switching_.empty()) {
      path.switching_.pop_back();
    }
    return path;
  }

  double GetLength() const { return length_; }

  PathSegment* GetPathSegment(double* s) const {
    if (segments_.empty()) {
      return nullptr;
    }
    for (std::size_t i = 0; i + 1 < segments_.size(); ++i) {
      if (*s < segments_[i + 1]->position) {
        *s -= segments_[i]->position;
        return segments_[i].get();
      }
    }
    *s -= segments_.back()->position;
    return segments_.back().get();
  }

  Config GetConfig(double s) const {
    PathSegment* seg = GetPathSegment(&s);
    return seg ? seg->GetConfig(s) : Config{};
  }
  Config GetTangent(double s) const {
    PathSegment* seg = GetPathSegment(&s);
    return seg ? seg->GetTangent(s) : Config{};
  }
  Config GetCurvature(double s) const {
    PathSegment* seg = GetPathSegment(&s);
    return seg ? seg->GetCurvature(s) : Config{};
  }

  const std::vector<std::pair<double, bool>>& SwitchingPoints() const {
    return switching_;
  }

 private:
  double length_ = 0.0;
  std::vector<std::unique_ptr<PathSegment>> segments_;
  std::vector<std::pair<double, bool>> switching_;
};

/** Path velocity limit from joint vmax / |q'(s)|. */
inline double VelocityMaxPathVelocity(const Config& tangent,
                                      const std::vector<double>& vmax) {
  double lim = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tangent.size() && i < vmax.size(); ++i) {
    const double t = std::abs(tangent[i]);
    if (t > 1e-12) {
      lim = std::min(lim, vmax[i] / t);
    }
  }
  return lim;
}

/**
 * Acceleration-limited path velocity (curvature term), MoveIt analogue of
 * getAccelerationMaxPathVelocity.
 */
inline double AccelerationMaxPathVelocity(const Config& tangent,
                                          const Config& curvature,
                                          const std::vector<double>& amax) {
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
                std::sqrt((amax[i] / std::abs(tangent[i]) +
                           amax[j] / std::abs(tangent[j])) /
                          std::abs(a_ij)));
          }
        }
      }
    } else if (std::abs(curvature[i]) > 1e-12) {
      max_path_velocity =
          std::min(max_path_velocity, std::sqrt(amax[i] / std::abs(curvature[i])));
    }
  }
  return max_path_velocity;
}

/**
 * Min/max path acceleration with curvature: a = ±(amax/|q'| − q'' v² / q').
 */
inline double MinMaxPathAcceleration(const Config& tangent,
                                     const Config& curvature, double path_vel,
                                     const std::vector<double>& amax,
                                     bool max) {
  const double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tangent.size() && i < amax.size(); ++i) {
    if (std::abs(tangent[i]) > 1e-12) {
      max_path_acceleration = std::min(
          max_path_acceleration,
          amax[i] / std::abs(tangent[i]) -
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

  static bool Create(const Path& path, const std::vector<double>& vmax,
                     const std::vector<double>& amax, double time_step,
                     std::vector<Step>* out) {
    if (!out || path.GetLength() < 1e-12 || time_step <= 0.0) {
      return false;
    }
    Trajectory traj(path, vmax, amax, time_step);
    traj.steps_.push_back({0.0, 0.0, 0.0});
    double after_acc = traj.MinMaxPathAcceleration(0.0, 0.0, true);
    int guard = 0;
    while (!traj.IntegrateForward(after_acc) && traj.valid_ && guard++ < 10000) {
      double before_acc = 0.0;
      Step sp;
      if (traj.NextSwitchingPoint(traj.steps_.back().path_pos, &sp, &before_acc,
                                  &after_acc)) {
        break;
      }
      traj.IntegrateBackward(sp.path_pos, sp.path_vel, before_acc);
    }
    if (!traj.valid_) {
      return false;
    }
    const double before_end =
        traj.MinMaxPathAcceleration(path.GetLength(), 0.0, false);
    traj.IntegrateBackward(path.GetLength(), 0.0, before_end);
    if (!traj.valid_ || traj.steps_.size() < 2) {
      return false;
    }
    // Assign times.
    traj.steps_.front().time = 0.0;
    for (std::size_t i = 1; i < traj.steps_.size(); ++i) {
      const double ds =
          traj.steps_[i].path_pos - traj.steps_[i - 1].path_pos;
      const double vavg =
          0.5 * (traj.steps_[i].path_vel + traj.steps_[i - 1].path_vel);
      traj.steps_[i].time =
          traj.steps_[i - 1].time +
          (vavg > 1e-12 ? ds / vavg : 0.0);
    }
    *out = std::move(traj.steps_);
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
  Trajectory(const Path& path, const std::vector<double>& vmax,
             const std::vector<double>& amax, double time_step)
      : path_(path), vmax_(vmax), amax_(amax), time_step_(time_step) {}

  double VelocityMax(double s) const {
    return VelocityMaxPathVelocity(path_.GetTangent(s), vmax_);
  }
  double AccelerationMax(double s) const {
    return AccelerationMaxPathVelocity(path_.GetTangent(s),
                                       path_.GetCurvature(s), amax_);
  }
  double MinMaxPathAcceleration(double s, double v, bool max) const {
    return kunz::MinMaxPathAcceleration(path_.GetTangent(s),
                                        path_.GetCurvature(s), v, amax_, max);
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
  std::vector<double> vmax_;
  std::vector<double> amax_;
  double time_step_ = 0.001;
  std::vector<Step> steps_;
  bool valid_ = true;
};

}  // namespace kunz
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
