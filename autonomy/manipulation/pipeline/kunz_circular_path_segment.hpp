/*
 * Copyright 2026 The Openbot Authors
 *
 * Circular blend PathSegment for Kunz–Stilman paths.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "autonomy/manipulation/pipeline/kunz_joint_configuration.hpp"
#include "autonomy/manipulation/pipeline/kunz_path_segment.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace kunz {

class CircularPathSegment : public PathSegment {
 public:
  CircularPathSegment(const JointConfiguration& start, const JointConfiguration& intersection,
                      const JointConfiguration& end, double max_deviation) {
    const JointConfiguration zero(start.size(), 0.0);
    if (EuclideanNorm(Subtract(intersection, start)) < 1e-6 ||
        EuclideanNorm(Subtract(end, intersection)) < 1e-6) {
      length_ = 0.0;
      radius_ = 1.0;
      center_ = intersection;
      x_ = zero;
      y_ = zero;
      return;
    }
    const JointConfiguration start_direction = Normalized(Subtract(intersection, start));
    const JointConfiguration end_direction = Normalized(Subtract(end, intersection));
    const double start_dot_end = DotProduct(start_direction, end_direction);
    if (start_dot_end > 0.999999 || start_dot_end < -0.999999) {
      length_ = 0.0;
      radius_ = 1.0;
      center_ = intersection;
      x_ = zero;
      y_ = zero;
      return;
    }
    const double angle = std::acos(std::clamp(start_dot_end, -1.0, 1.0));
    const double start_distance = EuclideanNorm(Subtract(start, intersection));
    const double end_distance = EuclideanNorm(Subtract(end, intersection));
    double distance = std::min(start_distance, end_distance);
    distance = std::min(
        distance, max_deviation * std::sin(0.5 * angle) /
                      std::max(1e-12, 1.0 - std::cos(0.5 * angle)));
    radius_ = distance / std::tan(0.5 * angle);
    length_ = angle * radius_;
    center_ = Add(
        intersection,
        Scale(Normalized(Subtract(end_direction, start_direction)),
              radius_ / std::cos(0.5 * angle)));
    x_ = Normalized(
        Subtract(Subtract(intersection, Scale(start_direction, distance)), center_));
    y_ = start_direction;
  }

  JointConfiguration GetConfiguration(double s) const override {
    const double angle = s / radius_;
    return Add(center_,
               Scale(Add(Scale(x_, std::cos(angle)), Scale(y_, std::sin(angle))),
                     radius_));
  }
  JointConfiguration GetTangent(double s) const override {
    const double angle = s / radius_;
    return Add(Scale(x_, -std::sin(angle)), Scale(y_, std::cos(angle)));
  }
  JointConfiguration GetCurvature(double s) const override {
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
  JointConfiguration center_;
  JointConfiguration x_;
  JointConfiguration y_;
};

}  // namespace kunz
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
