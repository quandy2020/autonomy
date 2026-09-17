/*
 * Copyright 2026 The Openbot Authors
 *
 * Linear PathSegment for Kunz–Stilman blended paths.
 */

#pragma once

#include <memory>
#include <vector>

#include "autonomy/manipulation/pipeline/kunz_joint_configuration.hpp"
#include "autonomy/manipulation/pipeline/kunz_path_segment.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace kunz {

class LinearPathSegment : public PathSegment {
 public:
  LinearPathSegment(const JointConfiguration& start, const JointConfiguration& end) : end_(end) {
    JointConfiguration diff = Subtract(end, start);
    length_ = EuclideanNorm(diff);
    tangent_ = length_ > 1e-12 ? Scale(diff, 1.0 / length_) : JointConfiguration(start.size(), 0.0);
    start_ = start;
  }

  JointConfiguration GetConfiguration(double s) const override {
    return Add(start_, Scale(tangent_, s));
  }
  JointConfiguration GetTangent(double /*s*/) const override { return tangent_; }
  JointConfiguration GetCurvature(double /*s*/) const override {
    return JointConfiguration(start_.size(), 0.0);
  }
  std::vector<double> GetSwitchingPoints() const override { return {}; }
  std::unique_ptr<PathSegment> Clone() const override {
    return std::make_unique<LinearPathSegment>(*this);
  }

 private:
  JointConfiguration start_;
  JointConfiguration end_;
  JointConfiguration tangent_;
};

}  // namespace kunz
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
