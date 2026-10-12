/*
 * Copyright 2026 The Openbot Authors
 *
 * Abstract PathSegment base for Kunz–Stilman blended paths.
 */

#pragma once

#include <memory>
#include <vector>

#include "automanip/pipeline/kunz_joint_configuration.hpp"

namespace automanip {
namespace trajectory {
namespace kunz {

class PathSegment {
 public:
  virtual ~PathSegment() = default;
  double GetLength() const { return length_; }
  double position = 0.0;  // absolute s offset on path
  virtual JointConfiguration GetConfiguration(double s) const = 0;
  virtual JointConfiguration GetTangent(double s) const = 0;
  virtual JointConfiguration GetCurvature(double s) const = 0;
  virtual std::vector<double> GetSwitchingPoints() const = 0;
  virtual std::unique_ptr<PathSegment> Clone() const = 0;

 protected:
  double length_ = 0.0;
};

}  // namespace kunz
}  // namespace trajectory
}  // namespace automanip
