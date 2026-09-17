/*
 * Copyright 2026 The Openbot Authors
 *
 * Joint configuration typedef and free math helpers for Kunz–Stilman path.
 */

#pragma once

#include <cmath>
#include <vector>

namespace autonomy {
namespace manipulation {
namespace trajectory {
namespace kunz {

/** @brief Configuration-space vector (joint positions). */
using JointConfiguration = std::vector<double>;

inline double EuclideanNorm(const JointConfiguration& a) {
  double s = 0.0;
  for (double v : a) {
    s += v * v;
  }
  return std::sqrt(s);
}

inline JointConfiguration Subtract(const JointConfiguration& a, const JointConfiguration& b) {
  JointConfiguration o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] - (i < b.size() ? b[i] : 0.0);
  }
  return o;
}

inline JointConfiguration Add(const JointConfiguration& a, const JointConfiguration& b) {
  JointConfiguration o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] + (i < b.size() ? b[i] : 0.0);
  }
  return o;
}

inline JointConfiguration Scale(const JointConfiguration& a, double s) {
  JointConfiguration o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] * s;
  }
  return o;
}

inline JointConfiguration Normalized(const JointConfiguration& a) {
  const double n = EuclideanNorm(a);
  return n > 1e-12 ? Scale(a, 1.0 / n) : JointConfiguration(a.size(), 0.0);
}

inline double DotProduct(const JointConfiguration& a, const JointConfiguration& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
    s += a[i] * b[i];
  }
  return s;
}

inline JointConfiguration LinearInterpolate(const JointConfiguration& a, const JointConfiguration& b, double t) {
  JointConfiguration o(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    o[i] = a[i] + t * ((i < b.size() ? b[i] : 0.0) - a[i]);
  }
  return o;
}

}  // namespace kunz
}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
