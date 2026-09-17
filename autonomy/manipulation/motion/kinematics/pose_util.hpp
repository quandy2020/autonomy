/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file pose_util.hpp
 * @brief geometry_msgs/Pose helpers (shared by cartesian / pilz).
 */

#pragma once

#include <cmath>

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/common/msg_types.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {

/**
 * @brief Linearly interpolate translation and hemisphere-aware quaternion blend.
 * @param[in] a Start pose.
 * @param[in] b End pose.
 * @param[in] t Factor in [0, 1].
 * @return Interpolated pose (normalized orientation).
 */
inline Pose InterpolatePose(const Pose& a, const Pose& b, double t) {
  Pose out;
  const double u = 1.0 - t;
  out.mutable_position()->set_x(u * a.position().x() + t * b.position().x());
  out.mutable_position()->set_y(u * a.position().y() + t * b.position().y());
  out.mutable_position()->set_z(u * a.position().z() + t * b.position().z());
  // Hemisphere-aware linear blend then normalize (good enough for short arcs).
  double dot = a.orientation().w() * b.orientation().w() +
               a.orientation().x() * b.orientation().x() +
               a.orientation().y() * b.orientation().y() +
               a.orientation().z() * b.orientation().z();
  const double s = dot < 0.0 ? -1.0 : 1.0;
  out.mutable_orientation()->set_w(u * a.orientation().w() +
                                   t * s * b.orientation().w());
  out.mutable_orientation()->set_x(u * a.orientation().x() +
                                   t * s * b.orientation().x());
  out.mutable_orientation()->set_y(u * a.orientation().y() +
                                   t * s * b.orientation().y());
  out.mutable_orientation()->set_z(u * a.orientation().z() +
                                   t * s * b.orientation().z());
  const double n = std::sqrt(
      out.orientation().w() * out.orientation().w() +
      out.orientation().x() * out.orientation().x() +
      out.orientation().y() * out.orientation().y() +
      out.orientation().z() * out.orientation().z());
  if (n > 1e-12) {
    out.mutable_orientation()->set_w(out.orientation().w() / n);
    out.mutable_orientation()->set_x(out.orientation().x() / n);
    out.mutable_orientation()->set_y(out.orientation().y() / n);
    out.mutable_orientation()->set_z(out.orientation().z() / n);
  }
  return out;
}

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
