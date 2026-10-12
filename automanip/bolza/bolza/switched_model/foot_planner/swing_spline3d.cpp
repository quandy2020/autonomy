//
// Created by rgrandia on 03.08.21.
//

#include "bolza/switched_model/foot_planner/swing_spline3d.hpp"

namespace switched_model {

SwingSpline3d::SwingSpline3d(const SwingNode3d& start, const SwingNode3d& mid, const SwingNode3d& end)
    : SwingSpline3d(std::vector<SwingNode3d>{start, mid, end}) {}

SwingSpline3d::SwingSpline3d(const std::vector<SwingNode3d>& nodes) {
  std::vector<SwingNode> x;
  x.reserve(nodes.size());
  std::vector<SwingNode> y;
  y.reserve(nodes.size());
  std::vector<SwingNode> z;
  y.reserve(nodes.size());
  for (const auto& node : nodes) {
    x.push_back({node.time, node.position.x(), node.velocity.x()});
    y.push_back({node.time, node.position.y(), node.velocity.y()});
    z.push_back({node.time, node.position.z(), node.velocity.z()});
  }
  x_ = QuinticSwing(x);
  y_ = QuinticSwing(y);
  z_ = QuinticSwing(z);
}

vector3_t SwingSpline3d::Position(scalar_t time) const {
  return {x_.Position(time), y_.Position(time), z_.Position(time)};
}

vector3_t SwingSpline3d::Velocity(scalar_t time) const {
  return {x_.Velocity(time), y_.Velocity(time), z_.Velocity(time)};
}

vector3_t SwingSpline3d::Acceleration(scalar_t time) const {
  return {x_.Acceleration(time), y_.Acceleration(time), z_.Acceleration(time)};
}

vector3_t SwingSpline3d::Jerk(scalar_t time) const {
  return {x_.Jerk(time), y_.Jerk(time), z_.Jerk(time)};
}

}  // namespace switched_model
