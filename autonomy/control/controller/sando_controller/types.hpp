/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file types.hpp
 * @brief Planner mode, automsgs state, and one cubic piece.
 *
 * A flat ground state is an automsgs CartesianPoint: pose, twist, and
 * acceleration. Half-planes and tracked movers are protobuf messages in
 * sando_controller.proto, built from Vector3 and CartesianPoint. A polynomial
 * piece has no automsgs equivalent and stays a fixed coefficient array.
 */

#pragma once

#include <cmath>

#include <automsgs/msgs/moveit_msgs/cartesian_point.pb.h>

#include "autonomy/control/proto/sando_controller.pb.h"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Discrete mode of one control cycle.
 *
 * The mode selects whether the planner replans, spins in place, tracks the
 * polynomial, or slides the hover point away from a threat. It is stored on
 * the planner and is not published as a message.
 */
enum class Status {
  kYawing = 0,         ///< Align heading before translation. Replanning is suppressed.
  kTraveling = 1,      ///< Track the sampled polynomial. Replan while the tail is outside the goal ball.
  kGoalSeen = 2,       ///< The committed plan tail already lies inside the goal radius. Stop replanning.
  kGoalReached = 3,    ///< Terminal condition reported to the controller. Hover may still force a replan.
  kHoverAvoiding = 4,  ///< Hover is latched and the goal may be slid away from nearby occupied cells.
};

/**
 * @brief Flat ground state: world pose, world twist, and world acceleration.
 *
 * pose.position is the planar position. pose.orientation stores yaw.
 * velocity.linear is the world velocity and velocity.angular.z is the yaw rate.
 * acceleration.linear is the world acceleration. There is no altitude channel.
 */
using State = automsgs::msgs::moveit_msgs::CartesianPoint;

/**
 * @brief Closed half-plane whose normal is a geometry Vector3.
 */
using HalfPlane = proto::HalfPlane;

/**
 * @brief Tracked mover stored as a CartesianPoint plus an axis-aligned extent.
 */
using DynObstacle = proto::DynamicObstacle;

/**
 * @brief Yaw of a planar orientation, in radians.
 *
 * An unset quaternion (all zeros) is the identity heading, not a division by zero.
 * @param state Cartesian state.
 * @return Heading in (-pi, pi], or 0 when the orientation has not been set.
 */
inline double Yaw(const State& state) {
  const auto& orientation = state.pose().orientation();
  if (orientation.x() == 0.0 && orientation.y() == 0.0 && orientation.z() == 0.0 && orientation.w() == 0.0) {
    return 0.0;
  }
  return transform::tf2::getYaw(orientation);
}

/**
 * @brief Store a planar heading in pose.orientation.
 * @param state Cartesian state. Required.
 * @param yaw Heading, radians.
 */
inline void SetYaw(State* state, double yaw) {
  auto* orientation = state->mutable_pose()->mutable_orientation();
  orientation->set_x(0.0);
  orientation->set_y(0.0);
  orientation->set_z(std::sin(yaw * 0.5));
  orientation->set_w(std::cos(yaw * 0.5));
}

/**
 * @brief One cubic piece of the jerk-minimizing trajectory.
 *
 * The curve on the normalized interval u in [0, 1] is
 * p(u) = c0 + c1 u + c2 u^2 + c3 u^3, independently on x and y.
 * Physical derivatives divide by duration^k. Control points are a linear
 * image of the coefficient vector [c3, c2, c1, c0], not of c itself.
 * No automsgs message represents a monomial piece, so the coefficients stay
 * a fixed array.
 */
struct Piece {
  double duration{0.2};  ///< Piece duration T, seconds. Uniform across a time-layered MIQP.
  double coefficients[4][2]{};      ///< coefficients[power][axis], power 0..3, axis 0 = x and 1 = y.
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
