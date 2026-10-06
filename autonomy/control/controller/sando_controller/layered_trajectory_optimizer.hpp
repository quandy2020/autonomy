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
 * @file layered_trajectory_optimizer.hpp
 * @brief Time-layered min-jerk MIQP encoded for the DAQP branch-and-bound solver.
 *
 * Decision variables are the cubic coefficients of every piece, then one
 * binary z_{t,p} per polytope of each time layer. The binary sum over p is
 * at least one, so the piece must pick at least one polytope. A face
 * n·q + M z <= c + M is inactive when z = 1 and enforces the half-plane when
 * z = 0. M is the maximum of n·corner - c over the four corners of a box
 * around the path, and the four position control points are constrained
 * into that same box, so the big-M is exact inside the box. An empty polytope
 * fixes its binary to zero. A small diagonal ridge keeps the Hessian positive
 * definite, which DAQP requires before it will branch.
 */

#pragma once

#include <vector>

#include "autonomy/control/controller/sando_controller/trajectory_optimizer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Solve a time-layered corridor as a convex mixed-integer quadratic program.
 *
 * The objective is integrated squared jerk, with Hessian jerk_weight * 72 / T^5
 * on each c3 and zeros elsewhere before regularization. Continuity of position,
 * velocity, and acceleration is an equality between adjacent pieces. The
 * initial state is an equality. stop_at_end adds zero terminal velocity and
 * acceleration. Dynamic limits are control-point inequalities.
 *
 * @param request Must contain a non-empty time_layered corridor and a positive uniform_piece_duration.
 * @param pieces Output cubics, one per spatial segment. Filled only on success.
 * @return False when the corridor is empty, a layer has no usable polytope, or the MIQP is not optimal.
 */
/**
 * @brief Encodes and solves the time-layered min-jerk mixed-integer program.
 */
class LayeredTrajectoryOptimizer {
 public:
  bool Optimize(const TrajRequest& request, std::vector<Piece>* pieces) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
