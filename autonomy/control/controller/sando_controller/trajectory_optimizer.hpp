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
 * @file trajectory_optimizer.hpp
 * @brief Request, optimizer entry, sampling, and limit test for one local trajectory.
 *
 * When time_layered is non-empty the request is a mixed-integer program: each
 * piece must lie in at least one polytope of its time layer. Otherwise the
 * request falls back to a penalized quadratic program on a single corridor
 * per segment. Both paths return cubic pieces. Sampling and the limit test
 * are shared.
 */

#pragma once

#include <string>
#include <vector>

#include "Eigen/Dense"
#include "autonomy/control/controller/sando_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Inputs of one trajectory optimization.
 *
 * The geometric path supplies the segment endpoints. corridors is the legacy
 * one-polytope-per-segment description used by the penalized solver.
 * time_layered replaces it with N time layers of P spatial polytopes. The
 * initial state is a position, velocity, and acceleration in the world frame.
 */
struct TrajRequest {
  std::vector<Eigen::Vector2d> path;  ///< Spatial knots, world meters. Segment i joins path[i] and path[i + 1].
  std::vector<std::vector<HalfPlane>> corridors;  ///< Legacy corridor, one polytope per segment. Ignored when time_layered is set.
  State initial;  ///< Initial world pose, twist, and acceleration. Fixed as equalities on the first piece.
  bool stop_at_end{false};  ///< When true, the last piece ends at zero velocity and acceleration.
  double maximum_velocity{1.0};        ///< Speed bound applied to velocity control points, m/s.
  double maximum_acceleration{2.0};        ///< Acceleration bound applied to acceleration control points, m/s^2.
  double maximum_jerk{5.0};        ///< Jerk bound. Jerk is the constant 6 c3 / T^3, m/s^3.
  double jerk_weight{10.0};  ///< Weight on integrated squared jerk. The Hessian on c3 is weight * 72 / T^5.
  double factor{1.5};         ///< Duration multiplier. Piece duration is factor * max(initial duration, 2 * control period).
  std::string dynamic_constraint{"Linf"};  ///< "Linf", "L1", or "L2". Anything else is treated as L-infinity.
  std::vector<std::vector<std::vector<HalfPlane>>> time_layered;  ///< [time layer][spatial polytope][half-plane].
  double uniform_piece_duration{0.0};          ///< Shared piece duration, seconds, used by the layered MIQP.
  double time_limit_seconds{0.0};  ///< Solver time limit, seconds. Zero leaves the solver default.
};

/**
 * @brief Solve the request and write one cubic piece per segment.
 *
 * A non-empty time_layered dispatches to the MIQP. Otherwise a penalized
 * least-squares iteration enforces continuity, the initial state, the
 * corridor, and the dynamic bounds. The penalized path uses eight iterations,
 * equality weight 1e5, and inequality weight 4e3.
 *
 * @param request Problem data. path must contain at least two points.
 * @param pieces Output pieces, one per segment. Cleared and filled on success.
 * @return False when the path is too short or the solver reports failure.
 */
/**
 * @brief Min-jerk trajectory: layered MIQP when a corridor is present, otherwise a penalized quadratic program.
 */
class TrajectoryOptimizer {
 public:
  bool Optimize(const TrajRequest& request, std::vector<Piece>* pieces) const;

/**
 * @brief Sample a piecewise cubic at a fixed period.
 *
 * start_time is ignored: the first sample is the start of the first piece. Position
 * uses the normalized monomial. Velocity, acceleration, and jerk divide by
 * duration, duration^2, and duration^3. Samples include every knot and the
 * interior points spaced by sample_period.
 *
 * @param pieces Cubic pieces in order.
 * @param sample_period Sample period, seconds. Non-positive values fall back to the piece duration.
 * @param start_time Unused. Kept so the call matches the planner's clock argument.
 * @param samples Output states. Cleared and filled. Yaw stays 0.
 */
  void Sample(const std::vector<Piece>& pieces, double sample_period, double start_time,
                        std::vector<State>* samples) const;

/**
 * @brief True when every velocity, acceleration, and jerk control point satisfies the norm.
 *
 * The test uses SatisfiesNormBound, so a five percent slack is allowed. Jerk
 * is tested as the single constant of each piece, not as a control polygon.
 *
 * @param pieces Candidate trajectory.
 * @param maximum_velocity Speed bound, m/s.
 * @param maximum_acceleration Acceleration bound, m/s^2.
 * @param maximum_jerk Jerk bound, m/s^3.
 * @param dynamic_constraint "L1", "L2", or L-infinity.
 * @return False when any control point exceeds the slackened bound.
 */
  bool IsWithinLimits(const std::vector<Piece>& pieces, double maximum_velocity,
                                double maximum_acceleration, double maximum_jerk,
                                const std::string& dynamic_constraint) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
