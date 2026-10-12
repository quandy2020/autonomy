//
// Created by rgrandia on 21.01.21.
//

#pragma once

/**
 * @file quintic_spline_swing.hpp
 * @brief Implements a quintic polynomial:.
 */


#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

struct SwingNode {
  scalar_t time;
  scalar_t position;
  scalar_t velocity;
};

/**
 * Implements a quintic polynomial:
 *  y = c_5 * tau^5 + ... + c_1 * tau + c_0,
 *  where tau = (t - t0) / dt
 */
class QuinticSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor creates a flat spline, y(t) = 0.0 for t in [-1e30, 1e30].
   * @param [in] e30: E30.
   */
  QuinticSpline() : c_(vector6_t::Zero()), t0_(-1e30), dt_(2e30) {}

  /**
   * Constructs a quintic spline. See class description for the definition.
   *
   * @param [in] coefficients: spline coefficients [c_5, ..., c_0]
   * @param [in] t0: start time of the spline.
   * @param [in] dt: time normalization factor. The normalized time is 1.0 at t = t0 + dt
   */
  QuinticSpline(const vector6_t& coefficients, scalar_t t0, scalar_t dt) : c_(coefficients), t0_(t0), dt_(dt) {}

  /**
   * @brief returns y(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Position(scalar_t time) const;

  /**
   * @brief returns dy/dt(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Velocity(scalar_t time) const;

  /**
   * @brief returns d2y/dt2(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Acceleration(scalar_t time) const;

  /**
   * @brief returns d3y/dt3(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Jerk(scalar_t time) const;

 private:
  /**
   * @brief Normalized Time.
   * @param [in] t: T.
   * @return Result value.
   */
  scalar_t NormalizedTime(scalar_t t) const;

  /// Coefficients [c_5, ..., c_0] for normalized time
  vector6_t c_;

  /// Start time of the spline
  scalar_t t0_;

  /// Spline time normalization
  scalar_t dt_;
};

/**
 * Implements a swing trajectory based on (at least 2) quintic splines
 * The trajectory is made with the following conditions
 *  start:
 *      position = given
 *      velocity = given
 *      acceleration = 0.0
 *  Mid (can be multiple):
 *      position = given & continuous
 *      velocity = given & continuous
 *      acceleration = continuous
 *      jerk = continuous
 *  End:
 *      position = given
 *      velocity = given
 *      acceleration = 0.0
 */
class QuinticSwing {
 public:
  /**
   * trajectory of constant zero
   */
  QuinticSwing();

  /**
   * Construct a swing trajectory with given boundary conditions
   *
   * @param [in] start: starting time and boundary conditions
   * @param [in] midHeight: desired height at the middle of the trajectory. Assumes zero velocity.
   * @param [in] end: ending time and boundary conditions
   */
  QuinticSwing(const SwingNode& start, scalar_t midHeight, const SwingNode& end);

  /**
   * Construct a swing trajectory with given boundary conditions
   *
   * @param [in] start: starting time and boundary conditions
   * @param [in] mid: waypoint time, position, and velocity. Time must be between start and end.
   * @param [in] end: ending time and boundary conditions
   */
  QuinticSwing(const SwingNode& start, const SwingNode& mid, const SwingNode& end);

  /**
   * Construct a swing trajectory with through the given nodes. At least 3 need to be passed (such that there are at least 2 splines)
   *
   * @param [in] nodes: each node is a waypoint in time, position, and velocity. Vector needs to be sorted in time.
   */
  explicit QuinticSwing(const std::vector<SwingNode>& nodes);

  /**
   * @brief returns z(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Position(scalar_t time) const;

  /**
   * @brief returns dz/dt(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Velocity(scalar_t time) const;

  /**
   * @brief returns d2z/dt2(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Acceleration(scalar_t time) const;

  /**
   * @brief returns d3z/dt3(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Jerk(scalar_t time) const;

 private:
  std::vector<scalar_t> nodeTimes;
  std::vector<QuinticSpline> splines;
};

}  // namespace switched_model