//
// Created by rgrandia on 03.08.21.
//

#pragma once

/**
 * @file swing_spline3d.hpp
 * @brief Construct a swing trajectory with given boundary conditions.
 */


#include "bolza/switched_model/core/switched_model.hpp"

#include "bolza/switched_model/foot_planner/quintic_spline_swing.hpp"

namespace switched_model {

struct SwingNode3d {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  scalar_t time;
  vector3_t position;
  vector3_t velocity;
};

class SwingSpline3d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Construct a swing trajectory with given boundary conditions
   *
   * @param [in] start: starting time and boundary conditions
   * @param [in] mid: waypoint time, position, and velocity. Time must be between start and end.
   * @param [in] end: ending time and boundary conditions
   */
  SwingSpline3d(const SwingNode3d& start, const SwingNode3d& mid, const SwingNode3d& end);

  /**
   * @brief Construct a swing trajectory through the provided nodes (at least 3).
   * @param [in] nodes: Nodes.
   */
  explicit SwingSpline3d(const std::vector<SwingNode3d>& nodes);

  /**
   * @brief returns p(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t Position(scalar_t time) const;

  /**
   * @brief returns dp/dt(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t Velocity(scalar_t time) const;

  /**
   * @brief returns d2p/dt2(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t Acceleration(scalar_t time) const;

  /**
   * @brief returns d3p/dt3(t).
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t Jerk(scalar_t time) const;

 private:
  QuinticSwing x_;
  QuinticSwing y_;
  QuinticSwing z_;
};

}  // namespace switched_model