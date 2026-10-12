/*
 * CubicSpline.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#pragma once

/**
 * @file cubic_spline.hpp
 * @brief cubic spline (bolza).
 */


#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

class CubicSpline {
 public:
  struct Node {
    scalar_t time;
    scalar_t position;
    scalar_t velocity;
  };

  /**
   * @brief Constructs a CubicSpline.
   * @param [in] start: Start.
   * @param [in] end: End.
   */
  CubicSpline(Node start, Node end);

  /**
   * @brief Position.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Position(scalar_t time) const;

  /**
   * @brief Velocity.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Velocity(scalar_t time) const;

  /**
   * @brief Acceleration.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t Acceleration(scalar_t time) const;

  /**
   * @brief Start Time Derivative.
   * @param [in] t: T.
   * @return Result value.
   */
  scalar_t StartTimeDerivative(scalar_t t) const;

  /**
   * @brief Final Time Derivative.
   * @param [in] t: T.
   * @return Result value.
   */
  scalar_t FinalTimeDerivative(scalar_t t) const;

 private:
  /**
   * @brief Normalized Time.
   * @param [in] t: T.
   * @return Result value.
   */
  scalar_t NormalizedTime(scalar_t t) const;

  scalar_t t0_;
  scalar_t t1_;
  scalar_t dt_;

  scalar_t c0_;
  scalar_t c1_;
  scalar_t c2_;
  scalar_t c3_;

  scalar_t dc0_;  // derivative w.r.t. dt_
  scalar_t dc1_;  // derivative w.r.t. dt_
  scalar_t dc2_;  // derivative w.r.t. dt_
  scalar_t dc3_;  // derivative w.r.t. dt_
};

}  // namespace switched_model
