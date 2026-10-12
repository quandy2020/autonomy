/*
 * SplineCPG.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#pragma once

/**
 * @file spline_cpg.hpp
 * @brief spline cpg (bolza).
 */


#include "bolza/switched_model/foot_planner/cubic_spline.hpp"

namespace switched_model {

class SplineCpg {
 public:
  /**
   * @brief Constructs a SplineCpg.
   * @param [in] liftOff: Lift Off.
   * @param [in] midHeight: Mid Height.
   * @param [in] touchDown: Touch Down.
   */
  SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown);

  /**
   * @brief Constructs a SplineCpg.
   * @param [in] liftOff: Lift Off.
   * @param [in] midNode: Mid Node.
   * @param [in] touchDown: Touch Down.
   */
  SplineCpg(CubicSpline::Node liftOff, CubicSpline::Node midNode, CubicSpline::Node touchDown);

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
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t StartTimeDerivative(scalar_t time) const;

  /**
   * @brief Final Time Derivative.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t FinalTimeDerivative(scalar_t time) const;

 private:
  scalar_t midTime_;
  CubicSpline leftSpline_;
  CubicSpline rightSpline_;
};

}  // namespace switched_model
