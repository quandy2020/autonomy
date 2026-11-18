/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file circle.hpp
 * @brief HC/CC circle geometry used by clothoid steering.
 */

#pragma once

#include "autonomy/common/math/steering/clothoid/configuration.hpp"

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {


class HcCcCircleParam
{
public:
  /**
   * @brief Set parameters.
   * @param[in] _kappa Maximum curvature.
   * @param[in] _sigma Maximum sharpness.
   * @param[in] _radius Outer-circle radius.
   * @param[in] _mu Tangent angle mu.
   * @param[in] _sin_mu sin(mu).
   * @param[in] _cos_mu cos(mu).
   * @param[in] _delta_min Minimum deflection.
   */
  void SetParam(double _kappa, double _sigma, double _radius, double _mu, double _sin_mu, double _cos_mu,
                 double _delta_min);

  /**
   * @brief Max. curvature, inverse of max. curvature, max. sharpness.
   */
  double kappa, kappa_inv, sigma;

  /**
   * @brief Radius of the outer circle.
   */
  double radius;

  /**
   * @brief Angle between the initial orientation and the tangent to the circle at the initial position.
   */
  double mu;

  /**
   * @brief Sine and cosine of mu.
   */
  double sin_mu, cos_mu;

  /**
   * @brief Minimal deflection.
   */
  double delta_min;
};

class HcCcCircle : public HcCcCircleParam
{
public:
  /**
   * @brief Constructor.
   * @param[in] _start Start configuration.
   * @param[in] _left True for a left turn.
   * @param[in] _forward True when driving forwards.
   * @param[in] _regular True for a regular circle.
   * @param[in] _param Circle geometric parameters.
   */
  HcCcCircle(const Configuration &_start, bool _left, bool _forward, bool _regular, const HcCcCircleParam &_param);

  /**
   * @brief Constructor.
   * @param[in] _xc Circle center x.
   * @param[in] _yc Circle center y.
   * @param[in] _left True for a left turn.
   * @param[in] _forward True when driving forwards.
   * @param[in] _regular True for a regular circle.
   * @param[in] _param Circle geometric parameters.
   */
  HcCcCircle(double _xc, double _yc, bool _left, bool _forward, bool _regular, const HcCcCircleParam &_param);

  /**
   * @brief Computation of Deflection (angle between start configuration of circle and configuration q).
   * @param[in] q Configuration on the circle or path.
   * @return Deflection angle in radians.
   */
  double Deflection(const Configuration &q) const;

  /**
   * @brief Calculation of D1 for the evaluation of an elementary path.
   * @param[in] alpha Angle in radians.
   * @return Elementary-path D1 term.
   */
  double D1(double alpha) const;

  /**
   * @brief Computation of a rs-turn's circular deflection.
   * @param[in] delta Deflection angle in radians.
   * @return Circular deflection of the RS turn.
   */
  double RsCircularDeflection(double delta) const;

  /**
   * @brief Length of a rs-turn.
   * @param[in] q Configuration on the circle or path.
   * @return RS-turn length.
   */
  double RsTurnLength(const Configuration &q) const;

  /**
   * @brief Computation of a hc-turn's circular deflection.
   * @param[in] delta Deflection angle in radians.
   * @return Circular deflection of the HC turn.
   */
  double HcCircularDeflection(double delta) const;

  /**
   * @brief Length of a hc-turn.
   * @param[in] q Configuration on the circle or path.
   * @return HC-turn length.
   */
  double HcTurnLength(const Configuration &q) const;

  /**
   * @brief Computation of an elementary path's sharpness.
   * @param[in] q Configuration on the circle or path.
   * @param[in] delta Deflection angle in radians.
   * @param[out] sigma0 Elementary-path sharpness.
   * @return True when the predicate holds.
   */
  bool CcElementarySharpness(const Configuration &q, double delta, double &sigma0) const;

  /**
   * @brief Computation of a cc-turn's circular deflection.
   * @param[in] delta Deflection angle in radians.
   * @return Circular deflection of the CC turn.
   */
  double CcCircularDeflection(double delta) const;

  /**
   * @brief Length of a cc-turn.
   * @param[in] q Configuration on the circle or path.
   * @return CC-turn length.
   */
  double CcTurnLength(const Configuration &q) const;

  /**
   * @brief Alphanumeric display.
   * @param[in] eol True to print a trailing newline.
   */
  void Print(bool eol) const;

  /**
   * @brief Start configuration.
   */
  Configuration start;

  /**
   * @brief Turning direction: left/right.
   */
  bool left;

  /**
   * @brief Driving direction: forwards/backwards.
   */
  bool forward;

  /**
   * @brief Type of the circle: regular/irregular.
   */
  bool regular;

  /**
   * @brief Center of the circle.
   */
  double xc, yc;
};

/**
 * @brief Cartesian distance between the centers of two circles.
 * @param[in] c1 First HC/CC circle.
 * @param[in] c2 Second HC/CC circle.
 * @return Distance between circle centers.
 */
double CenterDistance(const HcCcCircle &c1, const HcCcCircle &c2);

/**
 * @brief Whether configuration q lies on circle c.
 * @param[in] c HC/CC circle.
 * @param[in] q Configuration on the circle or path.
 * @return True when the predicate holds.
 */
bool ConfigurationOnHcCcCircle(const HcCcCircle &c, const Configuration &q);

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
