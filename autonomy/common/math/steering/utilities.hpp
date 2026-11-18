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
 * @file utilities.hpp
 * @brief Clothoid / arc helpers that are not already in common/math.
 */

#pragma once

#include <cmath>

#include "autonomy/common/math/math.hpp"
#include "autonomy/common/math/math_utils.hpp"

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {

using ::autonomy::common::SignOfNumber;
using ::autonomy::common::math::Cartesian2Polar;
using ::autonomy::common::math::NormalizeAngle;
using ::autonomy::common::math::RotateVector2d;
using ::autonomy::common::math::WrapAngle;

/** Geometric comparison tolerance used by clothoid / Dubins families. */
constexpr double kEpsilon = 1e-4;

inline const double kSqrtPi = std::sqrt(M_PI);
inline const double kSqrtPiInv = 1.0 / std::sqrt(M_PI);
inline const double kSqrtTwoPiInv = 1.0 / std::sqrt(2.0 * M_PI);

/**
 * @brief Return the steering geometric tolerance.
 * @return kEpsilon.
 */
inline double GetEpsilon() {
  return kEpsilon;
}

/**
 * @brief Fresnel integrals S(s), C(s) via Chebyshev approximation.
 * @param[in] s Fresnel argument.
 * @param[out] s_f Fresnel sine integral.
 * @param[out] c_f Fresnel cosine integral.
 */
void Fresnel(double s, double& s_f, double& c_f);

/**
 * @brief End pose of a clothoid of given length.
 */
void EndOfClothoid(double x_i, double y_i, double theta_i, double kappa_i, double sigma,
                   double direction, double length, double* x_f, double* y_f, double* theta_f,
                   double* kappa_f);

/**
 * @brief End pose of a circular arc of given length.
 */
void EndOfCircularArc(double x_i, double y_i, double theta_i, double kappa, double direction,
                      double length, double* x_f, double* y_f, double* theta_f);

/**
 * @brief End pose of a straight segment of given length.
 */
void EndOfStraightLine(double x_i, double y_i, double theta, double direction, double length,
                       double* x_f, double* y_f);

/**
 * @brief Map a local point into the global frame at (x, y, theta).
 */
inline void GlobalFrameChange(double x, double y, double theta, double local_x, double local_y,
                              double* global_x, double* global_y) {
  const Eigen::Vector2d g = RotateVector2d({local_x, local_y}, theta);
  *global_x = g.x() + x;
  *global_y = g.y() + y;
}

/**
 * @brief Map a global point into the local frame at (x, y, theta).
 */
inline void LocalFrameChange(double x, double y, double theta, double global_x, double global_y,
                             double* local_x, double* local_y) {
  const Eigen::Vector2d l = RotateVector2d({global_x - x, global_y - y}, -theta);
  *local_x = l.x();
  *local_y = l.y();
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
