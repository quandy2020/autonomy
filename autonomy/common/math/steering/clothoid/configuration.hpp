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
 * @file configuration.hpp
 * @brief Pose-and-curvature configuration on a clothoid path.
 */

#pragma once

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {


class Configuration
{
public:
  /**
   * @brief Constructor.
   * @param[in] _x X position.
   * @param[in] _y Y position.
   * @param[in] _theta Heading in radians.
   * @param[in] _kappa Curvature.
   */
  Configuration(double _x = 0.0, double _y = 0.0, double _theta = 0.0, double _kappa = 0.0);

  /**
   * @brief Alphanumeric display.
   * @param[in] eol True to print a trailing newline.
   */
  void Print(bool eol) const;

  /**
   * @brief Position.
   */
  double x, y;

  /**
   * @brief Orientation in rad between [0, 2*pi[.
   */
  double theta;

  /**
   * @brief Curvature.
   */
  double kappa;
};

/**
 * @brief Cartesian distance between two configurations.
 * @param[in] q1 First configuration.
 * @param[in] q2 Second configuration.
 * @return Cartesian distance.
 */
double ConfigurationDistance(const Configuration &q1, const Configuration &q2);

/**
 * @brief Whether two configurations are aligned.
 * @param[in] q1 First configuration.
 * @param[in] q2 Second configuration.
 * @return True when the predicate holds.
 */
bool ConfigurationAligned(const Configuration &q1, const Configuration &q2);

/**
 * @brief Whether two configurations are equal.
 * @param[in] q1 First configuration.
 * @param[in] q2 Second configuration.
 * @return True when the predicate holds.
 */
bool ConfigurationEqual(const Configuration &q1, const Configuration &q2);

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
