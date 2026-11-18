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
 * @file state_space.hpp
 * @brief Base clothoid state space with path integration.
 */

#pragma once

#include <vector>

#include "autonomy/common/math/steering/clothoid/circle.hpp"
#include "autonomy/common/math/steering/types.hpp"

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {


class HcCcStateSpace
{
public:
  /**
   * @brief Constructor.
   * @param[in] kappa Maximum curvature.
   * @param[in] sigma Maximum sharpness (d kappa / ds).
   * @param[in] discretization Path sampling step in meters.
   */
  HcCcStateSpace(double kappa, double sigma, double discretization);

  /**
   * @brief Sets the parameters required by the filter.
   * @param[in] motion_noise Longitudinal/lateral motion-noise parameters.
   * @param[in] measurement_noise Localization standard deviations.
   * @param[in] controller Feedback gains on longitudinal, lateral, and heading error.
   */
  void SetFilterParameters(const MotionNoise& motion_noise, const MeasurementNoise& measurement_noise,
                             const Controller& controller);

  /**
   * @brief Virtual function that returns controls of the shortest path from state1 to state2.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Controls of the shortest path.
   */
  virtual std::vector<Control> GetControls(const State& state1, const State& state2) const = 0;

  /**
   * @brief Returns path from state1 to state2.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Discretized path.
   */
  std::vector<State> GetPath(const State& state1, const State& state2) const;

  /**
   * @brief Returns path including covariances from state1 to state2.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Discretized path with covariance.
   */
  std::vector<StateWithCovariance> GetPathWithCovariance(const StateWithCovariance& state1,
                                                              const State& state2) const;

  /**
   * @brief Returns integrated states given a start state and controls.
   * @param[in] state Kinematic car state.
   * @param[in] controls Sequence of path-segment controls.
   * @return Integrated states along the controls.
   */
  std::vector<State> Integrate(const State& state, const std::vector<Control>& controls) const;

  /**
   * @brief Returns integrated states including covariance given a start state and controls.
   * @param[in] state Kinematic car state.
   * @param[in] controls Sequence of path-segment controls.
   * @return Integrated states with covariance.
   */
  std::vector<StateWithCovariance> IntegrateWithCovariance(const StateWithCovariance& state,
                                                               const std::vector<Control>& controls) const;

  /**
   * @brief Returns interpolated state at distance t in [0,1] (percentage of total path length).
   * @param[in] state Kinematic car state.
   * @param[in] controls Sequence of path-segment controls.
   * @param[in] t Normalized arc length in [0, 1].
   * @return Integrated or interpolated state.
   */
  State Interpolate(const State& state, const std::vector<Control>& controls, double t) const;

  /**
   * @brief Returns integrated state given a start state, a control, and an integration step.
   * @param[in] state Kinematic car state.
   * @param[in] control Signed length, curvature, and sharpness.
   * @param[in] integration_step Integration step length in meters.
   * @return Integrated or interpolated state.
   */
  inline State IntegrateOde(const State& state, const Control& control, double integration_step) const;

protected:
  /**
   * @brief Curvature, sharpness of clothoid.
   */
  double kappa_, sigma_;

  /**
   * @brief Discretization of path.
   */
  double discretization_;

  /**
   * @brief Parameters of a hc-/cc-circle.
   */
  HcCcCircleParam hc_cc_circle_param_;

  MotionNoise motion_noise_{};
  MeasurementNoise measurement_noise_{};
  Controller controller_{};
};

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
