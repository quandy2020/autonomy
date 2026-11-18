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
 * @file reeds_shepp.hpp
 * @brief SE(2) Reeds–Shepp state space.
 */

#pragma once

#include <cassert>
#include <limits>
#include <vector>

#include "autonomy/common/math/steering/types.hpp"

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {


/**
 * @class ReedsSheppStateSpace
 * @brief An SE(2) state space where distance is measured by the length of Reeds-Shepp curves. The notation and solutions are taken from: J.A. Reeds and L.A. Shepp, “Optimal paths for a car that goes both forwards and backwards,” Pacific Journal of Mathematics, 145(2):367–393, 1990. This implementation explicitly computes all 48 Reeds-Shepp curves and returns the shortest valid solution. This can be improved by using the configuration space partition described in: P. Souères and J.-P. Laumond, “Shortest paths synthesis for a car-like robot,” IEEE Trans. on Automatic Control, 41(5):672–688, May 1996.
 */
class ReedsSheppStateSpace
{
public:
  /**
   * @brief The Reeds-Shepp path segment types.
   */
  enum ReedsSheppPathSegmentType
  {
    RS_NOP = 0,
    RS_LEFT = 1,
    RS_STRAIGHT = 2,
    RS_RIGHT = 3
  };

  /**
   * @brief Reeds-Shepp path types.
   */
  static const ReedsSheppPathSegmentType reeds_shepp_path_type[18][5];

  /**
   * @class ReedsSheppPath
   * @brief Complete description of a ReedsShepp path.
   */
  class ReedsSheppPath
  {
  public:
    /**
     * @brief Constructor of the ReedsSheppPath.
     * @param[in] type Path segment type table.
     * @param[in] t Normalized arc length in [0, 1].
     */
    ReedsSheppPath(const ReedsSheppPathSegmentType *type = reeds_shepp_path_type[0],
                     double t = std::numeric_limits<double>::max(), double u = 0., double v = 0., double w = 0.,
                     double x = 0.);

    double length() const
    {
      return total_length_;
    }

  private:
    /**
     * @brief Path segment types.
     */
    const ReedsSheppPathSegmentType *type_;

    /**
     * @brief Path segment lengths.
     */
    double length_[5];

    /**
     * @brief Total length.
     */
    double total_length_;

    friend class ReedsSheppStateSpace;
  };

  /**
   * @brief Constructor of the ReedsSheppStateSpace.
   * @param[in] kappa Maximum curvature.
   * @param[in] discretization Path sampling step in meters.
   */
  ReedsSheppStateSpace(double kappa, double discretization = 0.1) : kappa_(kappa), discretization_(discretization)
  {
    assert(kappa > 0.0 && discretization > 0.0);
    kappa_inv_ = 1 / kappa;
  }

  /**
   * @brief Sets the parameters required by the filter.
   * @param[in] motion_noise Longitudinal/lateral motion-noise parameters.
   * @param[in] measurement_noise Localization standard deviations.
   * @param[in] controller Feedback gains on longitudinal, lateral, and heading error.
   */
  void SetFilterParameters(const MotionNoise &motion_noise, const MeasurementNoise &measurement_noise,
                             const Controller &controller);

  /**
   * @brief Returns type and length of segments of path from state1 to state2 with curvature = 1.0.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Reeds–Shepp path type and segment lengths.
   */
  ReedsSheppPath ReedsShepp(const State &state1, const State &state2) const;

  /**
   * @brief Returns shortest path length from state1 to state2 with curvature = kappa_.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Shortest path length in meters.
   */
  double GetDistance(const State &state1, const State &state2) const;

  /**
   * @brief Returns controls of the shortest path from state1 to state2 with curvature = kappa_.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Controls of the shortest path.
   */
  std::vector<Control> GetControls(const State &state1, const State &state2) const;

  /**
   * @brief Returns shortest path from state1 to state2 with curvature = kappa_.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Discretized path.
   */
  std::vector<State> GetPath(const State &state1, const State &state2) const;

  /**
   * @brief Returns shortest path including covariances from state1 to state2 with curvature = kappa_.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Discretized path with covariance.
   */
  std::vector<StateWithCovariance> GetPathWithCovariance(const StateWithCovariance &state1,
                                                              const State &state2) const;

  /**
   * @brief Returns integrated states given a start state and controls with curvature = kappa_.
   * @param[in] state Kinematic car state.
   * @param[in] controls Sequence of path-segment controls.
   * @return Integrated states along the controls.
   */
  std::vector<State> Integrate(const State &state, const std::vector<Control> &controls) const;

  /**
   * @brief Returns integrated states including covariance given a start state and controls with curvature = kappa_.
   * @param[in] state Kinematic car state.
   * @param[in] controls Sequence of path-segment controls.
   * @return Integrated states with covariance.
   */
  std::vector<StateWithCovariance> IntegrateWithCovariance(const StateWithCovariance &state,
                                                               const std::vector<Control> &controls) const;

  /**
   * @brief Returns interpolated state at distance t in [0,1] (percent of total path length with curvature = kappa_).
   * @param[in] state Kinematic car state.
   * @param[in] controls Sequence of path-segment controls.
   * @param[in] t Normalized arc length in [0, 1].
   * @return Integrated or interpolated state.
   */
  State Interpolate(const State &state, const std::vector<Control> &controls, double t) const;

  /**
   * @brief Returns integrated state given a start state, a control, and an integration step.
   * @param[in] state Kinematic car state.
   * @param[in] control Signed length, curvature, and sharpness.
   * @param[in] integration_step Integration step length in meters.
   * @return Integrated or interpolated state.
   */
  inline State IntegrateOde(const State &state, const Control &control, double integration_step) const;

private:
  /**
   * @brief Curvature.
   */
  double kappa_;

  /**
   * @brief Inverse of curvature.
   */
  double kappa_inv_;

  /**
   * @brief Discretization of path.
   */
  double discretization_;

  MotionNoise motion_noise_{};
  MeasurementNoise measurement_noise_{};
  Controller controller_{};
};

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
