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
 * @file dubins.hpp
 * @brief SE(2) Dubins state space.
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
 * @class DubinsStateSpace
 * @brief An SE(2) state space where distance is measured by the length of Dubins curves. Note that this Dubins distance is not a proper distance metric, so nearest neighbor methods that rely on distance() being a metric (such as ompl::NearestNeighborsGNAT) will not always return the true nearest neighbors or get stuck in an infinite loop. The notation and solutions in the code are taken from:<br> A.M. Shkel and V. Lumelsky, “Classification of the Dubins set,” Robotics and Autonomous Systems, 34(4):179-202, 2001. DOI: <a href="http://dx.doi.org/10.1016/S0921-8890(00)00127-5">10.1016/S0921-8890(00)00127-5</a> The classification scheme described there is not actually used, since it only applies to “long” paths.
 */
class DubinsStateSpace
{
public:
  /**
   * @brief The Dubins path segment type.
   */
  enum DubinsPathSegmentType
  {
    DUBINS_LEFT = 0,
    DUBINS_STRAIGHT = 1,
    DUBINS_RIGHT = 2
  };

  /**
   * @brief Dubins path types.
   */
  static const DubinsPathSegmentType dubins_path_type[6][3];

  /**
   * @class DubinsPath
   * @brief Complete description of a Dubins path.
   */
  class DubinsPath
  {
  public:
    /**
     * @brief Constructor of the DubinsPath.
     * @param[in] type Path segment type table.
     * @param[in] t First Dubins segment length.
     * @param[in] p Second Dubins segment length.
     * @param[in] q Third Dubins segment length.
     */
    DubinsPath(const DubinsPathSegmentType *type = dubins_path_type[0], double t = 0.,
                double p = std::numeric_limits<double>::max(), double q = 0.)
      : type_(type)
    {
      length_[0] = t;
      length_[1] = p;
      length_[2] = q;
      assert(t >= 0.);
      assert(p >= 0.);
      assert(q >= 0.);
    }

    double length() const
    {
      return length_[0] + length_[1] + length_[2];
    }

  private:
    /**
     * @brief Path segment types.
     */
    const DubinsPathSegmentType *type_;

    /**
     * @brief Path segment lengths.
     */
    double length_[3];

    friend class DubinsStateSpace;
  };

  /**
   * @brief Constructor of the DubinsStateSpace.
   * @param[in] kappa Maximum curvature.
   * @param[in] discretization Path sampling step in meters.
   * @param[in] forwards True to integrate forwards.
   */
  DubinsStateSpace(double kappa, double discretization = 0.1, bool forwards = true)
    : kappa_(kappa), discretization_(discretization), forwards_(forwards)
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
   * @return Dubins path type and segment lengths.
   */
  DubinsPath Dubins(const State &state1, const State &state2) const;

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

  /**
   * @brief Driving direction.
   */
  bool forwards_;

  MotionNoise motion_noise_{};
  MeasurementNoise measurement_noise_{};
  Controller controller_{};
};

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
