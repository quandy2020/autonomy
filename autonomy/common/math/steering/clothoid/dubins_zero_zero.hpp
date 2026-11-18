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
 * @file dubins_zero_zero.hpp
 * @brief Continuous-curvature Dubins with zero curvature at both ends.
 */

#pragma once

#include <memory>
#include <vector>

#include "autonomy/common/math/steering/clothoid/circle.hpp"
#include "autonomy/common/math/steering/clothoid/state_space.hpp"
#include "autonomy/common/math/steering/clothoid/paths.hpp"
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
 * @class ImplStateSpace
 * @brief An implementation of continuous curvature (CC) steer for a Dubins car with zero curvature at the start and goal configuration as described in: T. Fraichard and A. Scheuer, "From Reeds and Shepp's to continuous-curvature paths," IEEE Transactions on Robotics (Volume 20, Issue: 6, Dec. 2004). It evaluates all Dubins families and returns the shortest path.
 */
class Cc00DubinsStateSpace : public HcCcStateSpace
{
public:
  /**
   * @brief Constructor.
   * @param[in] kappa Maximum curvature.
   * @param[in] sigma Maximum sharpness (d kappa / ds).
   * @param[in] discretization Path sampling step in meters.
   * @param[in] forwards True to integrate forwards.
   */
  Cc00DubinsStateSpace(double kappa, double sigma, double discretization = 0.1, bool forwards = true);

  /**
   * @brief Destructor.
   */
  ~Cc00DubinsStateSpace();

  /**
   * @brief Returns a sequence of turns and straight lines connecting the two circles c1 and c2.
   * @param[in] c1 First HC/CC circle.
   * @param[in] c2 Second HC/CC circle.
   * @return Heap-allocated path; caller owns the pointer.
   */
  CcDubinsPath* Cc00CirclesDubinsPath(const HcCcCircle& c1, const HcCcCircle& c2) const;

  /**
   * @brief Returns a sequence of turns and straight lines connecting a start and an end configuration.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Heap-allocated path; caller owns the pointer.
   */
  CcDubinsPath* Cc00Dubins(const State& state1, const State& state2) const;

  /**
   * @brief Returns shortest path length from state1 to state2.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Shortest path length in meters.
   */
  double GetDistance(const State& state1, const State& state2) const;

  /**
   * @brief Returns controls of the shortest path from state1 to state2.
   * @param[in] state1 Start state.
   * @param[in] state2 Goal state.
   * @return Controls of the shortest path.
   */
  std::vector<Control> GetControls(const State& state1, const State& state2) const;

private:
  /**
   * @brief Driving direction.
   */
  bool forwards_;

  /**
   * @class Impl
   * @brief Pimpl Idiom: class that contains functions to compute the families.
   */
  class Impl;

  /**
   * @brief Pimpl Idiom: unique pointer on class with families.
   */
  std::unique_ptr<Impl> cc00_dubins_;
};

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
