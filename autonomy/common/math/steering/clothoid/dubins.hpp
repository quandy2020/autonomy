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
 * @brief Continuous-curvature Dubins with arbitrary endpoint curvature.
 */

#pragma once

#include <utility>
#include <vector>

#include "autonomy/common/math/steering/clothoid/state_space.hpp"
#include "autonomy/common/math/steering/clothoid/paths.hpp"
#include "autonomy/common/math/steering/clothoid/dubins_zero_zero.hpp"
#include "autonomy/common/math/steering/clothoid/dubins_zero_max.hpp"
#include "autonomy/common/math/steering/clothoid/dubins_max_zero.hpp"
#include "autonomy/common/math/steering/clothoid/dubins_max_max.hpp"
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
 * @class CcDubinsStateSpace
 * @brief An implementation of continuous curvature (CC) steer for a Dubins car with arbitrary curvature at the start and goal configuration.
 */
class CcDubinsStateSpace : public HcCcStateSpace
{
public:
  /**
   * @brief Constructor.
   * @param[in] kappa Maximum curvature.
   * @param[in] sigma Maximum sharpness (d kappa / ds).
   * @param[in] discretization Path sampling step in meters.
   * @param[in] forwards True to integrate forwards.
   */
  CcDubinsStateSpace(double kappa, double sigma, double discretization = 0.1, bool forwards = true);

  /**
   * @brief Predicts a state forwards or backwards to zero and max. curvature.
   * @param[in] state Kinematic car state.
   * @param[in] forwards True to integrate forwards.
   * @return Predicted (state, control) pairs.
   */
  std::vector<std::pair<State, Control>> PredictState(const State& state, bool forwards) const;

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
   * @brief Required state spaces.
   */
  Cc00DubinsStateSpace cc00_dubins_state_space_;
  Cc0pmDubinsStateSpace cc0pm_dubins_state_space_;
  Ccpm0DubinsStateSpace ccpm0_dubins_state_space_;
  CcpmpmDubinsStateSpace ccpmpm_dubins_state_space_;
};

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
