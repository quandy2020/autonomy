/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file state_cost.hpp
 * @brief State-only cost term.
 */


#include <type_traits>

#include <bolza/core/pre_computation.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

namespace bolza {

/** State-only cost term */
class StateCost {
 public:
  /**
   * @brief Constructs a StateCost.
   */
  StateCost() = default;
  /**
   * @brief Destructor.
   */
  virtual ~StateCost() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual StateCost* Clone() const = 0;

    /**
   * @brief Check if cost term is active 
   * @param [in] time: Time.
   * @return True on success.
   */
  virtual bool IsActive(scalar_t time) const { return true; }

    /**
   * @brief Get cost term value 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual scalar_t GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                            const PreComputation& preComp) const = 0;

    /**
   * @brief Get cost term quadratic approximation 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                         const TargetTrajectories& targetTrajectories,
                                                                         const PreComputation& preComp) const = 0;

 protected:
  /**
   * @brief Constructs a StateCost.
   * @param [in] rhs: Rhs.
   */
  StateCost(const StateCost& rhs) = default;
};

// Template for conditional compilation using SFINAE
template <typename T>
using EnableIfStateCost_t = typename std::enable_if<std::is_same<T, StateCost>::value, bool>::type;

}  // namespace bolza
