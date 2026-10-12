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
 * @file state_augmented_lagrangian_interface.hpp
 * @brief The base class for Augmented Lagrangian penalty of state constraint.
 */


#include <bolza/core/pre_computation.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_constraint.hpp>
#include <bolza/core/model_data/metrics.hpp>
#include <bolza/core/model_data/multiplier.hpp>

namespace bolza {

/** The base class for Augmented Lagrangian penalty of state constraint. */
class StateAugmentedLagrangianInterface {
 public:
  /**
   * @brief Constructs a StateAugmentedLagrangianInterface.
   */
  StateAugmentedLagrangianInterface() = default;
  /**
   * @brief Destructor.
   */
  virtual ~StateAugmentedLagrangianInterface() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual StateAugmentedLagrangianInterface* Clone() const = 0;

    /**
   * @brief /** Check penalty's activity */.
   * @param [in] time: Time.
   * @return True on success.
   */
  virtual bool IsActive(scalar_t time) const = 0;

    /**
   * @brief /** Get the size of the constraint vector at given time */.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual size_t GetNumConstraints(scalar_t time) const = 0;

    /**
   * @brief /** Get the constraint and its penalty value */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] multiplier: Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual LagrangianMetrics GetValue(scalar_t time, const vector_t& state, const Multiplier& multiplier,
                                     const PreComputation& preComp) const = 0;

    /**
   * @brief /** Get the constraint's penalty quadratic approximation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] multiplier: Multiplier.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const Multiplier& multiplier,
                                                                         const PreComputation& preComp) const = 0;

    /**
   * @brief /** Update Lagrange/penalty multipliers and the penalty function value. */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] constraint: Constraint.
   * @param [in] multiplier: Multiplier.
   * @return Result value.
   */
  virtual std::pair<Multiplier, scalar_t> UpdateLagrangian(scalar_t time, const vector_t& state, const vector_t& constraint,
                                                           const Multiplier& multiplier) const = 0;

    /**
   * @brief /** Initialize Lagrange/penalty multipliers. */.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual Multiplier InitializeLagrangian(scalar_t time) const = 0;

 protected:
  /**
   * @brief Constructs a StateAugmentedLagrangianInterface.
   * @param [in] rhs: Rhs.
   */
  StateAugmentedLagrangianInterface(const StateAugmentedLagrangianInterface& rhs) = default;
};

}  // namespace bolza
