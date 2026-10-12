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
 * @file loopshaping_state_constraint.hpp
 * @brief loopshaping state constraint (bolza).
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_constraint_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

class LoopshapingStateConstraint final : public StateConstraintCollection {
 public:
  /**
   * @brief Constructs a LoopshapingStateConstraint.
   * @param [in] systemConstraint: System Constraint.
   */
  LoopshapingStateConstraint(const StateConstraintCollection& systemConstraint,
                             std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief State Constraint Collection.
       * @return Result value.
       */
      : StateConstraintCollection(systemConstraint), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  /**
   * @brief Destructor.
   */
  ~LoopshapingStateConstraint() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingStateConstraint* Clone() const override { /**
  * @brief Loopshaping State Constraint.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingStateConstraint(*this); }

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_array_t GetValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  /**
   * @brief Get Linear Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a LoopshapingStateConstraint.
   * @param [in] other: Other.
   */
  LoopshapingStateConstraint(const LoopshapingStateConstraint& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
