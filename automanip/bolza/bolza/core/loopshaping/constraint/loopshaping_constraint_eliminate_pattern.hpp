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
 * @file loopshaping_constraint_eliminate_pattern.hpp
 * @brief loopshaping constraint eliminate pattern (bolza).
 */


#include <bolza/core/types.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>
#include <bolza/core/loopshaping/constraint/loopshaping_state_input_constraint.hpp>

namespace bolza {

class LoopshapingConstraintEliminatePattern final : public LoopshapingStateInputConstraint {
 public:
  using BASE = LoopshapingStateInputConstraint;

  /**
   * @brief Constructs a LoopshapingConstraintEliminatePattern.
   * @param [in] systemConstraint: System Constraint.
   */
  LoopshapingConstraintEliminatePattern(const StateInputConstraintCollection& systemConstraint,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief BASE.
       * @param [in] systemConstraint: System Constraint.
       * @return Result value.
       */
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  /**
   * @brief Destructor.
   */
  ~LoopshapingConstraintEliminatePattern() override = default;

  /**
   * @brief Constructs a LoopshapingConstraintEliminatePattern.
   * @param [in] obj: Obj.
   */
  LoopshapingConstraintEliminatePattern(const LoopshapingConstraintEliminatePattern& obj) = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingConstraintEliminatePattern* Clone() const override { /**
  * @brief Loopshaping Constraint Eliminate Pattern.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingConstraintEliminatePattern(*this); };

  /**
   * @brief Get Linear Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace bolza
