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
 * @file loopshaping_state_input_constraint.hpp
 * @brief loopshaping state input constraint (bolza).
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_input_constraint_collection.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

class LoopshapingStateInputConstraint : public StateInputConstraintCollection {
 public:
  /**
   * @brief Destructor.
   */
  ~LoopshapingStateInputConstraint() override = default;

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_array_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;

 protected:
  /**
   * @brief Constructs a LoopshapingStateInputConstraint.
   * @param [in] systemConstraint: System Constraint.
   */
  LoopshapingStateInputConstraint(const StateInputConstraintCollection& systemConstraint,
                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief State Input Constraint Collection.
       * @return Result value.
       */
      : StateInputConstraintCollection(systemConstraint), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  /**
   * @brief Constructs a LoopshapingStateInputConstraint.
   * @param [in] other: Other.
   */
  LoopshapingStateInputConstraint(const LoopshapingStateInputConstraint& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
