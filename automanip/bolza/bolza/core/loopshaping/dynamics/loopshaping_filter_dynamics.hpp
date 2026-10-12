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
 * @file loopshaping_filter_dynamics.hpp
 * @brief loopshaping filter dynamics (bolza).
 */


#include <memory>

#include <bolza/core/loopshaping/loopshaping_definition.hpp>
#include "bolza/core/integration/integrator.hpp"

namespace bolza {

class LoopshapingFilterDynamics {
 public:
  /**
   * @brief Constructs a LoopshapingFilterDynamics.
   */
  explicit LoopshapingFilterDynamics(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      /**
       * @brief Loopshaping Definition.
       * @return Result value.
       */
      : loopshapingDefinition_(std::move(loopshapingDefinition)), integrator_(NewIntegrator(IntegratorType::ODE45)) {
    /**
     * @brief Set Zero.
     * @return Result value.
     */
    filterState_.setZero(loopshapingDefinition_->GetInputFilter().GetNumStates());
  }

  /**
   * @brief Integrate.
   * @param [in] dt: Dt.
   * @param [in] input: Input.
   */
  void Integrate(scalar_t dt, const vector_t& input);

  /**
   * @brief Set Filter State.
   * @param [in] filterState: Filter State.
   */
  void SetFilterState(const vector_t& filterState) { filterState_ = filterState; };
  /**
   * @brief Get Filter State.
   * @return Result value.
   */
  const vector_t& GetFilterState() const { return filterState_; };

 private:
  /**
   * @brief Compute Flow Map.
   * @param [in] time: Time.
   * @param [in] filterState: Filter State.
   * @param [in] input: Input.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t time, const vector_t& filterState, const vector_t& input) const;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  vector_t filterState_;
  std::unique_ptr<IntegratorBase> integrator_;
};

}  // namespace bolza
