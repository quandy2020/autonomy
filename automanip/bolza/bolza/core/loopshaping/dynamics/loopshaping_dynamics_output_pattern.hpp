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
 * @file loopshaping_dynamics_output_pattern.hpp
 * @brief loopshaping dynamics output pattern (bolza).
 */


#include <bolza/core/loopshaping/dynamics/loopshaping_dynamics.hpp>

namespace bolza {

class LoopshapingDynamicsOutputPattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  /**
   * @brief Constructs a LoopshapingDynamicsOutputPattern.
   * @param [in] controlledSystem: Controlled System.
   * @param [in] loopshapingDefinition: Loopshaping Definition.
   * @param [in] PreComputation: Pre Computation.
   */
  LoopshapingDynamicsOutputPattern(const SystemDynamicsBase& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                                   const LoopshapingPreComputation& PreComputation)
      /**
       * @brief BASE.
       * @param [in] controlledSystem: Controlled System.
       * @param [in] PreComputation: Pre Computation.
       * @return Result value.
       */
      : BASE(controlledSystem, std::move(loopshapingDefinition), PreComputation) {}

  /**
   * @brief Destructor.
   */
  ~LoopshapingDynamicsOutputPattern() override = default;

  /**
   * @brief Constructs a LoopshapingDynamicsOutputPattern.
   * @param [in] obj: Obj.
   */
  LoopshapingDynamicsOutputPattern(const LoopshapingDynamicsOutputPattern& obj) = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingDynamicsOutputPattern* Clone() const override { /**
  * @brief Loopshaping Dynamics Output Pattern.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new LoopshapingDynamicsOutputPattern(*this); }

  /**
   * @brief Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override;

 private:
  /**
   * @brief Filter Flowmap.
   * @param [in] x_filter: X filter.
   * @param [in] u_filter: U filter.
   * @param [in] u_system: U system.
   * @return Result value.
   */
  vector_t FilterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) override;

  using BASE::loopshapingDefinition_;
};

}  // namespace bolza
