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
 * @file loopshaping_dynamics.hpp
 * @brief loopshaping dynamics (bolza).
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>
#include <bolza/core/loopshaping/loopshaping_pre_computation.hpp>

namespace bolza {
class LoopshapingDynamics : public SystemDynamicsBase {
 public:
  /**
   * @brief Destructor.
   */
  ~LoopshapingDynamics() override = default;

  /**
   * @brief Create.
   * @param [in] systemDynamics: System Dynamics.
   * @param [in] loopshapingDefinition: Loopshaping Definition.
   * @return Owning pointer.
   */
  static std::unique_ptr<LoopshapingDynamics> Create(const SystemDynamicsBase& systemDynamics,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  /**
   * @brief Compute Flow Map.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) final;
  /**
   * @brief Compute Jump Map.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_t ComputeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp) final;
  /**
   * @brief Compute Guard Surfaces.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  vector_t ComputeGuardSurfaces(scalar_t time, const vector_t& state) final;

  /**
   * @brief Jump Map Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation JumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) final;
  /**
   * @brief Guard Surfaces Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final;

  /**
   * @brief Flow Map Derivative Time.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t FlowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;
  /**
   * @brief Jump Map Derivative Time.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t JumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;
  /**
   * @brief Guard Surfaces Derivative Time.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t GuardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

 protected:
  /**
   * @brief Constructs a LoopshapingDynamics.
   * @param [in] loopshapingDefinition_: Loopshaping Definition.
   */
  LoopshapingDynamics(const LoopshapingDynamics& other)
      /**
       * @brief System Dynamics Base.
       * @param [in] loopshapingDefinition_: Loopshaping Definition.
       * @return Result value.
       */
      : SystemDynamicsBase(other), systemDynamics_(other.systemDynamics_->Clone()), loopshapingDefinition_(other.loopshapingDefinition_) {}

  /**
   * @brief Constructs a LoopshapingDynamics.
   * @param [in] systemDynamics: System Dynamics.
   * @param [in] loopshapingDefinition: Loopshaping Definition.
   */
  LoopshapingDynamics(const SystemDynamicsBase& systemDynamics, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                      const LoopshapingPreComputation& preComputation)
      /**
       * @brief System Dynamics Base.
       * @return Result value.
       */
      : SystemDynamicsBase(preComputation),
        systemDynamics_(systemDynamics.Clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  std::unique_ptr<SystemDynamicsBase> systemDynamics_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

 private:
  /**
   * @brief Filter Flowmap.
   * @param [in] x_filter: X filter.
   * @param [in] u_filter: U filter.
   * @param [in] u_system: U system.
   * @return Result value.
   */
  virtual vector_t FilterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) = 0;
};

}  // namespace bolza
