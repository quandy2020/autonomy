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
 * @file controlled_system_base.hpp
 * @brief The base class for non-autonomous system dynamics.
 */


#include <memory>

#include <bolza/core/types.hpp>

#include <bolza/core/pre_computation.hpp>
#include <bolza/core/control/controller_base.hpp>
#include <bolza/core/integration/ode_base.hpp>

namespace bolza {

/**
 * The base class for non-autonomous system dynamics.
 */
class ControlledSystemBase : public OdeBase {
 public:
  /**
   * Constructor
   *
   * @param [in] preComputation: The (optional) pre-computation module, internally keeps a copy.
   *                             @see PreComputation class documentation.
   */
  explicit ControlledSystemBase(const PreComputation& preComputation = PreComputation());

  /** Default destructor */
  ~ControlledSystemBase() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual ControlledSystemBase* Clone() const = 0;

  /** Resets the internal classes. */
  virtual void Reset() { controllerPtr_ = nullptr; }

    /**
   * @brief /**.
   *
   * Sets the control policy using the controller class.
   *
   * /
   * @param [in,out] controllerPtr: Controller Ptr.
   */
  void SetController(ControllerBase* controllerPtr) { controllerPtr_ = controllerPtr; };

  /**
   * @brief Returns the controller pointer.
   * @return Pointer; may be null.
   */
  ControllerBase* ControllerPtr() const { return controllerPtr_; };

  /**
   * Computes the flow map of a system.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @return The state time derivative.
   */
  vector_t ComputeFlowMap(scalar_t t, const vector_t& x) override final;

  /**
   * Computes the flow map of a system with exogenous input.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] u: The current input.
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return The state time derivative.
   */
  virtual vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) = 0;

  /**
   * State map at the transition time
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return mapped state after transition
   */
  virtual vector_t ComputeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp);

    /**
   * @brief /**.
   *
   * Computes the flow map of a system with exogenous input.
   *
   * ComputeFlowMap() with the preComputation as parameter.
   *
   * This interface is used by Rollout and SensitivityIntegrator.
   *
   * /
   * @note This method calls the internal preComputation Request() callback and the virtual
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief /**.
   *
   * State map at the transition time
   *
   * ComputeJumpMap() with the preComputation as parameter.
   *
   * This interface is used by Rollout.
   *
   * /
   * @note This method calls the internal preComputation RequestPreJump() callback and the virtual
   * @param [in] time: Time.
   * @param [in] state: State.
   * @return Result value.
   */
  vector_t ComputeJumpMap(scalar_t time, const vector_t& state) override final;

  /**
   * @brief Get the pre-computation module.
   * @return Result value.
   */
  const PreComputation& GetPreComputation() const { return *preCompPtr_; }

 protected:
    /**
   * @brief /**.
   *
   * Copy constructor
   *
   * /
   * @note Keeps the same controller pointer.
   * @note Clones the pre-computation object.
   * @param [in] other: Other.
   */
  ControlledSystemBase(const ControlledSystemBase& other);

  std::unique_ptr<PreComputation> preCompPtr_;  //! pointer to pre-computation module

 private:
  ControllerBase* controllerPtr_ = nullptr;  //! pointer to controller
};

}  // namespace bolza
