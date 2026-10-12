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
 * @file system_dynamics_base.hpp
 * @brief The system dynamics and linearization class.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>

namespace bolza {

/**
 * The system dynamics and linearization class.
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 */
class SystemDynamicsBase : public ControlledSystemBase {
 public:
  /**
   * Constructor
   *
   * @param [in] preComputation: The (optional) pre-computation module, internally keeps a copy.
   *                             @see PreComputation class documentation.
   */
  explicit SystemDynamicsBase(const PreComputation& preComputation = PreComputation());

  /** Default destructor */
  ~SystemDynamicsBase() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  SystemDynamicsBase* Clone() const override = 0;

  /**
   * Computes the linear approximation.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] u: The current input.
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return The state time derivative linear approximation.
   */
  virtual VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                const PreComputation& preComp) = 0;

  /** Computes the jump map linear approximation.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return The linear approximation of the mapped state after transition
   */
  virtual VectorFunctionLinearApproximation JumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp);

    /**
   * @brief Computes the guard surfaces linear approximation 
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  virtual VectorFunctionLinearApproximation GuardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief.
   *
   * Get partial time derivative of the system flow map.
   *
   * \f$ \frac{\partial f}{\partial t}  \f$.
   *
   * /
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return \f$ \frac{\partial f}{\partial t} \f$ matrix, size \f$ n_x \f$.
   */
  virtual vector_t FlowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief.
   *
   * Get partial time derivative of the system jump map.
   *
   * \f$ \frac{\partial g}{\partial t}  \f$.
   *
   * /
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return \f$ \frac{\partial g}{\partial t} \f$ matrix.
   */
  virtual vector_t JumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief.
   *
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * /
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Derivative of the guard surfaces w.r.t. time.
   */
  virtual vector_t GuardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief.
   *
   * Get at a given operating point the covariance of the dynamics.
   *
   * /
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return The covariance of the dynamics.
   */
  virtual matrix_t DynamicsCovariance(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief.
   *
   * Computes the flow map linear approximation.
   *
   * to the virtual LinearApproximation() with the preComputation parameter.
   *
   * This interface is used by SensitivityIntegrator.
   *
   * /
   * @note This method updates the internal preComputation with the Request() callback and passes it
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u);

    /**
   * @brief Computes the jump map linear approximation.
   *
   * passes it to the virtual JumpMapLinearApproximation() with the preComputation parameter.
   *
   * /
   * @note This method updates the internal preComputation with the RequestPreJump() callback and
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  VectorFunctionLinearApproximation JumpMapLinearApproximation(scalar_t t, const vector_t& x);

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  SystemDynamicsBase(const SystemDynamicsBase& other);
};

}  // namespace bolza
