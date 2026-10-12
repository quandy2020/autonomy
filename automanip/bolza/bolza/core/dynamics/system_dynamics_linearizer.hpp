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
 * @file system_dynamics_linearizer.hpp
 * @brief A class for linearizing system dynamics. The linearized system dynamics is defined as: \n.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/finite_difference_methods.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace bolza {

/**
 * A class for linearizing system dynamics. The linearized system dynamics is defined as: \n
 *
 * - Linearized system:   \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 */
class SystemDynamicsLinearizer final : public SystemDynamicsBase {
 public:
    /**
   * @brief Constructor 
   * @param [in] nonlinearSystemPtr: Nonlinear System Ptr.
   * @param [in] doubleSidedDerivative: Double Sided Derivative.
   * @param [in] isSecondOrderSystem: Is Second Order System.
   * @param [in] eps: Eps.
   */
  explicit SystemDynamicsLinearizer(std::unique_ptr<ControlledSystemBase> nonlinearSystemPtr, bool doubleSidedDerivative = true,
                                    bool isSecondOrderSystem = false, scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon());

  /** Default destructor */
  ~SystemDynamicsLinearizer() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  SystemDynamicsLinearizer* Clone() const override;

  /**
   * @brief Compute Flow Map.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;

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
   * @brief Copy constructor with pre-computation 
   * @param [in] other: Other.
   */
  SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other);

  std::unique_ptr<ControlledSystemBase> controlledSystemPtr_;
  bool doubleSidedDerivative_;
  bool isSecondOrderSystem_;
  scalar_t eps_;
};

}  // namespace bolza
