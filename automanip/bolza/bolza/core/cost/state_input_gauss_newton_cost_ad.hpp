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
 * @file state_input_gauss_newton_cost_ad.hpp
 * @brief State-input cost term of the form  0.5 ||f(x,u,p)||^2, where the linear approximation of f(x,u,p) will be used to form the Hessian.
 */


#include "bolza/core/cost/state_input_cost.hpp"

#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>

namespace bolza {

/**
 * State-input cost term of the form  0.5 ||f(x,u,p)||^2, where the linear approximation of f(x,u,p) will be used to form the Hessian.
 *
 * The cost approximation is guaranteed to be positive semi-definite, and is given by:
 *  c = 0.5 ||f(x,u,p)||^2
 *  dcdx = dfdx(x,u,p)' * f(x,u,p)
 *  dcdu = dfdu(x,u,p)' * f(x,u,p)
 *  dcdxx = dfdx(x,u,p)' * dfdx(x,u,p)
 *  dcdux = dfdu(x,u,p)' * dfdx(x,u,p)
 *  dcduu = dfdu(x,u,p)' * dfdu(x,u,p)
 */
class StateInputCostGaussNewtonAd : public StateInputCost {
 public:
  /**
   * @brief Constructs a StateInputCostGaussNewtonAd.
   */
  StateInputCostGaussNewtonAd() = default;
  /**
   * @brief Destructor.
   */
  ~StateInputCostGaussNewtonAd() override = default;

  /**
   * Initializes model libraries
   * @param [in] stateDim: state vector dimension.
   * @param [in] inputDim: state vector dimension.
   * @param [in] parameterDim: parameter vector dimension, set to 0 if GetParameters() is not used.
   * @param [in] modelName: Name of the generate model library.
   * @param [in] modelFolder: Folder where the model library files are saved.
   * @param [in] recompileLibraries: If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose: Print information.
   */
  void Initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                  const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

    /**
   * @brief /** Get the parameter vector */.
   * @param [in] time: Time.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  virtual vector_t GetParameters(scalar_t time, const TargetTrajectories& targetTrajectories,
                                 const PreComputation& /* preComputation */) const {
    /**
     * @brief Vector t.
     * @return Result value.
     */
    return vector_t(0);
  };

    /**
   * @brief /** Cost evaluation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComputation) const override;
  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComputation) const override;

 protected:
  /**
   * @brief Constructs a StateInputCostGaussNewtonAd.
   * @param [in] rhs: Rhs.
   */
  StateInputCostGaussNewtonAd(const StateInputCostGaussNewtonAd& rhs);

  /**
   * Interface method to the cost function. This method must be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] parameters: parameter vector.
   * @return vector of cost values f(x,u,p)
   */
  virtual ad_vector_t CostVectorFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                         const ad_vector_t& parameters) const = 0;

 private:
  std::unique_ptr<CppAdInterface> adInterfacePtr_;
};

}  // namespace bolza
