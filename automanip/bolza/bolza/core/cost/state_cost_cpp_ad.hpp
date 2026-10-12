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
 * @file state_cost_cpp_ad.hpp
 * @brief CppAD state-only cost base class.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include <bolza/core/cost/state_cost.hpp>

namespace bolza {

/** CppAD state-only cost base class*/
class StateCostCppAd : public StateCost {
 public:
  /**
   * @brief Constructs a StateCostCppAd.
   */
  StateCostCppAd() = default;
  /**
   * @brief Destructor.
   */
  ~StateCostCppAd() override = default;

  /** Initialize the CppAd interface. */
  void Initialize(size_t stateDim, size_t parameterDim, const std::string& modelName,
                  const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  /** Get the parameter vector. */
  virtual vector_t GetParameters(scalar_t time, const TargetTrajectories& targetTrajectories,
                                 const PreComputation& /* preComputation */) const {
    return vector_t(0);
  }

  scalar_t GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 protected:
  /**
   * @brief Constructs a StateCostCppAd.
   * @param [in] rhs: Rhs.
   */
  StateCostCppAd(const StateCostCppAd& rhs);

    /**
   * @brief The CppAD cost function 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  virtual ad_scalar_t CostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const = 0;

 private:
  std::unique_ptr<bolza::CppAdInterface> adInterfacePtr_;
};

}  // namespace bolza
