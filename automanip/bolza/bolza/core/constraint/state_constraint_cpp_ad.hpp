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
 * @file state_constraint_cpp_ad.hpp
 * @brief CppAD state-only constraint base class.
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include <bolza/core/constraint/state_constraint.hpp>

namespace bolza {

/** CppAD state-only constraint base class*/
class StateConstraintCppAd : public StateConstraint {
 public:
  /**
   * @brief Constructs a StateConstraintCppAd.
   * @param [in] order: Order.
   */
  explicit StateConstraintCppAd(ConstraintOrder order) : StateConstraint(order) {}
  /**
   * @brief Destructor.
   */
  ~StateConstraintCppAd() override = default;

  /** Initialize the CppAd interface
   * @param [in] stateDim: state vector dimension.
   * @param [in] parameterDim: parameter vector dimension, set to 0 if GetParameters() is not used.
   * @param [in] modelName: Name of the generate model library.
   * @param [in] modelFolder: Folder where the model library files are saved.
   * @param [in] recompileLibraries: If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose: Print information.
   */
  void Initialize(size_t stateDim, size_t parameterDim, const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                  bool recompileLibraries = true, bool verbose = true);

    /**
   * @brief /** Get the parameter vector */.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual vector_t GetParameters(scalar_t time, const PreComputation& /* preComputation */) const { /**
  * @brief Vector t.
  * @return Result value.
  */
 return vector_t(0); };

    /**
   * @brief /** Constraint evaluation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  /**
   * @brief Get Linear Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;
  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComputation) const override;

 protected:
  /**
   * @brief Constructs a StateConstraintCppAd.
   * @param [in] rhs: Rhs.
   */
  StateConstraintCppAd(const StateConstraintCppAd& rhs);

    /**
   * @brief /** The CppAD constraint function */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  virtual ad_vector_t ConstraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const = 0;

 private:
  std::unique_ptr<bolza::CppAdInterface> adInterfacePtr_;
};

}  // namespace bolza
