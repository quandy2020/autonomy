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
 * @file end_effector_distance_constraint_cpp_ad.hpp
 * @brief End-effector distance constraint Function.
 */


#include <memory>
#include <string>
#include <utility>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/constraint/state_input_constraint.hpp>
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>

#include "bolza/perceptive/distance_transform/distance_transform_interface.hpp"

namespace bolza {

/**
 * End-effector distance constraint Function.
 */
class EndEffectorDistanceConstraintCppAd final : public bolza::StateInputConstraint {
 public:
  struct Config {
    Config(scalar_t weightParam = 1.0, bool generateModelParam = true, bool verboseParam = true)
        /**
         * @brief Weight.
         * @param [in] verboseParam: Verbose Param.
         * @return Result value.
         */
        : Weight(weightParam), generateModel(generateModelParam), verbose(verboseParam) {}
    scalar_t weight;
    bool generateModel;
    bool verbose;
  };

  /**
   * @brief Constructor.
   * @param [in] stateDim: State Dim.
   * @param [in] inputDim: Input Dim.
   * @param [in] config: Config.
   * @param [in] adKinematicsPtr: Ad Kinematics Ptr.
   */
  EndEffectorDistanceConstraintCppAd(size_t stateDim, size_t inputDim, Config config,
                                     std::unique_ptr<EndEffectorKinematics<ad_scalar_t>> adKinematicsPtr);

  /** Default destructor */
  ~EndEffectorDistanceConstraintCppAd() override = default;

  /**
   * @brief Set.
   * @param [in] clearance: Clearance.
   * @param [in] distanceTransform: Distance Transform.
   */
  void set(scalar_t clearance, const DistanceTransformInterface& distanceTransform);
  /**
   * @brief Set.
   * @param [in] clearances: Clearances.
   * @param [in] distanceTransform: Distance Transform.
   */
  void set(vector_t clearances, const DistanceTransformInterface& distanceTransform);

  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  EndEffectorDistanceConstraintCppAd* Clone() const override { return new EndEffectorDistanceConstraintCppAd(*this); }
  /**
   * @brief Get Num Constraints.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const override { return adKinematicsPtr_->getIds().size(); }
  /**
   * @brief Get IDs.
   * @return Result value.
   */
  const std::vector<std::string>& GetIDs() const { return adKinematicsPtr_->getIds(); }

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  /**
   * @brief Get Linear Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

  /**
   * @brief Get Kinematics AD.
   * @return Result value.
   */
  EndEffectorKinematics<ad_scalar_t>& GetKinematicsAD() { return *adKinematicsPtr_; }
  /**
   * @brief Get State Input Dimensions.
   * @return Result value.
   */
  std::pair<size_t, size_t> GetStateInputDimensions() const { return {stateDim_, inputDim_}; }

 private:
  /**
   * @brief Constructs a EndEffectorDistanceConstraintCppAd.
   * @param [in] other: Other.
   */
  EndEffectorDistanceConstraintCppAd(const EndEffectorDistanceConstraintCppAd& other);

  const size_t stateDim_;
  const size_t inputDim_;
  const Config config_;

  std::unique_ptr<EndEffectorKinematics<ad_scalar_t>> adKinematicsPtr_;
  std::unique_ptr<CppAdInterface> kinematicsModelPtr_;

  vector_t clearances_;
  const DistanceTransformInterface* distanceTransformPtr_ = nullptr;
};

}  // namespace bolza
