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
 * @file quadratic_state_input_cost.hpp
 * @brief Quadratic state-input cost term.
 */


#include <utility>

#include <bolza/core/cost/state_input_cost.hpp>

namespace bolza {

/** Quadratic state-input cost term */
class QuadraticStateInputCost : public StateInputCost {
 public:
  /**
   * Constructor for the quadratic cost function defined as the following:
   * \f$ L = 0.5(x-x_{n})' Q (x-x_{n}) + 0.5(u-u_{n})' R (u-u_{n}) + (u-u_{n})' P (x-x_{n}) \f$
   * @param [in] Q: \f$ Q \f$
   * @param [in] R: \f$ R \f$
   * @param [in] P: \f$ P \f$
   */
  QuadraticStateInputCost(matrix_t Q, matrix_t R, matrix_t P = matrix_t());
  /**
   * @brief Destructor.
   */
  ~QuadraticStateInputCost() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  QuadraticStateInputCost* Clone() const override;

    /**
   * @brief Get cost term value 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation&) const final;

    /**
   * @brief Get cost term quadratic approximation 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation&) const final;

 protected:
  /**
   * @brief Constructs a QuadraticStateInputCost.
   * @param [in] rhs: Rhs.
   */
  QuadraticStateInputCost(const QuadraticStateInputCost& rhs) = default;

    /**
   * @brief Computes the state-input deviation pair around the nominal state and input.
   *
   * This method can be overwritten if desiredTrajectory has a different dimensions. 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  virtual std::pair<vector_t, vector_t> GetStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                               const TargetTrajectories& targetTrajectories) const;

 private:
  matrix_t Q_;
  matrix_t R_;
  matrix_t P_;
};

}  // namespace bolza
