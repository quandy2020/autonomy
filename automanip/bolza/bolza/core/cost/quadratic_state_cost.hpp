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
 * @file quadratic_state_cost.hpp
 * @brief Quadratic state-only cost term.
 */


#include <bolza/core/cost/state_cost.hpp>

namespace bolza {

/** Quadratic state-only cost term */
class QuadraticStateCost : public StateCost {
 public:
  /**
   * Constructor for the quadratic cost function defined as the following:
   * \f$ \l = 0.5(x-x_{n})' Q (x-x_{n}) \f$.
   * @param [in] Q: \f$ Q \f$
   */
  explicit QuadraticStateCost(matrix_t Q);
  /**
   * @brief Destructor.
   */
  ~QuadraticStateCost() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  QuadraticStateCost* Clone() const override;

    /**
   * @brief Get cost term value 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, const PreComputation&) const final;

    /**
   * @brief Get cost term quadratic approximation 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation&) const final;

 protected:
  /**
   * @brief Constructs a QuadraticStateCost.
   * @param [in] rhs: Rhs.
   */
  QuadraticStateCost(const QuadraticStateCost& rhs) = default;

    /**
   * @brief Computes the state deviation for the nominal state.
   *
   * This method can be overwritten if desiredTrajectory has a different dimensions. 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @return Result value.
   */
  virtual vector_t GetStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const;

 private:
  matrix_t Q_;
};

}  // namespace bolza
