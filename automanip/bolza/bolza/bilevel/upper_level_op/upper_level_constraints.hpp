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
 * @file upper_level_constraints.hpp
 * @brief This class is an interface to a NLP constraints.
 */


#include <bolza/frank_wolfe/nlp_constraints.hpp>

namespace bolza {

/**
 * This class is an interface to a NLP constraints.
 */
class UpperLevelConstraints final : public NLPConstraints {
 public:
  /**
   * Default constructor.
   */
  UpperLevelConstraints() = default;

  /**
   * Default destructor.
   */
  ~UpperLevelConstraints() override = default;

  /**
   * Sets the initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] finalTime: The final time.
   */
  void set(const scalar_t& initTime, const scalar_t& finalTime) {
    initTime_ = initTime;
    finalTime_ = finalTime;
  }

  /**
   * @brief Set Current Parameter.
   * @param [in] x: X.
   */
  void SetCurrentParameter(const dynamic_vector_t& x) override {
    eventTime_ = x;
    /**
     * @brief Constructs a LinearInequalityCoefficients.
     * @param [in] initTime_: Init Time.
     * @param [in] finalTime_: Final Time.
     * @param [in] Cm_: Cm.
     * @param [in] Dv_: Dv.
     */
    LinearInequalityCoefficients(x.size(), initTime_, finalTime_, Cm_, Dv_);
  }

  /**
   * @brief Get Linear Inequality Constraint.
   * @param [in,out] h: H.
   */
  void GetLinearInequalityConstraint(dynamic_vector_t& h) override { h = Cm_ * eventTime_ + Dv_; }

  /**
   * @brief Get Linear Inequality Constraint Derivative.
   * @param [in,out] dhdx: Dhdx.
   */
  void GetLinearInequalityConstraintDerivative(dynamic_matrix_t& dhdx) override { dhdx = Cm_; }

  /**
   * Computes the inequality constraints coefficients for the following conditions:
   *
   * \f$t_0 < s_0 < s_1 < ... < s_{n-1} < t_f\f$,
   *
   * where \f$t_0\f$ is the initial time and \f$t_f\f$ is the final time.
   * \f$n\f$ is the number of event times and \f$s_i\f$ is the ith event time.
   * The inequality constraints have the following format: \f$C_m [s_i] + D_v > 0\f$.
   *
   * @param [in] numEventTimes: The number of event times, \f$n\f$.
   * @param [in] initTime: The initial time, \f$t_0\f$.
   * @param [in] finalTime: The final time, \f$t_f\f$.
   * @param [out] Cm: \f$C_m\f$ matrix of the inequality constraints.
   * @param [out] Dv: \f$D_v\f$ vector of the inequality constraints.
   *
   */
  static void LinearInequalityCoefficients(size_t numEventTimes, scalar_t initTime, scalar_t finalTime, dynamic_matrix_t& Cm,
                                           dynamic_vector_t& Dv) {
    /**
     * @brief Zero.
     * @param [in] numEventTimes: Num Event Times.
     * @return Result value.
     */
    Cm = dynamic_matrix_t::Zero(numEventTimes + 1, numEventTimes);
    for (size_t i = 0; i < numEventTimes + 1; i++) {
      if (i < numEventTimes) {
        Cm(i, i) = +1.0;
      }
      if (i > 0) {
        Cm(i, i - 1) = -1.0;
      }
    }

    /**
     * @brief Zero.
     * @return Result value.
     */
    Dv = dynamic_vector_t::Zero(numEventTimes + 1);
    Dv(0) = -initTime;
    Dv(numEventTimes) = finalTime;
  }

 private:
  scalar_t initTime_;
  scalar_t finalTime_;

  dynamic_vector_t eventTime_;

  dynamic_matrix_t Cm_;
  dynamic_vector_t Dv_;
};

}  // namespace bolza
