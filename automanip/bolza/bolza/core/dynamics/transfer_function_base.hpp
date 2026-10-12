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
 * @file transfer_function_base.hpp
 * @brief transfer function base (bolza).
 */


#include <bolza/core/types.hpp>

namespace bolza {

class TransferFunctionBase {
 public:
  /**
   * @brief Constructs a TransferFunctionBase.
   * @param [in] numCoefficients: Num Coefficients.
   * @param [in] denCoefficients: Den Coefficients.
   * @param [in] timedelay: Timedelay.
   * @param [in] balance: Balance.
   */
  TransferFunctionBase(vector_t numCoefficients, vector_t denCoefficients, scalar_t timedelay = 0.0, bool balance = true);

  /**
   * @brief Absorb Delay.
   * @param [in] numZeros: Num Zeros.
   * @param [in] numPoles: Num Poles.
   */
  void AbsorbDelay(size_t numZeros, size_t numPoles);

  /**
   * @brief Normalize.
   */
  void normalize();

  /**
   * @brief Get State Space.
   * @param [in,out] A: A.
   * @param [in,out] B: B.
   * @param [in,out] C: C.
   * @param [in,out] D: D.
   */
  void GetStateSpace(matrix_t& A, matrix_t& B, matrix_t& C, matrix_t& D);

 private:
  vector_t numCoefficients_;
  vector_t denCoefficients_;
  scalar_t timeDelay_;
  const scalar_t delayTol_ = 1e-6;
  bool delayAbsorbed_ = false;
  bool balance_;
};

/**
 * @brief Tf2ss.
 * @param [in] numCoefficients: Num Coefficients.
 * @param [in] denCoefficients: Den Coefficients.
 * @param [in,out] A: A.
 * @param [in,out] B: B.
 * @param [in,out] C: C.
 * @param [in,out] D: D.
 * @param [in] timeDelay: Time Delay.
 * @param [in] balance: Balance.
 */
inline void Tf2ss(vector_t numCoefficients, vector_t denCoefficients, matrix_t& A, matrix_t& B, matrix_t& C, matrix_t& D,
                  scalar_t timeDelay = 0.0, bool balance = true) {
  /**
   * @brief Tf.
   * @param [in] numCoefficients: Num Coefficients.
   * @param [in] denCoefficients: Den Coefficients.
   * @param [in] timeDelay: Time Delay.
   * @param [in] balance: Balance.
   * @return Result value.
   */
  TransferFunctionBase tf(numCoefficients, denCoefficients, timeDelay, balance);
  /**
   * @brief Get State Space.
   * @param [in] A: A.
   * @param [in] B: B.
   * @param [in] C: C.
   * @param [in] D: D.
   * @return Result value.
   */
  tf.GetStateSpace(A, B, C, D);
}

};  // namespace bolza
