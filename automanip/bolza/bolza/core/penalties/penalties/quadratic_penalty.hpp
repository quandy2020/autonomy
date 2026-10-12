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
 * @file quadratic_penalty.hpp
 * @brief Implements the penalty for a single equality constraint \f$ h = 0 \f$. This leads to the following:.
 */


#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace bolza {

/**
 *  Implements the penalty for a single equality constraint \f$ h = 0 \f$. This leads to the following:
 *
 *  \f[
 *      L_{A} = \frac{\mu}{2} h^2.
 *  \f]
 *
 *  where \f$ \mu \f$ is the scale.
 */
class QuadraticPenalty final : public PenaltyBase {
 public:
  /**
   * Thos constructor sets both the scale and stepLength the same. This is a common practice in Augmented Lagrangian.
   * @param [in] scale: Scaling of the cost.
   */
  explicit QuadraticPenalty(scalar_t scale) : scale_(scale) {}

  /**
   * @brief Destructor.
   */
  ~QuadraticPenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  QuadraticPenalty* Clone() const override { /**
  * @brief Quadratic Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new QuadraticPenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "QuadraticPenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t h) const override { return 0.5 * scale_ * h * h; }
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t h) const override { return scale_ * h; }
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t h) const override { return scale_; }

 private:
  /**
   * @brief Constructs a QuadraticPenalty.
   * @param [in] other: Other.
   */
  QuadraticPenalty(const QuadraticPenalty& other) = default;

  const scalar_t scale_;
};

}  // namespace bolza
