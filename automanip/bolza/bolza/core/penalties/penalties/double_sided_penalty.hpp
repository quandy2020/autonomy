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
 * @file double_sided_penalty.hpp
 * @brief Implements the double sided inequality \f$ l \leq h \leq u \f$ with a given penalty function \f$ p() \f$.
 */


#include <memory>

#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace bolza {

/**
 * Implements the double sided inequality \f$ l \leq h \leq u \f$ with a given penalty function \f$ p() \f$.
 *
 * \f[
 *   p_{box}(h) = p(h - l) + p(u - h)
 * \f]
 */
class DoubleSidedPenalty final : public PenaltyBase {
 public:
  /**
   * Constructor
   * @param [in] lowerBound: The lower bound.
   * @param [in] upperBound: The upper bound.
   * @param [in] penalty: The penalty for the two inequality constraint.
   */
  DoubleSidedPenalty(scalar_t lowerBound, scalar_t upperBound, std::unique_ptr<PenaltyBase> penalty)
      /**
       * @brief Lower Bound.
       * @return Result value.
       */
      : lowerBound_(lowerBound), upperBound_(upperBound), penaltyPtr_(std::move(penalty)) {}

  /**
   * @brief Destructor.
   */
  ~DoubleSidedPenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  DoubleSidedPenalty* Clone() const override { /**
  * @brief Double Sided Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new DoubleSidedPenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "DoubleSidedPenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t h) const override {
    /**
     * @brief Get Value.
     * @param [in] t: T.
     * @param [in] h: H.
     * @return Result value.
     */
    return penaltyPtr_->getValue(t, h - lowerBound_) + penaltyPtr_->getValue(t, upperBound_ - h);
  }
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t h) const override {
    /**
     * @brief Get Derivative.
     * @param [in] t: T.
     * @param [in] h: H.
     * @return Result value.
     */
    return penaltyPtr_->getDerivative(t, h - lowerBound_) - penaltyPtr_->getDerivative(t, upperBound_ - h);
  }
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t h) const override {
    /**
     * @brief Get Second Derivative.
     * @param [in] t: T.
     * @param [in] h: H.
     * @return Result value.
     */
    return penaltyPtr_->getSecondDerivative(t, h - lowerBound_) + penaltyPtr_->getSecondDerivative(t, upperBound_ - h);
  }

 private:
  /**
   * @brief Constructs a DoubleSidedPenalty.
   */
  DoubleSidedPenalty(const DoubleSidedPenalty& other)
      /**
       * @brief Lower Bound.
       * @return Result value.
       */
      : lowerBound_(other.lowerBound_), upperBound_(other.upperBound_), penaltyPtr_(other.penaltyPtr_->clone()) {}

  const scalar_t lowerBound_;
  const scalar_t upperBound_;
  std::unique_ptr<PenaltyBase> penaltyPtr_;
};

}  // namespace bolza
