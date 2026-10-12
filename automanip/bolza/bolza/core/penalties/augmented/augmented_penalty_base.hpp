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
 * @file augmented_penalty_base.hpp
 * @brief The penalty function interface class is used to penalize constraint violation by adding a penalty term to the cost function.
 */


#include <string>

#include <bolza/core/types.hpp>

namespace bolza {
namespace augmented {

/**
 * The penalty function interface class is used to penalize constraint violation by adding a penalty term to the cost function.
 * We assume that the penalty function is convex. In general, the penalty is a function of time, Lagrange multiplier, and
 * constraint violation.
 */
class AugmentedPenaltyBase {
 public:
  /** Default constructor */
  AugmentedPenaltyBase() = default;

  /** Default destructor */
  virtual ~AugmentedPenaltyBase() = default;

  /**
   * @brief Clones the class.
   * @return Pointer; may be null.
   */
  virtual AugmentedPenaltyBase* Clone() const = 0;

  /**
   * @brief Get the name of the penalty function. This method is only used during error handling.
   * @return Result value.
   */
  virtual std::string Name() const = 0;

  /**
   * Compute the penalty value at a certain constraint value.
   *
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] l: The Lagrange multiplier.
   * @param [in] h: Constraint value.
   * @return penalty cost.
   */
  virtual scalar_t GetValue(scalar_t t, scalar_t l, scalar_t h) const = 0;

  /**
   * Compute the penalty derivative at a certain constraint value.
   *
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] l: The Lagrange multiplier.
   * @param [in] h: Constraint value.
   * @return penalty derivative with respect to constraint value.
   */
  virtual scalar_t GetDerivative(scalar_t t, scalar_t l, scalar_t h) const = 0;

  /**
   * Compute the penalty second derivative at a certain constraint value.
   *
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] l: The Lagrange multiplier.
   * @param [in] h: Constraint value.
   * @return penalty second derivative with respect to constraint value.
   */
  virtual scalar_t GetSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const = 0;

  /**
   * Updates the Lagrange multiplier.
   *
   * @param [in] t: The time stamp.
   * @param [in] l: The Lagrange multiplier.
   * @param [in] h: Constraint values.
   * @return updated Lagrange multiplier.
   */
  virtual scalar_t UpdateMultiplier(scalar_t t, scalar_t l, scalar_t h) const = 0;

  /**
   * Initializes the Lagrange multiplier.
   *
   * @return initial value of the Lagrange multiplier.
   */
  virtual scalar_t InitializeMultiplier() const = 0;

 protected:
  /**
   * @brief Constructs a AugmentedPenaltyBase.
   * @param [in] other: Other.
   */
  AugmentedPenaltyBase(const AugmentedPenaltyBase& other) = default;
};

}  // namespace augmented
}  // namespace bolza
