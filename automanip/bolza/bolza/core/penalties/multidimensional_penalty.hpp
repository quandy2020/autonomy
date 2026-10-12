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
 * @file multidimensional_penalty.hpp
 * @brief A helper class that implements the penalty for multidimensional constraint.
 */


#include <memory>

#include <bolza/core/types.hpp>

#include <bolza/core/penalties/penalties/penalty_base.hpp>
#include "bolza/core/penalties/augmented/augmented_penalty_base.hpp"

namespace bolza {

/**
 *   A helper class that implements the penalty for multidimensional constraint
 *   \f$ h_i(x, u) \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty(t, x, u) = \f$ \sum_{i=1}^{M} p(t, h_i(x, u)) \f$
 *
 *   This class uses the chain rule to compute the second-order approximation of the constraint-penalty. In the case that the
 *   second-order approximation of constraint is not provided, it employs a Gauss-Newton approximation technique which only
 *   relies on the first-order approximation. In general, the penalty function can be a function of time.
 */
class MultidimensionalPenalty final {
 public:
  /**
   * Constructor
   * @note This imposes a fixed number of constraints, where the corresponding penalty function in the array is applied.
   * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
   */
  template <class PenaltyType>
  /**
   * @brief Constructs a MultidimensionalPenalty.
   * @param [in] penaltyPtrArray: Penalty Ptr Array.
   */
  MultidimensionalPenalty(std::vector<std::unique_ptr<PenaltyType>> penaltyPtrArray);

  /**
   * Constructor with s single penalty function
   * @note This allows a varying number of constraints and uses the same penalty function for each constraint.
   * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
   */
  template <class PenaltyType>
  /**
   * @brief Constructs a MultidimensionalPenalty.
   * @param [in] penaltyPtr: Penalty Ptr.
   */
  MultidimensionalPenalty(std::unique_ptr<PenaltyType> penaltyPtr);

  /** Default destructor */
  ~MultidimensionalPenalty() = default;

    /**
   * @brief /** Copy constructor */.
   * @param [in] other: Other.
   */
  MultidimensionalPenalty(const MultidimensionalPenalty& other);

    /**
   * @brief /**.
   *
   * Get the penalty cost.
   *
   * /
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] l: L.
   * @return Penalty: The penalty cost.
   */
  scalar_t GetValue(scalar_t t, const vector_t& h, const vector_t* l = nullptr) const;

    /**
   * @brief /**.
   *
   * Get the derivative of the penalty cost.
   *
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * /
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] h: The constraint linear approximation.
   * @param [in] l: L.
   * @return The penalty cost quadratic approximation.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const VectorFunctionLinearApproximation& h,
                                                                 const vector_t* l = nullptr) const;

    /**
   * @brief /**.
   *
   * Get the derivative of the penalty cost.
   *
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * /
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] h: The constraint quadratic approximation.
   * @param [in] l: L.
   * @return The penalty cost quadratic approximation.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t t, const VectorFunctionQuadraticApproximation& h,
                                                                 const vector_t* l = nullptr) const;

  /**
   * Updates the Lagrange multipliers.
   *
   * @param [in] t: The time stamp.
   * @param [in] l: The Lagrange multipliers.
   * @param [in] h: The vector of constraint values.
   * @return updated Lagrange multipliers.
   */
  vector_t UpdateMultipliers(scalar_t t, const vector_t& h, const vector_t& l) const;

  /**
   * Initializes the Lagrange multipliers.
   *
   * @param [in] numConstraints: Number of constraints associated to this penalty.
   * @return Initial Lagrange multipliers.
   */
  vector_t InitializeMultipliers(size_t numConstraints) const;

 private:
  /**
   * @brief Get Penalty Value1st Dev2nd Dev.
   * @param [in] t: T.
   * @param [in] h: H.
   * @param [in] l: L.
   * @return Result value.
   */
  std::tuple<scalar_t, vector_t, vector_t> GetPenaltyValue1stDev2ndDev(scalar_t t, const vector_t& h, const vector_t* l) const;

  std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray_;
};

}  // namespace bolza
