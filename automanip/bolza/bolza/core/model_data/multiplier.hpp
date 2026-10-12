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
 * @file multiplier.hpp
 * @brief The penalty and Lagrangian multipliers associated to a constraint vector of size Multiplier::lagrangian.size().
 */


#include <algorithm>
#include <string>
#include <vector>

#include "bolza/core/types.hpp"
#include "bolza/core/constraint/state_constraint_collection.hpp"
#include "bolza/core/constraint/state_input_constraint_collection.hpp"
#include "bolza/core/misc/linear_interpolation.hpp"

namespace bolza {

/** The penalty and Lagrangian multipliers associated to a constraint vector of size Multiplier::lagrangian.size() */
struct Multiplier {
  /**
   * @brief Constructs a Multiplier.
   */
  Multiplier() : Multiplier(0.0, vector_t()) {}
  /**
   * @brief Constructs a Multiplier.
   * @param [in] penaltyArg: Penalty Arg.
   */
  Multiplier(scalar_t penaltyArg, vector_t lagrangianArg) : penalty(penaltyArg), lagrangian(std::move(lagrangianArg)) {}

  scalar_t penalty;
  vector_t lagrangian;
};

/** A const reference view to Multiplier. This is useful for having an array of references to Multipliers. */
struct MultiplierConstRef {
  /**
   * @brief Constructs a MultiplierConstRef.
   * @param [in] lagrangian: Lagrangian.
   */
  MultiplierConstRef(const Multiplier& multiplierArg) : penalty(multiplierArg.penalty), lagrangian(multiplierArg.lagrangian) {}
  operator Multiplier() const { return {penalty, lagrangian}; }

  const scalar_t& penalty;
  const vector_t& lagrangian;
};

/**
 * The collection of Multiplier structure for all possible constraint terms in a particular time point.
 * stateEq : An array of state equality constraint terms.
 * stateIneq : An array of state inequality constraint terms.
 * stateInputEq : An array of state-input equality constraint terms.
 * stateInputIneq : An array of state-input inequality constraint terms.
 */
struct MultiplierCollection {
  // state equality
  std::vector<Multiplier> stateEq;
  // state inequality
  std::vector<Multiplier> stateIneq;
  // state-input equality
  std::vector<Multiplier> stateInputEq;
  // state-input inequality
  std::vector<Multiplier> stateInputIneq;

    /**
   * @brief /** Exchanges the values of MultiplierCollection. */.
   * @param [in,out] other: Other.
   */
  void swap(MultiplierCollection& other) {
    /**
     * @brief Swap.
     * @param [in] stateEq: State Eq.
     * @return Result value.
     */
    stateEq.swap(other.stateEq);
    /**
     * @brief Swap.
     * @param [in] stateIneq: State Ineq.
     * @return Result value.
     */
    stateIneq.swap(other.stateIneq);
    /**
     * @brief Swap.
     * @param [in] stateInputEq: State Input Eq.
     * @return Result value.
     */
    stateInputEq.swap(other.stateInputEq);
    /**
     * @brief Swap.
     * @param [in] stateInputIneq: State Input Ineq.
     * @return Result value.
     */
    stateInputIneq.swap(other.stateInputIneq);
  }

  /** Clears the values of the MultiplierCollection. */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    stateEq.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    stateIneq.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    stateInputEq.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    stateInputIneq.clear();
  }

  /**
   * @brief Whether the MultiplierCollection is empty.
   * @return True on success.
   */
  bool empty() const { /**
  * @brief Empty.
  * @return Result value.
  */
 return stateEq.empty() && stateIneq.empty() && stateInputEq.empty() && stateInputIneq.empty(); }
};

/**
 * @brief /**.
 *
 * Serializes an array of Multiplier structures associated to an array of constraint terms.
 *
 * /
 * @param [in] termsMultiplier: Terms Multiplier.
 * @return Serialized vector of the format : (..., termsMultiplier[i].penalty, termsMultiplier[i].lagrangian, ...).
 */
vector_t ToVector(const std::vector<Multiplier>& termsMultiplier);

/**
 * @brief /**.
 *
 * Gets the size of constraint terms.
 *
 * /
 * @param [in] termsMultiplier: Terms Multiplier.
 * @return An array of constraint terms size. It has the same size as the input array.
 */
size_array_t GetSizes(const std::vector<Multiplier>& termsMultiplier);

/**
 * Deserializes the vector to an array of Multiplier structures based on size of constraint terms.
 *
 * @param [in] termsSize: An array of constraint terms size. It as the same size as the output array.
 * @param [in] vec: Serialized array of Multiplier structures of the format :
 *                   (..., termsMultiplier[i].penalty, termsMultiplier[i].lagrangian, ...)
 * @return An array of Multiplier structures associated to an array of constraint terms
 */
std::vector<Multiplier> ToMultipliers(const size_array_t& termsSize, const vector_t& vec);

}  // namespace bolza

namespace bolza {
namespace LinearInterpolation {

/**
 * Linearly interpolates a trajectory of Multipliers.
 *
 * @param [in] indexAlpha: index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray: A trajectory of MultiplierConstRef.
 * @return The interpolated Multiplier at indexAlpha.
 */
Multiplier Interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierConstRef>& dataArray);

/**
 * Linearly interpolates a trajectory of MultiplierCollections.
 *
 * @param [in] indexAlpha: index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray: A trajectory of MultiplierCollections.
 * @return The interpolated MultiplierCollection at indexAlpha.
 */
MultiplierCollection Interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierCollection>& dataArray);

}  // namespace LinearInterpolation
}  // namespace bolza
