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
 * @file pre_computation.hpp
 * @brief Pre-Computation base class.
 */


#include <bolza/core/computation_request.hpp>
#include <bolza/core/types.hpp>

namespace bolza {

/**
 * Pre-Computation base class.
 *
 * This is an optional module for sharing computation between system dynamics, cost and constraint
 * terms. The virtual request callback are called before getting the value or approximation.
 * The callbacks take a set of requested computation items, which must be computed and stored
 * in the PreComputation object. The same PreComputation is passed to the getter methods of
 * dynamics, cost and constraint terms, which can make use of the shared pre-computation.
 *
 * If pre-computation is not used, a default constructed PreComputation() can be passed to the getters.
 */
class PreComputation {
 public:
  /** Constructor */
  PreComputation() = default;

  /** Default destructor */
  virtual ~PreComputation() = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual PreComputation* Clone() const { /**
  * @brief Pre Computation.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new PreComputation(*this); }

    /**
   * @brief Request callback 
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   */
  virtual void Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {}

    /**
   * @brief Request callback at jump event time 
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   */
  virtual void RequestPreJump(RequestSet request, scalar_t t, const vector_t& x) {}

    /**
   * @brief Request callback at final time 
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   */
  virtual void RequestFinal(RequestSet request, scalar_t t, const vector_t& x) {}

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  PreComputation(const PreComputation& other) = default;
};

/** Helper to cast to const reference of derived class. */
template <typename Derived>
/**
 * @brief Cast.
 * @param [in] preComputation: Pre Computation.
 * @return Result value.
 */
const Derived& cast(const PreComputation& preComputation) {
  static_assert(std::is_base_of<PreComputation, Derived>::value, "Template argument must derive from PreComputation");
  /**
   * @brief Assert.
   */
  assert(dynamic_cast<const Derived*>(&preComputation) != nullptr);
  return static_cast<const Derived&>(preComputation);
}

/** Helper to cast to reference of derived class. */
template <typename Derived>
/**
 * @brief Cast.
 * @param [in,out] preComputation: Pre Computation.
 * @return Result value.
 */
Derived& cast(PreComputation& preComputation) {
  static_assert(std::is_base_of<PreComputation, Derived>::value, "Template argument must derive from PreComputation");
  /**
   * @brief Assert.
   */
  assert(dynamic_cast<Derived*>(&preComputation) != nullptr);
  return static_cast<Derived&>(preComputation);
}

}  // namespace bolza