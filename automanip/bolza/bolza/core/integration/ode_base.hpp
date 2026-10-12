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
 * @file ode_base.hpp
 * @brief The base class for autonomous system dynamics.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * The base class for autonomous system dynamics.
 */
class OdeBase {
 public:
  /** Default constructor */
  OdeBase() = default;

  /** Default destructor */
  virtual ~OdeBase() = default;

  /**
   * @brief Returns the number of function calls.
   * @return Result value.
   */
  size_t GetNumFunctionCalls() const { return numFunctionCalls_; }

  /** Resets the number of function calls to zero. */
  void ResetNumFunctionCalls() { numFunctionCalls_ = 0; }

  /**
   * @brief Increments the number of function calls.
   * @return Result value.
   */
  size_t IncrementNumFunctionCalls() { return ++numFunctionCalls_; }

  /**
   * Computes the autonomous system dynamics.
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current state time derivative
   */
  virtual vector_t ComputeFlowMap(scalar_t t, const vector_t& x) = 0;

  /**
   * State map at the transition time
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @return mapped state after transition
   */
  virtual vector_t ComputeJumpMap(scalar_t time, const vector_t& state);

  /**
   * Interface method to the guard surfaces.
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @return An array of guard surfaces values
   */
  virtual vector_t ComputeGuardSurfaces(scalar_t time, const vector_t& state);

 protected:
  /** Copy constructor */
  OdeBase(const OdeBase& rhs) : numFunctionCalls_(0) {}

 private:
  size_t numFunctionCalls_ = 0;
};

}  // namespace bolza
