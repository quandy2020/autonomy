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
 * @file nlp_cost.hpp
 * @brief This class is an interface to a NLP cost.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * This class is an interface to a NLP cost.
 */
class NLPCost {
 public:
  /**
   * Default constructor.
   */
  NLPCost() = default;

  /**
   * Default destructor.
   */
  virtual ~NLPCost() = default;

  /**
   * Sets the current parameter vector.
   *
   * @param [in] x: The value of parameter vector.
   * @return id: It returns a number which identifies the cached data.
   */
  virtual size_t SetCurrentParameter(const vector_t& x) = 0;

  /**
   * Gets the cost value.
   *
   * @param [in] id: The ID of the cached data.
   * @param [out] f: The value of the cost.
   * @return status: whether the cost computation was successful.
   */
  virtual bool GetCost(size_t id, scalar_t& f) = 0;

  /**
   * Gets the gradient of the cost w.r.t. parameter vector.
   *
   * @param [in] id: The ID of the cached data.
   * @param [out] g: The gradient of the cost.
   */
  virtual void GetCostDerivative(size_t id, vector_t& g) = 0;

  /**
   * Gets the Hessian of the cost w.r.t. parameter vector.
   *
   * @param [in] id: The ID of the cached data.
   * @param [out] H: The Hessian of the cost.
   */
  virtual void GetCostSecondDerivative(size_t id, matrix_t& H) = 0;

  /**
   * Clears the cache.
   */
  virtual void ClearCache() = 0;
};

}  // namespace bolza
