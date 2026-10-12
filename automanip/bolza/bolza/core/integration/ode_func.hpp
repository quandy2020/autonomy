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
 * @file ode_func.hpp
 * @brief ode func (bolza).
 */


#include <functional>

#include "ode_base.hpp"

namespace bolza {

class OdeFunc final : public OdeBase {
 public:
  /**
   * @brief Constructs a OdeFunc.
   */
  explicit OdeFunc(std::function<vector_t(scalar_t t, const vector_t& x)> flowMap) : flowMap_(std::move(flowMap)) {}

  /**
   * @brief Compute Flow Map.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t t, const vector_t& x) override { /**
  * @brief Flow Map.
  * @param [in] t: T.
  * @param [in] x: X.
  * @return Result value.
  */
 return flowMap_(t, x); }

  /**
   * @brief Set Flow Map.
   * @param [in] flowMap: Flow Map.
   */
  void SetFlowMap(std::function<vector_t(scalar_t t, const vector_t& x)>&& flowMap) { flowMap_ = flowMap; }

 private:
  std::function<vector_t(scalar_t t, const vector_t& x)> flowMap_;
};

}  // namespace bolza
