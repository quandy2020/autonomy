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
 * @file loopshaping_pre_computation.hpp
 * @brief Loopshaping Pre-Computation decorator class.
 */


#include <memory>

#include <bolza/core/pre_computation.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

/**
 * Loopshaping Pre-Computation decorator class.
 */
class LoopshapingPreComputation final : public PreComputation {
 public:
  /**
   * @brief Constructs a LoopshapingPreComputation.
   * @param [in] systemPreComputation: System Pre Computation.
   * @param [in] loopshapingDefinition: Loopshaping Definition.
   */
  LoopshapingPreComputation(const PreComputation& systemPreComputation, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);
  /**
   * @brief Destructor.
   */
  ~LoopshapingPreComputation() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingPreComputation* Clone() const override;

  /**
   * @brief Request.
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   */
  void Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
  /**
   * @brief Request Pre Jump.
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   */
  void RequestPreJump(RequestSet request, scalar_t t, const vector_t& x) override;
  /**
   * @brief Request Final.
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   */
  void RequestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  /**
   * @brief System state, computed for the last request.
   * @return Result value.
   */
  const vector_t& GetSystemState() const { return systemState_; }

  /**
   * @brief System input, computed for the last request.
   * @return Result value.
   */
  const vector_t& GetSystemInput() const { return systemInput_; }

  /**
   * @brief Filter state, computed for the last request.
   * @return Result value.
   */
  const vector_t& GetFilterState() const { return filterState_; }

  /**
   * @brief Filter Input, computed for the last request.
   * @return Result value.
   */
  const vector_t& GetFilteredInput() const { return filterInput_; }

  /**
   * @brief Precomputation evaluated for the system state and system input.
   * @return Result value.
   */
  const PreComputation& GetSystemPreComputation() const { return *systemPreCompPtr_; }

 private:
  /**
   * @brief Constructs a LoopshapingPreComputation.
   * @param [in] rhs: Rhs.
   */
  LoopshapingPreComputation(const LoopshapingPreComputation& rhs);

  vector_t systemState_;
  vector_t systemInput_;
  vector_t filterState_;
  vector_t filterInput_;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  std::unique_ptr<PreComputation> systemPreCompPtr_;
};

}  // namespace bolza