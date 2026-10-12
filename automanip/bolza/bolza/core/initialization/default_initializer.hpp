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
 * @file default_initializer.hpp
 * @brief This is a default implementation of the Initializer where it sets the input to zero and the next state to the current state.
 */


#include <bolza/core/initialization/initializer.hpp>

namespace bolza {

/**
 * This is a default implementation of the Initializer where it sets the input to zero and the next state to the current state.
 */
class DefaultInitializer final : public Initializer {
 public:
  /**
   * Constructor
   * @param [in] inputDim: The dimension of the input space.
   */
  explicit DefaultInitializer(size_t inputDim) : inputDim_(inputDim) {}

  /**
   * @brief Destructor.
   */
  ~DefaultInitializer() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  DefaultInitializer* Clone() const override { /**
  * @brief Default Initializer.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new DefaultInitializer(*this); }

  /**
   * @brief Compute.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] nextTime: Next Time.
   * @param [in,out] input: Input.
   * @param [in,out] nextState: Next State.
   */
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override {
    /**
     * @brief Set Zero.
     * @param [in] inputDim_: Input Dim.
     * @return Result value.
     */
    input.setZero(inputDim_);
    nextState = state;
  }

 protected:
  /**
   * @brief Constructs a DefaultInitializer.
   * @param [in] rhs: Rhs.
   */
  DefaultInitializer(const DefaultInitializer& rhs) = default;

  size_t inputDim_;
};

}  // namespace bolza
