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
 * @file initializer.hpp
 * @brief This is the interface class that the solvers use to initialize the state and the input for the time steps that no controller.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * This is the interface class that the solvers use to initialize the state and the input for the time steps that no controller
 * is available. For a trivial implementation of this class, refer to DefaultInitializer class.
 */
class Initializer {
 public:
  /**
   * @brief Constructs a Initializer.
   */
  Initializer() = default;
  /**
   * @brief Destructor.
   */
  virtual ~Initializer() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual Initializer* Clone() const = 0;

  /**
   * Computes the state and input of the next time step based on the current time and state. Note that it guaranteed that there is
   * no event times in the interval [time, nextTime), but there might be an event at nextTime (with nextTime = time + timeStep).
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] nextTime: The next time step.
   * @param [out] input: The current input.
   * @param [out] nextState: The next state.
   */
  virtual void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) = 0;

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] rhs: Rhs.
   */
  Initializer(const Initializer& rhs) = default;
};

}  // namespace bolza
