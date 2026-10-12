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
 * @file observer.hpp
 * @brief The Observer class stores data in given containers.
 */


#include <bolza/core/types.hpp>

namespace bolza {

/**
 * The Observer class stores data in given containers.
 */
class Observer {
 public:
  /**
   * Constructor.
   *
   * @param [in] stateTrajectoryPtr: A pinter to an state trajectory container to store resulting state trajectory.
   * @param [in] timeTrajectoryPtr: A pinter to an time trajectory container to store resulting time trajectory.
   */
  explicit Observer(vector_array_t* stateTrajectoryPtr = nullptr, scalar_array_t* timeTrajectoryPtr = nullptr);

  /**
   * Default destructor.
   */
  ~Observer() = default;

  /**
   * Observe function to retrieve the variable of interest.
   * @param [in] state: Current state.
   * @param [in] time: Current time.
   */
  void Observe(const vector_t& state, scalar_t time);

 private:
  scalar_array_t* timeTrajectoryPtr_;
  vector_array_t* stateTrajectoryPtr_;
};

}  // namespace bolza
