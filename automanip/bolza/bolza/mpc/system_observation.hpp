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
 * @file system_observation.hpp
 * @brief This class contains the observation information.
 */


#include <ostream>

#include <bolza/core/types.hpp>

namespace bolza {

/**
 * This class contains the observation information.
 */
struct SystemObservation {
  size_t mode = 0;
  scalar_t time = 0.0;
  vector_t state;
  vector_t input;

  /**
   * @brief Swap.
   * @param [in,out] a: A.
   * @param [in,out] b: B.
   */
  friend void swap(SystemObservation& a, SystemObservation& b) noexcept;
};

/**
 * @brief Swap.
 * @param [in,out] a: A.
 * @param [in,out] b: B.
 */
void swap(SystemObservation& a, SystemObservation& b) noexcept;

std::ostream& operator<<(std::ostream& out, const SystemObservation& observation);

}  // namespace bolza
