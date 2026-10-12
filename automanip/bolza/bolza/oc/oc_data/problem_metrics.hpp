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
 * @file problem_metrics.hpp
 * @brief Exchanges the content of ProblemMetrics.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/model_data/metrics.hpp>

namespace bolza {

struct ProblemMetrics {
  Metrics final;
  std::vector<Metrics> preJumps;
  std::vector<Metrics> intermediates;

  /**
   * @brief Exchanges the content of ProblemMetrics.
   * @param [in,out] other: Other.
   */
  void swap(ProblemMetrics& other) {
    /**
     * @brief Swap.
     * @param [in] final: Final.
     * @return Result value.
     */
    final.swap(other.final);
    /**
     * @brief Swap.
     * @param [in] preJumps: Pre Jumps.
     * @return Result value.
     */
    preJumps.swap(other.preJumps);
    /**
     * @brief Swap.
     * @param [in] intermediates: Intermediates.
     * @return Result value.
     */
    intermediates.swap(other.intermediates);
  }

  /** Clears the content of the ProblemMetrics */
  void clear() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    final.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    preJumps.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    intermediates.clear();
  }
};

}  // namespace bolza
