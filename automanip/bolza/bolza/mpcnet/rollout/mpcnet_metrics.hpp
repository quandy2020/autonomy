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
 * @file mpcnet_metrics.hpp
 * @brief Metrics computed during the policy evaluation rollout.
 */


#include <bolza/core/types.hpp>

namespace bolza {
namespace mpcnet {

/**
 * Metrics computed during the policy evaluation rollout.
 */
struct Metrics {
  /** Survival time. */
  scalar_t survivalTime = 0.0;
  /** Hamiltonian incurred over time. */
  scalar_t incurredHamiltonian = 0.0;
};
using metrics_t = Metrics;
using metrics_array_t = std::vector<metrics_t>;

}  // namespace mpcnet
}  // namespace bolza
