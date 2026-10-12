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

#include <bolza/core/penalties/penalties/relaxed_barrier_penalty.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierPenalty::GetValue(scalar_t t, scalar_t h) const {
  if (h > config_.delta) {
    return -config_.mu * log(h);
  } else {
    const scalar_t delta_h = (h - 2.0 * config_.delta) / config_.delta;
    return config_.mu * (-log(config_.delta) + 0.5 * delta_h * delta_h - 0.5);
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierPenalty::GetDerivative(scalar_t t, scalar_t h) const {
  if (h > config_.delta) {
    return -config_.mu / h;
  } else {
    return config_.mu * ((h - 2.0 * config_.delta) / (config_.delta * config_.delta));
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierPenalty::GetSecondDerivative(scalar_t t, scalar_t h) const {
  if (h > config_.delta) {
    return config_.mu / (h * h);
  } else {
    return config_.mu / (config_.delta * config_.delta);
  };
}

}  // namespace bolza
