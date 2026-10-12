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

#include <bolza/core/penalties/penalties/squared_hinge_penalty.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SquaredHingePenalty::GetValue(scalar_t t, scalar_t h) const {
  if (h < config_.delta) {
    const scalar_t delta_h = h - config_.delta;
    return config_.mu * 0.5 * delta_h * delta_h;
  } else {
    return 0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SquaredHingePenalty::GetDerivative(scalar_t t, scalar_t h) const {
  if (h < config_.delta) {
    return config_.mu * (h - config_.delta);
  } else {
    return 0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SquaredHingePenalty::GetSecondDerivative(scalar_t t, scalar_t h) const {
  if (h < config_.delta) {
    return config_.mu;
  } else {
    return 0;
  }
}

}  // namespace bolza
