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
 * @file riccati_modification_interpolation.hpp
 * @brief riccati modification interpolation (bolza).
 */


#include <bolza/core/misc/linear_interpolation.hpp>

#include "riccati_modification.hpp"

// Declares an access function of name FIELD (e.g., time, DmDagger, ...)
#define CREATE_INTERPOLATION_ACCESS_FUNCTION(FIELD)                                                                             \
  inline auto FIELD(const std::vector<bolza::riccati_modification::Data>& vec, size_t ind)->const decltype(vec[ind].FIELD##_)& { \
    return vec[ind].FIELD##_;                                                                                                   \
  }

namespace bolza {
namespace riccati_modification {

CREATE_INTERPOLATION_ACCESS_FUNCTION(time)

CREATE_INTERPOLATION_ACCESS_FUNCTION(deltaQm)
CREATE_INTERPOLATION_ACCESS_FUNCTION(deltaGm)
CREATE_INTERPOLATION_ACCESS_FUNCTION(deltaGv)

CREATE_INTERPOLATION_ACCESS_FUNCTION(hamiltonianHessian)
CREATE_INTERPOLATION_ACCESS_FUNCTION(constraintRangeProjector)
CREATE_INTERPOLATION_ACCESS_FUNCTION(constraintNullProjector)

}  // namespace riccati_modification
}  // namespace bolza

#undef CREATE_INTERPOLATION_ACCESS_FUNCTION
