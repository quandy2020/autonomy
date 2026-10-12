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

#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/model_data/model_data.hpp>

/*
 * @file
 * The linear interpolation of cost inpute-state derivative, Pm, at index-alpha pair given modelDataTrajectory can
 * be computed as:
 *
 * LinearInterpolation::Interpolate(indexAlpha, Pm, &modelDataTrajectory, model_data::cost_dfdux);
 */

/*
 * Declares an access function of name FIELD such as time, dynamics, dynamicsBias, ...
 * For example the signature of function for dynamics is:
 * const vector_t& dynamics(const std::vector<bolza::ModelData>& vec, size_t n) {
 *   return vec[n].dynamics;
 * }
 */
#define CREATE_INTERPOLATION_ACCESS_FUNCTION(FIELD) \
  inline auto FIELD(const std::vector<bolza::ModelData>& vec, size_t ind)->const decltype(vec[ind].FIELD)& { return vec[ind].FIELD; }

#define CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(FIELD, SUBFIELD)                                                            \
  inline auto FIELD##_##SUBFIELD(const std::vector<bolza::ModelData>& vec, size_t ind)->const decltype(vec[ind].FIELD.SUBFIELD)& { \
    return vec[ind].FIELD.SUBFIELD;                                                                                               \
  }

namespace bolza {
namespace model_data {

/**
 * Access method for different subfields of the ModelData.
 */

// Time
CREATE_INTERPOLATION_ACCESS_FUNCTION(time)

// Dynamics
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsBias)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsCovariance)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(dynamics, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(dynamics, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(dynamics, dfdu)

// Cost
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdxx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfduu)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdux)

// State equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateEqConstraint, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateEqConstraint, dfdx)

// State-input equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateInputEqConstraint, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateInputEqConstraint, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateInputEqConstraint, dfdu)

}  // namespace model_data
}  // namespace bolza

#undef CREATE_INTERPOLATION_ACCESS_FUNCTION
#undef CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD
