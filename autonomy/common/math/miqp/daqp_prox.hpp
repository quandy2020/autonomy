/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file daqp_prox.hpp
 * @brief Proximal-point wrapper around the dual active-set method.
 *
 * Vendored DAQP (MIT). A singular or poorly conditioned Hessian is solved as
 * a sequence of regularized least-distance programs. SANDO’S mixed-integer
 * front end does not call this path; it regularizes the Hessian on the
 * Eigen side instead.
 */

#ifndef DAQP_PROX_H
# define DAQP_PROX_H


#include "autonomy/common/math/miqp/types.hpp"
#include "autonomy/common/math/miqp/constants.hpp"
#include "autonomy/common/math/miqp/daqp.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Solve the quadratic in @p work by proximal regularization.
 *
 * @param[in,out] work Workspace whose Hessian needs a proximal shift.
 * @return DAQP exit flag (positive on success).
 */
int DaqpProx(DAQPWorkspace* work);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
