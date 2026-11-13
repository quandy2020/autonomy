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
 * @file daqp.hpp
 * @brief Dual active-set method for the least-distance program.
 *
 * Vendored DAQP (MIT). DaqpLdp walks the working set until the primal and
 * dual are feasible (or a resource limit is hit). Ldp2qpSolution maps the
 * least-distance iterate back to the original quadratic. ResetDaqpWorkspace
 * clears the active set without freeing memory.
 */

#ifndef DAQP_H
# define DAQP_H


#include "autonomy/common/math/miqp/factorization.hpp"
#include "autonomy/common/math/miqp/constants.hpp"
#include "autonomy/common/math/miqp/auxiliary.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Solve the least-distance program stored in @p work.
 *
 * @param[in,out] work Workspace whose least-distance data is already formed.
 * @return DAQP exit flag (positive on success).
 */
int DaqpLdp(DAQPWorkspace* work);

/**
 * @brief Map the least-distance solution in @p work to the original quadratic.
 *
 * @param[in,out] work Workspace whose x, lam, and fval are overwritten.
 */
void Ldp2qpSolution(DAQPWorkspace* work);

/**
 * @brief Clear the active set and singularity index of @p work.
 *
 * @param[in,out] work Workspace to reset; allocations are kept.
 */
void ResetDaqpWorkspace(DAQPWorkspace* work);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
