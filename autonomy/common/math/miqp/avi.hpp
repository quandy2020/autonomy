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
 * @file avi.hpp
 * @brief Affine variational inequality solver on a DAQP workspace.
 *
 * Vendored DAQP (MIT). The KKT system of the affine variational inequality
 * is factored once; DaqpSolveAvi then walks complementary cones until the
 * residual is optimal or a resource limit is hit.
 */

#ifndef DAQP_AVI_H
# define DAQP_AVI_H


#include "autonomy/common/math/miqp/types.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Solve the affine variational inequality stored in @p work.
 *
 * @param[in,out] work Workspace whose avi attachment is populated.
 * @return DAQP exit flag (positive on success).
 */
int DaqpSolveAvi(DAQPWorkspace* work);

/**
 * @brief Solve the KKT system of the current complementary cone.
 *
 * @param[in,out] work Workspace whose avi->x and related buffers are updated.
 */
void DaqpSolveAviKkt(DAQPWorkspace* work);

/**
 * @brief Test complementarity and feasibility of the current AVI iterate.
 *
 * @param[in] work Workspace after DaqpSolveAviKkt.
 * @return Nonzero when the iterate is optimal.
 */
int DaqpCheckOptimalAvi(DAQPWorkspace* work);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
