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
 * @file hierarchical.hpp
 * @brief Hierarchical quadratic program: lexicographic levels of constraints.
 *
 * Vendored DAQP (MIT). Each level is solved as a least-distance program with
 * the previous levels frozen. Unused by Solver::Solve, which installs a
 * single-level mixed-integer quadratic program.
 */

#ifndef DAQP_HIERARCHICAL_H
# define DAQP_HIERARCHICAL_H


#include "autonomy/common/math/miqp/types.hpp"
#include "autonomy/common/math/miqp/constants.hpp"
#include "autonomy/common/math/miqp/daqp.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Solve the hierarchical quadratic program in @p work.
 *
 * @param[in,out] work Workspace with nh > 1 and break_points set.
 * @param[out] lambda Optional dual of the hierarchical levels; may be NULL.
 * @return DAQP exit flag (positive on success).
 */
int DaqpHiqp(DAQPWorkspace* work, c_float* lambda);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
