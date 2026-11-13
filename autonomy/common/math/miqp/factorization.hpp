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
 * @file factorization.hpp
 * @brief Rank-one LDL' updates of the working-set Gramian.
 *
 * Vendored DAQP (MIT). Adding or removing an active constraint updates
 * Mk Mk' = L D L' without refactoring from scratch. rho is the reciprocal
 * quadratic weight of a free soft slack, and zero otherwise.
 */

#ifndef DAQP_FACTORIZATION_H
# define DAQP_FACTORIZATION_H


#include "autonomy/common/math/miqp/types.hpp"
#include "autonomy/common/math/miqp/constants.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Inner product of two length-@p n vectors.
 *
 * @param[in] v1 First vector.
 * @param[in] v2 Second vector.
 * @param[in] n Length.
 * @return Sum of v1[i] * v2[i].
 */
c_float DaqpDot(const c_float* v1, const c_float* v2, const int n);

/**
 * @brief Inlined inner product of two length-@p n vectors.
 *
 * @param[in] v1 First vector.
 * @param[in] v2 Second vector.
 * @param[in] n Length.
 * @return Sum of v1[i] * v2[i].
 */
static inline c_float DaqpDotInline(const c_float* v1, const c_float* v2,
                                      const int n) {
    c_float sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += v1[i] * v2[i];
    }
    return sum;
}

/**
 * @brief Rank-one update of L D L' after activating a constraint.
 *
 * @param[in,out] work Workspace whose L, D, and n_active are updated.
 * @param[in] add_ind Constraint index that was appended.
 * @param[in] rho Reciprocal quadratic weight of a free soft slack, else 0.
 */
void DaqpUpdateLDLAdd(DAQPWorkspace* work, const int add_ind,
                         const c_float rho);

/**
 * @brief Rank-one downdate of L D L' after dropping a working-set entry.
 *
 * @param[in,out] work Workspace whose L, D, and n_active are updated.
 * @param[in] rm_ind Index into the working set that was removed.
 */
void DaqpUpdateLDLRemove(DAQPWorkspace* work, const int rm_ind);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
