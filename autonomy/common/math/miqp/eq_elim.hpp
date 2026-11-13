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
 * @file eq_elim.hpp
 * @brief Eliminate equality constraints before the least-distance transform.
 *
 * Vendored DAQP (MIT). Equalities are removed from the quadratic program;
 * the reduced problem is an ordinary DAQPProblem whose least-distance
 * program is formed and solved by the usual routines. It is swapped into the
 * workspace (installed) while it is formed or solved, and the workspace
 * describes the original problem otherwise.
 */

#ifndef DAQP_EQ_ELIM_H
# define DAQP_EQ_ELIM_H


#include "autonomy/common/math/miqp/types.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief True when the equality constraints of @p work have been eliminated.
 *
 * @param work Workspace pointer.
 */
#define DAQP_IS_REDUCED(work) ((work)->eq != NULL && (work)->eq->active)

/**
 * @brief Return code of DaqpEqUpdate when nothing can be eliminated.
 */
#define DAQP_EQ_NOT_REDUCED 1

/**
 * @brief Whether @p qp should be reduced for an update with @p mask.
 *
 * AUTO only reduces when DAQP_UPDATE_eliminate is set in @p mask (one-shot
 * solves). ON also reduces a warm-started workspace that is updated and
 * solved repeatedly.
 *
 * @param[in] work Workspace whose settings->eq_reduction is read.
 * @param[in] qp Candidate quadratic program.
 * @param[in] mask Bitwise combination of DAQP_UPDATE_* flags.
 * @return Nonzero when a reduction should be attempted.
 */
int DaqpEqWanted(const DAQPWorkspace* work, const DAQPProblem* qp,
                   const int mask);

/**
 * @brief Form or update the reduction of @p qp and its least-distance program.
 *
 * @p update_ldp forms the least-distance program of a problem that is in the
 * workspace. Returns DAQP_EQ_NOT_REDUCED if nothing can be eliminated, a
 * negative exit flag if the equalities cannot be satisfied, and the return
 * value of @p update_ldp otherwise.
 *
 * @param[in,out] work Workspace that receives the reduction.
 * @param[in] qp Source quadratic program.
 * @param[in] mask Bitwise combination of DAQP_UPDATE_* flags.
 * @param[in] update_ldp Callback that forms the least-distance program.
 * @return DAQP_EQ_NOT_REDUCED, a negative exit flag, or the callback result.
 */
int DaqpEqUpdate(DAQPWorkspace* work, DAQPProblem* qp, int mask,
                   int (*update_ldp)(int, DAQPWorkspace*, DAQPProblem*));

/**
 * @brief Give up the reduction.
 *
 * The least-distance program of the original problem then has to be formed.
 *
 * @param[in,out] work Workspace whose reduction is dropped.
 */
void DaqpEqDeactivate(DAQPWorkspace* work);

/**
 * @brief Swap the reduced problem into the workspace.
 *
 * @param[in,out] work Workspace that receives the reduced arrays.
 * @return Nonzero when the reduced problem is installed.
 */
int DaqpEqInstall(DAQPWorkspace* work);

/**
 * @brief Swap the original problem back into the workspace.
 *
 * @param[in,out] work Workspace that currently holds the reduced problem.
 */
void DaqpEqRestore(DAQPWorkspace* work);

/**
 * @brief Set the starting iterate of the reduced problem from original @p x.
 *
 * @param[in,out] work Workspace that holds an active reduction.
 * @param[in] x Primal of the original problem.
 */
void DaqpEqSetPrimalStart(DAQPWorkspace* work, const c_float* x);

/**
 * @brief Free equality-elimination storage.
 *
 * @param[in,out] work Workspace whose eq attachment is released.
 */
void FreeDaqpEq(DAQPWorkspace* work);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
