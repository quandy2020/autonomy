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
 * @file auxiliary.hpp
 * @brief Working-set updates, primal recovery, and blocking tests.
 *
 * Vendored DAQP (MIT). These helpers implement add/remove of active
 * constraints, the constrained stationary point, and refinement of a
 * numerically tight working set.
 */

#ifndef DAQP_AUX_H
# define DAQP_AUX_H


#include "autonomy/common/math/miqp/types.hpp"
#include "autonomy/common/math/miqp/constants.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Drop working-set entry @p rm_ind and downdate the LDL' factor.
 *
 * @param[in,out] work Workspace whose working set shrinks.
 * @param[in] rm_ind Index into the working set, not a constraint Id.
 */
void DaqpRemoveConstraint(DAQPWorkspace* work, const int rm_ind);

/**
 * @brief Activate constraint @p add_ind with dual seed @p lam.
 *
 * @param[in,out] work Workspace whose working set grows.
 * @param[in] add_ind Constraint index.
 * @param[in] lam Initial dual for the new row (sign encodes lower/upper).
 */
void DaqpAddConstraint(DAQPWorkspace* work, const int add_ind, c_float lam);

/**
 * @brief Recover the primal x and the dual objective from the working set.
 *
 * @param[in,out] work Workspace whose x and fval are overwritten.
 */
void DaqpComputePrimalAndFval(DAQPWorkspace* work);

/**
 * @brief Soft slack of the constraint at working-set position @p i.
 *
 * @param[in] work Workspace with an active soft constraint.
 * @param[in] i Working-set index.
 * @return Slack in the units of the original problem.
 */
c_float DaqpSoftSlack(DAQPWorkspace* work, const int i);

/**
 * @brief Largest soft slack among the active soft constraints.
 *
 * @param[in] work Workspace to scan.
 * @return Maximum slack, or zero when none are soft.
 */
c_float DaqpMaxSoftSlack(DAQPWorkspace* work);

/**
 * @brief Raise the add-constraint tolerance to the rounding floor of u.
 *
 * @param[in,out] work Workspace that may set DAQP_STATE_NOISE_FLOOR.
 * @return Nonzero when the noise floor was applied.
 */
int DaqpSetNoiseFloor(DAQPWorkspace* work);

/**
 * @brief Rounding level of u = -M' lam used as a noise floor.
 *
 * @param[in] work Workspace whose dual is read.
 * @return Absolute noise estimate.
 */
c_float DaqpNoiseFloor(DAQPWorkspace* work);

/**
 * @brief Cache M u for the next feasibility scan.
 *
 * @param[in,out] work Workspace whose Mu is overwritten.
 */
void DaqpComputeMu(DAQPWorkspace* work);

/**
 * @brief Activate the most violated infeasible constraint, if any.
 *
 * @param[in,out] work Workspace that may grow the working set.
 * @return Nonzero when a constraint was added.
 */
int DaqpAddInfeasible(DAQPWorkspace* work);

/**
 * @brief Drop a blocking dual (lam_star of the wrong sign).
 *
 * @param[in,out] work Workspace that may shrink the working set.
 * @return Nonzero when a constraint was removed.
 */
int DaqpRemoveBlocking(DAQPWorkspace* work);

/**
 * @brief Compute the constrained stationary point lam_star.
 *
 * @param[in,out] work Workspace whose lam_star and u are overwritten.
 */
void DaqpComputeCSP(DAQPWorkspace* work);

/**
 * @brief Refactor a working set whose LDL' pivots are tiny.
 *
 * @param[in,out] work Workspace whose factors are rebuilt.
 */
void DaqpRefineActive(DAQPWorkspace* work);

/**
 * @brief Iterative refinement of the primal after a proximal inner solve.
 *
 * @param[in,out] work Workspace whose x is corrected.
 */
void DaqpRefinePrimal(DAQPWorkspace* work);

/**
 * @brief Solve Mk Mk' lam* = -d on the current working set.
 *
 * @param[in,out] work Workspace whose lam_star is overwritten.
 */
void DaqpSolveWorkingSet(DAQPWorkspace* work);

/**
 * @brief Subtract working-set rows of M times @p dlam from @p y.
 *
 * @param[in] work Workspace that supplies M and the working set.
 * @param[in] dlam Dual step, length n_active.
 * @param[in,out] y Vector that is updated in place.
 */
void DaqpSubWorkingSetRows(DAQPWorkspace* work, const c_float* dlam,
                               c_float* y);

/**
 * @brief Apply Rinv (or RinvD) to @p y in place.
 *
 * @param[in] work Workspace that supplies Rinv.
 * @param[in,out] y Vector of length n.
 */
void DaqpApplyRinv(DAQPWorkspace* work, c_float* y);

/**
 * @brief Compute a direction in the null space of a singular working set.
 *
 * @param[in,out] work Workspace whose lam_star holds the singular direction.
 */
void DaqpComputeSingularDirection(DAQPWorkspace* work);

/**
 * @brief True when the dual of a singular working set is inconsistent.
 *
 * @param[in] work Workspace after DaqpComputeSingularDirection.
 * @return Nonzero if the node is dual-infeasible.
 */
int DaqpInconsistentDual(DAQPWorkspace* work);

/**
 * @brief Repair a tiny last pivot of the LDL' factor.
 *
 * @param[in,out] work Workspace whose last diagonal is adjusted.
 */
void DaqpPivotLast(DAQPWorkspace* work);

/**
 * @brief Drop the last (singular) working-set entry.
 *
 * @param[in,out] work Workspace whose working set shrinks.
 * @return Nonzero on success.
 */
int DaqpDropSingularLast(DAQPWorkspace* work);

/**
 * @brief Activate every immutable equality that is currently inactive.
 *
 * @param[in,out] work Workspace whose equalities are forced into the working
 *                     set.
 * @return DAQP exit flag; negative if the equalities overdetermine the start.
 */
int DaqpActivateConstraints(DAQPWorkspace* work);

/**
 * @brief Clear mutable active flags without shrinking allocations.
 *
 * @param[in,out] work Workspace whose mutable actives are dropped.
 */
void DaqpDeactivateConstraints(DAQPWorkspace* work);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
