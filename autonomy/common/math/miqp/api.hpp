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
 * @file api.hpp
 * @brief Setup, solve, and teardown entry points of the DAQP C solver.
 *
 * Vendored DAQP (MIT). Solver::Solve uses SetupDaqpMain, DaqpSolve, and
 * FreeDaqpWorkspace / FreeDaqpLdp. The caller owns DAQPResult::x and
 * DAQPResult::lam.
 */

#ifndef DAQP_API_H
# define DAQP_API_H


#include "autonomy/common/math/miqp/daqp.hpp"
#include "autonomy/common/math/miqp/daqp_prox.hpp"
#include "autonomy/common/math/miqp/bnb.hpp"
#include "autonomy/common/math/miqp/hierarchical.hpp"
#include "autonomy/common/math/miqp/avi.hpp"
#include "autonomy/common/math/miqp/eq_elim.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @struct DAQPResult
 * @brief Primal, dual, timings, and the raw exit flag of one DAQP call.
 *
 * x and lam must already point at caller-owned buffers of length n and m.
 */
typedef struct {
    /**
     * @brief Primal solution, length n. Caller allocated.
     */
    c_float* x;

    /**
     * @brief Dual multipliers, length m. Caller allocated.
     */
    c_float* lam;

    /**
     * @brief Objective 0.5 x' H x + f' x at the returned x.
     */
    c_float fval;

    /**
     * @brief Largest violation of a soft constraint.
     */
    c_float soft_slack;

    /**
     * @brief Raw DAQP exit flag.
     */
    int exitflag;

    /**
     * @brief Active-set iterations.
     */
    int iter;

    /**
     * @brief Branch-and-bound nodes. Zero when there are no binaries.
     */
    int nodes;

    /**
     * @brief Solve time in seconds when PROFILING is enabled.
     */
    c_float solve_time;

    /**
     * @brief Setup time in seconds when PROFILING is enabled.
     */
    c_float setup_time;
} DAQPResult;

/**
 * @brief Solve the problem already installed in @p work and fill @p res.
 *
 * @param[out] res Result buffers; x and lam must be set.
 * @param[in,out] work Prepared workspace.
 */
void DaqpSolve(DAQPResult* res, DAQPWorkspace* work);

/**
 * @brief One-shot quadratic program: setup, solve, and free.
 *
 * @param[out] res Result buffers.
 * @param[in] qp Problem in DAQP layout.
 * @param[in] settings Solver settings, or NULL for defaults.
 */
void DaqpQuadprog(DAQPResult* res, DAQPProblem* qp, DAQPSettings* settings);

/**
 * @brief One-shot affine variational inequality: setup, solve, and free.
 *
 * @param[out] res Result buffers.
 * @param[in] problem Affine variational inequality in DAQP layout.
 * @param[in] settings Solver settings, or NULL for defaults.
 */
void DaqpAvi(DAQPResult* res, DAQPProblem* problem, DAQPSettings* settings);

/**
 * @brief Allocate and form the least-distance program for @p qp.
 *
 * @param[in] qp Problem to install.
 * @param[in,out] work Workspace to fill.
 * @param[out] setup_time Optional setup duration; may be NULL.
 * @return Non-negative on success, a DAQP exit flag otherwise.
 */
int SetupDaqp(DAQPProblem* qp, DAQPWorkspace* work, c_float* setup_time);

/**
 * @brief Setup with an explicit update mask (unconstrained, eliminate, ...).
 *
 * @param[in] qp Problem to install.
 * @param[in,out] work Workspace to fill.
 * @param[out] setup_time Optional setup duration; may be NULL.
 * @param[in] init_mask Bitwise combination of DAQP_UPDATE_* flags.
 * @return Non-negative on success, a DAQP exit flag otherwise.
 */
int SetupDaqpMain(DAQPProblem* qp, DAQPWorkspace* work, c_float* setup_time,
                    int init_mask);

/**
 * @brief Form the least-distance data of @p qp into @p work.
 *
 * @param[in,out] work Workspace that receives M, Rinv, v, and d.
 * @param[in] qp Source quadratic program.
 * @param[in] init_mask Bitwise combination of DAQP_UPDATE_* flags.
 * @return Non-negative on success, a DAQP exit flag otherwise.
 */
int SetupDaqpLdp(DAQPWorkspace* work, DAQPProblem* qp, const int init_mask);

/**
 * @brief Attach hierarchical breakpoints to @p work.
 *
 * @param[in,out] work Workspace that receives the hierarchy.
 * @param[in] break_points Level breaks.
 * @param[in] nh Number of levels.
 */
void SetupDaqpHiqp(DAQPWorkspace* work, int* break_points, int nh);

/**
 * @brief Allocate branch-and-bound storage for @p nb binaries.
 *
 * @param[in,out] work Workspace that receives bnb.
 * @param[in] sense Constraint sense used to count binaries.
 * @param[in] nb Number of binary constraints.
 * @param[in] ns Number of soft constraints (working-set capacity).
 * @return Non-negative on success.
 */
int SetupDaqpBnb(DAQPWorkspace* work, int* sense, int nb, int ns);

/**
 * @brief Allocate and factor the affine variational inequality.
 *
 * @param[in,out] avi Affine variational inequality state.
 * @param[in] p Source problem.
 * @param[in,out] work Workspace used during setup.
 * @param[out] setup_time Optional setup duration; may be NULL.
 * @return Non-negative on success.
 */
int SetupDaqpAvi(DAQPAVI* avi, DAQPProblem* p, DAQPWorkspace* work,
                   c_float* setup_time);

/**
 * @brief Allocate default settings on @p work if it has none.
 *
 * @param[in,out] work Workspace that may receive a settings block.
 */
void AllocateDaqpSettings(DAQPWorkspace* work);

/**
 * @brief Allocate primal/dual iterates and LDL' buffers.
 *
 * @param[in,out] work Workspace to allocate.
 * @param[in] n Number of primal variables.
 * @param[in] ns Number of extra (soft) working-set slots.
 */
void AllocateDaqpWorkspace(DAQPWorkspace* work, int n, int ns);

/**
 * @brief Allocate least-distance arrays M, Rinv, v, and related buffers.
 *
 * @param[in,out] work Workspace to allocate.
 * @param[in] n Number of primal variables.
 * @param[in] m Number of constraints.
 * @param[in] ms Number of simple bounds.
 * @param[in] alloc_R Nonzero to allocate Rinv.
 * @param[in] alloc_v Nonzero to allocate v.
 */
void AllocateDaqpLdp(DAQPWorkspace* work, int n, int m, int ms, int alloc_R,
                       int alloc_v);

/**
 * @brief Allocate affine variational inequality temporaries of size @p n.
 *
 * @param[in,out] avi State to allocate.
 * @param[in] n Number of primal variables.
 */
void AllocateDaqpAvi(DAQPAVI* avi, int n);

/**
 * @brief Allocate per-constraint soft-weight arrays.
 *
 * @param[in,out] work Workspace that receives rho_* and w_*.
 * @return 0 when the build has no per-constraint weights, 1 otherwise.
 */
int DaqpAllocateSoftWeights(DAQPWorkspace* work);

/**
 * @brief Copy per-constraint soft weights into @p work.
 *
 * @param[in,out] work Workspace that already allocated the weight arrays.
 * @param[in] rho_l Reciprocal quadratic weights on the lower side.
 * @param[in] rho_u Reciprocal quadratic weights on the upper side.
 * @param[in] w_l Linear weights on the lower side.
 * @param[in] w_u Linear weights on the upper side.
 * @return 0 when the build has no per-constraint weights, 1 otherwise.
 */
int DaqpSetSoftWeights(DAQPWorkspace* work, c_float* rho_l, c_float* rho_u,
                          c_float* w_l, c_float* w_u);

/**
 * @brief Rebuild derived soft weights after settings->rho_soft or w_soft
 *        change on a live workspace.
 *
 * @param[in,out] work Workspace whose uniform defaults changed.
 */
void DaqpRefreshSoftWeights(DAQPWorkspace* work);

/**
 * @brief Free least-distance arrays; iterates are kept.
 *
 * @param[in,out] work Workspace to release.
 */
void FreeDaqpLdp(DAQPWorkspace* work);

/**
 * @brief Free iterates, settings owned by the workspace, and attachments.
 *
 * @param[in,out] work Workspace to release.
 */
void FreeDaqpWorkspace(DAQPWorkspace* work);

/**
 * @brief Free branch-and-bound storage.
 *
 * @param[in,out] work Workspace whose bnb is released.
 */
void FreeDaqpBnb(DAQPWorkspace* work);

/**
 * @brief Free affine variational inequality storage.
 *
 * @param[in,out] work Workspace whose avi is released.
 */
void FreeDaqpAvi(DAQPWorkspace* work);

/**
 * @brief Copy primal, dual, iterations, and the exit flag into @p res.
 *
 * @param[out] res Result to fill.
 * @param[in] work Solved workspace.
 */
void DaqpExtractResult(DAQPResult* res, DAQPWorkspace* work);

/**
 * @brief Expand a reduced (equality-eliminated) result back to the original
 *        variables and working set.
 *
 * @param[in,out] res Result of the installed reduced problem.
 * @param[in,out] work Workspace that holds the reduction.
 */
void DaqpEqExpand(DAQPResult* res, DAQPWorkspace* work);

/**
 * @brief Scatter duals of the active set into the full multiplier vector.
 *
 * @param[out] res Result whose lam is filled.
 * @param[in] work Solved workspace.
 */
void DaqpExtractActiveDuals(DAQPResult* res, DAQPWorkspace* work);

/**
 * @brief Fill @p settings with the DAQP defaults.
 *
 * @param[out] settings Settings block to initialize.
 */
void DaqpDefaultSettings(DAQPSettings* settings);

/**
 * @brief Mark redundant inequality rows of A x <= b.
 *
 * @param[out] is_redundant Length-m flags, 1 if the row is redundant.
 * @param[in] A Constraint matrix, row-major.
 * @param[in] b Right-hand side.
 * @param[in] n Number of variables.
 * @param[in] m Number of general rows.
 * @param[in] ms Number of simple bounds preceding A.
 */
void DaqpMinrep(int* is_redundant, c_float* A, c_float* b, int n, int m, int ms);

/**
 * @brief Index of the first row that @p x violates, or -1 if none.
 *
 * @param[in] x Candidate primal.
 * @param[in] A General constraint matrix.
 * @param[in] bu Upper bounds.
 * @param[in] bl Lower bounds.
 * @param[in] n Number of variables.
 * @param[in] m Number of constraints.
 * @param[in] ms Number of simple bounds.
 * @param[in] tol Violation tolerance.
 * @return First violating row, or -1.
 */
int DaqpFirstViolating(c_float* x, c_float* A, c_float* bu, c_float* bl, int n,
                         int m, int ms, c_float tol);

/**
 * @brief Mark simple bounds that @p x saturates as active.
 *
 * @param[in,out] qp Problem whose sense is updated.
 * @param[in] x Primal used to detect bound activity.
 */
void DaqpPrimalInitActive(DAQPProblem* qp, c_float* x);

/**
 * @brief Mark constraints with a nonzero dual as active.
 *
 * @param[in,out] qp Problem whose sense is updated.
 * @param[in] lam Dual multipliers.
 */
void DaqpDualInitActive(DAQPProblem* qp, c_float* lam);

/**
 * @brief Install a primal starting point on @p work.
 *
 * @param[in,out] work Workspace that receives x.
 * @param[in] x Starting primal, length n.
 */
void DaqpSetPrimalStart(DAQPWorkspace* work, c_float* x);


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
