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
 * @file utils.hpp
 * @brief Quadratic-to-least-distance transforms, factorizations, and timers.
 *
 * Vendored DAQP (MIT). DaqpUpdateLdp rebuilds Rinv, M, v, and d according
 * to a DAQP_UPDATE_* mask. Profiling helpers are compiled only when
 * PROFILING is defined (required for DAQP_EXIT_TIMELIMIT).
 */

#ifndef DAQP_UTILS_H
# define DAQP_UTILS_H


#include "autonomy/common/math/miqp/daqp.hpp"
#include "autonomy/common/math/miqp/eq_elim.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Rebuild the least-distance program of @p qp into @p work.
 *
 * @param[in] mask Bitwise combination of DAQP_UPDATE_* flags.
 * @param[in,out] work Workspace that receives the transform.
 * @param[in] qp Source quadratic program.
 * @return Non-negative on success, a DAQP exit flag otherwise.
 */
int DaqpUpdateLdp(const int mask, DAQPWorkspace* work, DAQPProblem* qp);

/**
 * @brief Factor H and store R^{-1} (or a deferred Cholesky) in @p work.
 *
 * @param[in,out] work Workspace that receives Rinv.
 * @param[in] H Hessian, row-major, n by n.
 * @param[in] is_factored Nonzero when H already stores a factor.
 * @return Non-negative on success, DAQP_EXIT_NONCONVEX otherwise.
 */
int DaqpUpdateRinv(DAQPWorkspace* work, c_float* H, int is_factored);

/**
 * @brief Proximal diagonal shift implied by the current Hessian and settings.
 *
 * @param[in] work Workspace whose Hessian condition is inspected.
 * @return Regularization value, or zero when none is needed.
 */
c_float DaqpGetProximalRegularization(const DAQPWorkspace* work);

/**
 * @brief Rebuild M from the general constraint matrix @p A.
 *
 * @param[in,out] work Workspace that receives M.
 * @param[in] A General constraint matrix, row-major.
 * @return Non-negative on success.
 */
int DaqpUpdateM(DAQPWorkspace* work, c_float* A);

/**
 * @brief Rebuild v = R'\\f from the linear term @p f.
 *
 * @param[in] f Linear term, length n.
 * @param[in,out] work Workspace that receives v.
 */
void DaqpUpdateV(c_float* f, DAQPWorkspace* work);

/**
 * @brief Rebuild dual linear terms d from the bound vectors.
 *
 * @param[in,out] work Workspace that receives dupper and dlower.
 * @param[in] bupper Upper bounds, length m.
 * @param[in] blower Lower bounds, length m.
 * @return Non-negative on success.
 */
int DaqpUpdateD(DAQPWorkspace* work, c_float* bupper, c_float* blower);

/**
 * @brief Reject empty bound intervals and mark tight equalities.
 *
 * @param[in,out] work Workspace whose sense may gain AUTO_EQUALITY bits.
 * @param[in] bupper Upper bounds, length m.
 * @param[in] blower Lower bounds, length m.
 * @return Non-negative when the bounds are consistent.
 */
int DaqpCheckBounds(DAQPWorkspace* work, c_float* bupper, c_float* blower);

/**
 * @brief Normalize the first ms rows of Rinv.
 *
 * @param[in,out] work Workspace whose Rinv is scaled.
 */
void DaqpNormalizeRinv(DAQPWorkspace* work);

/**
 * @brief Normalize the general rows of M.
 *
 * @param[in,out] work Workspace whose M and scaling are updated.
 * @return Non-negative on success.
 */
int DaqpNormalizeM(DAQPWorkspace* work);

/**
 * @brief Detect an unconstrained optimum and skip the active-set loop.
 *
 * @param[in,out] work Workspace that may set DAQP_STATE_UNCONSTRAINED.
 * @param[in] mask Update mask of the current setup.
 * @return Nonzero when the unconstrained point is feasible.
 */
int DaqpCheckUnconstrained(DAQPWorkspace* work, const int mask);

/**
 * @brief Factor the affine variational inequality attached to @p avi.
 *
 * @param[in,out] avi Affine variational inequality state.
 * @param[in] problem Source problem.
 * @param[in] zero_tol Numerical zero.
 * @return Non-negative on success.
 */
int DaqpUpdateAvi(DAQPAVI* avi, DAQPProblem* problem, c_float zero_tol);

/**
 * @brief Retry an affine variational inequality with a smaller proximal rho.
 *
 * @param[in,out] work Workspace whose avi->retry_rho_needed is set.
 * @return Non-negative on success.
 */
int DaqpRetryAviWithReducedRho(DAQPWorkspace* work);

/**
 * @brief In-place LU factorization of an n-by-n matrix with pivoting.
 *
 * @param[in,out] A Matrix that is overwritten with LU.
 * @param[out] P Pivot permutation, length n.
 * @param[in] n Dimension.
 * @return Nonzero on success.
 */
int DaqpLu(c_float* A, int* P, int n);

/**
 * @brief Solve A x = b given the LU factorization of A.
 *
 * @param[in] LU Packed LU factors.
 * @param[in] P Pivot permutation.
 * @param[in] b Right-hand side, length n.
 * @param[out] x Solution, length n.
 * @param[in] n Dimension.
 */
void DaqpLuSolve(c_float* LU, int* P, c_float* b, c_float* x, int n);

/**
 * @brief Mark redundant inequalities using the least-distance data in @p work.
 *
 * @param[out] is_redundant Length-m flags, 1 if the row is redundant.
 * @param[in,out] work Workspace used as scratch.
 */
void DaqpMinrepWork(int* is_redundant, DAQPWorkspace* work);

/**
 * @brief True when a noise-floor solution still violates a hard constraint.
 *
 * @param[in] work Workspace after a solve that set DAQP_STATE_NOISE_FLOOR.
 * @return Nonzero if a hard constraint is violated beyond primal_tol.
 */
int DaqpViolatesHard(DAQPWorkspace* work);

#ifdef PROFILING
#ifdef _WIN32
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
#include <windows.h>
namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @struct DAQPtimer
 * @brief Wall-clock interval on Windows (QueryPerformanceCounter).
 */
typedef struct {
    /**
     * @brief Start counter.
     */
    LARGE_INTEGER start;

    /**
     * @brief Stop counter.
     */
    LARGE_INTEGER stop;
} DAQPtimer;
#else
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
#include <time.h>
namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @struct DAQPtimer
 * @brief Wall-clock interval on POSIX (CLOCK_MONOTONIC).
 */
typedef struct {
    /**
     * @brief Start timestamp.
     */
    struct timespec start;

    /**
     * @brief Stop timestamp.
     */
    struct timespec stop;
} DAQPtimer;
#endif

/**
 * @brief Record the start of a timing interval.
 *
 * @param[out] timer Timer to start.
 */
void Tic(DAQPtimer* timer);

/**
 * @brief Record the end of a timing interval.
 *
 * @param[in,out] timer Timer to stop.
 */
void Toc(DAQPtimer* timer);

/**
 * @brief Elapsed seconds between Tic and Toc.
 *
 * @param[in] timer Timer that has been started and stopped.
 * @return Duration in seconds.
 */
double GetTime(DAQPtimer* timer);
#endif


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
