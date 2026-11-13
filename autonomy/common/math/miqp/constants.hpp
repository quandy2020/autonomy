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
 * @file constants.hpp
 * @brief DAQP defaults, exit flags, update masks, and constraint sense bits.
 *
 * Vendored DAQP (MIT). Sense bits are stored in DAQPProblem::sense and
 * DAQPWorkspace::sense. Update masks tell DaqpUpdateLdp which parts of the
 * least-distance program to rebuild. Exit flags are mapped by Solver onto
 * miqp::Status.
 */

#ifndef DAQP_CONSTANTS_H
#define DAQP_CONSTANTS_H


#include <stddef.h>

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Enable per-constraint soft weights unless DAQP_NO_SOFT_WEIGHTS is set.
 */
#ifndef DAQP_NO_SOFT_WEIGHTS
#define DAQP_SOFT_WEIGHTS
#endif

/**
 * @brief Sentinel index: no constraint selected.
 */
#define DAQP_EMPTY_IND -1

/**
 * @brief Internal marker that the unconstrained point is optimal.
 */
#define DAQP_UNCONSTRAINED_OPTIMAL -2

/**
 * @brief Finite stand-in for an infinite bound (1e30).
 */
#define DAQP_INF ((c_float)1e30)

/**
 * @brief Default primal feasibility tolerance.
 */
#define DAQP_DEFAULT_PRIM_TOL 1e-6

/**
 * @brief Default dual feasibility tolerance.
 */
#define DAQP_DEFAULT_DUAL_TOL 1e-12

/**
 * @brief Default numerical zero used in factorizations.
 */
#define DAQP_DEFAULT_ZERO_TOL 1e-11

/**
 * @brief Default dual-progress tolerance against cycling.
 */
#define DAQP_DEFAULT_PROG_TOL 1e-14

/**
 * @brief Default LDL' pivot tolerance.
 */
#define DAQP_DEFAULT_PIVOT_TOL 1e-8

/**
 * @brief Default consecutive stall iterations before a cycle is declared.
 */
#define DAQP_DEFAULT_CYCLE_TOL 10

/**
 * @brief Default proximal eta; negative selects the automatic policy.
 */
#define DAQP_DEFAULT_ETA -1.0

/**
 * @brief Cap on the automatic proximal eta.
 */
#define DAQP_AUTO_ETA_CAP 1e-6

/**
 * @brief Default active-set iteration limit.
 */
#define DAQP_DEFAULT_ITER_LIMIT 10000

/**
 * @brief Default reciprocal quadratic soft weight.
 */
#define DAQP_DEFAULT_RHO_SOFT 1e-6

/**
 * @brief Default linear soft weight (zero is a pure quadratic penalty).
 */
#define DAQP_DEFAULT_W_SOFT 0

/**
 * @brief Default relative mixed-integer suboptimality tolerance.
 */
#define DAQP_DEFAULT_REL_SUBOPT 0

/**
 * @brief Default absolute mixed-integer suboptimality tolerance.
 */
#define DAQP_DEFAULT_ABS_SUBOPT 0

/**
 * @brief Default singularity tolerance on the working-set Gramian.
 */
#define DAQP_DEFAULT_SING_TOL (3.7e-11)

/**
 * @brief Weight of free soft slacks in a singular direction that is not
 *        rounding noise.
 */
#define DAQP_SOFT_CURV_TOL 1.5e-8

/**
 * @brief Default tolerance that triggers a working-set refactor.
 */
#define DAQP_DEFAULT_REFACTOR_TOL 1e-9

/**
 * @brief Default proximal epsilon; negative selects the automatic policy.
 */
#define DAQP_DEFAULT_EPS_PROX (-1e-6)

/**
 * @brief Equality reduction is disabled.
 *
 * DAQPSettings.eq_reduction. AUTO only reduces solves that start from
 * scratch, which setup/update marks with DAQP_UPDATE_eliminate
 * (DaqpQuadprog, one-shot interfaces, and Solver without warm start). ON
 * also reduces a warm-started workspace that is updated and solved
 * repeatedly.
 */
#define DAQP_EQ_REDUCTION_OFF (-1)

/**
 * @brief Equality reduction on cold starts only (see DAQP_EQ_REDUCTION_OFF).
 */
#define DAQP_EQ_REDUCTION_AUTO 0

/**
 * @brief Equality reduction on every update, including warm starts.
 */
#define DAQP_EQ_REDUCTION_ON 1

/**
 * @brief Minimum inner iterations before DaqpRefinePrimal runs (proximal).
 */
#define DAQP_REFINE_MIN_ITER 5

/**
 * @brief Hessian condition-number estimate above which the primal is refined.
 */
#define DAQP_REFINE_COND 1e6

/**
 * @brief Refine if rounding (about DAQP_REFINE_GAIN * eps * max(|u|,|v|) /
 *        min(D)) might exceed primal_tol.
 */
#define DAQP_REFINE_GAIN 1e3

/**
 * @brief Refine the active set at a solution if an LDL' pivot is below this.
 *
 * Independent of pivot_tol, because branch-and-bound does not refine x
 * afterwards.
 */
#define DAQP_REFINE_PIVOT 1e-6

/**
 * @brief Extra gain on eps * sum|lam| used as the add-constraint noise floor.
 *
 * When cycling persists after a refactor, a constraint is only added if its
 * violation also exceeds the rounding level of u = -M' lam. Below it, the
 * computed violation is rounding noise, on which a degenerate working set can
 * swap constraints indefinitely.
 */
#define DAQP_ADD_NOISE_GAIN 10

/**
 * @brief Relative-to-max(H_ii) cap used when a semi-proximal inner problem
 *        fails.
 */
#define DAQP_PROX_EPS_MAX 1e-3

/**
 * @brief Regularize a reduced Hessian of an H with zero rows if cond exceeds
 *        this, and any Hessian if n * eps * cond > DAQP_HESSIAN_COND_EPS.
 */
#define DAQP_HESSIAN_COND_MAX 1e8

/**
 * @brief Second Hessian-condition threshold; see DAQP_HESSIAN_COND_MAX.
 */
#define DAQP_HESSIAN_COND_EPS 0.1

/**
 * @brief Defer R^{-1} if the diagonal bound on cond(H), times this margin,
 *        is below the limits above.
 */
#define DAQP_COND_DEFER_MARGIN 100

/**
 * @brief Reduced problem is a least-distance program (identity Hessian).
 *
 * H is positive definite on the null space of the equalities.
 */
#define DAQP_EQ_PATH_LDP 0

/**
 * @brief Reduced problem keeps W' H W (singular quadratic or affine
 *        variational inequality).
 */
#define DAQP_EQ_PATH_QP 1

/**
 * @brief Reduced problem has no Hessian (linear program).
 */
#define DAQP_EQ_PATH_LP 2

/**
 * @brief Minimum number of equalities before elimination is considered.
 *
 * Equalities are eliminated if neq > DAQP_EQ_MIN_COUNT and
 * DAQP_EQ_MIN_RATIO * neq > n.
 */
#define DAQP_EQ_MIN_COUNT 5

/**
 * @brief Minimum neq / n ratio (times this scale) for elimination.
 */
#define DAQP_EQ_MIN_RATIO 10

/**
 * @brief Minimum primal dimension for equality elimination.
 */
#define DAQP_EQ_MIN_DIM 12

/**
 * @brief Diagonal Hessians require at least n / this many equalities.
 */
#define DAQP_EQ_DIAG_MIN_RATIO 4

/**
 * @brief Packed triangular length of an n-vector (n (n + 1) / 2).
 *
 * @param x Dimension.
 */
#define DAQP_ARSUM(x) ((x) * (x + 1) / 2)

/**
 * @brief Offset of packed row X in an upper-triangular n-by-n factor.
 *
 * @param X Row index.
 * @param Y Leading dimension n.
 */
#define DAQP_R_OFFSET(X, Y) (((2 * Y - X - 1) * X) / 2)

/**
 * @brief Optimal, but found after cycling with the rounding level as the
 *        add-constraint tolerance, and a hard constraint is still violated
 *        by more than primal_tol (see DAQP_ADD_NOISE_GAIN).
 */
#define DAQP_EXIT_OPTIMAL_INEXACT 4

/**
 * @brief Optimal with a positive soft slack.
 */
#define DAQP_EXIT_SOFT_OPTIMAL 2

/**
 * @brief Strictly optimal.
 */
#define DAQP_EXIT_OPTIMAL 1

/**
 * @brief Feasible set is empty.
 */
#define DAQP_EXIT_INFEASIBLE -1

/**
 * @brief Active-set method cycled.
 */
#define DAQP_EXIT_CYCLE -2

/**
 * @brief Quadratic is unbounded below.
 */
#define DAQP_EXIT_UNBOUNDED -3

/**
 * @brief Iteration budget exhausted.
 */
#define DAQP_EXIT_ITERLIMIT -4

/**
 * @brief Hessian is not positive definite on the free set.
 */
#define DAQP_EXIT_NONCONVEX -5

/**
 * @brief Immutable equalities overdetermine the initial working set.
 */
#define DAQP_EXIT_OVERDETERMINED_INITIAL -6

/**
 * @brief Time budget exhausted (requires PROFILING).
 */
#define DAQP_EXIT_TIMELIMIT -7

/**
 * @brief Requested feature is not supported in this build or problem type.
 */
#define DAQP_EXIT_UNSUPPORTED -8

/**
 * @brief Rebuild Rinv on the next least-distance update.
 */
#define DAQP_UPDATE_Rinv 1

/**
 * @brief Rebuild M on the next least-distance update.
 */
#define DAQP_UPDATE_M 2

/**
 * @brief Rebuild v on the next least-distance update.
 */
#define DAQP_UPDATE_v 4

/**
 * @brief Rebuild d on the next least-distance update.
 */
#define DAQP_UPDATE_d 8

/**
 * @brief Rebuild sense on the next least-distance update.
 */
#define DAQP_UPDATE_sense 16

/**
 * @brief Rebuild hierarchical data on the next update.
 */
#define DAQP_UPDATE_hierarchy 32

/**
 * @brief Allow the unconstrained-optimum shortcut during setup.
 */
#define DAQP_UPDATE_unconstrained 64

/**
 * @brief Let DAQP_EQ_REDUCTION_AUTO eliminate equalities of this setup.
 *
 * Used for a solve that starts from scratch.
 */
#define DAQP_UPDATE_eliminate 128

/**
 * @brief Bits in state that mark a pending least-distance rebuild.
 *
 * An earlier update skipped these parts; the next update forms them.
 */
#define DAQP_STATE_PENDING                                             \
    (DAQP_UPDATE_Rinv + DAQP_UPDATE_M + DAQP_UPDATE_v + DAQP_UPDATE_d + \
     DAQP_UPDATE_sense + DAQP_UPDATE_hierarchy)

/**
 * @brief The unconstrained optimum is the solution.
 */
#define DAQP_STATE_UNCONSTRAINED 256

/**
 * @brief The first ms rows of Rinv are normalized.
 */
#define DAQP_STATE_RINV_NORMALIZED 512

/**
 * @brief work->x holds a candidate integer-feasible solution.
 */
#define DAQP_STATE_INCUMBENT 1024

/**
 * @brief Estimated cond(H) is above DAQP_REFINE_COND.
 */
#define DAQP_STATE_ILL_CONDITIONED 2048

/**
 * @brief Rinv currently stores R (reciprocal diagonal), not R^{-1}.
 *
 * Kept while the unconstrained optimum is optimal; inverted before a
 * constrained solve.
 */
#define DAQP_STATE_CHOLESKY_PENDING 4096

/**
 * @brief Constraints are only added above the rounding level of u.
 *
 * Set by DaqpLdp when cycling persists; see DAQP_ADD_NOISE_GAIN.
 */
#define DAQP_STATE_NOISE_FLOOR 8192

/**
 * @brief Sense bit: constraint is in the working set.
 */
#define DAQP_ACTIVE 1

/**
 * @brief True when constraint @p x is active.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_ACTIVE(x) (work->sense[x] & 1)

/**
 * @brief Mark constraint @p x active.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_ACTIVE(x) (work->sense[x] |= 1)

/**
 * @brief Mark constraint @p x inactive.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_INACTIVE(x) (work->sense[x] &= ~1)

/**
 * @brief Sense bit: active at the lower bound.
 */
#define DAQP_LOWER 2

/**
 * @brief True when constraint @p x is active at its lower bound.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_LOWER(x) (work->sense[x] & 2)

/**
 * @brief Mark constraint @p x as a lower bound.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_LOWER(x) (work->sense[x] |= 2)

/**
 * @brief Mark constraint @p x as an upper bound.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_UPPER(x) (work->sense[x] &= ~2)

/**
 * @brief Sense bit: cannot be activated or deactivated.
 */
#define DAQP_IMMUTABLE 4

/**
 * @brief True when constraint @p x is immutable.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_IMMUTABLE(x) (work->sense[x] & 4)

/**
 * @brief Mark constraint @p x immutable.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_IMMUTABLE(x) (work->sense[x] |= 4)

/**
 * @brief Mark constraint @p x mutable.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_MUTABLE(x) (work->sense[x] &= ~4)

/**
 * @brief Sense bit: slack is penalized rather than required to vanish.
 */
#define DAQP_SOFT 8

/**
 * @brief True when constraint @p x is soft.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_SOFT(x) (work->sense[x] & 8)

/**
 * @brief Mark constraint @p x soft.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_SOFT(x) (work->sense[x] |= 8)

/**
 * @brief Mark constraint @p x hard.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_HARD(x) (work->sense[x] &= ~8)

/**
 * @brief Sense bit: must be active at either the upper or the lower bound.
 */
#define DAQP_BINARY 16

/**
 * @brief True when constraint @p x is binary.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_BINARY(x) (work->sense[x] & 16)

/**
 * @brief Sense bit: soft slack is currently zero (see auxiliary.cpp).
 */
#define DAQP_SLACK_FIXED 32

/**
 * @brief True when the soft slack of @p x is fixed at zero.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_SLACK_FIXED(x) (work->sense[x] & 32)

/**
 * @brief True when the soft slack of @p x is free.
 *
 * @param x Constraint index.
 */
#define DAQP_IS_SLACK_FREE(x) ((work->sense[x] & 32) == 0)

/**
 * @brief Mark the soft slack of @p x as fixed.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_SLACK_FIXED(x) (work->sense[x] |= 32)

/**
 * @brief Mark the soft slack of @p x as free.
 *
 * @param x Constraint index.
 */
#define DAQP_SET_SLACK_FREE(x) (work->sense[x] &= ~32)

/**
 * @brief Sense bit: constraint is temporarily set aside (proximal gradient
 *        step in DaqpProx.cpp).
 */
#define DAQP_SET_ASIDE 64

/**
 * @brief Internal marker: ACTIVE and IMMUTABLE were set by bound-equality
 *        detection.
 */
#define DAQP_AUTO_EQUALITY 128

/**
 * @brief Internal marker: soft slack switched at w, no primal step since.
 */
#define DAQP_SLACK_SWITCHED 256


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
