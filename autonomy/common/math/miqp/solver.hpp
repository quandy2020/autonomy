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
 * @file solver.hpp
 * @brief Eigen front end for the vendored DAQP mixed-integer quadratic solver.
 *
 * DAQP 0.10.3 solves a convex quadratic program and, through branch-and-bound,
 * a convex mixed-integer quadratic program. A binary variable is a simple
 * bound whose sense bit is DAQP_BINARY, so the variable is restricted to the
 * two-point set {lower, upper}. General rows follow the simple bounds. The
 * constraint matrix is row-major. Equal general bounds, |upper - lower| <=
 * 1e-12, are marked active and immutable. Infinite bounds are clamped to
 * ±DAQP_INF (1e30). The Hessian is symmetrized and then shifted by
 * hessian_regularization on the diagonal, because a singular or proximal
 * mixed-integer quadratic program is rejected as nonconvex and
 * branch-and-bound does not run.
 */

#pragma once

#include <vector>

#include "Eigen/Dense"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Outcome of one DAQP call, mapped from the solver exit flag.
 *
 * A positive DAQP flag (optimal, soft, or inexact) is reported as kOptimal.
 * The numeric values match the DAQP exit codes so a log line can be compared
 * with the upstream solver.
 */
enum class Status {
    /**
     * @brief DAQP exit greater than zero: optimal, soft-optimal, or inexact.
     */
    kOptimal = 1,

    /**
     * @brief The feasible set is empty.
     */
    kInfeasible = -1,

    /**
     * @brief The active-set method cycled.
     */
    kCycle = -2,

    /**
     * @brief The quadratic is unbounded below.
     */
    kUnbounded = -3,

    /**
     * @brief The iteration budget was exhausted.
     */
    kIterationLimit = -4,

    /**
     * @brief The Hessian is not positive definite on the free set.
     *
     * Branch-and-bound refuses a nonconvex Hessian.
     */
    kNonconvex = -5,

    /**
     * @brief The time budget was exhausted.
     *
     * Requires the DAQP PROFILING build.
     */
    kTimeLimit = -7,

    /**
     * @brief Dimensions disagree, or DAQP returned an unknown flag.
     */
    kError = -100,
};

/**
 * @brief Budgets and the ridge added to the Hessian.
 */
struct Options {
    /**
     * @brief Wall-clock budget in seconds.
     *
     * Zero leaves the DAQP default.
     */
    double time_limit_seconds{0.0};

    /**
     * @brief Active-set iteration budget.
     */
    int iteration_limit{10000};

    /**
     * @brief Added to every Hessian diagonal entry before factorization.
     */
    double hessian_regularization{1e-8};
};

/**
 * @brief Convex mixed-integer quadratic program in the DAQP layout.
 *
 * The objective is 0.5 x' H x + g' x. Simple bounds are lower <= x <= upper.
 * General constraints are constraint_lower <= A x <= constraint_upper, with A
 * stored row-major in constraint_matrix. binary lists the columns that must
 * take one of the two simple bounds rather than the whole interval.
 */
struct Problem {
    /**
     * @brief Symmetric n-by-n Hessian H.
     *
     * Read row-major and symmetrized before the DAQP call.
     */
    Eigen::MatrixXd hessian;

    /**
     * @brief Linear term g, length n.
     */
    Eigen::VectorXd gradient;

    /**
     * @brief Simple lower bounds, length n.
     *
     * Non-finite values become ±DAQP_INF.
     */
    Eigen::VectorXd lower;

    /**
     * @brief Simple upper bounds, length n.
     */
    Eigen::VectorXd upper;

    /**
     * @brief General constraint matrix A, m-by-n, row-major on the DAQP side.
     */
    Eigen::MatrixXd constraint_matrix;

    /**
     * @brief General lower bounds, length m.
     */
    Eigen::VectorXd constraint_lower;

    /**
     * @brief General upper bounds, length m.
     */
    Eigen::VectorXd constraint_upper;

    /**
     * @brief Columns restricted to {lower, upper}.
     *
     * Each index must lie in [0, n).
     */
    std::vector<int> binary;
};

/**
 * @brief Primal solution and the bookkeeping returned by DAQP.
 */
struct Result {
    /**
     * @brief Mapped exit status.
     */
    Status status{Status::kError};

    /**
     * @brief Raw DAQP exit flag, before the mapping.
     */
    int exit_flag{-100};

    /**
     * @brief Primal solution, length n.
     *
     * Uninitialized on a setup failure.
     */
    Eigen::VectorXd x;

    /**
     * @brief 0.5 x' H x + g' x at the returned x.
     */
    double objective{0.0};

    /**
     * @brief Active-set iterations reported by DAQP.
     */
    int iterations{0};

    /**
     * @brief Branch-and-bound nodes.
     *
     * Zero when the problem has no binaries.
     */
    int nodes{0};

    /**
     * @brief True when status is kOptimal.
     *
     * Soft-optimal and inexact DAQP exits are included, because both are mapped
     * to kOptimal.
     *
     * @return Whether the call is treated as a successful solve.
     */
    bool IsOptimal() const { return status == Status::kOptimal; }
};

/**
 * @class autonomy::common::math::miqp::Solver
 * @brief One-shot DAQP front end. Each Solve call owns its workspace.
 */
class Solver {
 public:
    /**
     * @brief Solve one convex mixed-integer quadratic program.
     *
     * The workspace is zeroed, set up with the unconstrained and eliminate
     * updates, solved, and then released. The caller does not keep a workspace
     * between calls. A dimension mismatch returns kError without calling DAQP.
     *
     * @param[in] problem Hessian, bounds, general rows, and the binary index
     *                    list.
     * @param[in] options Time limit, iteration limit, and diagonal ridge.
     * @return Result. IsOptimal is the success test the trajectory layer uses.
     */
    Result Solve(const Problem& problem, const Options& options = Options()) const;
};

}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
