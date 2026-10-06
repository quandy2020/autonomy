/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
 * @brief Eigen front end for the vendored DAQP mixed-integer QP solver.
 *
 * DAQP 0.10.3 solves a convex QP and, through branch-and-bound, a convex MIQP.
 * A binary variable is a simple bound whose sense bit is DAQP_BINARY, so the
 * variable is restricted to the two-point set {lower, upper}. General rows
 * follow the simple bounds. The constraint matrix is row-major. Equal general
 * bounds, |upper - lower| <= 1e-12, are marked active and immutable. Infinite
 * bounds are clamped to ±DAQP_INF (1e30). The Hessian is symmetrized and then
 * shifted by hessian_regularization on the diagonal, because a singular or
 * proximal MIQP is rejected as nonconvex and the branch-and-bound does not run.
 */

#pragma once

#include <vector>

#include "Eigen/Dense"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {
namespace miqp {

/**
 * @brief Outcome of one DAQP call, mapped from the solver exit flag.
 *
 * A positive DAQP flag (optimal, soft, or inexact) is reported as kOptimal.
 * The numeric values match the DAQP exit codes so a log line can be compared
 * with the upstream solver.
 */
enum class Status {
  kOptimal = 1,          ///< DAQP exit > 0: optimal, soft-optimal, or inexact.
  kInfeasible = -1,      ///< The feasible set is empty.
  kCycle = -2,           ///< The active-set method cycled.
  kUnbounded = -3,       ///< The quadratic is unbounded below.
  kIterationLimit = -4,  ///< The iteration budget was exhausted.
  kNonconvex = -5,       ///< The Hessian is not positive definite on the free set. Branch-and-bound refuses it.
  kTimeLimit = -7,       ///< The time budget was exhausted. Requires the DAQP PROFILING build.
  kError = -100,         ///< The problem dimensions disagree, or DAQP returned an unknown flag.
};

/**
 * @brief Budgets and the ridge added to the Hessian.
 */
struct Options {
  double time_limit_seconds{0.0};      ///< Wall-clock budget, seconds. Zero leaves the DAQP default.
  int iteration_limit{10000};           ///< Active-set iteration budget.
  double hessian_regularization{1e-8};  ///< Added to every Hessian diagonal entry before factorization.
};

/**
 * @brief Convex MIQP in the DAQP layout.
 *
 * The objective is 0.5 x' H x + g' x. Simple bounds are lower <= x <= upper.
 * General constraints are constraint_lower <= A x <= constraint_upper, with A
 * stored row-major in constraint_matrix. binary lists the columns that must
 * take one of the two simple bounds rather than the whole interval.
 */
struct Problem {
  Eigen::MatrixXd hessian;            ///< Symmetric n-by-n Hessian H. Read row-major and symmetrized.
  Eigen::VectorXd gradient;           ///< Linear term g, length n.
  Eigen::VectorXd lower;              ///< Simple lower bounds, length n. Non-finite values become ±DAQP_INF.
  Eigen::VectorXd upper;              ///< Simple upper bounds, length n.
  Eigen::MatrixXd constraint_matrix;  ///< General constraint matrix A, m-by-n, row-major on the DAQP side.
  Eigen::VectorXd constraint_lower;   ///< General lower bounds, length m.
  Eigen::VectorXd constraint_upper;   ///< General upper bounds, length m.
  std::vector<int> binary;            ///< Columns restricted to {lower, upper}. Each index must lie in [0, n).
};

/**
 * @brief Primal solution and the bookkeeping returned by DAQP.
 */
struct Result {
  Status status{Status::kError};  ///< Mapped exit status.
  int exit_flag{-100};            ///< Raw DAQP exit flag, before the mapping.
  Eigen::VectorXd x;              ///< Primal solution, length n. Uninitialized on a setup failure.
  double objective{0.0};          ///< 0.5 x' H x + g' x at the returned x.
  int iterations{0};              ///< Active-set iterations reported by DAQP.
  int nodes{0};                   ///< Branch-and-bound nodes. Zero when the problem has no binaries.

  /**
   * @brief True when status is kOptimal.
   *
   * Soft-optimal and inexact DAQP exits are included, because both are mapped
   * to kOptimal.
   */
  bool IsOptimal() const { return status == Status::kOptimal; }
};

/**
 * @brief Solve one convex MIQP with DAQP branch-and-bound.
 *
 * The workspace is zeroed, set up with the unconstrained and eliminate
 * updates, solved, and then released. The caller does not keep a workspace
 * between calls. A dimension mismatch returns kError without calling DAQP.
 *
 * @param problem Hessian, bounds, general rows, and the binary index list.
 * @param options Time limit, iteration limit, and diagonal ridge.
 * @return Result. IsOptimal is the success test the trajectory layer uses.
 */
/**
 * @brief One-shot DAQP front end. Each Solve call owns its workspace.
 */
class Solver {
 public:
  Result Solve(const Problem& problem, const Options& options = Options()) const;
};

}  // namespace miqp
}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
