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
 * @file solver.cpp
 * @brief Copy an Eigen MIQP into DAQP arrays, solve, and map the exit flag.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/miqp/solver.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include "api.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {
namespace miqp {
namespace {

/**
 * @brief Replace a non-finite or huge bound with ±DAQP_INF.
 * @param value Bound taken from an Eigen vector. NaN and infinities are finite-clamped.
 * @return A finite bound inside [-DAQP_INF, DAQP_INF].
 */
double ClampInfiniteBound(double value) {
  if (!std::isfinite(value)) {
    return std::copysign(static_cast<double>(DAQP_INF), value);
  }
  if (value > DAQP_INF) {
    return DAQP_INF;
  }
  if (value < -DAQP_INF) {
    return -DAQP_INF;
  }
  return value;
}

/**
 * @brief Map a DAQP exit flag onto Status.
 *
 * Every positive flag, including soft-optimal and inexact, becomes kOptimal.
 * Unknown negative flags become kError.
 *
 * @param flag Raw DAQP exit flag.
 * @return Status stored on the Result.
 */
Status MapExitStatus(int flag) {
  if (flag > 0) {
    return Status::kOptimal;
  }
  switch (flag) {
    case DAQP_EXIT_INFEASIBLE:
      return Status::kInfeasible;
    case DAQP_EXIT_CYCLE:
      return Status::kCycle;
    case DAQP_EXIT_UNBOUNDED:
      return Status::kUnbounded;
    case DAQP_EXIT_ITERLIMIT:
      return Status::kIterationLimit;
    case DAQP_EXIT_NONCONVEX:
      return Status::kNonconvex;
    case DAQP_EXIT_TIMELIMIT:
      return Status::kTimeLimit;
    default:
      return Status::kError;
  }
}

}  // namespace

Result Solver::Solve(const Problem& problem, const Options& options) const {
  Result result;
  const int n = static_cast<int>(problem.gradient.size());
  const int m_general = static_cast<int>(problem.constraint_matrix.rows());
  if (n <= 0 || problem.hessian.rows() != n || problem.hessian.cols() != n ||
      problem.lower.size() != n || problem.upper.size() != n) {
    return result;
  }
  if (m_general > 0 &&
      (problem.constraint_matrix.cols() != n || problem.constraint_lower.size() != m_general ||
       problem.constraint_upper.size() != m_general)) {
    return result;
  }

  std::vector<double> hessian(static_cast<size_t>(n * n), 0.0);
  const double ridge = std::max(0.0, options.hessian_regularization);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      hessian[static_cast<size_t>(i * n + j)] = 0.5 * (problem.hessian(i, j) + problem.hessian(j, i));
    }
    hessian[static_cast<size_t>(i * n + i)] += ridge;
  }
  std::vector<double> gradient(problem.gradient.data(), problem.gradient.data() + n);
  std::vector<double> constraints;
  if (m_general > 0) {
    constraints.resize(static_cast<size_t>(m_general * n));
    for (int i = 0; i < m_general; ++i) {
      for (int j = 0; j < n; ++j) {
        constraints[static_cast<size_t>(i * n + j)] = problem.constraint_matrix(i, j);
      }
    }
  }

  const int m = n + m_general;
  std::vector<double> lower(static_cast<size_t>(m));
  std::vector<double> upper(static_cast<size_t>(m));
  std::vector<int> sense(static_cast<size_t>(m), 0);
  for (int i = 0; i < n; ++i) {
    lower[static_cast<size_t>(i)] = ClampInfiniteBound(problem.lower(i));
    upper[static_cast<size_t>(i)] = ClampInfiniteBound(problem.upper(i));
    if (lower[static_cast<size_t>(i)] > upper[static_cast<size_t>(i)]) {
      return result;
    }
  }
  for (int index : problem.binary) {
    if (index < 0 || index >= n) {
      return result;
    }
    if (lower[static_cast<size_t>(index)] >= upper[static_cast<size_t>(index)]) {
      return result;
    }
    sense[static_cast<size_t>(index)] = DAQP_BINARY;
  }
  for (int i = 0; i < m_general; ++i) {
    const int row = n + i;
    lower[static_cast<size_t>(row)] = ClampInfiniteBound(problem.constraint_lower(i));
    upper[static_cast<size_t>(row)] = ClampInfiniteBound(problem.constraint_upper(i));
    if (lower[static_cast<size_t>(row)] > upper[static_cast<size_t>(row)]) {
      return result;
    }
    if (std::abs(upper[static_cast<size_t>(row)] - lower[static_cast<size_t>(row)]) <= 1e-12) {
      sense[static_cast<size_t>(row)] = DAQP_ACTIVE | DAQP_IMMUTABLE;
    }
  }

  DAQPProblem qp;
  std::memset(&qp, 0, sizeof(qp));
  qp.n = n;
  qp.m = m;
  qp.ms = n;
  qp.H = hessian.data();
  qp.f = gradient.data();
  qp.A = constraints.empty() ? nullptr : constraints.data();
  qp.bupper = upper.data();
  qp.blower = lower.data();
  qp.sense = sense.data();
  qp.nh = 1;

  DAQPSettings settings;
  daqp_default_settings(&settings);
  settings.time_limit = std::max(0.0, options.time_limit_seconds);
  if (options.iteration_limit > 0) {
    settings.iter_limit = options.iteration_limit;
  }

  std::vector<double> primal(static_cast<size_t>(n), 0.0);
  std::vector<double> dual(static_cast<size_t>(m), 0.0);
  DAQPResult raw;
  std::memset(&raw, 0, sizeof(raw));
  raw.x = primal.data();
  raw.lam = dual.data();

  DAQPWorkspace work;
  std::memset(&work, 0, sizeof(work));
  work.settings = &settings;
  const int setup = setup_daqp_main(&qp, &work, &raw.setup_time,
                                    DAQP_UPDATE_unconstrained | DAQP_UPDATE_eliminate);
  if (setup < 0) {
    result.exit_flag = setup;
    result.status = MapExitStatus(setup);
    return result;
  }
  daqp_solve(&raw, &work);
  work.settings = nullptr;
  free_daqp_workspace(&work);
  free_daqp_ldp(&work);

  result.exit_flag = raw.exitflag;
  result.status = MapExitStatus(raw.exitflag);
  result.iterations = raw.iter;
  result.nodes = raw.nodes;
  result.objective = raw.fval;
  if (result.ok()) {
    result.x = Eigen::Map<Eigen::VectorXd>(primal.data(), n);
  }
  return result;
}

}  // namespace miqp
}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
