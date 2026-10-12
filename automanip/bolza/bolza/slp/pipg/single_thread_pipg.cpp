/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bolza/slp/pipg/single_thread_pipg.hpp"

#include <iostream>
#include <numeric>

namespace bolza {
namespace pipg {

SolverStatus SingleThreadPipg(const pipg::Settings& settings, const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h,
                              const Eigen::SparseMatrix<scalar_t>& G, const vector_t& g, const vector_t& EInv, const PipgBounds& pipgBounds,
                              vector_t& stackedSolution) {
  // Cold start
  vector_t z = vector_t::Zero(H.cols());
  vector_t z_old = vector_t::Zero(H.cols());

  vector_t v = vector_t::Zero(g.rows());
  vector_t w = vector_t::Zero(g.rows());
  vector_t constraintsViolation(g.rows());

  // Iteration number
  size_t k = 0;
  bool isConverged = false;
  scalar_t constraintsViolationInfNorm;
  while (k < settings.maxNumIterations && !isConverged) {
    const auto beta = pipgBounds.dualStepSize(k);
    const auto alpha = pipgBounds.primalStepSize(k);

    z_old.swap(z);

    // v = w + beta * (G * z - g);
    v = -g;
    v.noalias() += G * z;
    v *= beta;
    v += w;

    // z = z_old - alpha * (H * z_old + h + G.transpose() * v);
    z = -h;
    z.noalias() -= (H * z_old);
    z.noalias() -= (G.transpose() * v);
    z *= alpha;
    z.noalias() += z_old;

    // w = w + beta * (G * z - g);
    w -= beta * g;
    w.noalias() += beta * (G * z);

    if (k % settings.checkTerminationInterval == 0) {
      const scalar_t zNorm = z.squaredNorm();

      constraintsViolation.noalias() = G * z;
      constraintsViolation -= g;
      constraintsViolation.cwiseProduct(EInv);
      constraintsViolationInfNorm = constraintsViolation.lpNorm<Eigen::Infinity>();

      const vector_t z_delta = z - z_old;
      const scalar_t z_deltaNorm = z_delta.squaredNorm();
      isConverged =
          constraintsViolationInfNorm <= settings.absoluteTolerance &&
          (z_deltaNorm <= settings.relativeTolerance * settings.relativeTolerance * zNorm || z_deltaNorm <= settings.absoluteTolerance);
    }

    ++k;
  }  // end of while loop

  stackedSolution.swap(z);
  pipg::SolverStatus status = isConverged ? pipg::SolverStatus::SUCCESS : pipg::SolverStatus::MAX_ITER;

  if (settings.displayShortSummary) {
    std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ PIPG +++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Solver status: " << pipg::ToString(status) << "\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings.maxNumIterations << "\n";
    std::cerr << "Norm of delta primal solution: " << (stackedSolution - z_old).norm() << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
  }

  return status;
}

}  // namespace pipg
}  // namespace bolza
