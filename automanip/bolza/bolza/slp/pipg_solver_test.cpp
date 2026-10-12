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

#include <gtest/gtest.h>
#include <Eigen/Sparse>

#include <bolza/oc/oc_problem/ocp_to_kkt.hpp>
#include "helpers/problems_generation_test.hpp"
#include <bolza/qp/qp_solver.hpp>

#include "bolza/slp/pipg/pipg_solver.hpp"
#include "bolza/slp/pipg/single_thread_pipg.hpp"

bolza::pipg::Settings configurePipg(size_t maxNumIterations, bolza::scalar_t absoluteTolerance, bolza::scalar_t relativeTolerance,
                                   bool verbose) {
  bolza::pipg::Settings settings;
  settings.maxNumIterations = maxNumIterations;
  settings.absoluteTolerance = absoluteTolerance;
  settings.relativeTolerance = relativeTolerance;
  settings.checkTerminationInterval = 1;
  settings.displayShortSummary = verbose;

  return settings;
}

class PIPGSolverTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 0;
  static constexpr size_t numDecisionVariables_ = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints_ = N_ * (nx_ + nc_);
  static constexpr size_t numThreads_ = 8;
  static constexpr bool verbose_ = true;

  PIPGSolverTest() : solver(configurePipg(30000, 1e-10, 1e-3, verbose_)) {
    srand(10);

    // Construct OCP problem
    x0 = bolza::vector_t::Random(nx_);

    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(bolza::GetRandomDynamics(nx_, nu_));
      costArray.push_back(bolza::GetRandomCost(nx_, nu_));
      constraintsArray.push_back(bolza::GetRandomConstraints(nx_, nu_, nc_));
    }
    costArray.push_back(bolza::GetRandomCost(nx_, nu_));
    constraintsArray.push_back(bolza::GetRandomConstraints(nx_, nu_, nc_));

    solver.resize(bolza::ExtractSizesFromProblem(dynamicsArray, costArray, &constraintsArray));
    bolza::GetCostMatrix(solver.size(), x0, costArray, costApproximation);
    bolza::GetConstraintMatrix(solver.size(), x0, dynamicsArray, nullptr, nullptr, constraintsApproximation);
  }

  bolza::vector_t x0;
  bolza::ScalarFunctionQuadraticApproximation costApproximation;
  bolza::VectorFunctionLinearApproximation constraintsApproximation;
  std::vector<bolza::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<bolza::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<bolza::VectorFunctionLinearApproximation> constraintsArray;

  bolza::PipgSolver solver;
  bolza::ThreadPool threadPool{numThreads_ - 1u, 50};
};

constexpr size_t PIPGSolverTest::N_;
constexpr size_t PIPGSolverTest::nx_;
constexpr size_t PIPGSolverTest::nu_;
constexpr size_t PIPGSolverTest::nc_;
constexpr size_t PIPGSolverTest::numDecisionVariables_;
constexpr size_t PIPGSolverTest::numConstraints_;
constexpr size_t PIPGSolverTest::numThreads_;
constexpr bool PIPGSolverTest::verbose_;

TEST_F(PIPGSolverTest, correctness) {
  // bolza::qp_solver::SolveDenseQP use  Gz + g = 0 for constraints
  auto QPconstraints = constraintsApproximation;
  QPconstraints.f = -QPconstraints.f;
  bolza::vector_t primalSolutionQP;
  std::tie(primalSolutionQP, std::ignore) = bolza::qp_solver::SolveDenseQp(costApproximation, QPconstraints);

  Eigen::JacobiSVD<bolza::matrix_t> Svd(costApproximation.dfdxx);
  bolza::vector_t s = svd.singularValues();
  const bolza::scalar_t lambda = s(0);
  const bolza::scalar_t mu = s(svd.Rank() - 1);
  Eigen::JacobiSVD<bolza::matrix_t> svdGTG(constraintsApproximation.dfdx.transpose() * constraintsApproximation.dfdx);
  const bolza::scalar_t sigma = svdGTG.singularValues()(0);
  const bolza::pipg::PipgBounds pipgBounds{mu, lambda, sigma};

  bolza::vector_t primalSolutionPIPG;
  std::ignore = bolza::pipg::SingleThreadPipg(solver.Settings(), costApproximation.dfdxx.sparseView(), costApproximation.dfdx,
                                             constraintsApproximation.dfdx.sparseView(), constraintsApproximation.f,
                                             bolza::vector_t::Ones(solver.GetNumDynamicsConstraints()), pipgBounds, primalSolutionPIPG);

  bolza::vector_array_t scalingVectors(N_, bolza::vector_t::Ones(nx_));
  bolza::vector_array_t X, U;
  std::ignore = solver.Solve(threadPool, x0, dynamicsArray, costArray, nullptr, scalingVectors, nullptr, pipgBounds, X, U);

  bolza::vector_t primalSolutionPIPGParallel;
  bolza::ToKktSolution(X, U, primalSolutionPIPGParallel);

  auto calculateConstraintViolation = [&](const bolza::vector_t& sol) -> bolza::scalar_t {
    return (constraintsApproximation.dfdx * sol - constraintsApproximation.f).cwiseAbs().maxCoeff();
  };
  auto calculateCost = [&](const bolza::vector_t& sol) -> bolza::scalar_t {
    return (0.5 * sol.transpose() * costApproximation.dfdxx * sol + costApproximation.dfdx.transpose() * sol)(0);
  };
  bolza::scalar_t QPCost = calculateCost(primalSolutionQP);
  bolza::scalar_t QPConstraintViolation = calculateConstraintViolation(primalSolutionQP);

  bolza::scalar_t PIPGCost = calculateCost(primalSolutionPIPG);
  bolza::scalar_t PIPGConstraintViolation = calculateConstraintViolation(primalSolutionPIPG);

  bolza::scalar_t PIPGParallelCost = calculateCost(primalSolutionPIPGParallel);
  bolza::scalar_t PIPGParallelCConstraintViolation = calculateConstraintViolation(primalSolutionPIPGParallel);

  if (verbose_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ [TestPIPG] Correctness ++++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";

    std::cerr << "mu:  " << mu << " lambda: " << Lambda << " sigma: " << sigma << "\n";

    std::cerr << "QP-PIPG:  " << (primalSolutionQP - primalSolutionPIPG).cwiseAbs().sum() << "\n";
    std::cerr << "PIPG-PIPGParallel:  " << (primalSolutionPIPG - primalSolutionPIPGParallel).cwiseAbs().sum() << "\n";

    std::cerr << "QP:\n";
    std::cerr << "cost:   " << QPCost << "    "
              << "constraint-violation:   " << QPConstraintViolation << "\n\n";

    std::cerr << "PIPG:\n";
    std::cerr << "cost:   " << PIPGCost << "    "
              << "constraint-violation:   " << PIPGConstraintViolation << "\n\n";

    std::cerr << "PIPGParallel:\n";
    std::cerr << "cost:   " << PIPGParallelCost << "    "
              << "constraint-violation:   " << PIPGParallelCConstraintViolation << "\n\n"
              << std::endl;
  }
  EXPECT_TRUE(primalSolutionQP.isApprox(primalSolutionPIPG, solver.Settings().absoluteTolerance * 10.0))
      << "Inf-norm of (QP - PIPG): " << (primalSolutionQP - primalSolutionPIPG).cwiseAbs().maxCoeff();

  EXPECT_TRUE(primalSolutionPIPGParallel.isApprox(primalSolutionPIPG, solver.Settings().absoluteTolerance * 10.0))
      << "Inf-norm of (PIPG - PIPGParallel): " << (primalSolutionPIPGParallel - primalSolutionPIPG).cwiseAbs().maxCoeff();

  // Threshold is the (absoluteTolerance) * (2-Norm of the hessian H)[lambda]
  EXPECT_TRUE(std::abs(QPCost - PIPGCost) < solver.Settings().absoluteTolerance * lambda)
      << "Absolute diff is [" << std::abs(QPCost - PIPGCost) << "] which is larger than [" << solver.Settings().absoluteTolerance * Lambda
      << "]";

  EXPECT_TRUE(std::abs(PIPGParallelCost - PIPGCost) < solver.Settings().absoluteTolerance)
      << "Absolute diff is [" << std::abs(PIPGParallelCost - PIPGCost) << "] which is larger than ["
      << solver.Settings().absoluteTolerance * Lambda << "]";

  ASSERT_TRUE(std::abs(PIPGConstraintViolation) < solver.Settings().absoluteTolerance);
  EXPECT_TRUE(std::abs(QPConstraintViolation - PIPGConstraintViolation) < solver.Settings().absoluteTolerance * 10.0);
  EXPECT_TRUE(std::abs(PIPGParallelCConstraintViolation - PIPGConstraintViolation) < solver.Settings().absoluteTolerance * 10.0);
}