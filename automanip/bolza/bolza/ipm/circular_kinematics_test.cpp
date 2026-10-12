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

#include "bolza/ipm/ipm_solver.hpp"

#include <bolza/core/constraint/linear_state_constraint.hpp>
#include <bolza/core/constraint/linear_state_input_constraint.hpp>
#include <bolza/core/initialization/default_initializer.hpp>
#include "helpers/circular_kinematics.hpp"

using namespace bolza;

class CircleKinematics_MixedStateInputIneqConstraints final : public StateInputConstraint {
 public:
  CircleKinematics_MixedStateInputIneqConstraints(scalar_t xumin, scalar_t xumax)
      : StateInputConstraint(ConstraintOrder::Linear), xumin_(xumin), xumax_(xumax) {}
  ~CircleKinematics_MixedStateInputIneqConstraints() override = default;

  CircleKinematics_MixedStateInputIneqConstraints* Clone() const override {
    return new CircleKinematics_MixedStateInputIneqConstraints(*this);
  }

  size_t GetNumConstraints(scalar_t time) const override { return 2; }

  vector_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(2);
    e << (x(0) * u(0) - xumin_), (xumax_ - x(1) * u(1));
    return e;
  }

  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e(2, x.size(), u.size());
    e.f = GetValue(t, x, u, preComp);
    e.dfdx << u(0), 0, 0, -u(1);
    e.dfdu << x(0), 0, 0, -x(1);
    return e;
  }

 private:
  const scalar_t xumin_, xumax_;
};

TEST(test_circular_kinematics, solve_projected_EqConstraints) {
  // optimal control problem
  OptimalControlProblem problem = CreateCircularKinematicsProblem("/tmp/bolza/ipm_test_generated");

  // Initializer
  DefaultInitializer zeroInitializer(2);

  // Solver settings
  const auto settings = []() {
    ipm::Settings s;
    s.dt = 0.01;
    s.ipmIteration = 20;
    s.useFeedbackPolicy = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.nThreads = 1;
    s.initialBarrierParameter = 1.0e-02;
    s.targetBarrierParameter = 1.0e-04;
    s.barrierLinearDecreaseFactor = 0.2;
    s.barrierSuperlinearDecreasePower = 1.5;
    s.fractionToBoundaryMargin = 0.995;
    return s;
  }();

  // Additional problem definitions
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 1.0;
  const vector_t initState = (vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.Run(startTime, initState, finalTime);

  const auto primalSolution = solver.PrimalSolution(finalTime);

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.GetPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->ComputeInput(t, x)));
  }
}

TEST(test_circular_kinematics, solve_projected_EqConstraints_IneqConstraints) {
  constexpr size_t STATE_DIM = 2;
  constexpr size_t INPUT_DIM = 2;

  // optimal control problem
  OptimalControlProblem problem = CreateCircularKinematicsProblem("/tmp/bolza/ipm_test_generated");

  auto getStateBoxConstraint = [&](const vector_t& minState, const vector_t& maxState) {
    constexpr size_t numIneqConstraint = 2 * STATE_DIM;
    const vector_t e = (vector_t(numIneqConstraint) << -minState, maxState).finished();
    const matrix_t C =
        (matrix_t(numIneqConstraint, STATE_DIM) << matrix_t::Identity(STATE_DIM, STATE_DIM), -matrix_t::Identity(STATE_DIM, STATE_DIM))
            .finished();
    return std::make_unique<LinearStateConstraint>(std::move(e), std::move(C));
  };

  auto getInputBoxConstraint = [&](const vector_t& minInput, const vector_t& maxInput) {
    constexpr size_t numIneqConstraint = 2 * INPUT_DIM;
    const vector_t e = (vector_t(numIneqConstraint) << -minInput, maxInput).finished();
    const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
    const matrix_t D =
        (matrix_t(numIneqConstraint, INPUT_DIM) << matrix_t::Identity(INPUT_DIM, INPUT_DIM), -matrix_t::Identity(INPUT_DIM, INPUT_DIM))
            .finished();
    return std::make_unique<LinearStateInputConstraint>(std::move(e), std::move(C), std::move(D));
  };

  // inequality constraints
  const vector_t umin = (vector_t(2) << -0.5, -0.5).finished();
  const vector_t umax = (vector_t(2) << 0.5, 0.5).finished();
  problem.inequalityConstraintPtr->Add("ubound", getInputBoxConstraint(umin, umax));
  const vector_t xmin = (vector_t(2) << -0.5, -0.5).finished();
  const vector_t xmax = (vector_t(2) << 1.0e03, 1.0e03).finished();  // no upper bound
  problem.stateInequalityConstraintPtr->Add("xbound", getStateBoxConstraint(xmin, xmax));
  problem.finalInequalityConstraintPtr->Add("xbound", getStateBoxConstraint(xmin, xmax));

  // Initializer
  DefaultInitializer zeroInitializer(2);

  // Solver settings
  const auto settings = []() {
    ipm::Settings s;
    s.dt = 0.01;
    s.ipmIteration = 20;
    s.useFeedbackPolicy = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.nThreads = 1;
    s.initialBarrierParameter = 1.0e-02;
    s.targetBarrierParameter = 1.0e-04;
    s.barrierLinearDecreaseFactor = 0.2;
    s.barrierSuperlinearDecreasePower = 1.5;
    s.fractionToBoundaryMargin = 0.995;
    return s;
  }();

  // Additional problem definitions
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 1.0;
  const vector_t initState = (vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.Run(startTime, initState, finalTime);

  const auto primalSolution = solver.PrimalSolution(finalTime);

  // check constraint satisfaction
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    if (primalSolution.inputTrajectory_[i].size() > 0) {
      const scalar_t projectionConstraintViolation = primalSolution.stateTrajectory_[i].dot(primalSolution.inputTrajectory_[i]);
      EXPECT_LT(std::abs(projectionConstraintViolation), 1.0e-06);
    }
  }

  // check constraint satisfaction
  for (const auto& x : primalSolution.stateTrajectory_) {
    if (x.size() > 0) {
      ASSERT_TRUE((x - xmin).minCoeff() >= 0);
      ASSERT_TRUE((xmax - x).minCoeff() >= 0);
    }
  }
  for (const auto& u : primalSolution.inputTrajectory_) {
    if (u.size() > 0) {
      ASSERT_TRUE((u - umin).minCoeff() >= 0);
      ASSERT_TRUE((umax - u).minCoeff() >= 0);
    }
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.GetPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->ComputeInput(t, x)));
  }

  // solve with shifted horizon
  const scalar_array_t shiftTime = {0.05, 0.1, 0.3, 0.5, 0.8, 0.12, 0.16, 0.19};
  for (const auto e : shiftTime) {
    solver.Run(startTime + e, initState, finalTime + e);
  }
}

TEST(test_circular_kinematics, solve_projected_EqConstraints_MixedIneqConstraints) {
  // optimal control problem
  OptimalControlProblem problem = CreateCircularKinematicsProblem("/tmp/bolza/ipm_test_generated");

  // inequality constraints
  const scalar_t xumin = -2.0;
  const scalar_t xumax = 2.0;
  auto stateInputIneqConstraint = std::make_unique<CircleKinematics_MixedStateInputIneqConstraints>(xumin, xumax);
  auto stateInputIneqConstraintCloned = stateInputIneqConstraint->Clone();
  problem.inequalityConstraintPtr->Add("xubound", std::move(stateInputIneqConstraint));

  // Initializer
  DefaultInitializer zeroInitializer(2);

  // Solver settings
  const auto settings = []() {
    ipm::Settings s;
    s.dt = 0.01;
    s.ipmIteration = 20;
    s.useFeedbackPolicy = true;
    s.computeLagrangeMultipliers = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.nThreads = 1;
    s.initialBarrierParameter = 1.0e-02;
    s.targetBarrierParameter = 1.0e-04;
    s.barrierLinearDecreaseFactor = 0.2;
    s.barrierSuperlinearDecreasePower = 1.5;
    s.fractionToBoundaryMargin = 0.995;
    return s;
  }();

  // Additional problem definitions
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 1.0;
  const vector_t initState = (vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.Run(startTime, initState, finalTime);

  const auto primalSolution = solver.PrimalSolution(finalTime);

  // check constraint satisfaction
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    if (primalSolution.inputTrajectory_[i].size() > 0) {
      const scalar_t projectionConstraintViolation = primalSolution.stateTrajectory_[i].dot(primalSolution.inputTrajectory_[i]);
      EXPECT_LT(std::abs(projectionConstraintViolation), 1.0e-06);
    }
  }

  // check constraint satisfaction
  const size_t N = primalSolution.inputTrajectory_.size();
  for (size_t i = 0; i < N; ++i) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    const auto constraintValue = stateInputIneqConstraintCloned->GetValue(t, x, u, PreComputation());
    ASSERT_TRUE(constraintValue.minCoeff() >= 0.0);
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.GetPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->ComputeInput(t, x)));
  }

  // Check Lagrange multipliers
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    ASSERT_NO_THROW(const auto multiplier = solver.GetStateInputEqualityConstraintLagrangian(t, x););
  }

  // solve with shifted horizon
  const scalar_array_t shiftTime = {0.05, 0.1, 0.3, 0.5, 0.8, 0.12, 0.16, 0.19};
  for (const auto e : shiftTime) {
    solver.Run(startTime + e, initState, finalTime + e);
  }
}