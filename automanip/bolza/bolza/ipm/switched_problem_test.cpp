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

#include <bolza/core/constraint/linear_state_input_constraint.hpp>
#include <bolza/core/constraint/state_input_constraint.hpp>
#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/oc/oc_data/time_discretization.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

#include "helpers/problems_generation_test.hpp"

namespace bolza {
namespace {
/**
 * A constraint that in mode 0 sets u[0] = 0 and in mode 1 sets u[1] = 1
 */
class SwitchedConstraint : public StateInputConstraint {
 public:
  explicit SwitchedConstraint(std::shared_ptr<ReferenceManager> referenceManagerPtr)
      : StateInputConstraint(ConstraintOrder::Linear), referenceManagerPtr_(std::move(referenceManagerPtr)) {
    constexpr int n = 3;
    constexpr int m = 2;
    constexpr int nc = 1;

    auto stateInputConstraints0 = VectorFunctionLinearApproximation::Zero(nc, n, m);
    stateInputConstraints0.dfdu << 1.0, 0.0;

    auto stateInputConstraints1 = VectorFunctionLinearApproximation::Zero(nc, n, m);
    stateInputConstraints1.dfdu << 0.0, 1.0;

    subsystemConstraintsPtr_.emplace_back(GetOcs2Constraints(stateInputConstraints0));
    subsystemConstraintsPtr_.emplace_back(GetOcs2Constraints(stateInputConstraints1));
  }

  ~SwitchedConstraint() override = default;
  SwitchedConstraint* Clone() const override { return new SwitchedConstraint(*this); }

  size_t GetNumConstraints(scalar_t time) const override {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(time);
    return subsystemConstraintsPtr_[activeMode]->GetNumConstraints(time);
  }

  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(time);
    return subsystemConstraintsPtr_[activeMode]->GetValue(time, state, input, preComp);
  };

  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const {
    const auto activeMode = referenceManagerPtr_->GetModeSchedule().ModeAtTime(time);
    return subsystemConstraintsPtr_[activeMode]->GetLinearApproximation(time, state, input, preComp);
  }

 private:
  SwitchedConstraint(const SwitchedConstraint& other)
      : bolza::StateInputConstraint(other), referenceManagerPtr_(other.referenceManagerPtr_) {
    for (const auto& constraint : other.subsystemConstraintsPtr_) {
      subsystemConstraintsPtr_.emplace_back(constraint->Clone());
    }
  }

  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::vector<std::unique_ptr<bolza::StateInputConstraint>> subsystemConstraintsPtr_;
};

std::pair<PrimalSolution, std::vector<PerformanceIndex>> solveWithEventTime(scalar_t eventTime) {
  constexpr int n = 3;
  constexpr int m = 2;

  bolza::OptimalControlProblem problem;

  // System
  const auto dynamics = bolza::GetRandomDynamics(n, m);
  const auto jumpMap = matrix_t::Random(n, n);
  problem.dynamicsPtr.reset(new bolza::LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // Cost
  problem.costPtr->Add("intermediateCost", bolza::GetOcs2Cost(bolza::GetRandomCost(n, m)));
  problem.softConstraintPtr->Add("intermediateCost", bolza::GetOcs2Cost(bolza::GetRandomCost(n, m)));
  problem.preJumpCostPtr->Add("eventCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(n, 0)));
  problem.preJumpSoftConstraintPtr->Add("eventCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(n, 0)));
  problem.finalCostPtr->Add("finalCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(n, 0)));
  problem.finalSoftConstraintPtr->Add("finalCost", bolza::GetOcs2StateCost(bolza::GetRandomCost(n, 0)));

  // Reference Manager
  const bolza::ModeSchedule ModeSchedule({eventTime}, {0, 1});
  const bolza::TargetTrajectories TargetTrajectories({0.0}, {bolza::vector_t::Random(n)}, {bolza::vector_t::Random(m)});
  auto referenceManagerPtr = std::make_shared<bolza::ReferenceManager>(targetTrajectories, modeSchedule);

  problem.targetTrajectoriesPtr = &targetTrajectories;

  // Constraint
  problem.equalityConstraintPtr->Add("switchedConstraint", std::make_unique<SwitchedConstraint>(referenceManagerPtr));

  bolza::DefaultInitializer zeroInitializer(m);

  // Solver settings
  bolza::ipm::Settings settings;
  settings.dt = 0.05;
  settings.ipmIteration = 20;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;

  // Additional problem definitions
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::vector_t initState = bolza::vector_t::Random(n);

  // Set up solver
  bolza::IpmSolver solver(settings, problem, zeroInitializer);
  solver.SetReferenceManager(referenceManagerPtr);

  // Solve
  solver.Run(startTime, initState, finalTime);
  return {solver.GetPrimalSolution(finalTime), solver.GetIterationsLog()};
}

}  // namespace
}  // namespace bolza

TEST(test_switched_problem, switched_constraint) {
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::scalar_t eventTime = 0.1875;
  const double tol = 1e-9;
  const auto solution = bolza::solveWithEventTime(eventTime);
  const auto& primalSolution = solution.first;
  const auto& performanceLog = solution.second;

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(performanceLog.size(), 2);
  ASSERT_LT(performanceLog.back().dynamicsViolationSSE, tol);

  // Should have correct node pre and post event time, with corresponding inputs
  ASSERT_EQ(primalSolution.timeTrajectory_[4], eventTime);
  ASSERT_EQ(primalSolution.timeTrajectory_[5], eventTime);
  ASSERT_LT(std::abs(primalSolution.inputTrajectory_[4][0]), tol);
  ASSERT_LT(std::abs(primalSolution.inputTrajectory_[5][1]), tol);

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  bolza::scalar_t t_check = startTime;
  bolza::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    bolza::vector_t xNominal =
        bolza::LinearInterpolation::Interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    bolza::vector_t uNominal =
        bolza::LinearInterpolation::Interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    bolza::vector_t uControl = primalSolution.controllerPtr_->ComputeInput(t_check, xNominal);
    if (t_check < eventTime) {
      ASSERT_LT(std::abs(uNominal[0]), tol);
      ASSERT_LT(std::abs(uControl[0]), tol);
    } else if (t_check > eventTime) {
      ASSERT_LT(std::abs(uNominal[1]), tol);
      ASSERT_LT(std::abs(uControl[1]), tol);
    }
    t_check += dt_check;
  }
}

TEST(test_switched_problem, event_at_beginning) {
  // The event should replace the start time, all inputs should be after the event.
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::scalar_t eventTime = 1e-8;
  const double tol = 1e-9;
  const auto solution = bolza::solveWithEventTime(eventTime);
  const auto& primalSolution = solution.first;
  const auto& performanceLog = solution.second;

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(performanceLog.size(), 2);
  ASSERT_LT(performanceLog.back().dynamicsViolationSSE, tol);

  // Should have correct post event time start
  ASSERT_EQ(primalSolution.timeTrajectory_[0], eventTime);
  ASSERT_NE(primalSolution.timeTrajectory_[1], eventTime);

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  bolza::scalar_t t_check = eventTime;
  bolza::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    bolza::vector_t xNominal =
        bolza::LinearInterpolation::Interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    bolza::vector_t uNominal =
        bolza::LinearInterpolation::Interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    bolza::vector_t uControl = primalSolution.controllerPtr_->ComputeInput(t_check, xNominal);
    ASSERT_LT(std::abs(uNominal[1]), tol);
    ASSERT_LT(std::abs(uControl[1]), tol);
    t_check += dt_check;
  }
}

TEST(test_switched_problem, event_at_end) {
  // The event should be ignored because its too close to the final time, all inputs should be before the event.
  const bolza::scalar_t startTime = 0.0;
  const bolza::scalar_t finalTime = 1.0;
  const bolza::scalar_t eventTime = 1.0 - 1e-8;
  const double tol = 1e-9;
  const auto solution = bolza::solveWithEventTime(eventTime);
  const auto& primalSolution = solution.first;
  const auto& performanceLog = solution.second;

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(performanceLog.size(), 2);
  ASSERT_LT(performanceLog.back().dynamicsViolationSSE, tol);

  // Should have correct node pre and post event time
  ASSERT_TRUE(std::none_of(primalSolution.timeTrajectory_.begin(), primalSolution.timeTrajectory_.end(),
                           [=](bolza::scalar_t t) { return t == eventTime; }));

  // Inspect solution, check at a dt smaller than solution to check interpolation around the switch.
  bolza::scalar_t t_check = startTime;
  bolza::scalar_t dt_check = 1e-5;
  while (t_check < finalTime) {
    bolza::vector_t xNominal =
        bolza::LinearInterpolation::Interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);
    bolza::vector_t uNominal =
        bolza::LinearInterpolation::Interpolate(t_check, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_);
    bolza::vector_t uControl = primalSolution.controllerPtr_->ComputeInput(t_check, xNominal);
    ASSERT_LT(std::abs(uNominal[0]), tol);
    ASSERT_LT(std::abs(uControl[0]), tol);
    t_check += dt_check;
  }
}
