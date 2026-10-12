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

#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include "helpers/double_integrator_reaching_task.hpp"

#include <bolza/ddp/ilqr.hpp>
#include <bolza/ddp/slq.hpp>

namespace bolza {

namespace {
// rollout settings
rollout::Settings getRolloutSettings(ddp::Algorithm alg) {
  rollout::Settings s;
  s.absTolODE = 1e-9;
  s.relTolODE = 1e-6;
  s.timeStep = DoubleIntegratorReachingTask::timeStep;
  s.integratorType = (alg == ddp::Algorithm::SLQ) ? IntegratorType::ODE45 : IntegratorType::RK4;
  s.maxNumStepsPerSecond = 100000;
  s.checkNumericalStability = true;
  return s;
}

// DDP settings
ddp::Settings getDdpSettings(ddp::Algorithm alg, bool display) {
  ddp::Settings s;
  s.algorithm_ = alg;
  s.nThreads_ = 2;
  s.maxNumIterations_ = 50;
  s.displayInfo_ = false;
  s.displayShortSummary_ = display;
  s.debugPrintRollout_ = false;
  s.minRelCost_ = DoubleIntegratorReachingTask::minRelCost;
  s.constraintTolerance_ = DoubleIntegratorReachingTask::constraintTolerance;
  s.timeStep_ = DoubleIntegratorReachingTask::timeStep;
  s.backwardPassIntegratorType_ = (alg == ddp::Algorithm::SLQ) ? IntegratorType::ODE45 : IntegratorType::RK4;
  s.strategy_ = search_strategy::Type::LINE_SEARCH;
  s.lineSearch_.minStepLength = 0.01;
  s.lineSearch_.maxStepLength = 1.0;
  return s;
}
}  // unnamed namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Reaching task at event Time (tGoal) for a problem with horizon 2*tGoal. The solution input should be zero in [tGoal, 2*tGoal]. */
class PreJumpDoubleIntegratorReachingTask : public DoubleIntegratorReachingTask,
                                            public testing::Test,
                                            public testing::WithParamInterface<DoubleIntegratorReachingTask::PenaltyType> {
 protected:
  PreJumpDoubleIntegratorReachingTask() {
    // reference manager
    referenceManagerPtr = GetReferenceManagerPtr();
    // optimal control problem
    ocp.dynamicsPtr = GetDynamicsPtr();
    ocp.costPtr->Add("cost", GetCostPtr());
    ocp.equalityConstraintPtr->Add("zero_force", std::make_unique<ZeroInputConstraint>(*referenceManagerPtr));
    ocp.preJumpEqualityLagrangianPtr->Add("goal_reaching", GetGoalReachingAugmentedLagrangian(xGoal, GetParam()));
  }

  void test(const PrimalSolution& primalSolution) const {
    for (size_t i = 0; i < primalSolution.timeTrajectory_.size(); ++i) {
      // zero input after tGoal
      if (primalSolution.timeTrajectory_[i] > tGoal) {
        EXPECT_NEAR(primalSolution.inputTrajectory_[i](0), 0.0, constraintTolerance) << " at time " << primalSolution.timeTrajectory_[i];
      }
      // reaching to the target
      if (primalSolution.timeTrajectory_[i] > tGoal) {
        EXPECT_TRUE(primalSolution.stateTrajectory_[i].isApprox(xGoal, constraintTolerance))
            << " at time " << primalSolution.timeTrajectory_[i];
      }
    }  // end of i loop
  }

  OptimalControlProblem ocp;
  std::shared_ptr<ReferenceManager> referenceManagerPtr;
};

TEST_P(PreJumpDoubleIntegratorReachingTask, SLQ) {
  constexpr bool display = true;
  constexpr auto alg = ddp::Algorithm::SLQ;
  // rollout
  TimeTriggeredRollout Rollout(*ocp.dynamicsPtr, getRolloutSettings(alg));
  // solver
  SLQ ddp(getDdpSettings(alg, display), rollout, ocp, *GetInitializer());
  ddp.SetReferenceManager(referenceManagerPtr);
  // run solver
  ddp.Run(0.0, xInit, 2.0 * tGoal);
  PrimalSolution primalSolution;
  ddp.GetPrimalSolution(2.0 * tGoal, &primalSolution);
  // test
  test(primalSolution);
  PrintSolution(primalSolution, display);
}

INSTANTIATE_TEST_CASE_P(PreJumpDoubleIntegratorReachingTaskCase, PreJumpDoubleIntegratorReachingTask,
                        testing::ValuesIn({DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty,
                                           DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty}),
                        [](const testing::TestParamInfo<PreJumpDoubleIntegratorReachingTask::ParamType>& info) {
                          if (info.param == DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty) {
                            return std::string("QuadraticPenalty");
                          } else if (info.param == DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty) {
                            return std::string("SmoothAbsolutePenalty");
                          }
                        });

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/* Reaching task at final time for a problem with horizon tGoal. */
class FinalDoubleIntegratorReachingTask : public DoubleIntegratorReachingTask,
                                          public testing::Test,
                                          public testing::WithParamInterface<DoubleIntegratorReachingTask::PenaltyType> {
 protected:
  FinalDoubleIntegratorReachingTask() {
    // reference manager
    referenceManagerPtr = GetReferenceManagerPtr();
    // optimal control problem
    ocp.dynamicsPtr = GetDynamicsPtr();
    ocp.costPtr->Add("cost", GetCostPtr());
    ocp.finalEqualityLagrangianPtr->Add("goal_reaching", GetGoalReachingAugmentedLagrangian(xGoal, GetParam()));
  }

  void test(const PrimalSolution& primalSolution) const {
    // reaching to the target
    EXPECT_TRUE(primalSolution.stateTrajectory_.back().isApprox(xGoal, constraintTolerance))
        << " at time " << primalSolution.timeTrajectory_.back();
  }

  OptimalControlProblem ocp;
  std::shared_ptr<ReferenceManager> referenceManagerPtr;
};

TEST_P(FinalDoubleIntegratorReachingTask, SLQ) {
  constexpr bool display = true;
  constexpr auto alg = ddp::Algorithm::SLQ;
  // rollout
  TimeTriggeredRollout Rollout(*ocp.dynamicsPtr, getRolloutSettings(alg));
  // solver
  SLQ ddp(getDdpSettings(alg, display), rollout, ocp, *GetInitializer());
  ddp.SetReferenceManager(referenceManagerPtr);
  // run solver
  ddp.Run(0.0, xInit, tGoal);
  PrimalSolution primalSolution;
  ddp.GetPrimalSolution(tGoal, &primalSolution);
  // test
  test(primalSolution);
  PrintSolution(primalSolution, display);
}

INSTANTIATE_TEST_CASE_P(FinalDoubleIntegratorReachingTaskCase, FinalDoubleIntegratorReachingTask,
                        testing::ValuesIn({DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty,
                                           DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty}),
                        [](const testing::TestParamInfo<PreJumpDoubleIntegratorReachingTask::ParamType>& info) {
                          if (info.param == DoubleIntegratorReachingTask::PenaltyType::QuadraticPenalty) {
                            return std::string("QuadraticPenalty");
                          } else if (info.param == DoubleIntegratorReachingTask::PenaltyType::SmoothAbsolutePenalty) {
                            return std::string("SmoothAbsolutePenalty");
                          } else {
                            throw std::runtime_error("Undefined PenaltyTyp!");
                          }
                        });

}  // namespace bolza
