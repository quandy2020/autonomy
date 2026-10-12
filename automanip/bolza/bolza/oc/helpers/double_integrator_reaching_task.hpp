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

#include <bolza/core/types.hpp>
#include <bolza/core/augmented_lagrangian/augmented_lagrangian.hpp>
#include <bolza/core/constraint/linear_state_constraint.hpp>
#include <bolza/core/constraint/linear_state_input_constraint.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/core/penalties/penalties.hpp>
#include <bolza/oc/oc_data/primal_solution.hpp>
#include <bolza/oc/synchronized_module/reference_manager.hpp>

namespace bolza {

class DoubleIntegratorReachingTask {
 public:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr scalar_t timeStep = 1e-2;
  static constexpr scalar_t minRelCost = 1e-12;  // to avoid early termination
  static constexpr scalar_t constraintTolerance = 1e-3;

  enum class PenaltyType {
    QuadraticPenalty,
    SmoothAbsolutePenalty,
  };

  DoubleIntegratorReachingTask() = default;
  virtual ~DoubleIntegratorReachingTask() = default;

 protected:
  const scalar_t tGoal = 1.0;
  const vector_t xInit = vector_t::Zero(STATE_DIM);
  const vector_t xGoal = (vector_t(STATE_DIM) << 2.0, 0.0).finished();

  std::unique_ptr<DefaultInitializer> GetInitializer() const { return std::make_unique<DefaultInitializer>(INPUT_DIM); }

  std::shared_ptr<ReferenceManager> GetReferenceManagerPtr() const {
    const ModeSchedule ModeSchedule({tGoal}, {0, 1});
    const TargetTrajectories TargetTrajectories({tGoal}, {xGoal}, {vector_t::Zero(INPUT_DIM)});
    return std::make_shared<ReferenceManager>(targetTrajectories, modeSchedule);
  }

  std::unique_ptr<StateInputCost> GetCostPtr() const {
    matrix_t Q = matrix_t::Zero(STATE_DIM, STATE_DIM);
    matrix_t R = 0.1 * matrix_t::Identity(INPUT_DIM, INPUT_DIM);
    return std::make_unique<QuadraticStateInputCost>(std::move(Q), std::move(R));
  }

  std::unique_ptr<SystemDynamicsBase> GetDynamicsPtr() const {
    matrix_t A = (matrix_t(STATE_DIM, STATE_DIM) << 0.0, 1.0, 0.0, 0.0).finished();
    matrix_t B = (matrix_t(STATE_DIM, INPUT_DIM) << 0.0, 1.0).finished();
    return std::make_unique<LinearSystemDynamics>(std::move(A), std::move(B));
  }

  std::unique_ptr<bolza::StateAugmentedLagrangian> GetGoalReachingAugmentedLagrangian(const vector_t& xGoal, PenaltyType penaltyType) {
    constexpr scalar_t scale = 10.0;
    constexpr scalar_t stepSize = 1.0;

    auto goalReachingConstraintPtr = std::make_unique<LinearStateConstraint>(-xGoal, matrix_t::Identity(xGoal.size(), xGoal.size()));

    switch (penaltyType) {
      case PenaltyType::QuadraticPenalty: {
        using penalty_type = augmented::QuadraticPenalty;
        penalty_type::Config boundsConfig{scale, stepSize};
        return Create(std::move(goalReachingConstraintPtr), penalty_type::Create(boundsConfig));
      }
      case PenaltyType::SmoothAbsolutePenalty: {
        using penalty_type = augmented::SmoothAbsolutePenalty;
        penalty_type::Config boundsConfig{scale, 0.1, stepSize};
        return Create(std::move(goalReachingConstraintPtr), penalty_type::Create(boundsConfig));
      }
      default:
        throw std::runtime_error("[TestCartpole::getPenalty] undefined penaltyType");
    }
  }

  /** This class enforces zero force at the second mode (mode = 1)*/
  class ZeroInputConstraint final : public StateInputConstraint {
   public:
    ZeroInputConstraint(const ReferenceManager& referenceManager)
        : StateInputConstraint(ConstraintOrder::Linear), referenceManagerPtr_(&referenceManager) {}

    ~ZeroInputConstraint() override = default;
    ZeroInputConstraint* Clone() const override { return new ZeroInputConstraint(*this); }

    size_t GetNumConstraints(scalar_t time) const override { return 1; }

    /** Only active after the fist mode */
    bool IsActive(scalar_t time) const override { return referenceManagerPtr_->GetModeSchedule().ModeAtTime(time) > 0 ? true : false; }

    vector_t GetValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override { return u; }

    VectorFunctionLinearApproximation GetLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                             const PreComputation&) const override {
      VectorFunctionLinearApproximation approx;
      approx.f = u;
      approx.dfdx.setZero(GetNumConstraints(t), x.size());
      approx.dfdu.setIdentity(GetNumConstraints(t), u.size());
      return approx;
    }

   private:
    ZeroInputConstraint(const ZeroInputConstraint&) = default;
    const ReferenceManager* referenceManagerPtr_;
  };

  /*
   * printout trajectory. Use the following commands for plotting in MATLAB:
   * subplot(2, 1, 1); plot(timeTrajectory, stateTrajectory);
   * xlabel("time [sec]"); legend("pos", "vel");
   * subplot(2, 1, 2); plot(timeTrajectory, inputTrajectory);
   * xlabel("time [sec]"); legend("force");
   */
  void PrintSolution(const PrimalSolution& primalSolution, bool display) const {
    if (display) {
      std::cerr << "\n";
      // time
      std::cerr << "timeTrajectory = [";
      for (const auto& t : primalSolution.timeTrajectory_) {
        std::cerr << t << "; ";
      }
      std::cerr << "];\n";
      // state
      std::cerr << "stateTrajectory = [";
      for (const auto& x : primalSolution.stateTrajectory_) {
        std::cerr << x.transpose() << "; ";
      }
      std::cerr << "];\n";
      // input
      std::cerr << "inputTrajectory = [";
      for (const auto& u : primalSolution.inputTrajectory_) {
        std::cerr << u.transpose() << "; ";
      }
      std::cerr << "];\n";
    }
  }
};

constexpr size_t DoubleIntegratorReachingTask::STATE_DIM;
constexpr size_t DoubleIntegratorReachingTask::INPUT_DIM;
constexpr scalar_t DoubleIntegratorReachingTask::timeStep;
constexpr scalar_t DoubleIntegratorReachingTask::minRelCost;
constexpr scalar_t DoubleIntegratorReachingTask::constraintTolerance;

}  // namespace bolza
