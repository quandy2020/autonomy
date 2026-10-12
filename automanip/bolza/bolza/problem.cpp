/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#include "bolza/problem.hpp"

#include <stdexcept>
#include <utility>

#include <bolza/core/cost/quadratic_state_cost.hpp>
#include <bolza/core/cost/quadratic_state_input_cost.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>

namespace bolza {

Problem::Problem() : reference_(std::make_shared<ReferenceManager>()) {}

Problem::~Problem() = default;

Problem::Problem(Problem&&) noexcept = default;

Problem& Problem::operator=(Problem&&) noexcept = default;

void Problem::SetDynamics(std::unique_ptr<SystemDynamicsBase> dynamics) {
  ocp_.dynamicsPtr = std::move(dynamics);
}

void Problem::SetLinearDynamics(const matrix_t& A, const matrix_t& B) {
  input_dim_ = static_cast<size_t>(B.cols());
  SetDynamics(std::make_unique<LinearSystemDynamics>(A, B));
}

void Problem::AddCost(std::string name, std::unique_ptr<StateInputCost> cost) {
  ocp_.costPtr->Add(std::move(name), std::move(cost));
}

void Problem::AddFinalCost(std::string name, std::unique_ptr<StateCost> cost) {
  ocp_.finalCostPtr->Add(std::move(name), std::move(cost));
}

void Problem::AddQuadraticCost(const matrix_t& Q, const matrix_t& R) {
  input_dim_ = static_cast<size_t>(R.cols());
  AddCost("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
}

void Problem::AddQuadraticFinalCost(const matrix_t& Qf) {
  AddFinalCost("finalCost", std::make_unique<QuadraticStateCost>(Qf));
}

void Problem::SetTarget(const vector_t& x_star, const vector_t& u_star, scalar_t time) {
  vector_t u = u_star;
  if (u.size() == 0) {
    if (input_dim_ == 0) {
      throw std::runtime_error("[bolza::Problem] SetTarget: set dynamics/cost first or pass u_star");
    }
    u = vector_t::Zero(static_cast<int>(input_dim_));
  } else {
    input_dim_ = static_cast<size_t>(u.size());
  }
  reference_->SetTargetTrajectories(TargetTrajectories({time}, {x_star}, {u}));
}

void Problem::SetTarget(TargetTrajectories trajectories) {
  if (!trajectories.inputTrajectory.empty()) {
    input_dim_ = static_cast<size_t>(trajectories.inputTrajectory.front().size());
  }
  reference_->SetTargetTrajectories(std::move(trajectories));
}

void Problem::SetInitialState(const vector_t& x0) { x0_ = x0; }

size_t Problem::GetInputDim() const { return input_dim_; }

const vector_t& Problem::GetInitialState() const { return x0_; }

OptimalControlProblem& Problem::GetOptimalControlProblem() { return ocp_; }

const OptimalControlProblem& Problem::GetOptimalControlProblem() const { return ocp_; }

std::shared_ptr<ReferenceManager> Problem::GetReferenceManager() const { return reference_; }

}  // namespace bolza
