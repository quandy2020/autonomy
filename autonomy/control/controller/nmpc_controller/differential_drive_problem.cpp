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
 * @file differential_drive_problem.cpp
 * @brief Implementation of nmpc_controller::DifferentialDriveProblem
 */

#include "autonomy/control/controller/nmpc_controller/differential_drive_problem.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

DifferentialDriveProblem::DifferentialDriveProblem(
    double dt, proto::NMPCControllerOptions options)
    : fmpc::FmpcProblem<3, 2, 4>(dt), options_(std::move(options)) {}

void DifferentialDriveProblem::SetReferences(
    const std::vector<PathReference::Pose2D>& refs,
    const PathReference::Pose2D& terminal) {
    refs_ = refs;
    terminal_ = terminal;
}

void DifferentialDriveProblem::SetOptions(
    const proto::NMPCControllerOptions& options) {
    options_ = options;
}

double DifferentialDriveProblem::NormalizeAngle(double yaw) {
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}

PathReference::Pose2D DifferentialDriveProblem::RefAt(double t) const {
    if (refs_.empty()) {
        return {};
    }
    const int idx = std::min(static_cast<int>(t / dt()), static_cast<int>(refs_.size()) - 1);
    return refs_[static_cast<size_t>(std::max(idx, 0))];
}

DifferentialDriveProblem::StateVector DifferentialDriveProblem::stateEq(
    double /*t*/, const StateVector& x, const InputVector& u) const {
    StateVector next = x;
    const double yaw = x(2);
    const double v = u(0);
    const double w = u(1);
    next(0) += dt() * v * std::cos(yaw);
    next(1) += dt() * v * std::sin(yaw);
    next(2) = NormalizeAngle(yaw + dt() * w);
    return next;
}

double DifferentialDriveProblem::runningCost(double t, const StateVector& x,
                                             const InputVector& u) const {
    const auto ref = RefAt(t);
    const double dx = x(0) - ref.x;
    const double dy = x(1) - ref.y;
    const double dyaw = NormalizeAngle(x(2) - ref.yaw);
    return options_.weight_pos() * (dx * dx + dy * dy) +
           options_.weight_yaw() * dyaw * dyaw +
           options_.weight_linear() * u(0) * u(0) +
           options_.weight_angular() * u(1) * u(1);
}

double DifferentialDriveProblem::terminalCost(double /*t*/,
                                              const StateVector& x) const {
    const double dx = x(0) - terminal_.x;
    const double dy = x(1) - terminal_.y;
    const double dyaw = NormalizeAngle(x(2) - terminal_.yaw);
    return options_.weight_terminal_pos() * (dx * dx + dy * dy) +
           options_.weight_terminal_yaw() * dyaw * dyaw;
}

void DifferentialDriveProblem::calcStateEqDeriv(
    double /*t*/, const StateVector& x, const InputVector& u,
    Eigen::Ref<StateStateDimMatrix> state_eq_deriv_x,
    Eigen::Ref<StateInputDimMatrix> state_eq_deriv_u) const {
    state_eq_deriv_x.setIdentity();
    const double yaw = x(2);
    const double v = u(0);
    state_eq_deriv_x(0, 2) = -dt() * v * std::sin(yaw);
    state_eq_deriv_x(1, 2) = dt() * v * std::cos(yaw);

    state_eq_deriv_u.setZero();
    state_eq_deriv_u(0, 0) = dt() * std::cos(yaw);
    state_eq_deriv_u(1, 0) = dt() * std::sin(yaw);
    state_eq_deriv_u(2, 1) = dt();
}

void DifferentialDriveProblem::calcStateEqDeriv(
    double t, const StateVector& x, const InputVector& u,
    Eigen::Ref<StateStateDimMatrix> state_eq_deriv_x,
    Eigen::Ref<StateInputDimMatrix> state_eq_deriv_u,
    std::vector<StateStateDimMatrix>& state_eq_deriv_xx,
    std::vector<InputInputDimMatrix>& state_eq_deriv_uu,
    std::vector<StateInputDimMatrix>& state_eq_deriv_xu) const {
    calcStateEqDeriv(t, x, u, state_eq_deriv_x, state_eq_deriv_u);
    state_eq_deriv_xx.assign(kStateDim, StateStateDimMatrix::Zero());
    state_eq_deriv_uu.assign(kInputDim, InputInputDimMatrix::Zero());
    state_eq_deriv_xu.assign(kStateDim, StateInputDimMatrix::Zero());
}

void DifferentialDriveProblem::calcRunningCostDeriv(
    double t, const StateVector& x, const InputVector& u,
    Eigen::Ref<StateVector> running_cost_deriv_x,
    Eigen::Ref<InputVector> running_cost_deriv_u) const {
    const auto ref = RefAt(t);
    running_cost_deriv_x(0) = 2.0 * options_.weight_pos() * (x(0) - ref.x);
    running_cost_deriv_x(1) = 2.0 * options_.weight_pos() * (x(1) - ref.y);
    running_cost_deriv_x(2) =
        2.0 * options_.weight_yaw() * NormalizeAngle(x(2) - ref.yaw);
    running_cost_deriv_u(0) = 2.0 * options_.weight_linear() * u(0);
    running_cost_deriv_u(1) = 2.0 * options_.weight_angular() * u(1);
}

void DifferentialDriveProblem::calcRunningCostDeriv(
    double t, const StateVector& x, const InputVector& u,
    Eigen::Ref<StateVector> running_cost_deriv_x,
    Eigen::Ref<InputVector> running_cost_deriv_u,
    Eigen::Ref<StateStateDimMatrix> running_cost_deriv_xx,
    Eigen::Ref<InputInputDimMatrix> running_cost_deriv_uu,
    Eigen::Ref<StateInputDimMatrix> running_cost_deriv_xu) const {
    calcRunningCostDeriv(t, x, u, running_cost_deriv_x, running_cost_deriv_u);
    running_cost_deriv_xx.setZero();
    running_cost_deriv_xx(0, 0) = 2.0 * options_.weight_pos();
    running_cost_deriv_xx(1, 1) = 2.0 * options_.weight_pos();
    running_cost_deriv_xx(2, 2) = 2.0 * options_.weight_yaw();
    running_cost_deriv_uu.setZero();
    running_cost_deriv_uu(0, 0) = 2.0 * options_.weight_linear();
    running_cost_deriv_uu(1, 1) = 2.0 * options_.weight_angular();
    running_cost_deriv_xu.setZero();
}

void DifferentialDriveProblem::calcTerminalCostDeriv(
    double /*t*/, const StateVector& x,
    Eigen::Ref<StateVector> terminal_cost_deriv_x) const {
    terminal_cost_deriv_x(0) =
        2.0 * options_.weight_terminal_pos() * (x(0) - terminal_.x);
    terminal_cost_deriv_x(1) =
        2.0 * options_.weight_terminal_pos() * (x(1) - terminal_.y);
    terminal_cost_deriv_x(2) = 2.0 * options_.weight_terminal_yaw() *
                               NormalizeAngle(x(2) - terminal_.yaw);
}

void DifferentialDriveProblem::calcTerminalCostDeriv(
    double t, const StateVector& x, Eigen::Ref<StateVector> terminal_cost_deriv_x,
    Eigen::Ref<StateStateDimMatrix> terminal_cost_deriv_xx) const {
    calcTerminalCostDeriv(t, x, terminal_cost_deriv_x);
    terminal_cost_deriv_xx.setZero();
    terminal_cost_deriv_xx(0, 0) = 2.0 * options_.weight_terminal_pos();
    terminal_cost_deriv_xx(1, 1) = 2.0 * options_.weight_terminal_pos();
    terminal_cost_deriv_xx(2, 2) = 2.0 * options_.weight_terminal_yaw();
}

std::array<DifferentialDriveProblem::InputVector, 2>
DifferentialDriveProblem::InputLimits() const {
    InputVector lower;
    lower << options_.min_linear_vel(), -options_.max_angular_vel();
    InputVector upper;
    upper << options_.max_linear_vel(), options_.max_angular_vel();
    return {lower, upper};
}

DifferentialDriveProblem::IneqDimVector DifferentialDriveProblem::ineqConst(
    double /*t*/, const StateVector& /*x*/, const InputVector& u) const {
    IneqDimVector g;
    g(0) = u(0) - options_.max_linear_vel();
    g(1) = options_.min_linear_vel() - u(0);
    g(2) = u(1) - options_.max_angular_vel();
    g(3) = -options_.max_angular_vel() - u(1);
    return g;
}

void DifferentialDriveProblem::calcIneqConstDeriv(
    double /*t*/, const StateVector& /*x*/, const InputVector& /*u*/,
    Eigen::Ref<IneqStateDimMatrix> ineq_const_deriv_x,
    Eigen::Ref<IneqInputDimMatrix> ineq_const_deriv_u) const {
    ineq_const_deriv_x.setZero();
    ineq_const_deriv_u.setZero();
    ineq_const_deriv_u(0, 0) = 1.0;
    ineq_const_deriv_u(1, 0) = -1.0;
    ineq_const_deriv_u(2, 1) = 1.0;
    ineq_const_deriv_u(3, 1) = -1.0;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
