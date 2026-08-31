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
 * @file differential_drive_problem.hpp
 * @brief Discrete unicycle OCP for DDP/FMPC backends
 */

#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/fmpc/fmpc_problem.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::DifferentialDriveProblem
 * @brief Discrete unicycle NMPC problem for differential-drive robots
 *
 * State x = [x, y, yaw], input u = [v, omega].
 *
 * Discrete dynamics (Euler):
 *   x_{k+1} = x_k + dt * v_k * cos(yaw_k)
 *   y_{k+1} = y_k + dt * v_k * sin(yaw_k)
 *   yaw_{k+1} = yaw_k + dt * omega_k
 *
 * Stage cost tracks path references; terminal cost penalizes goal pose error.
 * Four input inequalities encode box limits on v and omega for FMPC.
 */
class DifferentialDriveProblem
    : public fmpc::FmpcProblem<3, 2, 4> {
 public:
    static constexpr int kStateDim = 3;
    static constexpr int kInputDim = 2;

    using StateVector = Eigen::Matrix<double, kStateDim, 1>;
    using InputVector = Eigen::Matrix<double, kInputDim, 1>;

    /**
     * @brief Constructor
     * @param dt Model discretization timestep [s]
     * @param options Cost weights and velocity limits
     */
    explicit DifferentialDriveProblem(double dt, proto::NMPCControllerOptions options);

    /**
     * @brief Set horizon reference poses and terminal goal
     * @param refs Running reference poses along the horizon
     * @param terminal Terminal pose at horizon end
     */
    void SetReferences(const std::vector<PathReference::Pose2D>& refs,
                       const PathReference::Pose2D& terminal);

    /**
     * @brief Update cost weights and velocity limits at runtime
     * @param options Updated NMPC options
     */
    void SetOptions(const proto::NMPCControllerOptions& options);

    /**
     * @brief One-step discrete state transition f(x,u)
     */
    StateVector stateEq(double t, const StateVector& x,
                        const InputVector& u) const override;

    /**
     * @brief Stage cost L(x,u)
     */
    double runningCost(double t, const StateVector& x,
                       const InputVector& u) const override;

    /**
     * @brief Terminal cost phi(x)
     */
    double terminalCost(double t, const StateVector& x) const override;

    /**
     * @brief Jacobian of stateEq w.r.t. x and u (first order)
     */
    void calcStateEqDeriv(double t, const StateVector& x, const InputVector& u,
                          Eigen::Ref<StateStateDimMatrix> state_eq_deriv_x,
                          Eigen::Ref<StateInputDimMatrix> state_eq_deriv_u) const override;

    /**
     * @brief Higher-order stateEq derivatives (unused; zero-filled)
     */
    void calcStateEqDeriv(double t, const StateVector& x, const InputVector& u,
                          Eigen::Ref<StateStateDimMatrix> state_eq_deriv_x,
                          Eigen::Ref<StateInputDimMatrix> state_eq_deriv_u,
                          std::vector<StateStateDimMatrix>& state_eq_deriv_xx,
                          std::vector<InputInputDimMatrix>& state_eq_deriv_uu,
                          std::vector<StateInputDimMatrix>& state_eq_deriv_xu) const override;

    /**
     * @brief Gradient of running cost w.r.t. x and u
     */
    void calcRunningCostDeriv(double t, const StateVector& x, const InputVector& u,
                              Eigen::Ref<StateVector> running_cost_deriv_x,
                              Eigen::Ref<InputVector> running_cost_deriv_u) const override;

    /**
     * @brief Gradient and Hessian blocks of running cost
     */
    void calcRunningCostDeriv(double t, const StateVector& x, const InputVector& u,
                              Eigen::Ref<StateVector> running_cost_deriv_x,
                              Eigen::Ref<InputVector> running_cost_deriv_u,
                              Eigen::Ref<StateStateDimMatrix> running_cost_deriv_xx,
                              Eigen::Ref<InputInputDimMatrix> running_cost_deriv_uu,
                              Eigen::Ref<StateInputDimMatrix> running_cost_deriv_xu) const override;

    /**
     * @brief Gradient of terminal cost w.r.t. x
     */
    void calcTerminalCostDeriv(double t, const StateVector& x,
                               Eigen::Ref<StateVector> terminal_cost_deriv_x) const override;

    /**
     * @brief Gradient and Hessian of terminal cost w.r.t. x
     */
    void calcTerminalCostDeriv(double t, const StateVector& x,
                               Eigen::Ref<StateVector> terminal_cost_deriv_x,
                               Eigen::Ref<StateStateDimMatrix> terminal_cost_deriv_xx) const override;

    /**
     * @brief Input box constraints g(x,u) <= 0 for FMPC
     */
    IneqDimVector ineqConst(double t, const StateVector& x,
                            const InputVector& u) const override;

    /**
     * @brief Jacobian of inequality constraints
     */
    void calcIneqConstDeriv(double t, const StateVector& x, const InputVector& u,
                            Eigen::Ref<IneqStateDimMatrix> ineq_const_deriv_x,
                            Eigen::Ref<IneqInputDimMatrix> ineq_const_deriv_u) const override;

    /**
     * @brief Box input limits for DDP BoxQP
     * @return Pair of lower and upper input bounds
     */
    std::array<InputVector, 2> InputLimits() const;

 private:
    static double NormalizeAngle(double yaw);

    /**
     * @brief Reference pose at discrete time index t
     */
    PathReference::Pose2D RefAt(double t) const;

    // Cost weights and velocity limits for the OCP
    proto::NMPCControllerOptions options_;
    // Running reference poses indexed by discrete horizon step
    std::vector<PathReference::Pose2D> refs_;
    // Terminal reference pose at the horizon end
    PathReference::Pose2D terminal_;
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
