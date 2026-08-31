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
 * @file differential_drive_cgmres_problem.hpp
 * @brief Continuous-time unicycle OCP for the C/GMRES backend
 */

#pragma once

#include <vector>

#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/cgmres/cgmres_problem.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::DifferentialDriveCgmresProblem
 * @brief Continuous-time unicycle NMPC problem for the C/GMRES solver
 *
 * Implements the Hamiltonian-based C/GMRES interface:
 * stateEquation, costateEquation, calcDphiDx, calcDhDu.
 */
class DifferentialDriveCgmresProblem : public cgmres::CgmresProblem {
 public:
    /**
     * @brief Constructor
     * @param options Cost weights and model timestep
     */
    explicit DifferentialDriveCgmresProblem(proto::NMPCControllerOptions options);

    /**
     * @brief Set horizon reference poses and terminal goal
     * @param refs Running reference poses along the horizon
     * @param terminal Terminal pose at horizon end
     */
    void SetReferences(const std::vector<PathReference::Pose2D>& refs,
                       const PathReference::Pose2D& terminal);

    /**
     * @brief Update cost weights at runtime
     * @param options Updated NMPC options
     */
    void SetOptions(const proto::NMPCControllerOptions& options);

    /**
     * @brief Continuous dynamics dx/dt = f(x,u)
     */
    void stateEquation(double t, const Eigen::Ref<const Eigen::VectorXd>& x,
                       const Eigen::Ref<const Eigen::VectorXd>& u,
                       Eigen::Ref<Eigen::VectorXd> dotx) override;

    /**
     * @brief Costate dynamics d(lambda)/dt
     */
    void costateEquation(double t, const Eigen::Ref<const Eigen::VectorXd>& lmd,
                         const Eigen::Ref<const Eigen::VectorXd>& xu,
                         Eigen::Ref<Eigen::VectorXd> dotlmd) override;

    /**
     * @brief Terminal cost gradient d(phi)/d(x)
     */
    void calcDphiDx(double t, const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::Ref<Eigen::VectorXd> dphi_dx) override;

    /**
     * @brief Hamiltonian gradient d(H)/d(u)
     */
    void calcDhDu(double t, const Eigen::Ref<const Eigen::VectorXd>& x,
                  const Eigen::Ref<const Eigen::VectorXd>& u,
                  const Eigen::Ref<const Eigen::VectorXd>& lmd,
                  Eigen::Ref<Eigen::VectorXd> dh_du) override;

 private:
    static double NormalizeAngle(double yaw);

    PathReference::Pose2D RefAt(double t) const;

    // Cost weights and model timestep for the continuous OCP
    proto::NMPCControllerOptions options_;
    // Model discretization timestep [s]
    double dt_ = 0.05;
    // Running reference poses indexed by continuous time index
    std::vector<PathReference::Pose2D> refs_;
    // Terminal reference pose at the horizon end
    PathReference::Pose2D terminal_;
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
