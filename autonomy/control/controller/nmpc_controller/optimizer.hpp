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
 * @file optimizer.hpp
 * @brief NMPC optimizer facade over DDP, FMPC, and C/GMRES backends
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/control/controller/nmpc_controller/differential_drive_cgmres_problem.hpp"
#include "autonomy/control/controller/nmpc_controller/differential_drive_problem.hpp"
#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/cgmres/cgmres_solver.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/cgmres/euler_ode_solver.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/ddp/ddp_solver.hpp"
#include "autonomy/control/controller/nmpc_controller/solver/fmpc/fmpc_solver.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::NmpcOptimizer
 * @brief Main algorithm optimizer of the NMPC Controller (DDP / FMPC / C-GMRES)
 */
class NmpcOptimizer {
 public:
    /**
     * @brief Output of a single NMPC solve step
     */
    struct SolveResult {
        // First control in the optimized sequence [v, omega]
        DifferentialDriveProblem::InputVector cmd =
            DifferentialDriveProblem::InputVector::Zero();
        // Predicted state rollout used for collision checking
        std::vector<DifferentialDriveProblem::StateVector> predicted_states;
    };

    /**
     * @brief Constructor for nmpc_controller::NmpcOptimizer
     * @param options NMPC controller options
     */
    explicit NmpcOptimizer(proto::NMPCControllerOptions options);

    /**
     * @brief Solve one NMPC iteration for the current state and path
     * @param state Current robot state [x, y, yaw]
     * @param path Local path reference
     * @param path_progress_s Arc-length progress along the path
     * @param result Output command and predicted state rollout
     * @return True if optimization succeeded
     */
    bool Solve(const DifferentialDriveProblem::StateVector& state,
               const PathReference& path, double path_progress_s,
               SolveResult* result);

    /**
     * @brief Update runtime options (velocity limits, weights, solver type)
     * @param options Updated NMPC options
     */
    void UpdateOptions(const proto::NMPCControllerOptions& options);

    /**
     * @brief Active solver backend name ("ddp", "fmpc", or "cgmres")
     */
    const std::string& solver_type() const { return solver_type_; }

    /**
     * @brief Reset warm-start controls between navigation tasks
     */
    void Reset();

 private:
    // Internal solver backend selection
    enum class Backend { kDdp, kFmpc, kCgmres };

    /**
     * @brief (Re)create problem and solver objects from options_
     */
    void InitializeBackend();

    /**
     * @brief Fill running/terminal references on the active problem(s)
     */
    void BuildReferences(const PathReference& path, double path_progress_s);

    /**
     * @brief Run DDP with input box constraints (BoxQP)
     */
    bool SolveDdp(const DifferentialDriveProblem::StateVector& state,
                  SolveResult* result);

    /**
     * @brief Run FMPC with inequality constraints on inputs
     */
    bool SolveFmpc(const DifferentialDriveProblem::StateVector& state,
                   SolveResult* result);

    /**
     * @brief Run one C/GMRES control update step
     */
    bool SolveCgmres(const DifferentialDriveProblem::StateVector& state,
                     SolveResult* result);

    /**
     * @brief Forward-simulate FMPC initial primal-dual guess from warm start
     */
    fmpc::FmpcSolver<3, 2, 4>::Variable BuildFmpcInitialGuess(
        const DifferentialDriveProblem::StateVector& state) const;

    /**
     * @brief Map solver_type string to Backend enum
     */
    static Backend ParseSolverType(const std::string& solver_type);

    // Active NMPC cost weights, limits, and solver settings
    proto::NMPCControllerOptions options_;
    // Parsed solver backend selected from options_.solver_type()
    Backend backend_ = Backend::kDdp;
    // Solver type string copy ("ddp", "fmpc", or "cgmres")
    std::string solver_type_;
    // Shared discrete unicycle OCP used by DDP and FMPC
    std::shared_ptr<DifferentialDriveProblem> problem_;
    // DDP solver with box input constraints
    std::unique_ptr<ddp::DdpSolver<3, 2>> ddp_solver_;
    // FMPC interior-point solver with input inequalities
    std::unique_ptr<fmpc::FmpcSolver<3, 2, 4>> fmpc_solver_;
    // Continuous-time OCP definition for C/GMRES
    std::shared_ptr<DifferentialDriveCgmresProblem> cgmres_problem_;
    // Forward Euler integrator for C/GMRES rollouts
    std::shared_ptr<cgmres::EulerOdeSolver> cgmres_ode_solver_;
    // C/GMRES receding-horizon solver instance
    std::unique_ptr<cgmres::CgmresSolver> cgmres_solver_;
    // Previous optimal control sequence for warm starting
    std::vector<DifferentialDriveProblem::InputVector> warm_start_;
    // True after C/GMRES setup() has been called once
    bool cgmres_ready_ = false;
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
