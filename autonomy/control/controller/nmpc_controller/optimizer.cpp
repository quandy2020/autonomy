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
 * @file optimizer.cpp
 * @brief Implementation of nmpc_controller::NmpcOptimizer
 */

#include "autonomy/control/controller/nmpc_controller/optimizer.hpp"

#include <algorithm>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

namespace {

/**
 * @brief Use fallback when a positive scalar option is unset.
 */
double PositiveOr(double value, double fallback) {
    return value > 0.0 ? value : fallback;
}
}  // namespace

NmpcOptimizer::Backend NmpcOptimizer::ParseSolverType(
    const std::string& solver_type) {
    if (solver_type == "fmpc") {
        return Backend::kFmpc;
    }
    if (solver_type == "cgmres") {
        return Backend::kCgmres;
    }
    return Backend::kDdp;
}

NmpcOptimizer::NmpcOptimizer(proto::NMPCControllerOptions options)
    : options_(std::move(options)),
      backend_(ParseSolverType(options_.solver_type())),
      solver_type_(options_.solver_type().empty() ? "ddp" : options_.solver_type()) {
    InitializeBackend();
}

void NmpcOptimizer::InitializeBackend() {
    const double dt = PositiveOr(options_.model_dt(), 0.05);
    const int horizon_steps = std::max(options_.horizon_steps(), 10);
    const int max_iter = std::max(options_.max_solver_iter(), 20);

    problem_ = std::make_shared<DifferentialDriveProblem>(dt, options_);
    warm_start_.assign(static_cast<size_t>(horizon_steps),
                       DifferentialDriveProblem::InputVector::Zero());

    ddp_solver_.reset();
    fmpc_solver_.reset();
    cgmres_solver_.reset();
    cgmres_problem_.reset();
    cgmres_ode_solver_.reset();
    cgmres_ready_ = false;

    switch (backend_) {
        case Backend::kDdp: {
            ddp_solver_ = std::make_unique<ddp::DdpSolver<3, 2>>(problem_);
            ddp_solver_->config().horizon_steps = horizon_steps;
            ddp_solver_->config().max_iter = max_iter;
            ddp_solver_->config().with_input_constraint = true;
            ddp_solver_->config().print_level = 0;
            ddp_solver_->setInputLimitsFunc(
                [this](double /*t*/) { return problem_->InputLimits(); });
            break;
        }
        case Backend::kFmpc: {
            fmpc_solver_ =
                std::make_unique<fmpc::FmpcSolver<3, 2, 4>>(problem_);
            fmpc_solver_->config().horizon_steps = horizon_steps;
            fmpc_solver_->config().max_iter = max_iter;
            fmpc_solver_->config().print_level = 0;
            break;
        }
        case Backend::kCgmres: {
            cgmres_problem_ =
                std::make_shared<DifferentialDriveCgmresProblem>(options_);
            cgmres_ode_solver_ = std::make_shared<cgmres::EulerOdeSolver>();
            cgmres_solver_ = std::make_unique<cgmres::CgmresSolver>(
                cgmres_problem_, cgmres_ode_solver_, cgmres_ode_solver_);
            cgmres_solver_->dt_ = dt;
            cgmres_solver_->horizon_divide_num_ =
                std::max(options_.cgmres_horizon_divisions(), 10);
            cgmres_solver_->steady_horizon_duration_ =
                dt * static_cast<double>(horizon_steps);
            cgmres_solver_->k_max_ = std::max(options_.cgmres_k_max(), 3);
            break;
        }
    }
}

void NmpcOptimizer::Reset() {
    for (auto& u : warm_start_) {
        u.setZero();
    }
    cgmres_ready_ = false;
}

void NmpcOptimizer::UpdateOptions(const proto::NMPCControllerOptions& options) {
    const Backend new_backend = ParseSolverType(options.solver_type());
    options_ = options;
    if (new_backend != backend_ ||
        options.solver_type() != solver_type_) {
        backend_ = new_backend;
        solver_type_ =
            options.solver_type().empty() ? "ddp" : options.solver_type();
        InitializeBackend();
        return;
    }
    problem_->SetOptions(options_);
    if (cgmres_problem_) {
        cgmres_problem_->SetOptions(options_);
    }
    if (ddp_solver_) {
        ddp_solver_->setInputLimitsFunc(
            [this](double /*t*/) { return problem_->InputLimits(); });
    }
}

void NmpcOptimizer::BuildReferences(const PathReference& path,
                                    double path_progress_s) {
    const int steps = std::max(options_.horizon_steps(), 10);
    std::vector<PathReference::Pose2D> refs;
    refs.reserve(static_cast<size_t>(steps + 1));
    for (int i = 0; i <= steps; ++i) {
        refs.push_back(path.SampleAlongPath(path_progress_s, problem_->dt(), i));
    }
    problem_->SetReferences(refs, path.Goal());
    if (cgmres_problem_) {
        cgmres_problem_->SetReferences(refs, path.Goal());
    }
}

fmpc::FmpcSolver<3, 2, 4>::Variable NmpcOptimizer::BuildFmpcInitialGuess(
    const DifferentialDriveProblem::StateVector& state) const {
    const int horizon_steps = std::max(options_.horizon_steps(), 10);
    fmpc::FmpcSolver<3, 2, 4>::Variable variable(horizon_steps);
    variable.x_list.front() = state;
    for (int i = 0; i < horizon_steps; ++i) {
        const auto& u = warm_start_[static_cast<size_t>(i)];
        variable.u_list[static_cast<size_t>(i)] = u;
        variable.s_list[static_cast<size_t>(i)].setOnes();
        variable.nu_list[static_cast<size_t>(i)].setOnes();
        variable.x_list[static_cast<size_t>(i + 1)] = problem_->stateEq(
            static_cast<double>(i) * problem_->dt(), variable.x_list[i], u);
    }
    for (auto& lambda : variable.lambda_list) {
        lambda.setZero();
    }
    return variable;
}

bool NmpcOptimizer::SolveDdp(
    const DifferentialDriveProblem::StateVector& state, SolveResult* result) {
    if (!ddp_solver_->solve(0.0, state, warm_start_)) {
        AWARN << "NMPC DDP solve failed";
        return false;
    }
    result->cmd = ddp_solver_->controlData().u_list.front();
    result->predicted_states = ddp_solver_->controlData().x_list;
    const auto& solved_u = ddp_solver_->controlData().u_list;
    warm_start_.assign(solved_u.begin(), solved_u.end());
    if (!warm_start_.empty()) {
        warm_start_.push_back(warm_start_.back());
        warm_start_.erase(warm_start_.begin());
    }
    return true;
}

bool NmpcOptimizer::SolveFmpc(
    const DifferentialDriveProblem::StateVector& state, SolveResult* result) {
    auto variable = BuildFmpcInitialGuess(state);
    const auto status = fmpc_solver_->solve(0.0, state, variable);
    if (status != fmpc::FmpcSolver<3, 2, 4>::Status::Succeeded) {
        AWARN << "NMPC FMPC solve failed, status="
              << static_cast<int>(status);
        return false;
    }
    const auto& solved = fmpc_solver_->variable();
    result->cmd = solved.u_list.front();
    result->predicted_states = solved.x_list;
    warm_start_.assign(solved.u_list.begin(), solved.u_list.end());
    if (!warm_start_.empty()) {
        warm_start_.push_back(warm_start_.back());
        warm_start_.erase(warm_start_.begin());
    }
    return true;
}

bool NmpcOptimizer::SolveCgmres(
    const DifferentialDriveProblem::StateVector& state, SolveResult* result) {
    if (!cgmres_ready_) {
        cgmres_solver_->setup();
        cgmres_ready_ = true;
    }

    Eigen::VectorXd x = state;
    Eigen::VectorXd u = warm_start_.empty()
                            ? Eigen::VectorXd::Zero(cgmres_problem_->dim_uc_)
                            : warm_start_.front();
    if (u.size() != cgmres_problem_->dim_uc_) {
        u = Eigen::VectorXd::Zero(cgmres_problem_->dim_uc_);
        u(0) = warm_start_.empty() ? 0.0 : warm_start_.front()(0);
        u(1) = warm_start_.empty() ? 0.0 : warm_start_.front()(1);
    }

    Eigen::VectorXd next_x(cgmres_problem_->dim_x_);
    cgmres_problem_->stateEquation(0.0, x, u, next_x);
    next_x = x + cgmres_solver_->dt_ * next_x;
    cgmres_solver_->calcControlInput(0.0, x, next_x, u);

    result->cmd(0) = u(0);
    result->cmd(1) = u(1);
    if (!warm_start_.empty()) {
        warm_start_.front() = result->cmd;
    }

    result->predicted_states.clear();
    result->predicted_states.push_back(state);
    DifferentialDriveProblem::StateVector rollout = state;
    for (size_t i = 0; i < warm_start_.size(); ++i) {
        const auto input = i == 0 ? result->cmd : warm_start_[i];
        rollout = problem_->stateEq(static_cast<double>(i) * problem_->dt(),
                                  rollout, input);
        result->predicted_states.push_back(rollout);
    }
    return true;
}

bool NmpcOptimizer::Solve(const DifferentialDriveProblem::StateVector& state,
                          const PathReference& path, double path_progress_s,
                          SolveResult* result) {
    if (result == nullptr || path.empty()) {
        return false;
    }

    BuildReferences(path, path_progress_s);
    switch (backend_) {
        case Backend::kDdp:
            return SolveDdp(state, result);
        case Backend::kFmpc:
            return SolveFmpc(state, result);
        case Backend::kCgmres:
            return SolveCgmres(state, result);
    }
    return false;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
