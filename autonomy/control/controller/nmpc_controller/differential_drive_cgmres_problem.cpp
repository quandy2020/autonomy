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
 * @file differential_drive_cgmres_problem.cpp
 * @brief Implementation of nmpc_controller::DifferentialDriveCgmresProblem
 */

#include "autonomy/control/controller/nmpc_controller/differential_drive_cgmres_problem.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

DifferentialDriveCgmresProblem::DifferentialDriveCgmresProblem(
    proto::NMPCControllerOptions options)
    : options_(std::move(options)) {
    dim_x_ = 3;
    dim_u_ = 2;
    dim_c_ = 0;
    dim_uc_ = dim_u_ + dim_c_;
    dt_ = options_.model_dt() > 0.0 ? options_.model_dt() : 0.05;
    x_initial_.resize(dim_x_);
    x_initial_.setZero();
    u_initial_.resize(dim_uc_);
    u_initial_.setZero();
}

void DifferentialDriveCgmresProblem::SetReferences(
    const std::vector<PathReference::Pose2D>& refs,
    const PathReference::Pose2D& terminal) {
    refs_ = refs;
    terminal_ = terminal;
}

void DifferentialDriveCgmresProblem::SetOptions(
    const proto::NMPCControllerOptions& options) {
    options_ = options;
    dt_ = options_.model_dt() > 0.0 ? options_.model_dt() : 0.05;
}

double DifferentialDriveCgmresProblem::NormalizeAngle(double yaw) {
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}

PathReference::Pose2D DifferentialDriveCgmresProblem::RefAt(double t) const {
    if (refs_.empty()) {
        return {};
    }
    const int idx = std::min(static_cast<int>(t / dt_),
                             static_cast<int>(refs_.size()) - 1);
    return refs_[static_cast<size_t>(std::max(idx, 0))];
}

void DifferentialDriveCgmresProblem::stateEquation(
    double /*t*/, const Eigen::Ref<const Eigen::VectorXd>& x,
    const Eigen::Ref<const Eigen::VectorXd>& u, Eigen::Ref<Eigen::VectorXd> dotx) {
    const double yaw = x(2);
    dotx(0) = u(0) * std::cos(yaw);
    dotx(1) = u(0) * std::sin(yaw);
    dotx(2) = u(1);
}

void DifferentialDriveCgmresProblem::costateEquation(
    double t, const Eigen::Ref<const Eigen::VectorXd>& lmd,
    const Eigen::Ref<const Eigen::VectorXd>& xu, Eigen::Ref<Eigen::VectorXd> dotlmd) {
    const Eigen::Ref<const Eigen::VectorXd> x = xu.head(dim_x_);
    const Eigen::Ref<const Eigen::VectorXd> u = xu.tail(dim_uc_);
    const auto ref = RefAt(t);
    const double yaw = x(2);
    const double dyaw = NormalizeAngle(yaw - ref.yaw);

    dotlmd(0) = -2.0 * options_.weight_pos() * (x(0) - ref.x);
    dotlmd(1) = -2.0 * options_.weight_pos() * (x(1) - ref.y);
    dotlmd(2) = -2.0 * options_.weight_yaw() * dyaw -
                lmd(0) * (-u(0) * std::sin(yaw)) -
                lmd(1) * (u(0) * std::cos(yaw));
}

void DifferentialDriveCgmresProblem::calcDphiDx(
    double /*t*/, const Eigen::Ref<const Eigen::VectorXd>& x,
    Eigen::Ref<Eigen::VectorXd> dphi_dx) {
    dphi_dx(0) = 2.0 * options_.weight_terminal_pos() * (x(0) - terminal_.x);
    dphi_dx(1) = 2.0 * options_.weight_terminal_pos() * (x(1) - terminal_.y);
    dphi_dx(2) = 2.0 * options_.weight_terminal_yaw() *
                 NormalizeAngle(x(2) - terminal_.yaw);
}

void DifferentialDriveCgmresProblem::calcDhDu(
    double t, const Eigen::Ref<const Eigen::VectorXd>& x,
    const Eigen::Ref<const Eigen::VectorXd>& u,
    const Eigen::Ref<const Eigen::VectorXd>& lmd, Eigen::Ref<Eigen::VectorXd> dh_du) {
    const double yaw = x(2);
    dh_du(0) = 2.0 * options_.weight_linear() * u(0) +
               lmd(0) * std::cos(yaw) + lmd(1) * std::sin(yaw);
    dh_du(1) = 2.0 * options_.weight_angular() * u(1) + lmd(2);
    (void)t;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
