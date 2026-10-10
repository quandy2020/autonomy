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

/**
 * @file controller.cpp
 * @brief Gauss-Newton kinematic MPC (implementation).
 */

#include "arm/osc2/controller.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace automanip {
namespace arm {
namespace osc2 {
namespace {

double SafeSqrt(double weight) {
  return weight > 0.0 ? std::sqrt(weight) : 0.0;
}

}  // namespace

Eigen::Matrix<double, 6, 1> PoseError(const Eigen::Isometry3d& current,
                                       const Eigen::Isometry3d& desired) {
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = desired.translation() - current.translation();
  const Eigen::Matrix3d relative =
      desired.linear() * current.linear().transpose();
  const Eigen::AngleAxisd angle_axis(relative);
  const double angle = angle_axis.angle();
  if (std::abs(angle) < 1e-12 || !std::isfinite(angle)) {
    error.tail<3>().setZero();
  } else {
    error.tail<3>() = angle * angle_axis.axis();
  }
  return error;
}

Osc2Controller::Osc2Controller(SerialChain chain, Osc2Settings settings)
    : chain_(std::move(chain)), settings_(settings) {
  if (settings_.horizon < 1) {
    settings_.horizon = 1;
  }
  if (settings_.mpc_dt <= 0.0) {
    settings_.mpc_dt = 0.05;
  }
  if (settings_.iterations < 1) {
    settings_.iterations = 1;
  }
  joint_target_ = chain_.Home();
  pose_target_ = chain_.Forward(joint_target_);
}

void Osc2Controller::Reset(const Eigen::VectorXd& /*q*/) {
  // Drop the warm-started input only. Pose and joint targets stay so a mode
  // change (hold → track) does not erase a command that was just applied.
  u_prev_.resize(0);
}

void Osc2Controller::SetObjective(Osc2Objective objective) {
  if (objective_ != objective) {
    u_prev_.resize(0);
  }
  objective_ = objective;
}

void Osc2Controller::SetPoseTarget(const Eigen::Isometry3d& pose) {
  pose_target_ = pose;
}

void Osc2Controller::SetJointTarget(const Eigen::VectorXd& q) {
  if (q.size() == chain_.dof()) {
    joint_target_ = q;
  }
}

Eigen::VectorXd Osc2Controller::Compute(const Eigen::VectorXd& q) {
  const int n = chain_.dof();
  if (n == 0 || q.size() != n) {
    return Eigen::VectorXd();
  }
  const int horizon = settings_.horizon;
  const int input_size = n * horizon;
  const double dt = settings_.mpc_dt;

  Eigen::VectorXd u = Eigen::VectorXd::Zero(input_size);
  if (u_prev_.size() == input_size) {
    u.head(n * (horizon - 1)) = u_prev_.tail(n * (horizon - 1));
    u.tail(n) = u_prev_.tail(n);
  }
  const Eigen::VectorXd velocity_limit = chain_.VelocityLimit();
  const auto clamp_input = [&](Eigen::VectorXd* input) {
    for (int index = 0; index < input->size(); ++index) {
      const double limit = velocity_limit[index % n];
      (*input)[index] = std::clamp((*input)[index], -limit, limit);
    }
  };
  clamp_input(&u);

  const Eigen::VectorXd lower = chain_.Lower();
  const Eigen::VectorXd upper = chain_.Upper();
  const Eigen::VectorXd home = chain_.Home();
  const double position_scale = SafeSqrt(settings_.position_weight);
  const double orientation_scale = SafeSqrt(settings_.orientation_weight);
  const double input_scale = SafeSqrt(settings_.input_weight);
  const double joint_scale = SafeSqrt(settings_.joint_weight);
  const double posture_scale = SafeSqrt(settings_.posture_weight);
  const double limit_scale = SafeSqrt(settings_.limit_weight);
  const double terminal = std::sqrt(std::max(settings_.terminal_scale, 1.0));

  for (int iteration = 0; iteration < settings_.iterations; ++iteration) {
    std::vector<Eigen::VectorXd> knots(static_cast<std::size_t>(horizon + 1));
    knots[0] = q;
    for (int k = 0; k < horizon; ++k) {
      knots[static_cast<std::size_t>(k + 1)] =
          knots[static_cast<std::size_t>(k)] + dt * u.segment(k * n, n);
    }

    const int max_rows = horizon * (6 + 3 * n);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(max_rows, input_size);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(max_rows);
    int row = 0;

    for (int k = 1; k <= horizon; ++k) {
      const double stage = (k == horizon) ? terminal : 1.0;
      const Eigen::VectorXd& qk = knots[static_cast<std::size_t>(k)];

      if (objective_ == Osc2Objective::kPose) {
        const Eigen::Matrix<double, 6, 1> error =
            PoseError(chain_.Forward(qk), pose_target_);
        Eigen::Matrix<double, 6, 1> weight;
        weight.head<3>().setConstant(stage * position_scale);
        weight.tail<3>().setConstant(stage * orientation_scale);
        residual.segment<6>(row) = weight.cwiseProduct(error);
        const Eigen::MatrixXd state_jacobian =
            weight.asDiagonal() * (-chain_.Jacobian(qk));
        for (int j = 0; j < k; ++j) {
          jacobian.block(row, j * n, 6, n) = state_jacobian * dt;
        }
        row += 6;

        if (posture_scale > 0.0) {
          const double scale = stage * posture_scale;
          residual.segment(row, n) = scale * (qk - home);
          for (int j = 0; j < k; ++j) {
            jacobian.block(row, j * n, n, n).diagonal().setConstant(scale * dt);
          }
          row += n;
        }
      } else {
        const double scale = stage * joint_scale;
        residual.segment(row, n) = scale * (qk - joint_target_);
        for (int j = 0; j < k; ++j) {
          jacobian.block(row, j * n, n, n).diagonal().setConstant(scale * dt);
        }
        row += n;
      }

      if (input_scale > 0.0) {
        residual.segment(row, n) = input_scale * u.segment((k - 1) * n, n);
        jacobian.block(row, (k - 1) * n, n, n).diagonal().setConstant(input_scale);
        row += n;
      }

      if (limit_scale > 0.0) {
        for (int i = 0; i < n; ++i) {
          const double high = upper[i] - settings_.limit_margin;
          const double low = lower[i] + settings_.limit_margin;
          double violation = 0.0;
          if (qk[i] > high) {
            violation = qk[i] - high;
          } else if (qk[i] < low) {
            violation = qk[i] - low;
          } else {
            continue;
          }
          residual[row] = stage * limit_scale * violation;
          for (int j = 0; j < k; ++j) {
            jacobian(row, j * n + i) = stage * limit_scale * dt;
          }
          ++row;
        }
      }
    }

    if (row == 0) {
      break;
    }
    const Eigen::MatrixXd j_used = jacobian.topRows(row);
    const Eigen::VectorXd r_used = residual.head(row);
    Eigen::MatrixXd normal = j_used.transpose() * j_used;
    normal.diagonal().array() += settings_.damping;
    const Eigen::VectorXd gradient = j_used.transpose() * r_used;
    Eigen::LDLT<Eigen::MatrixXd> solver(normal);
    if (solver.info() != Eigen::Success) {
      normal.diagonal().array() += 1e-2;
      solver.compute(normal);
    }
    if (solver.info() != Eigen::Success) {
      break;
    }
    Eigen::VectorXd step = solver.solve(-gradient);
    const double max_abs = step.cwiseAbs().maxCoeff();
    if (max_abs > 2.0) {
      step *= 2.0 / max_abs;
    }
    u += step;
    clamp_input(&u);
  }

  u_prev_ = u;
  return u.head(n);
}

}  // namespace osc2
}  // namespace arm
}  // namespace automanip
