/*
 * Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
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
 * @file quadrotor.cpp
 * @brief Hover linearization of the quadrotor, solved with bolza SLQ.
 *
 * The absolute rigid-body map has a nonzero hover thrust, so it is not an
 * LTI system in absolute coordinates. The OCP is written in the deviation
 * from hover after one central-difference linearization.
 */

#include "quadrotor.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <sstream>

#include <glog/logging.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <assets/package_path.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/solver.hpp>
#include <bolza/tools/common/angular_velocity_mapping.hpp>

namespace quadrotor {
namespace {

/** @brief State size: [p, euler_xyz, v, w]. */
constexpr int kStateDim = 12;

/** @brief Input size: [Fz, Mx, My, Mz]. */
constexpr int kInputDim = 4;

/** @brief Mass, in kilograms. Matches the URDF. */
constexpr double kMass = 0.5;

/** @brief Roll and pitch inertia, in kg m^2. */
constexpr double kInertiaXy = 0.0023;

/** @brief Yaw inertia, in kg m^2. */
constexpr double kInertiaZ = 0.004;

/** @brief Gravity, along world -z of the absolute model. */
constexpr double kGravity = 9.81;

/** @brief Hover altitude used as the linearization point, in meters. */
constexpr double kHoverAltitude = 1.0;

/** @brief Target deviation along x, in meters. */
constexpr double kTargetX = 0.3;

/** @brief Target deviation along z, in meters. */
constexpr double kTargetZ = 0.2;

/** @brief Open-loop horizon, in seconds. */
constexpr double kTimeHorizon = 2.0;

/** @brief Allowed distance from the terminal deviation to the target. */
constexpr double kAcceptanceTolerance = 0.05;

/** @brief SLQ iteration limit. */
constexpr int kMaxIterations = 20;

/** @brief Weight on the squared input deviation. */
constexpr double kInputWeight = 0.2;

/** @brief Extra weight on the three position deviations. */
constexpr double kPositionWeight = 5.0;

/** @brief Terminal cost is this many times the stage state cost. */
constexpr double kTerminalScale = 8.0;

/** @brief Central-difference step. */
constexpr double kFiniteDifferenceStep = 1e-6;

/** @brief How many state entries to log on each sample. */
constexpr int kLoggedStateColumns = 6;

using Flow = std::function<bolza::vector_t(const bolza::vector_t&, const bolza::vector_t&)>;

/**
 * @brief Central-difference linearization of a flow map.
 * @param flow Absolute dynamics.
 * @param state Linearization state.
 * @param input Linearization input.
 * @param state_matrix Output A.
 * @param input_matrix Output B.
 */
void Linearize(const Flow& flow, const bolza::vector_t& state, const bolza::vector_t& input, bolza::matrix_t* state_matrix,
               bolza::matrix_t* input_matrix) {
  const int state_dim = static_cast<int>(state.size());
  const int input_dim = static_cast<int>(input.size());
  *state_matrix = bolza::matrix_t::Zero(state_dim, state_dim);
  *input_matrix = bolza::matrix_t::Zero(state_dim, input_dim);
  for (int index = 0; index < state_dim; ++index) {
    bolza::vector_t plus = state;
    bolza::vector_t minus = state;
    plus(index) += kFiniteDifferenceStep;
    minus(index) -= kFiniteDifferenceStep;
    state_matrix->col(index) = (flow(plus, input) - flow(minus, input)) / (2.0 * kFiniteDifferenceStep);
  }
  for (int index = 0; index < input_dim; ++index) {
    bolza::vector_t plus = input;
    bolza::vector_t minus = input;
    plus(index) += kFiniteDifferenceStep;
    minus(index) -= kFiniteDifferenceStep;
    input_matrix->col(index) = (flow(state, plus) - flow(state, minus)) / (2.0 * kFiniteDifferenceStep);
  }
}

/**
 * @brief Log one sampled deviation. Extra coordinates are omitted.
 * @param summary Time, state, and input sequences.
 * @param sample_index Index into those sequences.
 */
void LogSample(const bolza::Solver::Summary& summary, std::size_t sample_index) {
  const int state_dim = static_cast<int>(summary.state_trajectory[sample_index].size());
  const int state_cols = std::min(state_dim, kLoggedStateColumns);
  std::ostringstream line;
  line << std::fixed << std::setprecision(4);
  line << "t=" << std::setw(6) << summary.time_trajectory[sample_index] << "  x";
  for (int index = 0; index < state_cols; ++index) {
    line << ' ' << std::setw(8) << summary.state_trajectory[sample_index](index);
  }
  if (state_cols < state_dim) {
    line << " ...";
  }
  line << "  u";
  for (int index = 0; index < kInputDim; ++index) {
    line << ' ' << std::setw(8) << summary.input_trajectory[sample_index](index);
  }
  LOG(INFO) << line.str();
}

/**
 * @brief Log status and a short sampling of the deviation trajectory.
 * @param summary Result returned by bolza.
 * @param target Target deviation.
 * @return True when the solve succeeded and the terminal error is within tolerance.
 */
bool LogSolution(const bolza::Solver::Summary& summary, const bolza::vector_t& target) {
  const double error = summary.ok ? (summary.x_final - target).norm() : -1.0;
  std::ostringstream status;
  status << std::fixed << std::setprecision(4) << std::boolalpha;
  status << "solve ok=" << summary.ok << "  " << summary.message << "  samples=" << summary.time_trajectory.size()
         << "  |x(tf)-x*|=" << error;
  if (!summary.ok || summary.time_trajectory.empty()) {
    LOG(ERROR) << status.str();
    return false;
  }
  LOG(INFO) << status.str();

  const std::size_t sample_count = summary.time_trajectory.size();
  const std::size_t stride = std::max<std::size_t>(1, sample_count / 4);
  for (std::size_t sample_index = 0; sample_index < sample_count; sample_index += stride) {
    LogSample(summary, sample_index);
  }
  if (sample_count > 1 && (sample_count - 1) % stride != 0) {
    LogSample(summary, sample_count - 1);
  }
  std::ostringstream terminal;
  terminal << std::fixed << std::setprecision(4);
  terminal << "target x=" << target(0) << " z=" << target(2) << "  final x=" << summary.x_final(0)
           << " z=" << summary.x_final(2);
  LOG(INFO) << terminal.str();
  return error < kAcceptanceTolerance;
}

}  // namespace

Quadrotor::Quadrotor() : Quadrotor(bolza::robotic_assets::GetPath() + "/quadrotor/urdf/quadrotor.urdf") {}

Quadrotor::Quadrotor(std::string urdf_path) : urdf_path_(std::move(urdf_path)) {}

bolza::vector_t Quadrotor::ComputeFlowMap(const bolza::vector_t& state, const bolza::vector_t& input) const {
  const Eigen::Matrix<bolza::scalar_t, 3, 1> euler = state.segment<3>(3);
  const Eigen::Matrix<bolza::scalar_t, 3, 3> mapping =
      bolza::GetMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative<bolza::scalar_t>(euler);
  const Eigen::Matrix<bolza::scalar_t, 3, 1> euler_dot = mapping * state.segment<3>(9);

  const double roll = state(3);
  const double pitch = state(4);
  const double yaw = state(5);
  const double roll_rate = state(9);
  const double pitch_rate = state(10);
  const double yaw_rate = state(11);
  const double thrust = input(0);
  const double moment_x = input(1);
  const double moment_y = input(2);
  const double moment_z = input(3);

  const double inverse_mass = 1.0 / kMass;
  const double cos_pitch = std::cos(pitch);
  const double sin_pitch = std::sin(pitch);
  const double inverse_inertia_xy = 1.0 / kInertiaXy;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double roll_rate_sq = roll_rate * roll_rate;
  const double sin_two_pitch = std::sin(2.0 * pitch);
  const double inverse_cos_pitch = 1.0 / cos_pitch;
  const double inertia_z_sq = kInertiaZ * kInertiaZ;
  const double cos_pitch_sq = cos_pitch * cos_pitch;

  bolza::vector_t derivative(kStateDim);
  derivative(0) = state(6);
  derivative(1) = state(7);
  derivative(2) = state(8);
  derivative(3) = euler_dot(0);
  derivative(4) = euler_dot(1);
  derivative(5) = euler_dot(2);
  derivative(6) = thrust * inverse_mass * sin_pitch;
  derivative(7) = -thrust * inverse_mass * cos_pitch * std::sin(roll);
  derivative(8) = inverse_mass * (kMass * kGravity - thrust * cos_pitch * std::cos(roll)) * (-1.0);
  derivative(9) = -inverse_inertia_xy * inverse_cos_pitch *
                  (-moment_x * cos_yaw + moment_y * sin_yaw + kInertiaZ * yaw_rate * pitch_rate -
                   kInertiaXy * roll_rate * pitch_rate * sin_pitch * 2.0 + kInertiaZ * roll_rate * pitch_rate * sin_pitch);
  derivative(10) = inverse_inertia_xy * (moment_x * sin_yaw + moment_y * cos_yaw - kInertiaXy * roll_rate_sq * sin_two_pitch * 0.5 +
                                         kInertiaZ * roll_rate_sq * sin_two_pitch * 0.5 + kInertiaZ * roll_rate * yaw_rate * cos_pitch);
  derivative(11) = (inverse_inertia_xy * inverse_cos_pitch *
                    (moment_z * kInertiaXy * cos_pitch + roll_rate * pitch_rate * inertia_z_sq -
                     roll_rate * pitch_rate * inertia_z_sq * cos_pitch_sq + yaw_rate * pitch_rate * sin_pitch * inertia_z_sq -
                     kInertiaXy * kInertiaZ * roll_rate * pitch_rate * 2.0 - moment_x * kInertiaZ * sin_pitch * cos_yaw +
                     moment_y * kInertiaZ * sin_pitch * sin_yaw + kInertiaXy * kInertiaZ * roll_rate * pitch_rate * cos_pitch_sq)) /
                   kInertiaZ;
  return derivative;
}

int Quadrotor::Run() {
  LOG(INFO) << "quadrotor";
  LOG(INFO) << "urdf " << urdf_path_;
  LOG(INFO) << "deviation from hover  x=[p, euler_xyz, v, w]  u=[Fz, Mx, My, Mz]-hover  mass=" << kMass;
  try {
    const bolza::PinocchioInterface interface = bolza::GetPinocchioInterfaceFromUrdfFile(urdf_path_);
    const auto& model = interface.GetModel();
    LOG(INFO) << "model " << model.name << " nq=" << model.nq << " nv=" << model.nv;
  } catch (const std::exception& error) {
    LOG(WARNING) << "urdf not used for the dynamics (" << error.what() << ")";
  }

  bolza::vector_t hover_state = bolza::vector_t::Zero(kStateDim);
  hover_state(2) = kHoverAltitude;
  bolza::vector_t hover_input(kInputDim);
  hover_input << kMass * kGravity, 0.0, 0.0, 0.0;
  const double hover_residual = ComputeFlowMap(hover_state, hover_input).norm();
  LOG(INFO) << "|f(hover)|=" << hover_residual;

  const Flow flow = [this](const bolza::vector_t& state, const bolza::vector_t& input) {
    return ComputeFlowMap(state, input);
  };
  bolza::matrix_t state_matrix;
  bolza::matrix_t input_matrix;
  Linearize(flow, hover_state, hover_input, &state_matrix, &input_matrix);

  bolza::Problem problem;
  problem.SetLinearDynamics(state_matrix, input_matrix);
  bolza::matrix_t state_cost = bolza::matrix_t::Identity(kStateDim, kStateDim);
  state_cost.diagonal().head<3>() *= kPositionWeight;
  problem.AddQuadraticCost(state_cost, bolza::matrix_t::Identity(kInputDim, kInputDim) * kInputWeight);
  problem.AddQuadraticFinalCost(kTerminalScale * state_cost);

  bolza::vector_t target = bolza::vector_t::Zero(kStateDim);
  target(0) = kTargetX;
  target(2) = kTargetZ;
  problem.SetInitialState(bolza::vector_t::Zero(kStateDim));
  problem.SetTarget(target);

  bolza::Solver::Options options;
  options.tf = kTimeHorizon;
  options.max_num_iterations = static_cast<std::size_t>(kMaxIterations);

  bolza::Solver::Summary summary;
  bolza::Solve(options, &problem, &summary);
  const bool reached = LogSolution(summary, target);
  if (hover_residual >= 1e-6) {
    LOG(ERROR) << "hover is not an equilibrium";
    return 1;
  }
  return reached ? 0 : 1;
}

}  // namespace quadrotor
