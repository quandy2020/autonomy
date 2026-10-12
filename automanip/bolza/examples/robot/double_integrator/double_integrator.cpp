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
 * @file double_integrator.cpp
 * @brief Linear double integrator solved with bolza SLQ.
 */

#include "double_integrator.hpp"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <sstream>

#include <glog/logging.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <assets/package_path.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/solver.hpp>

namespace double_integrator {
namespace {

/** @brief State size: [p, pdot]. */
constexpr int kStateDim = 2;

/** @brief Input size: acceleration. */
constexpr int kInputDim = 1;

/** @brief Target position, in meters. */
constexpr double kTargetPosition = 1.0;

/** @brief Open-loop horizon, in seconds. */
constexpr double kTimeHorizon = 3.0;

/** @brief Allowed distance from the terminal state to the target. */
constexpr double kAcceptanceTolerance = 0.05;

/** @brief SLQ iteration limit. */
constexpr int kMaxIterations = 20;

/** @brief Weight on the squared acceleration. */
constexpr double kInputWeight = 0.1;

/** @brief Terminal cost is this many times the stage state cost. */
constexpr double kTerminalScale = 10.0;

/**
 * @brief Log one sampled instant.
 * @param summary Time, state, and input sequences.
 * @param sample_index Index into those sequences.
 */
void LogSample(const bolza::Solver::Summary& summary, std::size_t sample_index) {
  std::ostringstream line;
  line << std::fixed << std::setprecision(4);
  line << "t=" << std::setw(6) << summary.time_trajectory[sample_index];
  line << "  p=" << std::setw(8) << summary.state_trajectory[sample_index](0);
  line << "  pdot=" << std::setw(8) << summary.state_trajectory[sample_index](1);
  line << "  u=" << std::setw(8) << summary.input_trajectory[sample_index](0);
  LOG(INFO) << line.str();
}

/**
 * @brief Log status and a short sampling of the trajectory.
 * @param summary Result returned by bolza.
 * @param target Target state.
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
  terminal << "target p=" << target(0) << "  final p=" << summary.x_final(0) << " pdot=" << summary.x_final(1);
  LOG(INFO) << terminal.str();
  return error < kAcceptanceTolerance;
}

}  // namespace

DoubleIntegrator::DoubleIntegrator()
    : DoubleIntegrator(bolza::robotic_assets::GetPath() + "/double_integrator/urdf/double_integrator.urdf") {}

DoubleIntegrator::DoubleIntegrator(std::string urdf_path) : urdf_path_(std::move(urdf_path)) {}

int DoubleIntegrator::Run() {
  LOG(INFO) << "double_integrator";
  LOG(INFO) << "urdf " << urdf_path_;
  LOG(INFO) << "x=[p, pdot]  u=pddot  target p=" << kTargetPosition;
  try {
    const bolza::PinocchioInterface interface = bolza::GetPinocchioInterfaceFromUrdfFile(urdf_path_);
    const auto& model = interface.GetModel();
    LOG(INFO) << "model " << model.name << " nq=" << model.nq << " nv=" << model.nv;
  } catch (const std::exception& error) {
    LOG(WARNING) << "urdf not used for the dynamics (" << error.what() << ")";
  }

  bolza::Problem problem;
  const bolza::matrix_t state_matrix = (bolza::matrix_t(kStateDim, kStateDim) << 0.0, 1.0, 0.0, 0.0).finished();
  const bolza::matrix_t input_matrix = (bolza::matrix_t(kStateDim, kInputDim) << 0.0, 1.0).finished();
  problem.SetLinearDynamics(state_matrix, input_matrix);
  problem.AddQuadraticCost(bolza::matrix_t::Identity(kStateDim, kStateDim),
                           bolza::matrix_t::Identity(kInputDim, kInputDim) * kInputWeight);
  problem.AddQuadraticFinalCost(kTerminalScale * bolza::matrix_t::Identity(kStateDim, kStateDim));

  const bolza::vector_t target = (bolza::vector_t(kStateDim) << kTargetPosition, 0.0).finished();
  problem.SetInitialState(bolza::vector_t::Zero(kStateDim));
  problem.SetTarget(target);

  bolza::Solver::Options options;
  options.tf = kTimeHorizon;
  options.max_num_iterations = static_cast<std::size_t>(kMaxIterations);

  bolza::Solver::Summary summary;
  bolza::Solve(options, &problem, &summary);
  return LogSolution(summary, target) ? 0 : 1;
}

}  // namespace double_integrator
