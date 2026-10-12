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
 * @file quadruped.cpp
 * @brief ANYmal C joint-velocity tracking solved with bolza SLQ.
 */

#include "quadruped.hpp"

#include <algorithm>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <glog/logging.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <assets/package_path.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/solver.hpp>

namespace quadruped {
namespace {

/** @brief Open-loop horizon, in seconds. */
constexpr double kTimeHorizon = 1.0;

/** @brief Allowed distance from the terminal joints to the target. */
constexpr double kAcceptanceTolerance = 0.05;

/** @brief SLQ iteration limit. */
constexpr int kMaxIterations = 15;

/** @brief Weight on squared joint velocity. */
constexpr double kInputWeight = 0.05;

/** @brief Terminal cost is this many times the stage state cost. */
constexpr double kTerminalScale = 10.0;

/** @brief Magnitude of the alternating joint target, in radians. */
constexpr double kTargetAmplitude = 0.15;

/** @brief How many state and input entries to log on each sample. */
constexpr int kLoggedColumns = 4;

/**
 * @brief Read a whole file into a string.
 * @param path File path.
 * @return File contents.
 * @throws std::runtime_error If the file cannot be opened.
 */
std::string ReadText(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot read " + path);
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

/**
 * @brief Replace URDF limit sentinels that mean "unset".
 * @param urdf_xml Raw URDF text.
 * @return Text after the replacement.
 */
std::string SanitizeEffortLimits(std::string urdf_xml) {
  const std::pair<const char*, const char*> replacements[] = {
      {"effort=\"-1.0\"", "effort=\"10\""},
      {"velocity=\"-1.0\"", "velocity=\"10\""},
  };
  for (const auto& [from, to] : replacements) {
    const std::string source(from);
    std::size_t found = urdf_xml.find(source);
    while (found != std::string::npos) {
      urdf_xml.replace(found, source.size(), to);
      found = urdf_xml.find(source, found + 1);
    }
  }
  return urdf_xml;
}

/**
 * @brief Parse a URDF, retrying after in-memory limit cleanup.
 * @param urdf_path URDF path.
 * @return Pinocchio interface. The file on disk is not modified.
 */
bolza::PinocchioInterface LoadInterface(const std::string& urdf_path) {
  try {
    return bolza::GetPinocchioInterfaceFromUrdfFile(urdf_path);
  } catch (const std::exception& error) {
    LOG(WARNING) << "urdf parse retry (" << error.what() << ")";
    return bolza::GetPinocchioInterfaceFromUrdfString(SanitizeEffortLimits(ReadText(urdf_path)));
  }
}

/**
 * @brief Log one sampled instant. Extra coordinates are omitted.
 * @param summary Time, state, and input sequences.
 * @param sample_index Index into those sequences.
 */
void LogSample(const bolza::Solver::Summary& summary, std::size_t sample_index) {
  const int state_dim = static_cast<int>(summary.state_trajectory[sample_index].size());
  const int input_dim = static_cast<int>(summary.input_trajectory[sample_index].size());
  const int state_cols = std::min(state_dim, kLoggedColumns);
  const int input_cols = std::min(input_dim, kLoggedColumns);
  std::ostringstream line;
  line << std::fixed << std::setprecision(4);
  line << "t=" << std::setw(6) << summary.time_trajectory[sample_index] << "  q";
  for (int index = 0; index < state_cols; ++index) {
    line << ' ' << std::setw(8) << summary.state_trajectory[sample_index](index);
  }
  if (state_cols < state_dim) {
    line << " ...";
  }
  line << "  qdot";
  for (int index = 0; index < input_cols; ++index) {
    line << ' ' << std::setw(8) << summary.input_trajectory[sample_index](index);
  }
  if (input_cols < input_dim) {
    line << " ...";
  }
  LOG(INFO) << line.str();
}

/**
 * @brief Log status and a short sampling of the trajectory.
 * @param summary Result returned by bolza.
 * @param target Target joint vector.
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
  terminal << "target q0=" << target(0) << "  final q0=" << summary.x_final(0);
  LOG(INFO) << terminal.str();
  return error < kAcceptanceTolerance;
}

}  // namespace

Quadruped::Quadruped() : Quadruped(bolza::robotic_assets::GetPath() + "/anymal_c/urdf/anymal.urdf") {}

Quadruped::Quadruped(std::string urdf_path) : urdf_path_(std::move(urdf_path)) {}

int Quadruped::Run() {
  LOG(INFO) << "quadruped";
  LOG(INFO) << "urdf " << urdf_path_;
  LOG(INFO) << "x=q  u=qdot  target alternates +/- " << kTargetAmplitude << " rad";

  const bolza::PinocchioInterface interface = LoadInterface(urdf_path_);
  const auto& model = interface.GetModel();
  const int velocity_dim = static_cast<int>(model.nv);
  LOG(INFO) << "model " << model.name << " nq=" << model.nq << " nv=" << velocity_dim;
  if (velocity_dim <= 0) {
    LOG(ERROR) << "expected at least one velocity coordinate";
    return 1;
  }

  std::ostringstream joints;
  int shown = 0;
  for (std::size_t index = 1; index < model.names.size() && shown < 8; ++index) {
    if (model.joints[index].nv() > 0) {
      joints << (shown == 0 ? "" : " ") << model.names[index];
      ++shown;
    }
  }
  LOG(INFO) << "joints " << joints.str();

  bolza::Problem problem;
  problem.SetLinearDynamics(bolza::matrix_t::Zero(velocity_dim, velocity_dim),
                            bolza::matrix_t::Identity(velocity_dim, velocity_dim));
  problem.AddQuadraticCost(bolza::matrix_t::Identity(velocity_dim, velocity_dim),
                           bolza::matrix_t::Identity(velocity_dim, velocity_dim) * kInputWeight);
  problem.AddQuadraticFinalCost(kTerminalScale * bolza::matrix_t::Identity(velocity_dim, velocity_dim));

  bolza::vector_t target = bolza::vector_t::Zero(velocity_dim);
  for (int index = 0; index < velocity_dim; ++index) {
    target(index) = (index % 2 == 0) ? kTargetAmplitude : -kTargetAmplitude;
  }
  problem.SetInitialState(bolza::vector_t::Zero(velocity_dim));
  problem.SetTarget(target);

  bolza::Solver::Options options;
  options.tf = kTimeHorizon;
  options.max_num_iterations = static_cast<std::size_t>(kMaxIterations);

  bolza::Solver::Summary summary;
  bolza::Solve(options, &problem, &summary);
  return LogSolution(summary, target) ? 0 : 1;
}

}  // namespace quadruped
