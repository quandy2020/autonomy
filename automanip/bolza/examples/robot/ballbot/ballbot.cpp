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
 * @file ballbot.cpp
 * @brief Ballbot rigid-body model and the bolza optimal-control solve.
 *
 * Wheel torques are mapped to generalized forces by a fixed geometric map.
 * Pinocchio ABA supplies the accelerations. Bolza linearizes that flow map;
 * the demo does not hand-write A and B.
 */

#include "ballbot.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <glog/logging.h>

#include <Eigen/Geometry>

#include <pinocchio/algorithm/aba.hpp>

#include <assets/package_path.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>
#include <bolza/core/dynamics/system_dynamics_linearizer.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/solver.hpp>

namespace ballbot {
namespace {

/** @brief State size: [q(5), v(5)]. */
constexpr int kStateDim = 10;

/** @brief Input size: three wheel torques. */
constexpr int kInputDim = 3;

/** @brief Generalized-coordinate size. Matches the movable joints in the URDF. */
constexpr int kVelocityDim = 5;

/** @brief Position indices. Order matches the URDF joints. */
constexpr int kPx = 0;
constexpr int kPy = 1;
constexpr int kYaw = 2;
constexpr int kPitch = 3;
constexpr int kRoll = 4;

/** @brief Ball and wheel radii, in meters. Matches the URDF geometry. */
constexpr double kBallRadius = 0.125;
constexpr double kWheelRadius = 0.064;

/** @brief Gravity, along world -z. */
constexpr double kGravity = 9.81;

/** @brief Target translation along x, in meters. */
constexpr double kTargetPositionX = 0.2;

/** @brief Open-loop horizon, in seconds. */
constexpr double kTimeHorizon = 2.0;

/** @brief Allowed Euclidean distance from the terminal state to the target. */
constexpr double kAcceptanceTolerance = 0.15;

/** @brief SLQ iteration limit. */
constexpr int kMaxIterations = 15;

/**
 * @brief Central-difference step used to linearize the dynamics.
 *
 * Machine epsilon is too small for this rigid-body model: the difference is
 * lost in roundoff.
 */
constexpr double kFiniteDifferenceStep = 1e-5;

/**
 * @brief Adapts an arbitrary flow map to a bolza controlled system.
 *
 * The linearizer only needs ControlledSystemBase. This type stores no extra
 * state; each evaluation forwards to the function given at construction.
 */
class FlowMapDynamics final : public bolza::ControlledSystemBase {
 public:
  using FlowMap = std::function<bolza::vector_t(const bolza::vector_t&, const bolza::vector_t&)>;

  /**
   * @brief Store the flow map.
   * @param flow_map Maps (state, input) to the state derivative.
   */
  explicit FlowMapDynamics(FlowMap flow_map) : flow_map_(std::move(flow_map)) {}

  /** @brief Copy the flow map. The linearizer calls this while rolling out. */
  FlowMapDynamics* Clone() const override { return new FlowMapDynamics(*this); }

  /**
   * @brief Evaluate the state derivative.
   * @param state Current state.
   * @param input Current input.
   * @return flow_map_(state, input). Time and the precomputation object are unused.
   */
  bolza::vector_t ComputeFlowMap(bolza::scalar_t, const bolza::vector_t& state, const bolza::vector_t& input,
                                  const bolza::PreComputation&) override {
    return flow_map_(state, input);
  }

 private:
  FlowMap flow_map_;
};

/**
 * @brief Problem and target state for one solve.
 *
 * bolza::Problem is move-only, so this aggregate is moved rather than copied.
 */
struct Task {
  bolza::Problem problem;
  bolza::vector_t target;
};

/**
 * @brief Read a whole file into a string.
 * @param path File path.
 * @return File contents, with the original newlines kept.
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
 *
 * Pinocchio requires the lower effort and velocity limits to be no greater
 * than the upper ones. The ballbot revolute joints write both as -1, and the
 * parser then reports "min_effort are greater than max_effort". The sentinels
 * are replaced with 10 in memory. The URDF on disk is not modified, and this
 * demo's quadratic cost does not use the limits.
 *
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
 * @brief Linear map from three wheel torques to five generalized forces.
 *
 * Force order is [px, py, yaw, pitch, roll], matching the URDF joints.
 * The body attitude is R = Rz(yaw) Ry(pitch) Rx(roll). Wheel geometry sits
 * in one constant 3x3 layout; planar and yaw rows are that layout rotated by
 * R, and pitch and roll rows are the same layout rotated by Rx(roll) only.
 * The result is tau = S(q)^T u. Ball radius 0.125 m, wheel radius 0.064 m.
 *
 * @param configuration Generalized positions. Yaw, pitch, and roll are read.
 * @param wheel_torque Three wheel torques.
 * @return Five generalized forces.
 */
bolza::vector_t WheelTorqueMap(const bolza::vector_t& configuration, const bolza::vector_t& wheel_torque) {
  const double sqrt_3 = std::sqrt(3.0);
  const double scale = std::sqrt(2.0) / (4.0 * kWheelRadius);
  // Columns are the three wheels. Rows are the body axes used by R.
  const Eigen::Matrix3d wheel_layout = (Eigen::Matrix3d() << 2.0, -1.0, -1.0, 0.0, sqrt_3, -sqrt_3, -2.0, -2.0, -2.0).finished();

  const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(configuration(kYaw), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(configuration(kPitch), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(configuration(kRoll), Eigen::Vector3d::UnitX()))
                                        .toRotationMatrix();
  const Eigen::Matrix3d roll_rotation = Eigen::AngleAxisd(configuration(kRoll), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d rotated = scale * rotation * wheel_layout;
  const Eigen::Matrix3d roll_only = scale * kBallRadius * roll_rotation * wheel_layout;

  Eigen::Matrix<double, kVelocityDim, kInputDim> map;
  map.row(kPx) = -rotated.row(1);
  map.row(kPy) = rotated.row(0);
  map.row(kYaw) = kBallRadius * rotated.row(2);
  map.row(kPitch) = roll_only.row(1);
  map.row(kRoll) = roll_only.row(0);
  return map * wheel_torque;
}

/**
 * @brief Build dynamics, quadratic cost, the initial state, and the target.
 *
 * Diagonal state weights, in order
 * [px, py, yaw, pitch, roll, vx, vy, yaw_rate, pitch_rate, roll_rate],
 * are [100, 100, 400, 1, 1, 40, 40, 40, 20, 20]. Yaw is weighted heavily so
 * the heading stays put. Pitch and roll are weighted lightly so the body may
 * lean while translating. The input weight is 2. The terminal cost is five
 * times the stage state cost.
 *
 * The state layout is [q, v]. The second-order flag fixes the top block of
 * A to [0 I] and the top block of B to 0, so the difference quotient
 * identifies only the acceleration rows. flow_map must keep the Ballbot
 * alive for the whole solve; Run() does, because the task is local.
 *
 * @param flow_map State derivative of the loaded ballbot.
 * @return A task ready for bolza::Solve.
 */
Task MakeTask(const FlowMapDynamics::FlowMap& flow_map) {
  Task task{bolza::Problem(), bolza::vector_t::Zero(kStateDim)};
  task.problem.SetDynamics(std::make_unique<bolza::SystemDynamicsLinearizer>(
      std::make_unique<FlowMapDynamics>(flow_map),
      /*doubleSidedDerivative=*/true, /*isSecondOrderSystem=*/true, kFiniteDifferenceStep));

  bolza::vector_t state_weight(kStateDim);
  state_weight << 100.0, 100.0, 400.0, 1.0, 1.0, 40.0, 40.0, 40.0, 20.0, 20.0;
  const bolza::matrix_t state_cost = state_weight.asDiagonal();
  const bolza::matrix_t input_cost = bolza::matrix_t::Identity(kInputDim, kInputDim) * 2.0;
  task.problem.AddQuadraticCost(state_cost, input_cost);
  task.problem.AddQuadraticFinalCost(5.0 * state_cost);

  task.target(kPx) = kTargetPositionX;
  task.problem.SetInitialState(bolza::vector_t::Zero(kStateDim));
  task.problem.SetTarget(task.target, bolza::vector_t::Zero(kInputDim));
  return task;
}

/**
 * @brief Log one sampled instant of pose and wheel torque.
 * @param summary Time, state, and input sequences from the solve.
 * @param sample_index Index into those sequences.
 */
void LogSample(const bolza::Solver::Summary& summary, std::size_t sample_index) {
  const char* labels[] = {"px", "py", "yaw", "pitch", "roll"};
  std::ostringstream line;
  line << std::fixed << std::setprecision(4);
  line << "t=" << std::setw(6) << summary.time_trajectory[sample_index];
  for (int label = 0; label < kVelocityDim; ++label) {
    line << ' ' << labels[label] << '=' << std::setw(8) << summary.state_trajectory[sample_index](label);
  }
  line << "  tau";
  for (int torque = 0; torque < kInputDim; ++torque) {
    line << ' ' << std::setw(8) << summary.input_trajectory[sample_index](torque);
  }
  LOG(INFO) << line.str();
}

/**
 * @brief Log the solve status and, on success, a sampled trajectory.
 *
 * The full trajectory is long. About one sample is kept from each quarter of
 * the horizon, and the final sample is always logged.
 *
 * @param summary Result returned by bolza.
 * @param target Target state, used for the terminal error and the last line.
 * @return True when the solve succeeded and the terminal error is within tolerance.
 */
bool LogSolution(const bolza::Solver::Summary& summary, const bolza::vector_t& target) {
  const double error = summary.ok ? (summary.x_final - target).norm() : -1.0;
  std::ostringstream status;
  status << std::fixed << std::setprecision(4) << std::boolalpha;
  status << "solve ok=" << summary.ok << "  " << summary.message << "  samples=" << summary.time_trajectory.size()
         << "  |x(tf)-x*|=" << error;
  if (!summary.ok) {
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
  terminal << "target px=" << target(kPx) << "  final px=" << summary.x_final(kPx) << " py=" << summary.x_final(kPy)
           << " pitch=" << summary.x_final(kPitch) << " roll=" << summary.x_final(kRoll);
  LOG(INFO) << terminal.str();
  return error < kAcceptanceTolerance;
}

}  // namespace

pinocchio::Model LoadModel(const std::string& urdf_path) {
  const std::string urdf_xml = SanitizeEffortLimits(ReadText(urdf_path));
  bolza::PinocchioInterface interface = bolza::GetPinocchioInterfaceFromUrdfString(urdf_xml);
  return interface.GetModel();
}

Ballbot::Ballbot() : Ballbot(bolza::robotic_assets::GetPath() + "/ballbot/urdf/ballbot.urdf") {}

Ballbot::Ballbot(std::string urdf_path)
    : urdf_path_(std::move(urdf_path)), model_(LoadModel(urdf_path_)), data_(model_) {
  model_.gravity.linear() = -kGravity * Eigen::Vector3d::UnitZ();
}

bolza::vector_t Ballbot::ComputeFlowMap(const bolza::vector_t& state, const bolza::vector_t& input) {
  const bolza::vector_t position = state.head<kVelocityDim>();
  const bolza::vector_t velocity = state.tail<kVelocityDim>();
  const bolza::vector_t generalized_force = WheelTorqueMap(position, input);
  pinocchio::aba(model_, data_, position, velocity, generalized_force);

  bolza::vector_t derivative(kStateDim);
  derivative << velocity, data_.ddq;
  return derivative;
}

int Ballbot::Run() {
  LOG(INFO) << "ballbot";
  LOG(INFO) << "urdf " << urdf_path_;
  LOG(INFO) << "x=[px, py, yaw, pitch, roll, v]  u=three wheel torques";
  LOG(INFO) << "model " << model_.name << " nq=" << model_.nq << " nv=" << model_.nv;
  if (model_.nv != kVelocityDim) {
    LOG(ERROR) << "expected 5 velocity coordinates";
    return 1;
  }

  Task task = MakeTask([this](const bolza::vector_t& state, const bolza::vector_t& input) {
    return ComputeFlowMap(state, input);
  });
  bolza::Solver::Options options;
  options.tf = kTimeHorizon;
  options.max_num_iterations = static_cast<std::size_t>(kMaxIterations);

  bolza::Solver::Summary summary;
  bolza::Solve(options, &task.problem, &summary);
  return LogSolution(summary, task.target) ? 0 : 1;
}

}  // namespace ballbot
