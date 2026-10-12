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
 * @file double_integrator.cpp
 * @brief Double-integrator example implementation and Autolink demo entry.
 *
 * This translation unit provides:
 * 1. @ref bolza::double_integrator::DoubleIntegrator construction: load INFO,
 *    assemble @c bolza::Problem, create rollout / initializer;
 * 2. Autolink callbacks: target pose → setpoint, observation → Marker / TF / Path;
 * 3. @c main: @c GaussNewtonDDPMPC + @c examples::RunDemo.
 *
 * For headless open-loop / MPC solves, use
 * @c examples/tutorial/solve_double_integrator.cpp (target @c bolza_tutorial_di).
 *
 * @par Run
 * @code
 * double_integrator_demo [max_steps]
 * @endcode
 * If @p max_steps is 0, the demo runs until Ctrl+C.
 */

#include "examples/double_integrator/double_integrator.hpp"

#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/core/misc/load_data.hpp>
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace bolza {
namespace double_integrator {

/**
 * @brief Load task.info and assemble the LTI double-integrator OCP plus MPC helpers.
 * @param [in] taskFile: Absolute path to the INFO configuration file.
 * @param [in] verbose: If true, print load info and weight matrices to stderr.
 */
DoubleIntegrator::DoubleIntegrator(const std::string& taskFile, bool verbose) {
  if (!std::filesystem::exists(taskFile)) {
    throw std::invalid_argument("[DoubleIntegrator] Task file not found: " + taskFile);
  }
  if (verbose) {
    std::cerr << "[DoubleIntegrator] Loading task file: " << taskFile << '\n';
  }

  vector_t x0(STATE_DIM);
  loadData::LoadEigenMatrix(taskFile, "initialState", x0);
  loadData::LoadEigenMatrix(taskFile, "finalGoal", initialTarget_);

  ddpSettings_ = ddp::LoadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::LoadSettings(taskFile, "mpc", verbose);

  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::LoadEigenMatrix(taskFile, "Q", Q);
  loadData::LoadEigenMatrix(taskFile, "R", R);
  loadData::LoadEigenMatrix(taskFile, "Q_final", Qf);
  if (verbose) {
    std::cerr << "Q:\n" << Q << "\nR:\n" << R << "\nQ_final:\n" << Qf << '\n';
  }

  // ẋ = A x + B u,  A = [[0,1],[0,0]],  B = [0; 1]  (p̈ = u)
  problem_.SetLinearDynamics(
      (matrix_t(STATE_DIM, STATE_DIM) << 0.0, 1.0, 0.0, 0.0).finished(),
      (matrix_t(STATE_DIM, INPUT_DIM) << 0.0, 1.0).finished());
  problem_.AddQuadraticCost(Q, R);
  problem_.AddQuadraticFinalCost(Qf);
  problem_.SetInitialState(x0);
  problem_.SetTarget(initialTarget_);

  const auto rolloutSettings = rollout::LoadSettings(taskFile, "rollout", verbose);
  rollout_ = std::make_unique<TimeTriggeredRollout>(*problem_.GetOptimalControlProblem().dynamicsPtr,
                                                    rolloutSettings);
  initializer_ = std::make_unique<DefaultInitializer>(INPUT_DIM);
}

}  // namespace double_integrator
}  // namespace bolza

namespace {

using bolza::CommandData;
using bolza::PrimalSolution;
using bolza::SystemObservation;
using bolza::TargetTrajectories;
using bolza::vector_t;
using bolza::double_integrator::INPUT_DIM;
using bolza::examples::JointStateMsg;
using bolza::examples::MarkerArray;
using bolza::examples::PathMsg;
using bolza::examples::PoseStamped;
using bolza::examples::TfMsg;

/**
 * @brief Map an Autolink / autoviz @c PoseStamped to a double-integrator target trajectory.
 *
 * Uses only the pose @f$ x @f$ component as the slider position setpoint @f$ p^\star @f$,
 * sets velocity to zero, and uses a zero input reference.
 *
 * @param [in] pose: Desired pose (@c position.x is the position).
 * @param [in] previous: Previous target state (unused coordinates are overwritten; velocity → 0).
 * @param [in] time: Timestamp associated with this setpoint.
 * @return Single-point @c TargetTrajectories: state @f$ [p^\star;\ 0] @f$, input 0.
 */
TargetTrajectories TargetFromPose(const PoseStamped& pose, const vector_t& previous,
                                  bolza::scalar_t time) {
  vector_t target = previous;
  target(0) = pose.pose().position().x();
  target(1) = 0.0;
  return TargetTrajectories({time}, {target}, {vector_t::Zero(INPUT_DIM)});
}

/**
 * @brief Publish the current observation, MPC policy, and command as visualization messages.
 *
 * Publishes:
 * - Gray sphere @c cart: current slider position;
 * - Red sphere @c target: commanded target position;
 * - Joints @c slider_to_cart / @c slider_to_target;
 * - TF: @c map → @c slideBar → @c cart / @c target;
 * - Predicted path: sparsely sampled policy state trajectory (at most ~40 points).
 *
 * @param [in] observation: Current system observation (@c state(0) is position).
 * @param [in] policy: MPC primal solution (state trajectory for Path).
 * @param [in] command: Command data including the MPC target trajectory.
 * @param [out] markers: Filled MarkerArray.
 * @param [out] joints: Filled JointState.
 * @param [out] path: Filled predicted Path.
 * @param [out] tf: Filled TF tree.
 */
void Publish(const SystemObservation& observation, const PrimalSolution& policy,
             const CommandData& command, MarkerArray* markers, JointStateMsg* joints, PathMsg* path,
             TfMsg* tf) {
  const double position = observation.state(0);
  double target = position;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target = command.mpcTargetTrajectories_.stateTrajectory.front()(0);
  }

  using Marker = automsgs::msgs::visualization_msgs::Marker;
  constexpr double kDiameter = 0.4;
  *markers->add_markers() = bolza::examples::MakeShape(
      "cart", 0, Marker::SPHERE, position, 0.0, 0.0, kDiameter, kDiameter, kDiameter, 0.75f, 0.75f,
      0.75f);
  *markers->add_markers() = bolza::examples::MakeShape(
      "target", 1, Marker::SPHERE, target, 0.0, 0.0, kDiameter, kDiameter, kDiameter, 1.0f, 0.0f,
      0.0f);

  bolza::examples::FillJoints(joints, {"slider_to_cart", "slider_to_target"}, {position, target});
  bolza::examples::AddTransform(tf, "map", "slideBar", 0.0, 0.0, 0.0);
  bolza::examples::AddTransform(tf, "slideBar", "cart", position, 0.0, 0.0);
  bolza::examples::AddTransform(tf, "slideBar", "target", target, 0.0, 0.0);

  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    bolza::examples::AddPathPose(path, trajectory[i](0), 0.0, 0.0);
  }
}

}  // namespace

/**
 * @brief Double-integrator Autolink demo: Problem → GaussNewtonDDPMPC → RunDemo.
 * @param [in] argc: Argument count; optional @c argv[1] is the max simulation steps.
 * @param [in] argv: Command-line arguments; missing or 0 means run indefinitely.
 * @return Exit code from @c examples::RunDemo (typically 0 on success).
 */
int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;

  bolza::double_integrator::DoubleIntegrator di(AUTOMANIP_EXAMPLE_TASK, /*verbose=*/false);
  auto& problem = di.GetProblem();

  bolza::GaussNewtonDDPMPC mpc(di.MpcSettings(), di.DdpSettings(), di.GetRollout(),
                               problem.GetOptimalControlProblem(), di.GetInitializer());
  mpc.GetSolverPtr()->SetReferenceManager(problem.GetReferenceManager());

  bolza::examples::DemoRequest request;
  request.name = "double_integrator";
  request.mpc = &mpc;
  request.rollout = &di.GetRollout();
  request.reference = problem.GetReferenceManager();
  request.mpc_settings = di.MpcSettings();
  request.thread_priority = di.DdpSettings().threadPriority_;
  request.initial_state = problem.GetInitialState();
  request.initial_target = di.GetInitialTarget();
  request.initial_input = vector_t::Zero(INPUT_DIM);
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description =
      bolza::examples::RobotDescriptionWithAbsoluteMeshes(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  return bolza::examples::RunDemo(request);
}
