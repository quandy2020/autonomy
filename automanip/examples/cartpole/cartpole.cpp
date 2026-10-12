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
 * @file cartpole.cpp
 * @brief Cart-pole example implementation and Autolink demo entry.
 *
 * This translation unit provides:
 * 1. @ref bolza::cartpole::CartPole construction: load INFO, assemble @c bolza::Problem,
 *    soft input limits, rollout / initializer;
 * 2. Autolink callbacks: target pose → cart position setpoint, observation → viz;
 * 3. @c main: @c GaussNewtonDDPMPC + @c examples::RunDemo.
 *
 * @par Run
 * @code
 * cartpole_demo [max_steps]
 * @endcode
 * If @p max_steps is 0, the demo runs until Ctrl+C.
 */

#include "examples/cartpole/cartpole.hpp"

#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <bolza/core/augmented_lagrangian/augmented_lagrangian.hpp>
#include <bolza/core/constraint/linear_state_input_constraint.hpp>
#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/core/misc/load_data.hpp>
#include <bolza/core/penalties/penalties.hpp>
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

#include "examples/cartpole/cart_pole_parameters.hpp"
#include "examples/cartpole/dynamics/cart_pole_system_dynamics.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace bolza {
namespace cartpole {

/**
 * @brief Load task.info and assemble the cart-pole OCP plus MPC helpers.
 * @param [in] taskFile: Absolute path to the INFO configuration file.
 * @param [in] libraryFolder: Directory for CppAD-generated libraries.
 * @param [in] verbose: If true, print load info and weight matrices to stderr.
 */
CartPole::CartPole(const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
  if (!std::filesystem::exists(taskFile)) {
    throw std::invalid_argument("[CartPole] Task file not found: " + taskFile);
  }
  std::filesystem::create_directories(libraryFolder);
  if (verbose) {
    std::cerr << "[CartPole] Loading task file: " << taskFile << '\n';
    std::cerr << "[CartPole] Generated library path: " << libraryFolder << '\n';
  }

  vector_t x0(STATE_DIM);
  loadData::LoadEigenMatrix(taskFile, "initialState", x0);
  loadData::LoadEigenMatrix(taskFile, "x_final", initialTarget_);
  if (verbose) {
    std::cerr << "x_init:   " << x0.transpose() << '\n';
    std::cerr << "x_final:  " << initialTarget_.transpose() << '\n';
  }

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

  CartPoleParameters parameters;
  parameters.LoadSettings(taskFile, "cartpole_parameters", verbose);

  problem_.SetDynamics(std::make_unique<CartPoleSytemDynamics>(parameters, libraryFolder, verbose));
  problem_.AddQuadraticCost(Q, R);
  problem_.AddQuadraticFinalCost(Qf);
  problem_.SetInitialState(x0);
  problem_.SetTarget(initialTarget_);

  auto& ocp = problem_.GetOptimalControlProblem();
  auto getPenalty = [&]() {
    using penalty_type = augmented::SlacknessSquaredHingePenalty;
    penalty_type::Config boundsConfig;
    loadData::LoadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig, verbose);
    return penalty_type::Create(boundsConfig);
  };
  auto getConstraint = [&]() {
    constexpr size_t numIneqConstraint = 2;
    const vector_t e =
        (vector_t(numIneqConstraint) << parameters.maxInput_, parameters.maxInput_).finished();
    const vector_t D = (vector_t(numIneqConstraint) << 1.0, -1.0).finished();
    const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
    return std::make_unique<LinearStateInputConstraint>(e, C, D);
  };
  ocp.inequalityLagrangianPtr->Add("InputLimits", Create(getConstraint(), getPenalty()));

  const auto rolloutSettings = rollout::LoadSettings(taskFile, "rollout", verbose);
  rollout_ = std::make_unique<TimeTriggeredRollout>(*ocp.dynamicsPtr, rolloutSettings);
  initializer_ = std::make_unique<DefaultInitializer>(INPUT_DIM);
}

}  // namespace cartpole
}  // namespace bolza

namespace {

using bolza::CommandData;
using bolza::PrimalSolution;
using bolza::SystemObservation;
using bolza::TargetTrajectories;
using bolza::vector_t;
using bolza::cartpole::INPUT_DIM;
using bolza::examples::JointStateMsg;
using bolza::examples::MarkerArray;
using bolza::examples::PathMsg;
using bolza::examples::PoseStamped;
using bolza::examples::TfMsg;

/**
 * @brief Map a @c PoseStamped to a cart-pole target (cart position from @c position.x).
 * @param [in] pose: Desired pose.
 * @param [in] previous: Previous target state.
 * @param [in] time: Timestamp for this setpoint.
 * @return Single-point @c TargetTrajectories with zero input.
 */
TargetTrajectories TargetFromPose(const PoseStamped& pose, const vector_t& previous,
                                  bolza::scalar_t time) {
  vector_t target = previous;
  target(1) = pose.pose().position().x();
  return TargetTrajectories({time}, {target}, {vector_t::Zero(INPUT_DIM)});
}

/**
 * @brief Publish cart-pole markers, joints, TF, and predicted path.
 * @param [in] observation: Current system observation.
 * @param [in] policy: MPC primal solution.
 * @param [in] command: Command data with MPC target trajectory.
 * @param [out] markers: Filled MarkerArray.
 * @param [out] joints: Filled JointState.
 * @param [out] path: Filled predicted Path.
 * @param [out] tf: Filled TF tree.
 */
void Publish(const SystemObservation& observation, const PrimalSolution& policy,
             const CommandData& command, MarkerArray* markers, JointStateMsg* joints, PathMsg* path,
             TfMsg* tf) {
  const double theta = observation.state(0);
  const double cart = observation.state(1);
  double target_x = cart;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target_x = command.mpcTargetTrajectories_.stateTrajectory.front()(1);
  }
  constexpr double kRailZ = 2.0;
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  *markers->add_markers() = bolza::examples::MakeShape(
      "target", 0, Marker::SPHERE, target_x, 0.0, kRailZ + 0.35, 0.12, 0.12, 0.12, 0.1f, 0.8f, 0.2f);
  bolza::examples::FillJoints(joints, {"slider_to_cart", "cart_to_pole"}, {cart, theta});
  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
  bolza::examples::AxisAngle(theta, 0.0, 1.0, 0.0, &qw, &qx, &qy, &qz);
  bolza::examples::AddTransform(tf, "map", "world", 0.0, 0.0, 0.0);
  bolza::examples::AddTransform(tf, "world", "slideBar", 0.0, 0.0, kRailZ);
  bolza::examples::AddTransform(tf, "slideBar", "cart", cart, 0.0, 0.0);
  bolza::examples::AddTransform(tf, "cart", "pole", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    bolza::examples::AddPathPose(path, trajectory[i](1), 0.0, kRailZ);
  }
}

}  // namespace

/**
 * @brief Cart-pole Autolink demo: Problem → GaussNewtonDDPMPC → RunDemo.
 * @param [in] argc: Argument count; optional @c argv[1] is the max simulation steps.
 * @param [in] argv: Command-line arguments; missing or 0 means run indefinitely.
 * @return Exit code from @c examples::RunDemo.
 */
int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;

  bolza::cartpole::CartPole cartpole(AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip_cartpole",
                                     /*verbose=*/false);
  auto& problem = cartpole.GetProblem();

  bolza::GaussNewtonDDPMPC mpc(cartpole.MpcSettings(), cartpole.DdpSettings(), cartpole.GetRollout(),
                               problem.GetOptimalControlProblem(), cartpole.GetInitializer());
  mpc.GetSolverPtr()->SetReferenceManager(problem.GetReferenceManager());

  bolza::examples::DemoRequest request;
  request.name = "cartpole";
  request.mpc = &mpc;
  request.rollout = &cartpole.GetRollout();
  request.reference = problem.GetReferenceManager();
  request.mpc_settings = cartpole.MpcSettings();
  request.thread_priority = cartpole.DdpSettings().threadPriority_;
  request.initial_state = problem.GetInitialState();
  request.initial_target = cartpole.GetInitialTarget();
  request.initial_input = vector_t::Zero(INPUT_DIM);
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = bolza::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  return bolza::examples::RunDemo(request);
}
