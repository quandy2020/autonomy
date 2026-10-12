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
 * @file quadrotor.cpp
 * @brief Quadrotor example implementation and Autolink demo entry.
 *
 * Tracks a 3D position and yaw from autoviz Goal Pose. Roll/pitch remain free.
 *
 * @par Run
 * @code
 * quadrotor_demo [max_steps]
 * @endcode
 */

#include "examples/quadrotor/quadrotor.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/misc/load_data.hpp>
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>

#include "examples/quadrotor/dynamics/quadrotor_system_dynamics.hpp"
#include "examples/quadrotor/quadrotor_parameters.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace bolza {
namespace quadrotor {

namespace {

/**
 * @brief Initializer that holds a constant hover input and freezes the state.
 */
class HoverInitializer final : public Initializer {
 public:
  /**
   * @brief Construct with a hover input.
   * @param [in] hover_input: Steady thrust / torque used at every node.
   */
  explicit HoverInitializer(vector_t hover_input) : hover_input_(std::move(hover_input)) {}

  HoverInitializer* Clone() const override { return new HoverInitializer(hover_input_); }

  void Compute(scalar_t, const vector_t& state, scalar_t, vector_t& input,
               vector_t& next_state) override {
    input = hover_input_;
    next_state = state;
  }

 private:
  vector_t hover_input_;
};

}  // namespace

/**
 * @brief Load task.info and assemble the quadrotor OCP plus MPC helpers.
 * @param [in] taskFile: Absolute path to the INFO configuration file.
 * @param [in] libraryFolder: Directory created for generated artifacts.
 * @param [in] verbose: If true, print load info and weight matrices to stderr.
 */
Quadrotor::Quadrotor(const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
  if (!std::filesystem::exists(taskFile)) {
    throw std::invalid_argument("[Quadrotor] Task file not found: " + taskFile);
  }
  std::filesystem::create_directories(libraryFolder);
  if (verbose) {
    std::cerr << "[Quadrotor] Loading task file: " << taskFile << '\n';
    std::cerr << "[Quadrotor] Generated library path: " << libraryFolder << '\n';
  }

  vector_t x0(STATE_DIM);
  loadData::LoadEigenMatrix(taskFile, "initialState", x0);
  if (verbose) {
    std::cerr << "x_init:   " << x0.transpose() << '\n';
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

  const auto parameters = LoadSettings(taskFile, "QuadrotorParameters", verbose);

  problem_.SetDynamics(std::make_unique<QuadrotorSystemDynamics>(parameters));
  problem_.AddQuadraticCost(Q, R);
  problem_.AddQuadraticFinalCost(Qf);
  problem_.SetInitialState(x0);
  problem_.SetTarget(x0);  // hover at initial pose until Autolink goal arrives

  hoverInput_ = vector_t::Zero(INPUT_DIM);
  hoverInput_(0) = parameters.quadrotorMass_ * parameters.gravity_;

  const auto rolloutSettings = rollout::LoadSettings(taskFile, "rollout", verbose);
  rollout_ = std::make_unique<TimeTriggeredRollout>(*problem_.GetOptimalControlProblem().dynamicsPtr,
                                                    rolloutSettings);
  initializer_ = std::make_unique<HoverInitializer>(hoverInput_);
}

}  // namespace quadrotor
}  // namespace bolza

namespace {

using bolza::CommandData;
using bolza::PrimalSolution;
using bolza::SystemObservation;
using bolza::TargetTrajectories;
using bolza::vector_t;
using bolza::examples::JointStateMsg;
using bolza::examples::MarkerArray;
using bolza::examples::PathMsg;
using bolza::examples::PoseStamped;
using bolza::examples::TfMsg;
using bolza::quadrotor::INPUT_DIM;

/**
 * @brief Resolve the absolute path to the quadrotor mesh.
 * @return Filesystem path to @c meshes/quadrotor.obj.
 */
std::string MeshPath() {
  const std::string urdf_path = AUTOMANIP_EXAMPLE_URDF;
  const auto slash = urdf_path.find_last_of('/');
  const std::string urdf_dir =
      slash == std::string::npos ? std::string(".") : urdf_path.substr(0, slash);
  const auto parent = urdf_dir.find_last_of('/');
  const std::string example_dir =
      parent == std::string::npos ? std::string(".") : urdf_dir.substr(0, parent);
  return example_dir + "/meshes/quadrotor.obj";
}

/**
 * @brief Load URDF and rewrite the package mesh URI to a local path.
 * @return Robot description XML for Autolink.
 */
std::string RobotDescription() {
  std::string xml = bolza::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  const std::string package_uri =
      "package://ocs2_robotic_assets/resources/quadrotor/meshes/quadrotor.obj";
  const auto pos = xml.find(package_uri);
  if (pos != std::string::npos) {
    xml.replace(pos, package_uri.size(), MeshPath());
  }
  return xml;
}

/**
 * @brief Convert ZYX Euler angles to a quaternion.
 * @param [in] yaw: Yaw angle [rad].
 * @param [in] pitch: Pitch angle [rad].
 * @param [in] roll: Roll angle [rad].
 * @param [out] qw: Quaternion w.
 * @param [out] qx: Quaternion x.
 * @param [out] qy: Quaternion y.
 * @param [out] qz: Quaternion z.
 */
void EulerZyxToQuaternion(double yaw, double pitch, double roll, double* qw, double* qx, double* qy,
                          double* qz) {
  const Eigen::Quaterniond rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  *qw = rotation.w();
  *qx = rotation.x();
  *qy = rotation.y();
  *qz = rotation.z();
}

/**
 * @brief Build a two-point command from the current state to a position/yaw goal.
 * @param [in] current: Current (or previous) state used as the first waypoint.
 * @param [in] x: Goal position x.
 * @param [in] y: Goal position y.
 * @param [in] z: Goal position z.
 * @param [in] yaw: Goal yaw [rad], unwrapped relative to @p current.
 * @param [in] time: Start time of the command.
 * @return Target trajectories with zero input reference.
 */
TargetTrajectories CommandToGoal(const vector_t& current, double x, double y, double z, double yaw,
                                 bolza::scalar_t time) {
  vector_t goal = current;
  goal(0) = x;
  goal(1) = y;
  goal(2) = z;
  const double yaw_error = std::atan2(std::sin(yaw - current(5)), std::cos(yaw - current(5)));
  goal(5) = current(5) + yaw_error;
  if (goal.size() > 6) {
    goal.tail(goal.size() - 6).setZero();
  }
  const double distance = (goal - current).head<6>().norm();
  constexpr double kAverageSpeed = 2.0;
  const bolza::scalar_t arrival = time + std::max(0.2, distance / kAverageSpeed);
  const vector_t input = vector_t::Zero(INPUT_DIM);
  return TargetTrajectories({time, arrival}, {current, goal}, {input, input});
}

/**
 * @brief Map a Nav Goal pose to a hover command (random altitude in [0.5, 3] m).
 * @param [in] pose: Desired pose (x/y and yaw from orientation).
 * @param [in] previous: Previous target / live state (see @c command_from_state).
 * @param [in] time: Timestamp for this setpoint.
 * @return Two-point @c TargetTrajectories.
 */
TargetTrajectories TargetFromPose(const PoseStamped& pose, const vector_t& previous,
                                  bolza::scalar_t time) {
  const auto& orientation = pose.pose().orientation();
  const double yaw = std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
  thread_local std::mt19937 generator{std::random_device{}()};
  std::uniform_real_distribution<double> altitude(0.5, 3.0);
  return CommandToGoal(previous, pose.pose().position().x(), pose.pose().position().y(),
                       altitude(generator), yaw, time);
}

/**
 * @brief Publish quadrotor mesh, command frame, and a short path to the hover goal.
 * @param [in] observation: Current system observation.
 * @param [in] policy: MPC primal solution (unused for path; command segment is drawn).
 * @param [in] command: Command data with MPC target trajectory.
 * @param [out] markers: Filled MarkerArray.
 * @param [out] joints: Filled JointState (empty for this model).
 * @param [out] path: Filled predicted Path.
 * @param [out] tf: Filled TF tree.
 */
void Publish(const SystemObservation& observation, const PrimalSolution& /*policy*/,
             const CommandData& command, MarkerArray* markers, JointStateMsg* joints, PathMsg* path,
             TfMsg* tf) {
  vector_t target = observation.state;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target = command.mpcTargetTrajectories_.stateTrajectory.back();
  }
  const double x = observation.state(0);
  const double y = observation.state(1);
  const double z = observation.state(2);
  const double roll = observation.state(3);
  const double pitch = observation.state(4);
  const double yaw = observation.state(5);
  bolza::examples::FillJoints(joints, {}, {});

  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
  EulerZyxToQuaternion(yaw, pitch, roll, &qw, &qx, &qy, &qz);
  bolza::examples::AddTransform(tf, "map", "base", x, y, z, qw, qx, qy, qz);
  EulerZyxToQuaternion(target(5), target(4), target(3), &qw, &qx, &qy, &qz);
  bolza::examples::AddTransform(tf, "map", "command", target(0), target(1), target(2), qw, qx, qy,
                                qz);

  using Marker = automsgs::msgs::visualization_msgs::Marker;
  EulerZyxToQuaternion(yaw, pitch, roll, &qw, &qx, &qy, &qz);
  auto mesh = bolza::examples::MakeShape("body", 0, Marker::MESH_RESOURCE, x, y, z, 0.4, 0.4, 0.4,
                                         0.08f, 0.08f, 0.08f, qw, qx, qy, qz);
  mesh.set_mesh_resource(MeshPath());
  *markers->add_markers() = std::move(mesh);

  const Eigen::Vector3d position = observation.state.head<3>();
  const Eigen::Vector3d goal = target.head<3>();
  constexpr double kGoalTolerance = 0.2;
  if ((position - goal).norm() <= kGoalTolerance) {
    return;
  }
  bolza::examples::AddPathPose(path, position.x(), position.y(), position.z());
  bolza::examples::AddPathPose(path, goal.x(), goal.y(), goal.z());
}

}  // namespace

/**
 * @brief Quadrotor Autolink demo: Problem → GaussNewtonDDPMPC → RunDemo.
 * @param [in] argc: Argument count; optional @c argv[1] is the max simulation steps.
 * @param [in] argv: Command-line arguments; missing or 0 means run indefinitely.
 * @return Exit code from @c examples::RunDemo.
 */
int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;

  bolza::quadrotor::Quadrotor quadrotor(AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip_quadrotor",
                                        /*verbose=*/false);
  auto& problem = quadrotor.GetProblem();

  bolza::GaussNewtonDDPMPC mpc(quadrotor.MpcSettings(), quadrotor.DdpSettings(),
                               quadrotor.GetRollout(), problem.GetOptimalControlProblem(),
                               quadrotor.GetInitializer());
  mpc.GetSolverPtr()->SetReferenceManager(problem.GetReferenceManager());

  bolza::examples::DemoRequest request;
  request.name = "quadrotor";
  request.mpc = &mpc;
  request.rollout = &quadrotor.GetRollout();
  request.reference = problem.GetReferenceManager();
  request.mpc_settings = quadrotor.MpcSettings();
  request.thread_priority = quadrotor.DdpSettings().threadPriority_;
  request.initial_state = problem.GetInitialState();
  request.initial_target = quadrotor.GetInitialTarget();
  request.command_from_state = true;
  request.initial_input = quadrotor.GetInitialInput();
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = RobotDescription();
  request.max_steps = steps;
  return bolza::examples::RunDemo(request);
}
