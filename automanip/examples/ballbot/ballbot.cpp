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
 * @file ballbot.cpp
 * @brief Ballbot example implementation and Autolink demo entry.
 *
 * PoseStamped x/y is the ground target; yaw is taken from the orientation.
 *
 * @par Run
 * @code
 * ballbot_demo [max_steps] [ddp|slp]
 * @endcode
 */

#include "examples/ballbot/ballbot.hpp"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <bolza/core/initialization/default_initializer.hpp>
#include <bolza/core/misc/load_data.hpp>
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
#include <bolza/oc/rollout/time_triggered_rollout.hpp>
#include <bolza/slp/slp_mpc.hpp>

#include "examples/ballbot/dynamics/ballbot_system_dynamics.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace bolza {
namespace ballbot {

/**
 * @brief Load task.info and assemble the ballbot OCP plus MPC helpers.
 * @param [in] taskFile: Absolute path to the INFO configuration file.
 * @param [in] libraryFolder: Directory for CppAD-generated libraries.
 * @param [in] verbose: If true, print load info and weight matrices to stderr.
 */
Ballbot::Ballbot(const std::string& taskFile, const std::string& libraryFolder, bool verbose) {
  if (!std::filesystem::exists(taskFile)) {
    throw std::invalid_argument("[Ballbot] Task file not found: " + taskFile);
  }
  std::filesystem::create_directories(libraryFolder);
  if (verbose) {
    std::cerr << "[Ballbot] Loading task file: " << taskFile << '\n';
    std::cerr << "[Ballbot] Generated library path: " << libraryFolder << '\n';
  }

  vector_t x0(STATE_DIM);
  loadData::LoadEigenMatrix(taskFile, "initialState", x0);
  if (verbose) {
    std::cerr << "x_init:   " << x0.transpose() << '\n';
  }

  ddpSettings_ = ddp::LoadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::LoadSettings(taskFile, "mpc", verbose);
  slpSettings_ = slp::LoadSettings(taskFile, "slp", verbose);

  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::LoadEigenMatrix(taskFile, "Q", Q);
  loadData::LoadEigenMatrix(taskFile, "R", R);
  loadData::LoadEigenMatrix(taskFile, "Q_final", Qf);
  if (verbose) {
    std::cerr << "Q:\n" << Q << "\nR:\n" << R << "\nQ_final:\n" << Qf << '\n';
  }

  bool recompileLibraries = false;
  loadData::LoadCppDataType(taskFile, "ballbot_interface.recompileLibraries", recompileLibraries);

  problem_.SetDynamics(std::make_unique<BallbotSystemDynamics>(libraryFolder, recompileLibraries));
  problem_.AddQuadraticCost(Q, R);
  problem_.AddQuadraticFinalCost(Qf);
  problem_.SetInitialState(x0);
  problem_.SetTarget(x0);

  const auto rolloutSettings = rollout::LoadSettings(taskFile, "rollout", verbose);
  rollout_ = std::make_unique<TimeTriggeredRollout>(*problem_.GetOptimalControlProblem().dynamicsPtr,
                                                    rolloutSettings);
  initializer_ = std::make_unique<DefaultInitializer>(INPUT_DIM);
}

}  // namespace ballbot
}  // namespace bolza

namespace {

using bolza::CommandData;
using bolza::PrimalSolution;
using bolza::SystemObservation;
using bolza::TargetTrajectories;
using bolza::vector_t;
using bolza::ballbot::INPUT_DIM;
using bolza::examples::JointStateMsg;
using bolza::examples::MarkerArray;
using bolza::examples::PathMsg;
using bolza::examples::PoseStamped;
using bolza::examples::TfMsg;
using bolza::examples::TwistStampedMsg;

/**
 * @brief Build a two-point XY/yaw command; pitch/roll remain free.
 * @param [in] current: Current (or previous) state used as the first waypoint.
 * @param [in] x: Goal ball position x.
 * @param [in] y: Goal ball position y.
 * @param [in] yaw: Goal yaw [rad], unwrapped relative to @p current.
 * @param [in] time: Start time of the command.
 * @return Target trajectories with zero input reference.
 */
TargetTrajectories CommandToGoal(const vector_t& current, double x, double y, double yaw,
                                 bolza::scalar_t time) {
  vector_t goal = current;
  goal(0) = x;
  goal(1) = y;
  const double yaw_error = std::atan2(std::sin(yaw - current(2)), std::cos(yaw - current(2)));
  goal(2) = current(2) + yaw_error;
  if (goal.size() > 5) {
    goal.tail(goal.size() - 5).setZero();
  }
  const double distance = (goal - current).head<5>().norm();
  constexpr double kAverageSpeed = 2.0;
  const bolza::scalar_t arrival = time + std::max(0.2, distance / kAverageSpeed);
  const vector_t input = vector_t::Zero(INPUT_DIM);
  return TargetTrajectories({time, arrival}, {current, goal}, {input, input});
}

/**
 * @brief Map a @c PoseStamped ground goal to a ballbot target trajectory.
 * @param [in] pose: Desired pose (x/y and yaw from orientation).
 * @param [in] previous: Previous target / live state.
 * @param [in] time: Timestamp for this setpoint.
 * @return Two-point @c TargetTrajectories.
 */
TargetTrajectories TargetFromPose(const PoseStamped& pose, const vector_t& previous,
                                  bolza::scalar_t time) {
  const auto& orientation = pose.pose().orientation();
  const double yaw = std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
  return CommandToGoal(previous, pose.pose().position().x(), pose.pose().position().y(), yaw, time);
}

/**
 * @brief Resolve the absolute path to the ballbot body mesh.
 * @return Filesystem path to @c meshes/base.obj.
 */
std::string MeshPath() {
  const std::string urdf_path = AUTOMANIP_EXAMPLE_URDF;
  const auto slash = urdf_path.find_last_of('/');
  const std::string urdf_dir =
      slash == std::string::npos ? std::string(".") : urdf_path.substr(0, slash);
  const auto parent = urdf_dir.find_last_of('/');
  const std::string example_dir =
      parent == std::string::npos ? std::string(".") : urdf_dir.substr(0, parent);
  return example_dir + "/meshes/base.obj";
}

/**
 * @brief Load URDF and rewrite the package mesh URI to a local path.
 * @return Robot description XML for Autolink.
 */
std::string RobotDescription() {
  std::string xml = bolza::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  const std::string package_uri =
      "package://ocs2_robotic_assets/resources/ballbot/meshes/base.obj";
  const auto pos = xml.find(package_uri);
  if (pos != std::string::npos) {
    xml.replace(pos, package_uri.size(), MeshPath());
  }
  return xml;
}

/**
 * @brief Publish body-frame velocity for autoviz.
 * @param [in] observation: Current system observation.
 * @param [out] twist: Filled TwistStamped message.
 */
void PublishVelocity(const SystemObservation& observation, TwistStampedMsg* twist) {
  const double yaw = observation.state(2);
  const double vx = observation.state(5);
  const double vy = observation.state(6);
  const Eigen::Quaterniond rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(observation.state(3), Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(observation.state(4), Eigen::Vector3d::UnitX());
  const Eigen::Vector3d v_body = rotation.conjugate() * Eigen::Vector3d(vx, vy, 0.0);
  bolza::examples::StampHeader(twist->mutable_header(), "base");
  twist->mutable_twist()->mutable_linear()->set_x(v_body.x());
  twist->mutable_twist()->mutable_linear()->set_y(v_body.y());
  twist->mutable_twist()->mutable_linear()->set_z(v_body.z());
  twist->mutable_twist()->mutable_angular()->set_x(observation.state(9));
  twist->mutable_twist()->mutable_angular()->set_y(observation.state(8));
  twist->mutable_twist()->mutable_angular()->set_z(observation.state(7));
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
 * @brief Publish ball, body mesh, velocity label, joints, TF, and predicted path.
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
  vector_t target = observation.state;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target = command.mpcTargetTrajectories_.stateTrajectory.back();
  }
  const double x = observation.state(0);
  const double y = observation.state(1);
  const double yaw = observation.state(2);
  const double pitch = observation.state(3);
  const double roll = observation.state(4);
  bolza::examples::FillJoints(joints, {"jball_x", "jball_y", "jbase_z", "jbase_y", "jbase_x"},
                              {x, y, yaw, pitch, roll});

  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
  EulerZyxToQuaternion(yaw, pitch, roll, &qw, &qx, &qy, &qz);
  bolza::examples::AddTransform(tf, "map", "base", x, y, 0.125, qw, qx, qy, qz);
  EulerZyxToQuaternion(target(2), target(3), target(4), &qw, &qx, &qy, &qz);
  bolza::examples::AddTransform(tf, "map", "command", target(0), target(1), 0.0, qw, qx, qy, qz);

  using Marker = automsgs::msgs::visualization_msgs::Marker;
  *markers->add_markers() =
      bolza::examples::MakeShape("ball", 0, Marker::SPHERE, x, y, 0.125, 0.25, 0.25, 0.25, 0.0f,
                                 0.0f, 0.8f);
  const Eigen::Quaterniond body = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(0.275, Eigen::Vector3d::UnitZ());
  auto mesh = bolza::examples::MakeShape("body", 1, Marker::MESH_RESOURCE, x, y, 0.125, 1.0, 1.0,
                                         1.0, 0.5f, 0.5f, 0.5f, body.w(), body.x(), body.y(),
                                         body.z());
  mesh.set_mesh_resource(MeshPath());
  *markers->add_markers() = std::move(mesh);

  const double body_vx = std::cos(yaw) * observation.state(5) + std::sin(yaw) * observation.state(6);
  const double body_vy = -std::sin(yaw) * observation.state(5) + std::cos(yaw) * observation.state(6);
  const double speed = std::hypot(body_vx, body_vy);
  std::ostringstream label;
  label << std::fixed << std::setprecision(2) << "v " << speed << " m/s   vx " << body_vx
        << "  vy " << body_vy << "  wz " << observation.state(7);
  auto text = bolza::examples::MakeShape("velocity", 2, Marker::TEXT_VIEW_FACING, x, y, 1.05, 0.0,
                                         0.0, 0.12, 1.0f, 0.9f, 0.2f);
  text.set_text(label.str());
  *markers->add_markers() = std::move(text);

  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    bolza::examples::AddPathPose(path, trajectory[i](0), trajectory[i](1), 0.0);
  }
}

}  // namespace

/**
 * @brief Ballbot Autolink demo: Problem → DDP or SLP MPC → RunDemo.
 * @param [in] argc: Argument count.
 * @param [in] argv: @c argv[1] max steps; @c argv[2] solver @c ddp (default) or @c slp.
 * @return Exit code from @c examples::RunDemo, or 2 on unknown solver.
 */
int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  const std::string solver = argc > 2 ? argv[2] : "ddp";

  bolza::ballbot::Ballbot ballbot(AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip_ballbot",
                                  /*verbose=*/false);
  auto& problem = ballbot.GetProblem();

  std::unique_ptr<bolza::MPCBase> mpc;
  int thread_priority = ballbot.DdpSettings().threadPriority_;
  if (solver == "slp") {
    mpc = std::make_unique<bolza::SlpMpc>(ballbot.MpcSettings(), ballbot.SlpSettings(),
                                          problem.GetOptimalControlProblem(),
                                          ballbot.GetInitializer());
    thread_priority = ballbot.SlpSettings().threadPriority;
  } else if (solver == "ddp") {
    mpc = std::make_unique<bolza::GaussNewtonDDPMPC>(
        ballbot.MpcSettings(), ballbot.DdpSettings(), ballbot.GetRollout(),
        problem.GetOptimalControlProblem(), ballbot.GetInitializer());
  } else {
    return 2;
  }
  mpc->GetSolverPtr()->SetReferenceManager(problem.GetReferenceManager());

  bolza::examples::DemoRequest request;
  request.name = "ballbot";
  request.mpc = mpc.get();
  request.rollout = &ballbot.GetRollout();
  request.reference = problem.GetReferenceManager();
  request.mpc_settings = ballbot.MpcSettings();
  request.thread_priority = thread_priority;
  request.initial_state = problem.GetInitialState();
  request.initial_target = ballbot.GetInitialTarget();
  request.command_from_state = true;
  request.initial_input = vector_t::Zero(INPUT_DIM);
  request.publish = Publish;
  request.velocity = PublishVelocity;
  request.on_target = TargetFromPose;
  request.robot_description = RobotDescription();
  request.max_steps = steps;
  return bolza::examples::RunDemo(request);
}
