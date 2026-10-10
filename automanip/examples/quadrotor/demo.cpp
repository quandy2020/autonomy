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
 * @file demo.cpp
 * @brief Quadrotor MPC. Tracks a 3D position and yaw from autoviz Goal Pose.
 */

#include <cmath>
#include <cstdlib>
#include <random>
#include <string>

#include <Eigen/Geometry>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/quadrotor/definitions.hpp"
#include "examples/quadrotor/quadrotor_interface.hpp"
#include "examples/quadrotor/quadrotor_parameters.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

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

std::string RobotDescription() {
  std::string xml = automanip::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  const std::string package_uri =
      "package://ocs2_robotic_assets/resources/quadrotor/meshes/quadrotor.obj";
  const auto pos = xml.find(package_uri);
  if (pos != std::string::npos) {
    xml.replace(pos, package_uri.size(), MeshPath());
  }
  return xml;
}

void EulerZyxToQuaternion(double yaw, double pitch, double roll, double* qw, double* qx,
                          double* qy, double* qz) {
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  *qw = rotation.w();
  *qx = rotation.x();
  *qy = rotation.y();
  *qz = rotation.z();
}

/** Official command: reach a position and yaw. Roll and pitch stay with the robot. */
automanip::TargetTrajectories CommandToGoal(const automanip::vector_t& current, double x,
                                            double y, double z, double yaw,
                                            automanip::scalar_t time) {
  automanip::vector_t goal = current;
  goal(0) = x;
  goal(1) = y;
  goal(2) = z;
  const double yaw_error =
      std::atan2(std::sin(yaw - current(5)), std::cos(yaw - current(5)));
  goal(5) = current(5) + yaw_error;
  if (goal.size() > 6) {
    goal.tail(goal.size() - 6).setZero();
  }
  const double distance = (goal - current).head<6>().norm();
  constexpr double kAverageSpeed = 2.0;
  const automanip::scalar_t arrival =
      time + std::max(0.2, distance / kAverageSpeed);
  const automanip::vector_t input =
      automanip::vector_t::Zero(automanip::quadrotor::INPUT_DIM);
  return automanip::TargetTrajectories({time, arrival}, {current, goal}, {input, input});
}

automanip::TargetTrajectories TargetFromPose(
    const automanip::examples::PoseStamped& pose, const automanip::vector_t& previous,
    automanip::scalar_t time) {
  const auto& orientation = pose.pose().orientation();
  const double yaw = std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
  // Nav Goal is a ground click. Each target picks its own hover height.
  thread_local std::mt19937 generator{std::random_device{}()};
  std::uniform_real_distribution<double> altitude(0.5, 3.0);
  const double z = altitude(generator);
  return CommandToGoal(previous, pose.pose().position().x(), pose.pose().position().y(), z, yaw,
                       time);
}

void Publish(const automanip::SystemObservation& observation,
             const automanip::PrimalSolution& policy,
             const automanip::CommandData& command,
             automanip::examples::MarkerArray* markers,
             automanip::examples::JointStateMsg* joints,
             automanip::examples::PathMsg* path,
             automanip::examples::TfMsg* tf) {
  automanip::vector_t target = observation.state;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target = command.mpcTargetTrajectories_.stateTrajectory.back();
  }
  const double x = observation.state(0);
  const double y = observation.state(1);
  const double z = observation.state(2);
  const double roll = observation.state(3);
  const double pitch = observation.state(4);
  const double yaw = observation.state(5);
  automanip::examples::FillJoints(joints, {}, {});

  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  EulerZyxToQuaternion(yaw, pitch, roll, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "map", "base", x, y, z, qw, qx, qy, qz);
  EulerZyxToQuaternion(target(5), target(4), target(3), &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "map", "command", target(0), target(1), target(2), qw, qx,
                                    qy, qz);

  using Marker = automsgs::msgs::visualization_msgs::Marker;
  EulerZyxToQuaternion(yaw, pitch, roll, &qw, &qx, &qy, &qz);
  auto mesh = automanip::examples::MakeShape(
      "body", 0, Marker::MESH_RESOURCE, x, y, z, 0.4, 0.4, 0.4, 0.08f, 0.08f, 0.08f, qw, qx, qy,
      qz);
  mesh.set_mesh_resource(MeshPath());
  *markers->add_markers() = std::move(mesh);

  // The drawn path is the open-loop horizon. Feedback already holds the hover,
  // so that horizon is not a path still left to fly.
  const Eigen::Vector3d position = observation.state.head<3>();
  const Eigen::Vector3d goal = target.head<3>();
  constexpr double kGoalTolerance = 0.2;
  if ((position - goal).norm() <= kGoalTolerance) {
    return;
  }
  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    automanip::examples::AddPathPose(path, trajectory[i](0), trajectory[i](1), trajectory[i](2));
  }
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  automanip::quadrotor::QuadrotorInterface interface(AUTOMANIP_EXAMPLE_TASK,
                                                     "/tmp/automanip_quadrotor");
  automanip::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(),
                                    interface.getRollout(), interface.getOptimalControlProblem(),
                                    interface.getInitializer());
  const auto parameters =
      automanip::quadrotor::loadSettings(AUTOMANIP_EXAMPLE_TASK, "QuadrotorParameters", false);
  automanip::examples::DemoRequest request;
  request.name = "quadrotor";
  request.mpc = &mpc;
  request.rollout = &interface.getRollout();
  request.reference = interface.getReferenceManagerPtr();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = interface.ddpSettings().threadPriority_;
  request.initial_state = interface.getInitialState();
  request.initial_target = interface.getInitialState();
  request.command_from_state = true;
  request.initial_input = automanip::vector_t::Zero(automanip::quadrotor::INPUT_DIM);
  request.initial_input(0) = parameters.quadrotorMass_ * parameters.gravity_;
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = RobotDescription();
  request.max_steps = steps;
  return automanip::examples::RunDemo(request);
}
