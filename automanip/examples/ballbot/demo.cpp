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
 * @brief Ballbot MPC. PoseStamped.x/y is the ground target.
 */

#include <cstddef>
#include <cstdlib>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "automanip/slp/slp_mpc.hpp"
#include "examples/ballbot/ballbot_interface.hpp"
#include "examples/ballbot/definitions.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

automanip::TargetTrajectories TargetFromPose(
    const automanip::examples::PoseStamped& pose, const automanip::vector_t& previous,
    automanip::scalar_t time) {
  automanip::vector_t target = previous;
  target.setZero();
  target(0) = pose.pose().position().x();
  target(1) = pose.pose().position().y();
  return automanip::TargetTrajectories(
      {time}, {target}, {automanip::vector_t::Zero(automanip::ballbot::INPUT_DIM)});
}

std::string RobotDescription() {
  std::string xml = automanip::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  const std::string urdf_path = AUTOMANIP_EXAMPLE_URDF;
  const auto slash = urdf_path.find_last_of('/');
  const std::string mesh =
      (slash == std::string::npos ? std::string(".") : urdf_path.substr(0, slash)) +
      "/../meshes/base.obj";
  const std::string package_uri =
      "package://ocs2_robotic_assets/resources/ballbot/meshes/base.obj";
  const auto pos = xml.find(package_uri);
  if (pos != std::string::npos) {
    xml.replace(pos, package_uri.size(), mesh);
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
  (void)markers;
  const double x = observation.state(0);
  const double y = observation.state(1);
  const double yaw = observation.state(2);
  const double pitch = observation.state(3);
  const double roll = observation.state(4);
  automanip::examples::FillJoints(
      joints, {"jball_x", "jball_y", "jbase_z", "jbase_y", "jbase_x"},
      {x, y, yaw, pitch, roll});

  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  // Root frame is map. The official URDF calls this link world; that name is not published.
  automanip::examples::AddTransform(tf, "map", "world_inertia", 0.0, 0.0, 0.0);
  automanip::examples::AddTransform(tf, "world_inertia", "dummy_ball1", x, 0.0, 0.125);
  automanip::examples::AddTransform(tf, "dummy_ball1", "ball", 0.0, y, 0.0);
  automanip::examples::AxisAngle(yaw, 0.0, 0.0, 1.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "ball", "dummy_base1", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  automanip::examples::AxisAngle(pitch, 0.0, 1.0, 0.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "dummy_base1", "dummy_base2", 0.0, 0.0, 0.0, qw, qx, qy,
                                    qz);
  automanip::examples::AxisAngle(roll, 1.0, 0.0, 0.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "dummy_base2", "base", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  EulerZyxToQuaternion(target(2), target(3), target(4), &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "map", "command", target(0), target(1), 0.0, qw, qx, qy,
                                    qz);

  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    automanip::examples::AddPathPose(path, trajectory[i](0), trajectory[i](1), 0.0);
  }
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  const std::string solver = argc > 2 ? argv[2] : "ddp";
  automanip::ballbot::BallbotInterface interface(AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip_ballbot");
  std::unique_ptr<automanip::MPC_BASE> mpc;
  int thread_priority = interface.ddpSettings().threadPriority_;
  if (solver == "slp") {
    mpc = std::make_unique<automanip::SlpMpc>(interface.mpcSettings(), interface.slpSettings(),
                                              interface.getOptimalControlProblem(),
                                              interface.getInitializer());
    thread_priority = interface.slpSettings().threadPriority;
  } else if (solver == "ddp") {
    mpc = std::make_unique<automanip::GaussNewtonDDP_MPC>(
        interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
        interface.getOptimalControlProblem(), interface.getInitializer());
  } else {
    return 2;
  }
  automanip::examples::DemoRequest request;
  request.name = "ballbot";
  request.mpc = mpc.get();
  request.rollout = &interface.getRollout();
  request.reference = interface.getReferenceManagerPtr();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = thread_priority;
  request.initial_state = interface.getInitialState();
  request.initial_target = interface.getInitialState();
  request.initial_input = automanip::vector_t::Zero(automanip::ballbot::INPUT_DIM);
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = RobotDescription();
  request.max_steps = steps;
  return automanip::examples::RunDemo(request);
}
