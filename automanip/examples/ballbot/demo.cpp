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

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "automanip/slp/slp_mpc.hpp"
#include "examples/ballbot/ballbot_interface.hpp"
#include "examples/ballbot/definitions.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

/** Official ballbot command: reach XY and yaw, and let pitch/roll stay free. */
automanip::TargetTrajectories CommandToGoal(const automanip::vector_t& current,
                                            double x, double y, double yaw,
                                            automanip::scalar_t time) {
  automanip::vector_t goal = current;
  goal(0) = x;
  goal(1) = y;
  // Keep the reference yaw continuous with the live heading.
  const double yaw_error =
      std::atan2(std::sin(yaw - current(2)), std::cos(yaw - current(2)));
  goal(2) = current(2) + yaw_error;
  if (goal.size() > 5) {
    goal.tail(goal.size() - 5).setZero();
  }
  const double distance = (goal - current).head<5>().norm();
  constexpr double kAverageSpeed = 2.0;
  const automanip::scalar_t arrival =
      time + std::max(0.2, distance / kAverageSpeed);
  const automanip::vector_t input =
      automanip::vector_t::Zero(automanip::ballbot::INPUT_DIM);
  return automanip::TargetTrajectories({time, arrival}, {current, goal}, {input, input});
}

automanip::TargetTrajectories TargetFromPose(
    const automanip::examples::PoseStamped& pose, const automanip::vector_t& previous,
    automanip::scalar_t time) {
  const auto& orientation = pose.pose().orientation();
  const double yaw = std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
  return CommandToGoal(previous, pose.pose().position().x(), pose.pose().position().y(), yaw,
                       time);
}

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

std::string RobotDescription() {
  std::string xml = automanip::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  const std::string package_uri =
      "package://ocs2_robotic_assets/resources/ballbot/meshes/base.obj";
  const auto pos = xml.find(package_uri);
  if (pos != std::string::npos) {
    xml.replace(pos, package_uri.size(), MeshPath());
  }
  return xml;
}

/** Body-frame velocity. The arrow is drawn on the base frame. */
void PublishVelocity(const automanip::SystemObservation& observation,
                     automanip::examples::TwistStampedMsg* twist) {
  const double yaw = observation.state(2);
  const double vx = observation.state(5);
  const double vy = observation.state(6);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(observation.state(3), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(observation.state(4), Eigen::Vector3d::UnitX());
  const Eigen::Vector3d v_body =
      rotation.conjugate() * Eigen::Vector3d(vx, vy, 0.0);
  automanip::examples::StampHeader(twist->mutable_header(), "base");
  twist->mutable_twist()->mutable_linear()->set_x(v_body.x());
  twist->mutable_twist()->mutable_linear()->set_y(v_body.y());
  twist->mutable_twist()->mutable_linear()->set_z(v_body.z());
  twist->mutable_twist()->mutable_angular()->set_x(observation.state(9));
  twist->mutable_twist()->mutable_angular()->set_y(observation.state(8));
  twist->mutable_twist()->mutable_angular()->set_z(observation.state(7));
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
  // Official RViz enables base and command only. The prismatic dummy links
  // sit meters apart (x, then y), so publishing them draws a broken triangle.
  EulerZyxToQuaternion(yaw, pitch, roll, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "map", "base", x, y, 0.125, qw, qx, qy, qz);
  EulerZyxToQuaternion(target(2), target(3), target(4), &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "map", "command", target(0), target(1), 0.0, qw, qx, qy,
                                    qz);

  // The ball is a 0.25 m blue sphere centered 0.125 m above the ground.
  // The body mesh sits on that center; its URDF visual origin yaws by 0.275 rad.
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  *markers->add_markers() = automanip::examples::MakeShape(
      "ball", 0, Marker::SPHERE, x, y, 0.125, 0.25, 0.25, 0.25, 0.0f, 0.0f, 0.8f);
  const Eigen::Quaterniond body =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.275, Eigen::Vector3d::UnitZ());
  auto mesh = automanip::examples::MakeShape(
      "body", 1, Marker::MESH_RESOURCE, x, y, 0.125, 1.0, 1.0, 1.0, 0.5f, 0.5f, 0.5f,
      body.w(), body.x(), body.y(), body.z());
  mesh.set_mesh_resource(MeshPath());
  *markers->add_markers() = std::move(mesh);

  const double body_vx = std::cos(yaw) * observation.state(5) + std::sin(yaw) * observation.state(6);
  const double body_vy = -std::sin(yaw) * observation.state(5) + std::cos(yaw) * observation.state(6);
  const double speed = std::hypot(body_vx, body_vy);
  std::ostringstream label;
  label << std::fixed << std::setprecision(2) << "v " << speed << " m/s   vx " << body_vx
        << "  vy " << body_vy << "  wz " << observation.state(7);
  auto text = automanip::examples::MakeShape(
      "velocity", 2, Marker::TEXT_VIEW_FACING, x, y, 1.05, 0.0, 0.0, 0.12, 1.0f, 0.9f,
      0.2f);
  text.set_text(label.str());
  *markers->add_markers() = std::move(text);

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
  request.command_from_state = true;
  request.initial_input = automanip::vector_t::Zero(automanip::ballbot::INPUT_DIM);
  request.publish = Publish;
  request.velocity = PublishVelocity;
  request.on_target = TargetFromPose;
  request.robot_description = RobotDescription();
  request.max_steps = steps;
  return automanip::examples::RunDemo(request);
}
