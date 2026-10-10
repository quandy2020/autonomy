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
 * @brief Quadrotor MPC. PoseStamped position is the xyz target.
 */

#include <cstdlib>
#include <string>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/quadrotor/definitions.hpp"
#include "examples/quadrotor/quadrotor_interface.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

automanip::TargetTrajectories TargetFromPose(
    const automanip::examples::PoseStamped& pose, const automanip::vector_t& previous,
    automanip::scalar_t time) {
  automanip::vector_t target = previous;
  target(0) = pose.pose().position().x();
  target(1) = pose.pose().position().y();
  target(2) = pose.pose().position().z();
  target.tail(automanip::quadrotor::STATE_DIM - 3).setZero();
  return automanip::TargetTrajectories(
      {time}, {target}, {automanip::vector_t::Zero(automanip::quadrotor::INPUT_DIM)});
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
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  *markers->add_markers() = automanip::examples::MakeShape(
      "target", 0, Marker::SPHERE, target(0), target(1), target(2), 0.1, 0.1, 0.1, 0.1f, 0.8f,
      0.2f);
  const double yaw = observation.state(5);
  const double pitch = observation.state(4);
  const double roll = observation.state(3);
  automanip::examples::FillJoints(
      joints, {"x", "y", "z", "yaw", "pitch", "roll"},
      {observation.state(0), observation.state(1), observation.state(2), yaw, pitch, roll});
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  automanip::examples::AddTransform(tf, "map", "world", 0.0, 0.0, 0.0);
  automanip::examples::AddTransform(tf, "world", "dx", observation.state(0), 0.0, 0.0);
  automanip::examples::AddTransform(tf, "dx", "dy", 0.0, observation.state(1), 0.0);
  automanip::examples::AddTransform(tf, "dy", "dz", 0.0, 0.0, observation.state(2));
  automanip::examples::AxisAngle(yaw, 0.0, 0.0, 1.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "dz", "yaw_link", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  automanip::examples::AxisAngle(pitch, 0.0, 1.0, 0.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "yaw_link", "pitch_link", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  automanip::examples::AxisAngle(roll, 1.0, 0.0, 0.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "pitch_link", "body", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  automanip::examples::AddTransform(tf, "body", "rotor_fr", 0.18, 0.18, 0.02);
  automanip::examples::AddTransform(tf, "body", "rotor_fl", 0.18, -0.18, 0.02);
  automanip::examples::AddTransform(tf, "body", "rotor_rr", -0.18, 0.18, 0.02);
  automanip::examples::AddTransform(tf, "body", "rotor_rl", -0.18, -0.18, 0.02);
  for (const auto& state : policy.stateTrajectory_) {
    automanip::examples::AddPathPose(path, state(0), state(1), state(2));
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
  automanip::examples::DemoRequest request;
  request.name = "quadrotor";
  request.mpc = &mpc;
  request.rollout = &interface.getRollout();
  request.reference = interface.getReferenceManagerPtr();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = interface.ddpSettings().threadPriority_;
  request.initial_state = interface.getInitialState();
  request.initial_target = interface.getInitialState();
  request.initial_input = automanip::vector_t::Zero(automanip::quadrotor::INPUT_DIM);
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = automanip::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  return automanip::examples::RunDemo(request);
}
