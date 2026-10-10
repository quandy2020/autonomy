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
 * @brief Cart-pole MPC. PoseStamped.x is the cart target.
 */

#include <cstddef>
#include <cstdlib>
#include <string>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/cartpole/cart_pole_interface.hpp"
#include "examples/cartpole/definitions.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

automanip::TargetTrajectories TargetFromPose(
    const automanip::examples::PoseStamped& pose, const automanip::vector_t& previous,
    automanip::scalar_t time) {
  automanip::vector_t target = previous;
  target(1) = pose.pose().position().x();
  return automanip::TargetTrajectories({time}, {target},
                                       {automanip::vector_t::Zero(automanip::cartpole::INPUT_DIM)});
}

void Publish(const automanip::SystemObservation& observation,
             const automanip::PrimalSolution& policy,
             const automanip::CommandData& command,
             automanip::examples::MarkerArray* markers,
             automanip::examples::JointStateMsg* joints,
             automanip::examples::PathMsg* path,
             automanip::examples::TfMsg* tf) {
  const double theta = observation.state(0);
  const double cart = observation.state(1);
  double target_x = cart;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target_x = command.mpcTargetTrajectories_.stateTrajectory.front()(1);
  }
  // Same joint names as ocs2_cartpole_ros CartpoleDummyVisualization.
  // The rail (slideBar) sits at z = 2, matching the upstream URDF.
  constexpr double kRailZ = 2.0;
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  *markers->add_markers() = automanip::examples::MakeShape(
      "target", 0, Marker::SPHERE, target_x, 0.0, kRailZ + 0.35, 0.12, 0.12, 0.12, 0.1f,
      0.8f, 0.2f);
  automanip::examples::FillJoints(joints, {"slider_to_cart", "cart_to_pole"}, {cart, theta});
  double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
  automanip::examples::AxisAngle(theta, 0.0, 1.0, 0.0, &qw, &qx, &qy, &qz);
  automanip::examples::AddTransform(tf, "map", "world", 0.0, 0.0, 0.0);
  automanip::examples::AddTransform(tf, "world", "slideBar", 0.0, 0.0, kRailZ);
  automanip::examples::AddTransform(tf, "slideBar", "cart", cart, 0.0, 0.0);
  automanip::examples::AddTransform(tf, "cart", "pole", 0.0, 0.0, 0.0, qw, qx, qy, qz);
  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride =
      trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    automanip::examples::AddPathPose(path, trajectory[i](1), 0.0, kRailZ);
  }
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  automanip::cartpole::CartPoleInterface interface(AUTOMANIP_EXAMPLE_TASK,
                                                   "/tmp/automanip_cartpole", false);
  automanip::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(),
                                    interface.getRollout(), interface.getOptimalControlProblem(),
                                    interface.getInitializer());
  automanip::examples::DemoRequest request;
  request.name = "cartpole";
  request.mpc = &mpc;
  request.rollout = &interface.getRollout();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = interface.ddpSettings().threadPriority_;
  request.initial_state = interface.getInitialState();
  request.initial_target = interface.getInitialTarget();
  request.initial_input = automanip::vector_t::Zero(automanip::cartpole::INPUT_DIM);
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = automanip::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  return automanip::examples::RunDemo(request);
}
