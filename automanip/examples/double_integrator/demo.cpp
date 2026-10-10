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
 * @brief Double integrator. PoseStamped.x is the position target.
 */

#include <cstddef>
#include <cstdlib>
#include <string>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/double_integrator/definitions.hpp"
#include "examples/double_integrator/double_integrator_interface.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

automanip::TargetTrajectories TargetFromPose(
    const automanip::examples::PoseStamped& pose, const automanip::vector_t& previous,
    automanip::scalar_t time) {
  automanip::vector_t target = previous;
  target(0) = pose.pose().position().x();
  target(1) = 0.0;
  return automanip::TargetTrajectories(
      {time}, {target}, {automanip::vector_t::Zero(automanip::double_integrator::INPUT_DIM)});
}

void Publish(const automanip::SystemObservation& observation,
             const automanip::PrimalSolution& policy,
             const automanip::CommandData& command,
             automanip::examples::MarkerArray* markers,
             automanip::examples::JointStateMsg* joints,
             automanip::examples::PathMsg* path,
             automanip::examples::TfMsg* tf) {
  const double position = observation.state(0);
  double target = position;
  if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
    target = command.mpcTargetTrajectories_.stateTrajectory.front()(0);
  }
  // Gray cart and red target, same geometry as the OCS2 double-integrator URDF.
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  constexpr double kDiameter = 0.4;
  *markers->add_markers() = automanip::examples::MakeShape(
      "cart", 0, Marker::SPHERE, position, 0.0, 0.0, kDiameter, kDiameter, kDiameter,
      0.75f, 0.75f, 0.75f);
  *markers->add_markers() = automanip::examples::MakeShape(
      "target", 1, Marker::SPHERE, target, 0.0, 0.0, kDiameter, kDiameter, kDiameter,
      1.0f, 0.0f, 0.0f);
  automanip::examples::FillJoints(joints, {"slider_to_cart", "slider_to_target"},
                                  {position, target});
  automanip::examples::AddTransform(tf, "map", "slideBar", 0.0, 0.0, 0.0);
  automanip::examples::AddTransform(tf, "slideBar", "cart", position, 0.0, 0.0);
  automanip::examples::AddTransform(tf, "slideBar", "target", target, 0.0, 0.0);
  const auto& trajectory = policy.stateTrajectory_;
  const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
  for (std::size_t i = 0; i < trajectory.size(); i += stride) {
    automanip::examples::AddPathPose(path, trajectory[i](0), 0.0, 0.0);
  }
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  automanip::double_integrator::DoubleIntegratorInterface interface(
      AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip_double_integrator", false);
  automanip::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(),
                                    interface.getRollout(), interface.getOptimalControlProblem(),
                                    interface.getInitializer());
  automanip::examples::DemoRequest request;
  request.name = "double_integrator";
  request.mpc = &mpc;
  request.rollout = &interface.getRollout();
  request.reference = interface.getReferenceManagerPtr();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = interface.ddpSettings().threadPriority_;
  request.initial_state = interface.getInitialState();
  request.initial_target = interface.getInitialTarget();
  request.initial_input =
      automanip::vector_t::Zero(automanip::double_integrator::INPUT_DIM);
  request.publish = Publish;
  request.on_target = TargetFromPose;
  request.robot_description = automanip::examples::ReadTextFile(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  return automanip::examples::RunDemo(request);
}
