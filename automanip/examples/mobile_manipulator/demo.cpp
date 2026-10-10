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
 * @brief Franka mobile-manipulator MPC. PoseStamped is the end-effector target.
 */

#include <cstddef>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/mobile_manipulator/access_helper_functions.hpp"
#include "examples/mobile_manipulator/mobile_manipulator_interface.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

automanip::vector_t EndEffectorTarget(const automanip::examples::PoseStamped& pose) {
  automanip::vector_t target(7);
  target(0) = pose.pose().position().x();
  target(1) = pose.pose().position().y();
  target(2) = pose.pose().position().z();
  target(3) = pose.pose().orientation().x();
  target(4) = pose.pose().orientation().y();
  target(5) = pose.pose().orientation().z();
  target(6) = pose.pose().orientation().w();
  return target;
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  automanip::mobile_manipulator::MobileManipulatorInterface interface(
      AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip/mobile_manipulator", AUTOMANIP_EXAMPLE_URDF);
  const auto& model = interface.getManipulatorModelInfo();

  automanip::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(),
                                    interface.getRollout(), interface.getOptimalControlProblem(),
                                    interface.getInitializer());
  automanip::vector_t initial_target(7);
  initial_target << 0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0;

  automanip::examples::DemoRequest request;
  request.name = "mobile_manipulator";
  request.mpc = &mpc;
  request.rollout = &interface.getRollout();
  request.reference = interface.getReferenceManagerPtr();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = interface.ddpSettings().threadPriority_;
  request.initial_state = interface.getInitialState();
  request.initial_target = initial_target;
  request.initial_input = automanip::vector_t::Zero(model.inputDim);
  request.robot_description =
      automanip::examples::RobotDescriptionWithAbsoluteMeshes(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  request.on_target = [&](const automanip::examples::PoseStamped& pose,
                          const automanip::vector_t&, automanip::scalar_t time) {
    return automanip::TargetTrajectories({time}, {EndEffectorTarget(pose)},
                                         {automanip::vector_t::Zero(model.inputDim)});
  };
  request.publish = [&](const automanip::SystemObservation& observation,
                        const automanip::PrimalSolution&,
                        const automanip::CommandData& command,
                        automanip::examples::MarkerArray* markers,
                        automanip::examples::JointStateMsg* joints,
                        automanip::examples::PathMsg*,
                        automanip::examples::TfMsg* tf) {
    automanip::vector_t state = observation.state;
    const auto arm = automanip::mobile_manipulator::getArmJointAngles(state, model);
    std::vector<std::string> names = model.dofNames;
    std::vector<double> positions(static_cast<std::size_t>(arm.size()));
    for (std::size_t i = 0; i < positions.size(); ++i) {
      positions[i] = arm(static_cast<Eigen::Index>(i));
    }
    names.push_back("panda_finger_joint1");
    names.push_back("panda_finger_joint2");
    positions.push_back(0.0);
    positions.push_back(0.0);
    automanip::examples::FillJoints(joints, names, positions);
    const auto base = automanip::mobile_manipulator::getBasePosition(observation.state, model);
    const auto rotation = automanip::mobile_manipulator::getBaseOrientation(observation.state, model);
    automanip::examples::AddTransform(tf, "map", model.baseFrame, base.x(), base.y(), base.z(),
                                      rotation.w(), rotation.x(), rotation.y(), rotation.z());
    automanip::vector_t goal = request.initial_target;
    if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
      goal = command.mpcTargetTrajectories_.stateTrajectory.back();
    }
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    *markers->add_markers() = automanip::examples::MakeShape(
        "target", 0, Marker::SPHERE, goal(0), goal(1), goal(2), 0.05, 0.05, 0.05, 0.1f, 0.8f, 0.2f,
        goal(6), goal(3), goal(4), goal(5));
  };
  return automanip::examples::RunDemo(request);
}
