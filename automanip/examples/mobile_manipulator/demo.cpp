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

#include "bolza/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/mobile_manipulator/access_helper_functions.hpp"
#include "examples/mobile_manipulator/mobile_manipulator_interface.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

bolza::vector_t EndEffectorTarget(const bolza::examples::PoseStamped& pose) {
  bolza::vector_t target(7);
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
  bolza::mobile_manipulator::MobileManipulatorInterface interface(
      AUTOMANIP_EXAMPLE_TASK, "/tmp/automanip/mobile_manipulator", AUTOMANIP_EXAMPLE_URDF);
  const auto& model = interface.GetManipulatorModelInfo();

  bolza::GaussNewtonDDPMPC mpc(interface.MpcSettings(), interface.DdpSettings(),
                                    interface.GetRollout(), interface.GetOptimalControlProblem(),
                                    interface.GetInitializer());
  bolza::vector_t initial_target(7);
  initial_target << 0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0;

  bolza::examples::DemoRequest request;
  request.name = "mobile_manipulator";
  request.mpc = &mpc;
  request.rollout = &interface.GetRollout();
  request.reference = interface.GetReferenceManagerPtr();
  request.mpc_settings = interface.MpcSettings();
  request.thread_priority = interface.DdpSettings().threadPriority_;
  request.initial_state = interface.GetInitialState();
  request.initial_target = initial_target;
  request.initial_input = bolza::vector_t::Zero(model.inputDim);
  request.robot_description =
      bolza::examples::RobotDescriptionWithAbsoluteMeshes(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  request.on_target = [&](const bolza::examples::PoseStamped& pose,
                          const bolza::vector_t&, bolza::scalar_t time) {
    const bolza::vector_t goal = EndEffectorTarget(pose);
    const bolza::vector_t input = bolza::vector_t::Zero(model.inputDim);
    return bolza::TargetTrajectories({time}, {goal}, {input});
  };
  request.publish = [&](const bolza::SystemObservation& observation,
                        const bolza::PrimalSolution&,
                        const bolza::CommandData& command,
                        bolza::examples::MarkerArray* markers,
                        bolza::examples::JointStateMsg* joints,
                        bolza::examples::PathMsg*,
                        bolza::examples::TfMsg* tf) {
    const bolza::vector_t& state = observation.state;
    const auto arm = bolza::mobile_manipulator::getArmJointAngles(state, model);
    std::vector<std::string> names = model.dofNames;
    std::vector<double> positions(static_cast<std::size_t>(arm.size()));
    for (std::size_t i = 0; i < positions.size(); ++i) {
      positions[i] = arm(static_cast<Eigen::Index>(i));
    }
    names.push_back("panda_finger_joint1");
    names.push_back("panda_finger_joint2");
    positions.push_back(0.0);
    positions.push_back(0.0);
    bolza::examples::FillJoints(joints, names, positions);
    const auto base = bolza::mobile_manipulator::GetBasePosition(observation.state, model);
    const auto rotation = bolza::mobile_manipulator::GetBaseOrientation(observation.state, model);
    bolza::examples::AddTransform(tf, "map", model.baseFrame, base.x(), base.y(), base.z(),
                                      rotation.w(), rotation.x(), rotation.y(), rotation.z());
    bolza::vector_t goal = request.initial_target;
    if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
      goal = command.mpcTargetTrajectories_.stateTrajectory.back();
    }
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    *markers->add_markers() = bolza::examples::MakeShape(
        "target", 0, Marker::SPHERE, goal(0), goal(1), goal(2), 0.05, 0.05, 0.05, 0.1f, 0.8f, 0.2f,
        goal(6), goal(3), goal(4), goal(5));
  };
  return bolza::examples::RunDemo(request);
}
