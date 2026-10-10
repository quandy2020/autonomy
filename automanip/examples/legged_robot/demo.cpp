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
 * @brief ANYmal C legged MPC. PoseStamped moves the torso in x, y, and yaw.
 */

#include <pinocchio/fwd.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "automanip/ddp/gauss_newton_ddp_mpc.hpp"
#include "automanip/ipm/ipm_mpc.hpp"
#include "automanip/pinocchio/centroidal/access_helper_functions.hpp"
#include "automanip/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp"
#include "automanip/pinocchio/interface/pinocchio_end_effector_kinematics.hpp"
#include "automanip/sqp/sqp_mpc.hpp"
#include "examples/legged_robot/gait/motion_phase_definition.hpp"
#include "examples/legged_robot/legged_robot_interface.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

namespace {

constexpr std::array<std::array<float, 3>, 4> kFootColor{{
    {{0.0f, 0.4470f, 0.7410f}},
    {{0.8500f, 0.3250f, 0.0980f}},
    {{0.9290f, 0.6940f, 0.1250f}},
    {{0.4940f, 0.1840f, 0.5560f}},
}};
constexpr std::array<float, 3> kGreen{{0.4660f, 0.6740f, 0.1880f}};
constexpr std::array<float, 3> kRed{{0.6350f, 0.0780f, 0.1840f}};
constexpr std::array<float, 3> kBlack{{0.25f, 0.25f, 0.25f}};

void AddPoint(automsgs::msgs::visualization_msgs::Marker* marker, const Eigen::Vector3d& point) {
  auto* corner = marker->add_points();
  corner->set_x(point.x());
  corner->set_y(point.y());
  corner->set_z(point.z());
}

double Yaw(const automanip::examples::PoseStamped& pose) {
  const double w = pose.pose().orientation().w();
  const double x = pose.pose().orientation().x();
  const double y = pose.pose().orientation().y();
  const double z = pose.pose().orientation().z();
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

int main(int argc, char** argv) {
  const int steps = argc > 1 ? std::atoi(argv[1]) : 0;
  const std::string solver = argc > 2 ? argv[2] : "ddp";
  automanip::legged_robot::LeggedRobotInterface interface(
      AUTOMANIP_EXAMPLE_TASK, AUTOMANIP_EXAMPLE_URDF, AUTOMANIP_EXAMPLE_REFERENCE);
  const auto& info = interface.getCentroidalModelInfo();
  const auto& joint_names = interface.modelSettings().jointNames;

  std::unique_ptr<automanip::MPC_BASE> mpc;
  int thread_priority = interface.ddpSettings().threadPriority_;
  if (solver == "sqp") {
    mpc = std::make_unique<automanip::SqpMpc>(interface.mpcSettings(), interface.sqpSettings(),
                                              interface.getOptimalControlProblem(),
                                              interface.getInitializer());
    thread_priority = interface.sqpSettings().threadPriority;
  } else if (solver == "ipm") {
    mpc = std::make_unique<automanip::IpmMpc>(interface.mpcSettings(), interface.ipmSettings(),
                                              interface.getOptimalControlProblem(),
                                              interface.getInitializer());
    thread_priority = interface.ipmSettings().threadPriority;
  } else if (solver == "ddp") {
    mpc = std::make_unique<automanip::GaussNewtonDDP_MPC>(
        interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
        interface.getOptimalControlProblem(), interface.getInitializer());
  } else {
    return 2;
  }

  automanip::PinocchioInterface pinocchio_view(interface.getPinocchioInterface());
  automanip::CentroidalModelPinocchioMapping pinocchio_mapping(info);
  automanip::PinocchioEndEffectorKinematics end_effector_kinematics(
      pinocchio_view, pinocchio_mapping, interface.modelSettings().contactNames3DoF);

  automanip::examples::DemoRequest request;
  request.name = "legged_robot";
  request.mpc = mpc.get();
  request.rollout = &interface.getRollout();
  request.reference = interface.getReferenceManagerPtr();
  request.mpc_settings = interface.mpcSettings();
  request.thread_priority = thread_priority;
  request.initial_state = interface.getInitialState();
  request.initial_target = interface.getInitialState();
  request.initial_input = automanip::vector_t::Zero(info.inputDim);
  request.initial_mode = automanip::legged_robot::ModeNumber::STANCE;
  request.robot_description =
      automanip::examples::RobotDescriptionWithAbsoluteMeshes(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  request.on_target = [&](const automanip::examples::PoseStamped& pose,
                          const automanip::vector_t& previous, automanip::scalar_t time) {
    automanip::vector_t target = previous;
    target.head(6).setZero();
    target(6) = pose.pose().position().x();
    target(7) = pose.pose().position().y();
    target(9) = Yaw(pose);
    return automanip::TargetTrajectories({time}, {target},
                                         {automanip::vector_t::Zero(info.inputDim)});
  };
  request.publish = [&](const automanip::SystemObservation& observation,
                        const automanip::PrimalSolution& policy,
                        const automanip::CommandData& command,
                        automanip::examples::MarkerArray* markers,
                        automanip::examples::JointStateMsg* joints,
                        automanip::examples::PathMsg* path,
                        automanip::examples::TfMsg* tf) {
    automanip::vector_t state = observation.state;
    const auto base = automanip::centroidal_model::getBasePose(state, info);
    const auto angles = automanip::centroidal_model::getJointAngles(state, info);
    std::vector<double> positions(static_cast<std::size_t>(angles.size()));
    for (std::size_t i = 0; i < positions.size(); ++i) {
      positions[i] = angles(static_cast<Eigen::Index>(i));
    }
    automanip::examples::FillJoints(joints, joint_names, positions);
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(base(3), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(base(4), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(base(5), Eigen::Vector3d::UnitX());
    automanip::examples::AddTransform(tf, "map", "base", base(0), base(1), base(2), rotation.w(),
                                      rotation.x(), rotation.y(), rotation.z());

    const auto& trajectory = policy.stateTrajectory_;
    const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    auto com = automanip::examples::MakeShape("CoM Trajectory", 0, Marker::LINE_STRIP, 0.0, 0.0, 0.0,
                                              0.01, 0.01, 0.01, kRed[0], kRed[1], kRed[2]);
    for (std::size_t i = 0; i < trajectory.size(); i += stride) {
      automanip::examples::AddPathPose(path, trajectory[i](6), trajectory[i](7), trajectory[i](8));
      AddPoint(&com, Eigen::Vector3d(trajectory[i](6), trajectory[i](7), trajectory[i](8)));
    }
    if (com.points_size() >= 2) {
      *markers->add_markers() = std::move(com);
    }
    const auto& desired = command.mpcTargetTrajectories_.stateTrajectory;
    auto desired_line = automanip::examples::MakeShape(
        "Desired Trajectory", 0, Marker::LINE_STRIP, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01, kGreen[0],
        kGreen[1], kGreen[2]);
    for (const auto& desired_state : desired) {
      if (desired_state.size() > 8) {
        AddPoint(&desired_line, Eigen::Vector3d(desired_state(6), desired_state(7), desired_state(8)));
      }
    }
    if (desired_line.points_size() >= 2) {
      *markers->add_markers() = std::move(desired_line);
    }

    auto& model = pinocchio_view.getModel();
    auto& data = pinocchio_view.getData();
    const auto q = automanip::centroidal_model::getGeneralizedCoordinates(state, info);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    end_effector_kinematics.setPinocchioInterface(pinocchio_view);
    const auto feet = end_effector_kinematics.getPosition(state);
    const auto contact = automanip::legged_robot::modeNumber2StanceLeg(observation.mode);
    const auto input = observation.input;
    const std::size_t contacts = std::min(feet.size(), contact.size());

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double force_z = 0.0;
    int stance_count = 0;
    for (std::size_t i = 0; i < contacts; ++i) {
      const Eigen::Vector3d foot = feet[i];
      auto sphere = automanip::examples::MakeShape(
          "EE Positions", static_cast<int>(i), Marker::SPHERE, foot.x(), foot.y(), foot.z(), 0.03,
          0.03, 0.03, kFootColor[i][0], kFootColor[i][1], kFootColor[i][2]);
      if (!contact[i]) {
        sphere.mutable_color()->set_a(0.3f);
      }
      *markers->add_markers() = std::move(sphere);

      const Eigen::Vector3d force =
          automanip::centroidal_model::getContactForces(input, i, info);
      if (contact[i]) {
        const Eigen::Vector3d arrow = force / 1000.0;
        const double length = arrow.norm();
        if (length > 1e-4) {
          const Eigen::Quaterniond aim =
              Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), arrow / length);
          const Eigen::Vector3d start = foot - arrow;
          *markers->add_markers() = automanip::examples::MakeShape(
              "EE Forces", static_cast<int>(i), Marker::ARROW, start.x(), start.y(), start.z(),
              length, 0.01, 0.02, kGreen[0], kGreen[1], kGreen[2], aim.w(), aim.x(), aim.y(),
              aim.z());
        }
        center += force.z() * foot;
        force_z += force.z();
        ++stance_count;
      }
    }
    if (force_z > 0.0 && stance_count > 0) {
      center /= force_z;
      *markers->add_markers() = automanip::examples::MakeShape(
          "Center of Pressure", 0, Marker::SPHERE, center.x(), center.y(), center.z(), 0.03, 0.03,
          0.03, kGreen[0], kGreen[1], kGreen[2]);
    }
    auto polygon = automanip::examples::MakeShape("Support Polygon", 0, Marker::LINE_LIST, 0.0, 0.0,
                                                  0.0, 0.005, 0.005, 0.005, kBlack[0], kBlack[1],
                                                  kBlack[2]);
    for (std::size_t i = 0; i < contacts; ++i) {
      if (!contact[i]) {
        continue;
      }
      for (std::size_t j = i + 1; j < contacts; ++j) {
        if (!contact[j]) {
          continue;
        }
        AddPoint(&polygon, feet[i]);
        AddPoint(&polygon, feet[j]);
      }
    }
    if (polygon.points_size() >= 2) {
      *markers->add_markers() = std::move(polygon);
    }
  };
  return automanip::examples::RunDemo(request);
}
