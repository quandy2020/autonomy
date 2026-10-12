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

#include "bolza/core/misc/linear_interpolation.hpp"
#include "bolza/ddp/gauss_newton_ddp_mpc.hpp"
#include "bolza/ipm/ipm_mpc.hpp"
#include "bolza/pinocchio/centroidal/access_helper_functions.hpp"
#include "bolza/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp"
#include "bolza/pinocchio/interface/pinocchio_end_effector_kinematics.hpp"
#include "bolza/sqp/sqp_mpc.hpp"
#include "examples/legged_robot/gait/motion_phase_definition.hpp"
#include "examples/legged_robot/legged_robot_interface.hpp"
#include "examples/legged_robot/terrain/terrain.hpp"
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

void AppendTerrain(bolza::examples::MarkerArray* markers, const bolza::legged_robot::Terrain& terrain) {
  if (markers == nullptr || !terrain.hasRelief() || !(terrain.resolution() > 0.0)) {
    return;
  }
  const double resolution = terrain.resolution();
  const int stride = std::max(1, static_cast<int>(std::lround(0.1 / resolution)));
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  auto mesh = bolza::examples::MakeShape("Terrain", 0, Marker::TRIANGLE_LIST, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.45f,
                                        0.42f, 0.36f);
  mesh.set_mesh_resource(terrain.description());
  auto add = [&](double x, double y) { AddPoint(&mesh, Eigen::Vector3d(x, y, terrain.height(x, y))); };
  for (int iy = 0; iy + stride < terrain.rows(); iy += stride) {
    for (int ix = 0; ix + stride < terrain.columns(); ix += stride) {
      const double x0 = terrain.originX() + static_cast<double>(ix) * resolution;
      const double y0 = terrain.originY() + static_cast<double>(iy) * resolution;
      const double x1 = terrain.originX() + static_cast<double>(ix + stride) * resolution;
      const double y1 = terrain.originY() + static_cast<double>(iy + stride) * resolution;
      add(x0, y0);
      add(x0, y1);
      add(x1, y0);
      add(x1, y0);
      add(x0, y1);
      add(x1, y1);
    }
  }
  if (mesh.points_size() >= 3) {
    *markers->add_markers() = std::move(mesh);
  }
}

double Yaw(const bolza::examples::PoseStamped& pose) {
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
  bolza::legged_robot::LeggedRobotInterface interface(
      AUTOMANIP_EXAMPLE_TASK, AUTOMANIP_EXAMPLE_URDF, AUTOMANIP_EXAMPLE_REFERENCE);
  const auto& info = interface.GetCentroidalModelInfo();

  std::unique_ptr<bolza::MPCBase> mpc;
  int thread_priority = interface.DdpSettings().threadPriority_;
  if (solver == "sqp") {
    mpc = std::make_unique<bolza::SqpMpc>(interface.MpcSettings(), interface.SqpSettings(),
                                              interface.GetOptimalControlProblem(),
                                              interface.GetInitializer());
    thread_priority = interface.SqpSettings().threadPriority;
  } else if (solver == "ipm") {
    mpc = std::make_unique<bolza::IpmMpc>(interface.MpcSettings(), interface.IpmSettings(),
                                              interface.GetOptimalControlProblem(),
                                              interface.GetInitializer());
    thread_priority = interface.IpmSettings().threadPriority;
  } else if (solver == "ddp") {
    mpc = std::make_unique<bolza::GaussNewtonDDPMPC>(
        interface.MpcSettings(), interface.DdpSettings(), interface.GetRollout(),
        interface.GetOptimalControlProblem(), interface.GetInitializer());
  } else {
    return 2;
  }

  bolza::PinocchioInterface pinocchio_view(interface.GetPinocchioInterface());
  // ModelSettings().jointNames is LF, RF, LH, RH. Pinocchio q, and therefore
  // getJointAngles, is LF, LH, RF, RH. Publishing the settings list would put
  // the hind-left angles on the right-front leg.
  const auto& pinocchio_model = pinocchio_view.GetModel();
  std::vector<std::string> joint_names;
  joint_names.reserve(static_cast<std::size_t>(info.actuatedDofNum));
  for (pinocchio::JointIndex joint = 1; joint < pinocchio_model.njoints; ++joint) {
    if (pinocchio_model.nqs[joint] == 1) {
      joint_names.push_back(pinocchio_model.names[joint]);
    }
  }
  bolza::CentroidalModelPinocchioMapping pinocchio_mapping(info);
  bolza::PinocchioEndEffectorKinematics end_effector_kinematics(
      pinocchio_view, pinocchio_mapping, interface.GetModelSettings().contactNames3DoF);

  bolza::examples::DemoRequest request;
  request.name = "legged_robot";
  request.mpc = mpc.get();
  request.rollout = &interface.GetRollout();
  request.reference = interface.GetReferenceManagerPtr();
  request.mpc_settings = interface.MpcSettings();
  request.thread_priority = thread_priority;
  request.initial_state = interface.GetInitialState();
  request.initial_target = interface.GetInitialState();
  request.initial_input = bolza::vector_t::Zero(info.inputDim);
  request.initial_mode = bolza::legged_robot::ModeNumber::STANCE;
  request.robot_description =
      bolza::examples::RobotDescriptionWithAbsoluteMeshes(AUTOMANIP_EXAMPLE_URDF);
  request.max_steps = steps;
  // A Nav Goal is an absolute base xy/yaw. One sample at the current time
  // makes DDP demand the goal immediately and the rollout diverges. Yaw and
  // xy in the same segment make the base translate while it is still facing
  // sideways. Turn in place onto the path heading, then walk with that yaw
  // held, at the speeds in reference.info (0.5 m/s, 0.3 rad/s).
  auto switched = interface.GetSwitchedModelReferenceManagerPtr();
  request.on_gait = [switched](const std::string& name) { switched->SetGait(name); };
  request.on_terrain = [switched](const std::string& command) { switched->SetTerrainCommand(command); };
  request.command_from_state = true;
  request.on_target = [&](const bolza::examples::PoseStamped& pose,
                          const bolza::vector_t& previous, bolza::scalar_t time) {
    constexpr double kComHeight = 0.575;
    constexpr double kDisplacementVelocity = 0.5;
    constexpr double kRotationVelocity = 0.3;
    constexpr double kTurnInPlace = 0.05;
    constexpr double kMinTravel = 0.05;
    constexpr double kBearingDistance = 0.15;
    constexpr double kFinalYaw = 0.2;
    const auto wrap = [](double angle) {
      return std::atan2(std::sin(angle), std::cos(angle));
    };
    const bolza::vector_t& stance = interface.GetInitialState();
    bolza::vector_t current_pose = stance.segment<6>(6);
    if (previous.size() >= 12) {
      current_pose = previous.segment<6>(6);
    }
    const double goal_x = pose.pose().position().x();
    const double goal_y = pose.pose().position().y();
    const double dx = goal_x - current_pose(0);
    const double dy = goal_y - current_pose(1);
    const double distance = std::hypot(dx, dy);
    const double arrow_yaw = current_pose(3) + wrap(Yaw(pose) - current_pose(3));

    bolza::scalar_array_t times;
    bolza::vector_array_t states;
    bolza::vector_array_t inputs;
    const bolza::vector_t input = bolza::vector_t::Zero(info.inputDim);
    bolza::vector_t sample = stance;
    sample.head(6).setZero();
    bolza::vector_t base = current_pose;
    bolza::scalar_t stamp = time;
    auto push = [&](const bolza::vector_t& base_pose) {
      if (!times.empty() && !(stamp > times.back())) {
        stamp = times.back() + 1e-3;
      }
      sample.segment<6>(6) = base_pose;
      times.push_back(stamp);
      states.push_back(sample);
      inputs.push_back(input);
    };
    auto advance = [&](double delta, double speed) {
      stamp += std::max(0.2, std::abs(delta) / speed);
    };
    push(base);

    if (distance > kBearingDistance) {
      const double heading = current_pose(3) + wrap(std::atan2(dy, dx) - current_pose(3));
      const double turn = heading - base(3);
      if (std::abs(turn) >= kTurnInPlace) {
        advance(turn, kRotationVelocity);
        base(2) = kComHeight;
        base(3) = heading;
        base(4) = 0.0;
        base(5) = 0.0;
        push(base);
      }
      advance(distance, kDisplacementVelocity);
      base(0) = goal_x;
      base(1) = goal_y;
      base(2) = kComHeight;
      base(3) = heading;
      base(4) = 0.0;
      base(5) = 0.0;
      push(base);
      const double settle = wrap(arrow_yaw - heading);
      if (std::abs(settle) >= kFinalYaw) {
        advance(settle, kRotationVelocity);
        base(3) = heading + settle;
        push(base);
      }
    } else {
      const double turn = wrap(arrow_yaw - base(3));
      if (std::abs(turn) >= kTurnInPlace) {
        advance(turn, kRotationVelocity);
        base(2) = kComHeight;
        base(3) += turn;
        base(4) = 0.0;
        base(5) = 0.0;
        push(base);
      }
      if (distance >= kMinTravel) {
        advance(distance, kDisplacementVelocity);
        base(0) = goal_x;
        base(1) = goal_y;
        base(2) = kComHeight;
        base(4) = 0.0;
        base(5) = 0.0;
        push(base);
      }
    }
    if (states.size() < 2) {
      stamp = time + 0.2;
      base(0) = goal_x;
      base(1) = goal_y;
      base(2) = kComHeight;
      base(3) = arrow_yaw;
      base(4) = 0.0;
      base(5) = 0.0;
      push(base);
    }
    return bolza::TargetTrajectories(std::move(times), std::move(states), std::move(inputs));
  };
  request.publish = [&](const bolza::SystemObservation& observation,
                        const bolza::PrimalSolution& policy,
                        const bolza::CommandData& command,
                        bolza::examples::MarkerArray* markers,
                        bolza::examples::JointStateMsg* joints,
                        bolza::examples::PathMsg* path,
                        bolza::examples::TfMsg* tf) {
    static int terrain_tick = 0;
    if (terrain_tick++ % 60 == 0) {
      AppendTerrain(markers, interface.GetSwitchedModelReferenceManagerPtr()->TerrainCopy());
    }
    bolza::vector_t state = observation.state;
    const auto base = bolza::centroidal_model::GetBasePose(state, info);
    const auto angles = bolza::centroidal_model::getJointAngles(state, info);
    std::vector<double> positions(static_cast<std::size_t>(angles.size()));
    for (std::size_t i = 0; i < positions.size(); ++i) {
      positions[i] = angles(static_cast<Eigen::Index>(i));
    }
    bolza::examples::FillJoints(joints, joint_names, positions);
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(base(3), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(base(4), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(base(5), Eigen::Vector3d::UnitX());
    bolza::examples::AddTransform(tf, "map", "base", base(0), base(1), base(2), rotation.w(),
                                      rotation.x(), rotation.y(), rotation.z());

    const auto& trajectory = policy.stateTrajectory_;
    const std::size_t stride = trajectory.size() > 40 ? trajectory.size() / 40 : 1;
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    auto com = bolza::examples::MakeShape("CoM Trajectory", 0, Marker::LINE_STRIP, 0.0, 0.0, 0.0,
                                              0.01, 0.01, 0.01, kRed[0], kRed[1], kRed[2]);
    for (std::size_t i = 0; i < trajectory.size(); i += stride) {
      bolza::examples::AddPathPose(path, trajectory[i](6), trajectory[i](7), trajectory[i](8));
      AddPoint(&com, Eigen::Vector3d(trajectory[i](6), trajectory[i](7), trajectory[i](8)));
    }
    if (com.points_size() >= 2) {
      *markers->add_markers() = std::move(com);
    }
    const auto& desired = command.mpcTargetTrajectories_.stateTrajectory;
    auto desired_line = bolza::examples::MakeShape(
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

    auto& model = pinocchio_view.GetModel();
    auto& data = pinocchio_view.GetData();
    end_effector_kinematics.SetPinocchioInterface(pinocchio_view);
    auto feetAt = [&](const bolza::vector_t& sample) {
      const auto sample_q =
          bolza::centroidal_model::getGeneralizedCoordinates(sample, info);
      pinocchio::forwardKinematics(model, data, sample_q);
      pinocchio::updateFramePlacements(model, data);
      return end_effector_kinematics.GetPosition(sample);
    };
    constexpr std::size_t kFootCount = 4;
    if (trajectory.size() >= 2) {
      std::array<Marker, kFootCount> foot_lines;
      for (std::size_t foot = 0; foot < kFootCount; ++foot) {
        foot_lines[foot] = bolza::examples::MakeShape(
            "EE Trajectories", static_cast<int>(foot), Marker::LINE_STRIP, 0.0, 0.0, 0.0, 0.01,
            0.01, 0.01, kFootColor[foot][0], kFootColor[foot][1], kFootColor[foot][2]);
      }
      const std::size_t foot_stride = trajectory.size() > 60 ? trajectory.size() / 60 : 1;
      auto append_feet = [&](const bolza::vector_t& sample) {
        if (sample.size() < info.stateDim) {
          return;
        }
        const auto positions = feetAt(sample);
        const std::size_t n = std::min(kFootCount, positions.size());
        for (std::size_t foot = 0; foot < n; ++foot) {
          AddPoint(&foot_lines[foot], positions[foot]);
        }
      };
      for (std::size_t i = 0; i < trajectory.size(); i += foot_stride) {
        append_feet(trajectory[i]);
      }
      if ((trajectory.size() - 1) % foot_stride != 0) {
        append_feet(trajectory.back());
      }
      for (std::size_t foot = 0; foot < kFootCount; ++foot) {
        if (foot_lines[foot].points_size() >= 2) {
          *markers->add_markers() = std::move(foot_lines[foot]);
        }
      }
    }

    const auto& plan_time = policy.timeTrajectory_;
    const auto& event_times = policy.modeSchedule_.eventTimes;
    const auto& event_modes = policy.modeSchedule_.modeSequence;
    if (trajectory.size() >= 2 && plan_time.size() == trajectory.size() &&
        event_modes.size() == event_times.size() + 1) {
      std::array<Marker, kFootCount> footholds;
      for (std::size_t foot = 0; foot < kFootCount; ++foot) {
        footholds[foot] = bolza::examples::MakeShape(
            "Future footholds", static_cast<int>(foot), Marker::SPHERE_LIST, 0.0, 0.0, 0.0,
            0.03, 0.03, 0.03, kFootColor[foot][0], kFootColor[foot][1], kFootColor[foot][2]);
      }
      const auto t_start = plan_time.front();
      const auto t_end = plan_time.back();
      for (std::size_t event = 0; event < event_times.size(); ++event) {
        if (!(t_start < event_times[event] && event_times[event] < t_end)) {
          continue;
        }
        const auto before = bolza::legged_robot::ModeNumber2StanceLeg(event_modes[event]);
        const auto after = bolza::legged_robot::ModeNumber2StanceLeg(event_modes[event + 1]);
        const auto landing = bolza::LinearInterpolation::Interpolate(event_times[event],
                                                                         plan_time, trajectory);
        if (landing.size() < info.stateDim) {
          continue;
        }
        const auto positions = feetAt(landing);
        const std::size_t n = std::min(kFootCount, positions.size());
        for (std::size_t foot = 0; foot < n; ++foot) {
          if (!before[foot] && after[foot]) {
            AddPoint(&footholds[foot], positions[foot]);
          }
        }
      }
      for (std::size_t foot = 0; foot < kFootCount; ++foot) {
        if (footholds[foot].points_size() > 0) {
          *markers->add_markers() = std::move(footholds[foot]);
        }
      }
    }

    const auto feet = feetAt(state);
    const auto contact = bolza::legged_robot::ModeNumber2StanceLeg(observation.mode);
    const auto input = observation.input;
    const std::size_t contacts = std::min(feet.size(), contact.size());

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double force_z = 0.0;
    int stance_count = 0;
    for (std::size_t i = 0; i < contacts; ++i) {
      const Eigen::Vector3d foot = feet[i];
      auto sphere = bolza::examples::MakeShape(
          "EE Positions", static_cast<int>(i), Marker::SPHERE, foot.x(), foot.y(), foot.z(), 0.03,
          0.03, 0.03, kFootColor[i][0], kFootColor[i][1], kFootColor[i][2]);
      if (!contact[i]) {
        sphere.mutable_color()->set_a(0.3f);
      }
      *markers->add_markers() = std::move(sphere);

      const Eigen::Vector3d force =
          bolza::centroidal_model::GetContactForces(input, i, info);
      if (contact[i]) {
        const Eigen::Vector3d arrow = force / 1000.0;
        const double length = arrow.norm();
        if (length > 1e-4) {
          const Eigen::Quaterniond aim =
              Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), arrow / length);
          const Eigen::Vector3d start = foot - arrow;
          *markers->add_markers() = bolza::examples::MakeShape(
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
      *markers->add_markers() = bolza::examples::MakeShape(
          "Center of Pressure", 0, Marker::SPHERE, center.x(), center.y(), center.z(), 0.03, 0.03,
          0.03, kGreen[0], kGreen[1], kGreen[2]);
    }
    auto polygon = bolza::examples::MakeShape("Support Polygon", 0, Marker::LINE_LIST, 0.0, 0.0,
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
  return bolza::examples::RunDemo(request);
}
