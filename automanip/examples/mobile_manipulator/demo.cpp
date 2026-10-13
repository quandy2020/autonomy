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
 * @brief Mobile-manipulator MPC for the robots in bolza/examples/assets.
 *
 * Selects Franka Panda, Willow Garage PR2, Clearpath Ridgeback with UR5,
 * Kinova Jaco2 (j2n6s300 / j2n7s300), or Mabi-Mobile. PoseStamped is the
 * end-effector target. No ROS node graph: visualization goes out on Autolink.
 */

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "bolza/core/misc/load_data.hpp"
#include "bolza/ddp/gauss_newton_ddp_mpc.hpp"
#include "examples/mobile_manipulator/access_helper_functions.hpp"
#include "examples/mobile_manipulator/mobile_manipulator_interface.hpp"
#include "examples/runtime/demo_loop.hpp"
#include "examples/runtime/markers.hpp"

#include <automsgs/msgs/visualization_msgs/interactive_marker.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_control.pb.h>
#include <automsgs/msgs/visualization_msgs/menu_entry.pb.h>

namespace {

struct RobotChoice {
  const char* key;
  const char* task;
  const char* urdf;
};

constexpr RobotChoice kRobots[] = {
    {"franka", "config/franka/task.info",
     "mobile_manipulator/franka/urdf/panda.urdf"},
    {"panda", "config/franka/task.info",
     "mobile_manipulator/franka/urdf/panda.urdf"},
    {"pr2", "config/pr2/task.info", "mobile_manipulator/pr2/urdf/pr2.urdf"},
    {"ridgeback", "config/ridgeback_ur5/task.info",
     "mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf"},
    {"ridgeback_ur5", "config/ridgeback_ur5/task.info",
     "mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf"},
    {"kinova", "config/kinova/task_j2n6.info",
     "mobile_manipulator/kinova/urdf/j2n6s300.urdf"},
    {"j2n6", "config/kinova/task_j2n6.info",
     "mobile_manipulator/kinova/urdf/j2n6s300.urdf"},
    {"j2n6s300", "config/kinova/task_j2n6.info",
     "mobile_manipulator/kinova/urdf/j2n6s300.urdf"},
    {"j2n7", "config/kinova/task_j2n7.info",
     "mobile_manipulator/kinova/urdf/j2n7s300.urdf"},
    {"j2n7s300", "config/kinova/task_j2n7.info",
     "mobile_manipulator/kinova/urdf/j2n7s300.urdf"},
    {"mabi", "config/mabi_mobile/task.info",
     "mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf"},
    {"mabi_mobile", "config/mabi_mobile/task.info",
     "mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf"},
};

void PrintUsage() {
  std::cerr << "Usage: mobile_manipulator_demo [robot] [steps]\n"
            << "  robot: franka (default), pr2, ridgeback, kinova (j2n6), "
               "j2n7, mabi\n"
            << "  steps: 0 runs until interrupted\n";
}

const RobotChoice* FindRobot(const std::string& key) {
  for (const RobotChoice& robot : kRobots) {
    if (key == robot.key) {
      return &robot;
    }
  }
  return nullptr;
}

struct Options {
  const RobotChoice* robot = &kRobots[0];
  int steps = 0;
};

Options ParseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    char* end = nullptr;
    const long value = std::strtol(argv[i], &end, 10);
    if (end != argv[i] && *end == '\0') {
      options.steps = static_cast<int>(value);
      continue;
    }
    options.robot = FindRobot(argv[i]);
    if (options.robot == nullptr) {
      PrintUsage();
      std::exit(1);
    }
  }
  return options;
}

std::string ResolveRobotUrdf(const std::string& relative) {
  namespace fs = std::filesystem;
#ifdef AUTOMANIP_ROBOTIC_ASSETS
  const fs::path candidate = fs::path(AUTOMANIP_ROBOTIC_ASSETS) / relative;
  if (fs::exists(candidate)) {
    return fs::weakly_canonical(candidate).string();
  }
#endif
  return bolza::examples::ResolveExampleUrdf("", relative);
}

void AddAxisControl(
    automsgs::msgs::visualization_msgs::InteractiveMarker* marker,
    const std::string& name, double w, double x, double y, double z,
    automsgs::msgs::visualization_msgs::InteractiveMarkerControl::InteractionMode
        mode) {
  auto* control = marker->add_controls();
  control->set_name(name);
  control->mutable_orientation()->set_w(w);
  control->mutable_orientation()->set_x(x);
  control->mutable_orientation()->set_y(y);
  control->mutable_orientation()->set_z(z);
  control->set_interaction_mode(mode);
}

/**
 * 6-DoF goal marker in the `map` frame, matching OCS2
 * `TargetTrajectoriesInteractiveMarker`.
 * Dragging only moves the marker. Menu entry 1, "Send target pose",
 * commits that pose and the arm starts moving.
 */
std::shared_ptr<automsgs::msgs::visualization_msgs::InteractiveMarker>
MakeGoalInteractiveMarker() {
  using Control = automsgs::msgs::visualization_msgs::InteractiveMarkerControl;
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  auto marker =
      std::make_shared<automsgs::msgs::visualization_msgs::InteractiveMarker>();
  bolza::examples::StampHeader(marker->mutable_header(), "map");
  marker->mutable_pose()->mutable_position()->set_z(1.0);
  marker->mutable_pose()->mutable_orientation()->set_w(1.0);
  marker->set_name("Goal");
  marker->set_description("Right click to send command");
  // OCS2 uses 0.2. That leaves the rings and the hub only a few centimeters
  // across, which is hard to grab. 1 m is the usual 6-DoF handle size.
  marker->set_scale(1.0f);

  auto* menu = marker->add_menu_entries();
  menu->set_id(1);
  menu->set_title("Send target pose");
  menu->set_command_type(
      automsgs::msgs::visualization_msgs::MenuEntry::FEEDBACK);

  auto* box = marker->add_controls();
  box->set_always_visible(true);
  box->set_interaction_mode(Control::MOVE_ROTATE_3D);
  auto* cube = box->add_markers();
  cube->set_type(Marker::CUBE);
  cube->mutable_scale()->set_x(0.25);
  cube->mutable_scale()->set_y(0.25);
  cube->mutable_scale()->set_z(0.25);
  // Opaque dark-gray hub (RViz basic_controls / 6-DoF reference look).
  cube->mutable_color()->set_r(0.45f);
  cube->mutable_color()->set_g(0.45f);
  cube->mutable_color()->set_b(0.45f);
  cube->mutable_color()->set_a(1.0f);
  cube->mutable_pose()->mutable_orientation()->set_w(1.0);

  // OCS2 stores (w,x,y,z) = (1,1,0,0) and the two cyclic permutations.
  // Those are the same rotations as these unit quaternions.
  constexpr double kHalf = 0.7071067811865476;
  AddAxisControl(marker.get(), "rotate_x", kHalf, kHalf, 0.0, 0.0,
                 Control::ROTATE_AXIS);
  AddAxisControl(marker.get(), "move_x", kHalf, kHalf, 0.0, 0.0,
                 Control::MOVE_AXIS);
  AddAxisControl(marker.get(), "rotate_z", kHalf, 0.0, kHalf, 0.0,
                 Control::ROTATE_AXIS);
  AddAxisControl(marker.get(), "move_z", kHalf, 0.0, kHalf, 0.0,
                 Control::MOVE_AXIS);
  AddAxisControl(marker.get(), "rotate_y", kHalf, 0.0, 0.0, kHalf,
                 Control::ROTATE_AXIS);
  AddAxisControl(marker.get(), "move_y", kHalf, 0.0, 0.0, kHalf,
                 Control::MOVE_AXIS);
  return marker;
}

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

void AddLineStrip(bolza::examples::MarkerArray* markers, const std::string& ns,
                  int id, const std::vector<Eigen::Vector3d>& points, float r,
                  float g, float b) {
  if (points.size() < 2) {
    return;
  }
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  Marker marker;
  bolza::examples::StampHeader(marker.mutable_header(), "map");
  marker.set_ns(ns);
  marker.set_id(id);
  marker.set_type(Marker::LINE_STRIP);
  marker.set_action(Marker::ADD);
  marker.mutable_scale()->set_x(0.005);
  marker.mutable_color()->set_r(r);
  marker.mutable_color()->set_g(g);
  marker.mutable_color()->set_b(b);
  marker.mutable_color()->set_a(1.0f);
  marker.mutable_pose()->mutable_orientation()->set_w(1.0);
  const std::size_t stride = std::max<std::size_t>(1, points.size() / 40);
  for (std::size_t i = 0; i < points.size(); i += stride) {
    auto* point = marker.add_points();
    point->set_x(points[i].x());
    point->set_y(points[i].y());
    point->set_z(points[i].z());
  }
  *markers->add_markers() = std::move(marker);
}

void AddPlacement(bolza::examples::TfMsg* tf, const std::string& parent,
                  const std::string& child, const pinocchio::SE3& pose) {
  const Eigen::Vector3d& translation = pose.translation();
  Eigen::Quaterniond rotation(pose.rotation());
  rotation.normalize();
  bolza::examples::AddTransform(tf, parent, child, translation.x(),
                                translation.y(), translation.z(), rotation.w(),
                                rotation.x(), rotation.y(), rotation.z());
}

/**
 * True when `frame` is the child link of a joint that still has configuration.
 * Fixed joints (cameras, shells, and joints listed in `removeJoints`) do not.
 * RobotModel still poses those links from joint states and `map` → `base`.
 */
bool IsMovingBody(const pinocchio::Model& model, const pinocchio::Frame& frame) {
  if ((frame.type & pinocchio::FrameType::BODY) == 0) {
    return false;
  }
  const pinocchio::JointIndex joint = frame.parentJoint;
  if (joint == 0 || joint >= static_cast<pinocchio::JointIndex>(model.njoints)) {
    return false;
  }
  return model.joints[joint].nq() > 0;
}

/** Tool frame named in the task file. Joint frames with the same name are skipped. */
bool IsToolFrame(const pinocchio::Frame& frame, const std::string& end_effector) {
  if (end_effector.empty() || frame.name != end_effector) {
    return false;
  }
  const auto type = frame.type;
  return (type & pinocchio::FrameType::BODY) != 0 || (type & pinocchio::FrameType::OP_FRAME) != 0;
}

/**
 * Publishes `map` → `base_frame`, each moving link, and the end effector.
 *
 * Every kept link is placed from forward kinematics relative to the base, so
 * the joint angle is included. Cameras, covers, and locked joints are omitted;
 * each of those would otherwise draw its own coordinate triad.
 */
bool PublishModelFrames(const pinocchio::Model& model, const bolza::vector_t& state,
                        bolza::examples::TfMsg* tf, const std::string& base_frame,
                        const std::string& end_effector) {
  if (tf == nullptr || base_frame.empty() || state.size() != model.nq || model.nframes < 2) {
    return false;
  }
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);

  pinocchio::FrameIndex base_index = static_cast<pinocchio::FrameIndex>(model.nframes);
  for (pinocchio::FrameIndex i = 1; i < static_cast<pinocchio::FrameIndex>(model.nframes); ++i) {
    const pinocchio::Frame& frame = model.frames[i];
    if (frame.name == base_frame && (frame.type & pinocchio::FrameType::BODY) != 0) {
      base_index = i;
      break;
    }
  }
  if (base_index >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
    return false;
  }

  AddPlacement(tf, "map", base_frame, data.oMf[base_index]);
  std::unordered_set<std::string> published;
  published.reserve(static_cast<std::size_t>(model.nframes));
  published.insert(base_frame);
  for (pinocchio::FrameIndex i = 1; i < static_cast<pinocchio::FrameIndex>(model.nframes); ++i) {
    const pinocchio::Frame& frame = model.frames[i];
    if (frame.name.empty() || frame.name == "universe" || frame.name == "command") {
      continue;
    }
    if (!IsMovingBody(model, frame) && !IsToolFrame(frame, end_effector)) {
      continue;
    }
    if (!published.insert(frame.name).second) {
      continue;
    }
    const pinocchio::SE3 relative = data.oMf[base_index].inverse() * data.oMf[i];
    AddPlacement(tf, base_frame, frame.name, relative);
  }
  return true;
}

Eigen::Vector3d EndEffectorPosition(const pinocchio::Model& model,
                                    const std::string& frame,
                                    const bolza::vector_t& state) {
  if (state.size() != model.nq || !model.existFrame(frame)) {
    return Eigen::Vector3d::Zero();
  }
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);
  return data.oMf[model.getFrameId(frame)].translation();
}

/** Position plus quaternion (x, y, z, qx, qy, qz, qw) of `frame` at `state`. */
bolza::vector_t EndEffectorPose(const pinocchio::Model& model,
                                const std::string& frame,
                                const bolza::vector_t& state) {
  bolza::vector_t pose(7);
  pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  if (state.size() != model.nq || !model.existFrame(frame)) {
    return pose;
  }
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);
  const pinocchio::SE3& placement = data.oMf[model.getFrameId(frame)];
  const Eigen::Quaterniond rotation(placement.rotation());
  pose.head<3>() = placement.translation();
  pose.tail<4>() = rotation.coeffs();
  return pose;
}

/**
 * Time for the end effector to travel from `start` to `goal`.
 * A single sample at the current time makes the solver spend the whole
 * horizon on the goal, so Kinova's 15 rad/s limits empty the motion in a
 * fraction of a second.
 */
double TravelDuration(const bolza::vector_t& start, const bolza::vector_t& goal) {
  constexpr double kLinearSpeed = 0.8;
  constexpr double kAngularSpeed = 2.0;
  constexpr double kMinDuration = 0.2;
  const double distance = (goal.head<3>() - start.head<3>()).norm();
  const Eigen::Quaterniond from(start(6), start(3), start(4), start(5));
  const Eigen::Quaterniond to(goal(6), goal(3), goal(4), goal(5));
  const double angle = from.normalized().angularDistance(to.normalized());
  return std::max(kMinDuration, std::max(distance / kLinearSpeed, angle / kAngularSpeed));
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = ParseArgs(argc, argv);
  const std::string root = AUTOMANIP_MOBILE_MANIPULATOR_DIR;
  const std::string task_file = root + "/" + options.robot->task;
  const std::string library =
      std::string("/tmp/automanip/mobile_manipulator/") + options.robot->key;
  const std::string urdf_path = ResolveRobotUrdf(options.robot->urdf);

  std::cerr << "[mobile_manipulator] robot " << options.robot->key
            << " task " << task_file << " urdf " << urdf_path << "\n";

  bolza::mobile_manipulator::MobileManipulatorInterface interface(
      task_file, library, urdf_path);
  const auto& model = interface.GetManipulatorModelInfo();

  std::vector<std::string> fixed_joints;
  bolza::loadData::LoadStdVector<std::string>(
      task_file, "model_information.removeJoints", fixed_joints, false);

  bolza::GaussNewtonDDPMPC mpc(interface.MpcSettings(), interface.DdpSettings(),
                               interface.GetRollout(),
                               interface.GetOptimalControlProblem(),
                               interface.GetInitializer());
  // Same initial end-effector command as the upstream dummy node.
  bolza::vector_t initial_target(7);
  initial_target << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  const pinocchio::Model pinocchio_model =
      interface.GetPinocchioInterface().GetModel();

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
  request.robot_description = bolza::examples::LoadExampleRobotDescription(
      urdf_path, options.robot->urdf);
  if (request.robot_description.empty()) {
    std::cerr << "[mobile_manipulator] Failed to load URDF " << urdf_path
              << "\n";
  }
  request.max_steps = options.steps;
  request.goal_interactive_marker = MakeGoalInteractiveMarker();
  // `previous` is the live joint state, so the reference can start at the
  // end effector instead of the dummy (1, 0, 1) pose.
  request.command_from_state = true;
  request.on_target = [&](const bolza::examples::PoseStamped& pose,
                          const bolza::vector_t& previous, bolza::scalar_t time) {
    const bolza::vector_t goal = EndEffectorTarget(pose);
    const bolza::vector_t input = bolza::vector_t::Zero(model.inputDim);
    const bolza::vector_t start = EndEffectorPose(pinocchio_model, model.eeFrame, previous);
    const double duration = TravelDuration(start, goal);
    return bolza::TargetTrajectories({time, time + duration}, {start, goal}, {input, input});
  };
  request.publish = [&](const bolza::SystemObservation& observation,
                        const bolza::PrimalSolution& policy,
                        const bolza::CommandData& command,
                        bolza::examples::MarkerArray* markers,
                        bolza::examples::JointStateMsg* joints,
                        bolza::examples::PathMsg* path,
                        bolza::examples::TfMsg* tf) {
    const auto arm =
        bolza::mobile_manipulator::getArmJointAngles(observation.state, model);
    std::vector<std::string> names = model.dofNames;
    std::vector<double> positions(static_cast<std::size_t>(arm.size()));
    for (std::size_t i = 0; i < positions.size(); ++i) {
      positions[i] = arm(static_cast<Eigen::Index>(i));
    }
    for (const std::string& name : fixed_joints) {
      names.push_back(name);
      positions.push_back(0.0);
    }
    bolza::examples::FillJoints(joints, names, positions);

    if (!PublishModelFrames(pinocchio_model, observation.state, tf, model.baseFrame,
                            model.eeFrame)) {
      const auto base =
          bolza::mobile_manipulator::GetBasePosition(observation.state, model);
      const auto rotation = bolza::mobile_manipulator::GetBaseOrientation(
          observation.state, model);
      bolza::examples::AddTransform(tf, "map", model.baseFrame, base.x(),
                                    base.y(), base.z(), rotation.w(),
                                    rotation.x(), rotation.y(), rotation.z());
    }

    bolza::vector_t goal = request.initial_target;
    if (!command.mpcTargetTrajectories_.stateTrajectory.empty()) {
      goal = command.mpcTargetTrajectories_.stateTrajectory.back();
    }
    bolza::examples::AddTransform(tf, "map", "command", goal(0), goal(1),
                                  goal(2), goal(6), goal(3), goal(4), goal(5));

    using Marker = automsgs::msgs::visualization_msgs::Marker;
    *markers->add_markers() = bolza::examples::MakeShape(
        "target", 0, Marker::SPHERE, goal(0), goal(1), goal(2), 0.05, 0.05,
        0.05, 0.1f, 0.8f, 0.2f, goal(6), goal(3), goal(4), goal(5));

    std::vector<Eigen::Vector3d> base_line;
    std::vector<Eigen::Vector3d> ee_line;
    const auto& states = policy.stateTrajectory_;
    const std::size_t stride =
        std::max<std::size_t>(1, states.size() / 40);
    for (std::size_t i = 0; i < states.size(); i += stride) {
      const bolza::vector_t& state = states[i];
      if (state.size() != static_cast<Eigen::Index>(model.stateDim)) {
        continue;
      }
      const auto point =
          bolza::mobile_manipulator::GetBasePosition(state, model);
      base_line.emplace_back(point.x(), point.y(), point.z());
      bolza::examples::AddPathPose(path, point.x(), point.y(), point.z());
      ee_line.push_back(
          EndEffectorPosition(pinocchio_model, model.eeFrame, state));
    }
    AddLineStrip(markers, "base_trajectory", 1, base_line, 0.635f, 0.078f,
                 0.184f);
    AddLineStrip(markers, "ee_trajectory", 2, ee_line, 0.0f, 0.447f, 0.741f);
  };
  return bolza::examples::RunDemo(request);
}
