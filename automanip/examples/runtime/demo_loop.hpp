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
 * @file demo_loop.hpp
 * @brief MPC + rollout loop. Targets and visualization travel on Autolink.
 */

#ifndef AUTOMANIP_EXAMPLES_RUNTIME_DEMO_LOOP_HPP_
#define AUTOMANIP_EXAMPLES_RUNTIME_DEMO_LOOP_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "bolza/core/reference/target_trajectories.hpp"
#include "bolza/core/types.hpp"
#include "bolza/mpc/command_data.hpp"
#include "bolza/mpc/mpc_base.hpp"
#include "bolza/mpc/system_observation.hpp"
#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/rollout/rollout_base.hpp"
#include "bolza/oc/synchronized_module/reference_manager_interface.hpp"

namespace bolza {
namespace examples {

using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
using TwistStampedMsg = automsgs::msgs::geometry_msgs::TwistStamped;
using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;
using JointStateMsg = automsgs::msgs::sensor_msgs::JointState;
using PathMsg = automsgs::msgs::nav_msgs::Path;
using TfMsg = automsgs::msgs::tf2_msgs::TFMessage;

/** Reads a URDF (or any text file). Empty when the path cannot be opened. */
inline std::string ReadTextFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    return {};
  }
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

inline void ReplaceAll(std::string* text, const std::string& from, const std::string& to) {
  if (text == nullptr || from.empty()) {
    return;
  }
  for (std::string::size_type pos = 0; (pos = text->find(from, pos)) != std::string::npos;) {
    text->replace(pos, from.size(), to);
    pos += to.size();
  }
}

/**
 * Rewrites package and relative mesh URIs to absolute paths under the
 * robotic-assets tree. Robot descriptions arrive on a topic, so autoviz has
 * no URDF directory and cannot resolve `package://`.
 */
inline std::string RobotDescriptionWithAbsoluteMeshes(const std::string& urdf_path) {
  std::string xml = ReadTextFile(urdf_path);
#ifdef AUTOMANIP_ROBOTIC_ASSETS
  const std::string root = std::string(AUTOMANIP_ROBOTIC_ASSETS) + "/";
  ReplaceAll(&xml, "package://ocs2_robotic_assets/resources/", root);
  ReplaceAll(&xml, "file://$(find ocs2_robotic_assets)/resources/", root);
#endif
  const std::filesystem::path mesh_dir =
      std::filesystem::path(urdf_path).parent_path() / ".." / "meshes";
  if (std::filesystem::exists(mesh_dir)) {
    const std::string absolute =
        std::filesystem::weakly_canonical(mesh_dir).string() + "/";
    ReplaceAll(&xml, "../meshes/", absolute);
  }
  return xml;
}

/** Fills markers, joints, the predicted path, and the TF tree. */
using ViewPublisher = std::function<void(const SystemObservation& observation,
                                         const PrimalSolution& policy,
                                         const CommandData& command,
                                         MarkerArray* markers,
                                         JointStateMsg* joints, PathMsg* path,
                                         TfMsg* tf)>;

/**
 * Builds a new target from a PoseStamped on `/<name>/target_pose`.
 * The previous target state is passed so unused coordinates can stay put.
 */
using TargetFromPose = std::function<TargetTrajectories(
    const PoseStamped& pose, const vector_t& previous_target, scalar_t time)>;

/** Fills `/cmd_vel` (`geometry_msgs/TwistStamped`) from the live state. */
using VelocityPublisher =
    std::function<void(const SystemObservation& observation, TwistStampedMsg* twist)>;

struct DemoRequest {
  std::string name;
  MPCBase* mpc = nullptr;
  const RolloutBase* rollout = nullptr;
  std::shared_ptr<ReferenceManagerInterface> reference;
  mpc::Settings mpc_settings;
  int thread_priority = 0;
  vector_t initial_state;
  vector_t initial_target;
  vector_t initial_input;
  /** Initial contact mode. Legged robots start in stance (15). */
  std::size_t initial_mode = 0;
  ViewPublisher publish;
  TargetFromPose on_target;
  /** URDF XML published on `/robot_description` for a RobotModel display. */
  std::string robot_description;
  /**
   * Replaces the single-point target built from `initial_target`.
   * Ballbot uses the official two-point command (current pose, then goal).
   */
  TargetTrajectories initial_trajectories;
  /**
   * When set, a new pose starts from the live state instead of the last goal.
   * Ballbot needs this so a Nav Goal click tracks from where the robot is.
   */
  bool command_from_state = false;
  /**
   * When set, each rollout step publishes this twist on `/cmd_vel`.
   * Ballbot reports planar speed and yaw rate so autoviz can draw it.
   */
  VelocityPublisher velocity;
  /** String on `/<name>/gait`. Legged robots select a gait from gait.info. */
  std::function<void(const std::string&)> on_gait;
  /** String on `/<name>/terrain`: `flat`, `stairs`, or a height-file path. */
  std::function<void(const std::string&)> on_terrain;
  /** 0 runs until Ctrl+C. */
  int max_steps = 0;
};

/**
 * Runs Gauss-Newton MPC and a rollout, and publishes
 * `/joint_states`, `/tf`, `/robot_description`, plus the same streams
 * under `/<name>/`. When `velocity` is set, also publishes `/cmd_vel`.
 * Subscribes to `/<name>/target_pose` and `/goal_pose`
 * (autoviz Nav Goal).
 */
int RunDemo(const DemoRequest& request);

}  // namespace examples
}  // namespace bolza

#endif  // AUTOMANIP_EXAMPLES_RUNTIME_DEMO_LOOP_HPP_
