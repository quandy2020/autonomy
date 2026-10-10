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
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "automanip/core/reference/target_trajectories.hpp"
#include "automanip/core/types.hpp"
#include "automanip/mpc/command_data.hpp"
#include "automanip/mpc/mpc_base.hpp"
#include "automanip/mpc/system_observation.hpp"
#include "automanip/oc/oc_data/primal_solution.hpp"
#include "automanip/oc/rollout/rollout_base.hpp"
#include "automanip/oc/synchronized_module/reference_manager_interface.hpp"

namespace automanip {
namespace examples {

using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
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

/**
 * Rewrites `../meshes/` in a URDF to an absolute directory beside the file.
 * Robot descriptions arrive on a topic, so autoviz has no URDF directory.
 */
inline std::string RobotDescriptionWithAbsoluteMeshes(const std::string& urdf_path) {
  std::string xml = ReadTextFile(urdf_path);
  const std::filesystem::path mesh_dir = std::filesystem::weakly_canonical(
      std::filesystem::path(urdf_path).parent_path() / ".." / "meshes");
  const std::string absolute = mesh_dir.string() + "/";
  const std::string token = "../meshes/";
  for (std::string::size_type pos = 0; (pos = xml.find(token, pos)) != std::string::npos;) {
    xml.replace(pos, token.size(), absolute);
    pos += absolute.size();
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

struct DemoRequest {
  std::string name;
  MPC_BASE* mpc = nullptr;
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
  /** 0 runs until Ctrl+C. */
  int max_steps = 0;
};

/**
 * Runs Gauss-Newton MPC and a rollout, and publishes
 * `/joint_states`, `/tf`, `/robot_description`, plus the same streams
 * under `/<name>/`.
 * Subscribes to `/<name>/target_pose`.
 */
int RunDemo(const DemoRequest& request);

}  // namespace examples
}  // namespace automanip

#endif  // AUTOMANIP_EXAMPLES_RUNTIME_DEMO_LOOP_HPP_
