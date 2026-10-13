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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker.pb.h>
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

/**
 * Resolves a robot URDF path for demos. Prefers @p compile_time_urdf when the
 * file exists; otherwise tries compile-time @c AUTOMANIP_ROBOTIC_ASSETS and the
 * env vars @c BOLZA_ROBOTIC_ASSETS / @c AUTOMANIP_ROBOTIC_ASSETS /
 * @c OCS2_ROBOTIC_ASSETS plus @p robot_relative
 * (e.g. @c "ballbot/urdf/ballbot.urdf").
 */
inline std::string ResolveExampleUrdf(const std::string& compile_time_urdf,
                                      const std::string& robot_relative) {
  namespace fs = std::filesystem;
  if (!compile_time_urdf.empty() && fs::exists(compile_time_urdf)) {
    return fs::weakly_canonical(compile_time_urdf).string();
  }
  auto try_root = [&](const fs::path& root) -> std::string {
    if (root.empty()) {
      return {};
    }
    const fs::path candidate = root / robot_relative;
    if (fs::exists(candidate)) {
      return fs::weakly_canonical(candidate).string();
    }
    return {};
  };
#ifdef AUTOMANIP_ROBOTIC_ASSETS
  if (const std::string found = try_root(AUTOMANIP_ROBOTIC_ASSETS); !found.empty()) {
    return found;
  }
#endif
  for (const char* key : {"BOLZA_ROBOTIC_ASSETS", "AUTOMANIP_ROBOTIC_ASSETS",
                          "OCS2_ROBOTIC_ASSETS"}) {
    if (const char* root = std::getenv(key)) {
      if (const std::string found = try_root(root); !found.empty()) {
        return found;
      }
    }
  }
  // Source-tree fallback next to this header's assets sibling is handled by
  // callers that know bolza::robotic_assets::GetPath(); keep compile path last.
  return compile_time_urdf;
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
 * Loads a URDF for `/robot_description`. Mesh URIs are rewritten to absolute
 * filesystem paths when the sibling `../meshes/` directory exists, so Autoviz
 * (and RViz with a file-backed description) can load assets without
 * `resource_retriever` / ament package discovery.
 *
 * Handles both flat robots (@c ballbot/meshes) and nested ones
 * (@c mobile_manipulator/franka/meshes) by rewriting
 * @c package://ocs2_robotic_assets/resources/<rel>/meshes/ when an assets root
 * is known. Remaining package URIs are left for MeshResourceResolver.
 */
inline std::string RobotDescriptionWithAbsoluteMeshes(const std::string& urdf_path) {
  namespace fs = std::filesystem;
  if (urdf_path.empty() || !fs::exists(urdf_path)) {
    return {};
  }
  std::string xml = ReadTextFile(urdf_path);
  if (xml.empty() || xml.find("<robot") == std::string::npos) {
    return {};
  }
  // Normalize file://$(find pkg)/… → package://pkg/… (RViz / rosrun style).
  ReplaceAll(&xml, "file://$(find ocs2_robotic_assets)/",
             "package://ocs2_robotic_assets/");
  const fs::path mesh_dir =
      fs::path(urdf_path).parent_path() / ".." / "meshes";
  if (!fs::exists(mesh_dir)) {
    return xml;
  }
  const std::string absolute = fs::weakly_canonical(mesh_dir).string() + "/";
  ReplaceAll(&xml, "../meshes/", absolute);
  // package://ocs2_robotic_assets/resources/<rel>/meshes/<file>
  // → absolute sibling meshes/ (source-tree / bolza assets layout).
  const fs::path robot_dir = fs::path(urdf_path).parent_path().parent_path();
  auto rewrite_package = [&](const fs::path& assets_root) {
    if (assets_root.empty()) {
      return false;
    }
    std::error_code ec;
    const fs::path rel = fs::relative(robot_dir, assets_root, ec);
    if (ec || rel.empty() || *rel.begin() == "..") {
      return false;
    }
    ReplaceAll(&xml,
               "package://ocs2_robotic_assets/resources/" + rel.generic_string() +
                   "/meshes/",
               absolute);
    return true;
  };
#ifdef AUTOMANIP_ROBOTIC_ASSETS
  rewrite_package(AUTOMANIP_ROBOTIC_ASSETS);
#endif
  for (const char* key : {"BOLZA_ROBOTIC_ASSETS", "AUTOMANIP_ROBOTIC_ASSETS",
                          "OCS2_ROBOTIC_ASSETS"}) {
    if (const char* root = std::getenv(key)) {
      rewrite_package(root);
    }
  }
  // Flat layout fallback: resources/<robot_leaf>/meshes/
  const std::string robot_name = robot_dir.filename().string();
  if (!robot_name.empty()) {
    ReplaceAll(&xml,
               "package://ocs2_robotic_assets/resources/" + robot_name +
                   "/meshes/",
               absolute);
  }
  return xml;
}

/**
 * Resolve @p robot_relative under assets roots, then load URDF with absolute
 * mesh paths for `/robot_description`.
 */
inline std::string LoadExampleRobotDescription(
    const std::string& compile_time_urdf, const std::string& robot_relative) {
  return RobotDescriptionWithAbsoluteMeshes(
      ResolveExampleUrdf(compile_time_urdf, robot_relative));
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
  /**
   * Goal marker published on `/simple_marker/update`.
   * Dragging updates the latched pose only. Menu entry "Send target pose"
   * (id 1) applies `on_target` and then the rollout starts, matching
   * OCS2 `TargetTrajectoriesInteractiveMarker`.
   */
  std::shared_ptr<automsgs::msgs::visualization_msgs::InteractiveMarker>
      goal_interactive_marker;
  /** 0 runs until Ctrl+C. */
  int max_steps = 0;
};

/**
 * Runs Gauss-Newton MPC and a rollout, and publishes
 * `/joint_states`, `/tf`, `/robot_description`, plus the same streams
 * under `/<name>/`. When `velocity` is set, also publishes `/cmd_vel`.
 * Subscribes to `/<name>/target_pose` and `/goal_pose`
 * (autoviz Nav Goal). When `goal_interactive_marker` is set, also publishes
 * `/simple_marker/update` and subscribes to `/simple_marker/feedback`.
 */
int RunDemo(const DemoRequest& request);

}  // namespace examples
}  // namespace bolza

#endif  // AUTOMANIP_EXAMPLES_RUNTIME_DEMO_LOOP_HPP_
