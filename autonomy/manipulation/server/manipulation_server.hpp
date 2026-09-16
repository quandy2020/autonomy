/*
 * Copyright 2026 The Openbot Authors
 *
 * Runtime server (MoveIt move_group analogue) — capability container.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "autolink/node/node.hpp"
#include "autonomy/manipulation/execution/joint_state_subscriber.hpp"
#include "autonomy/manipulation/execution/trajectory_execution_manager.hpp"
#include "autonomy/manipulation/core/simple_robot_model.hpp"
#include "autonomy/manipulation/kinematics/kinematics_base.hpp"
#include "autonomy/manipulation/planning/planning_pipeline.hpp"
#include "autonomy/manipulation/planning/planner_base.hpp"
#include "autonomy/manipulation/proto/manipulation_options.pb.h"
#include "autonomy/manipulation/scene/planning_scene.hpp"
#include "autonomy/manipulation/scene/scene_monitor.hpp"
#include "autonomy/manipulation/server/capability.hpp"
#include "autonomy/manipulation/servo/servo.hpp"

namespace autonomy {
namespace manipulation {

class ManipulationActionServer;

/**
 * @brief Runtime manipulation server (MoveIt move_group analogue).
 *
 * Owns model, scene, kinematics, planning pipeline, execution manager,
 * action server, and capability plugins.
 */
class ManipulationServer {
 public:
  /** @brief Construct an uninitialized server. */
  ManipulationServer() = default;

  /** @brief Stop the server and release owned resources. */
  ~ManipulationServer();

  /**
   * @brief Load options, resolve URDF, and construct plugins / subsystems.
   * @param[in] options Proto config (defaults may be loaded from conf files).
   * @return true on successful initialization.
   */
  bool Init(const proto::ManipulationOptions& options = {});

  /**
   * @brief Start Autolink node, scene monitor, action server, and servo.
   * @return true if the server is running.
   */
  bool Start();

  /** @brief Stop subsystems and clear the running flag. */
  void Stop();

  /**
   * @brief Run the planning pipeline for @p request.
   * @param[in] request Motion plan request (group, goals, planner id, …).
   * @return Planner response with trajectory and error code.
   */
  planning::MotionPlanResponse Plan(
      const planning::MotionPlanRequest& request);

  /**
   * @brief Execute a trajectory via TrajectoryExecutionManager.
   * @param[in] trajectory Joint-space waypoints to follow.
   * @param[in] replace If true, preempt an in-flight goal; otherwise fail if busy.
   * @return Error code from the execution manager.
   */
  ErrorCode ExecuteTrajectory(const core::RobotTrajectory& trajectory,
                              bool replace = false);

  /**
   * @brief Execute a trajectory; returns success as bool.
   * @param[in] trajectory Joint-space waypoints to follow.
   * @param[in] replace If true, preempt an in-flight goal; otherwise fail if busy.
   * @return true if execution completed with ErrorCode::kSuccess.
   */
  bool Execute(const core::RobotTrajectory& trajectory, bool replace = false);

  /** @brief Cancel in-flight trajectory execution. */
  void CancelExecution();

  /** @brief Planning pipeline, or nullptr if not initialized. */
  planning::PlanningPipeline* pipeline();

  /** @brief Planning scene, or nullptr if not initialized. */
  scene::PlanningScene* scene();

  /** @brief Robot model, or nullptr if not initialized. */
  core::SimpleRobotModel* model();

  /** @brief Active kinematics plugin, or nullptr if not initialized. */
  kinematics::KinematicsBase* kinematics();

  /** @brief Shared ownership of the kinematics plugin (may be empty). */
  std::shared_ptr<kinematics::KinematicsBase> SharedKinematics() const {
    return kinematics_;
  }

  /** @brief Scene monitor, or nullptr if not initialized. */
  scene::SceneMonitor* scene_monitor();

  /** @brief Trajectory execution manager, or nullptr if not initialized. */
  execution::TrajectoryExecutionManager* execution_manager();

  /**
   * @brief Look up a loaded capability by name.
   * @param[in] name Capability name (e.g. "plan", "execute").
   * @return Capability pointer, or nullptr if not found.
   */
  server::Capability* GetCapability(const std::string& name);

  /** @brief Effective ManipulationOptions used at Init. */
  const proto::ManipulationOptions& options() const { return options_; }

  /**
   * @brief Update runtime planner-related options (query_planners set params).
   * @param[in] planner_id Optional planner id override.
   * @param[in] params Key/value map (planner_id, planning_time_ms, …).
   * @return true if at least one field changed.
   */
  bool SetPlannerParams(
      const std::string& planner_id,
      const std::unordered_map<std::string, std::string>& params);

  /** @brief Resolved URDF path loaded at Init (may be empty). */
  const std::string& UrdfPath() const { return urdf_path_; }

 private:
  bool ResolveUrdfPath(std::string* urdf_path) const;
  bool SetupPlugins(const std::string& urdf_path, const std::string& group);

  proto::ManipulationOptions options_;
  std::string urdf_path_;
  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<core::SimpleRobotModel> model_;
  std::shared_ptr<core::SimpleRobotState> state_;
  std::shared_ptr<scene::PlanningScene> scene_;
  std::shared_ptr<scene::SceneMonitor> scene_monitor_;
  std::shared_ptr<kinematics::KinematicsBase> kinematics_;
  std::shared_ptr<planning::PlannerBase> planner_;
  std::unique_ptr<planning::PlanningPipeline> pipeline_;
  std::unique_ptr<execution::TrajectoryExecutionManager> execution_;
  std::shared_ptr<execution::JointStateSubscriber> joint_states_;
  std::unique_ptr<ManipulationActionServer> action_server_;
  std::shared_ptr<servo::DampedLeastSquaresServo> servo_;
  std::unique_ptr<servo::ServoNode> servo_node_;
  std::unordered_map<std::string, std::shared_ptr<server::Capability>>
      capabilities_;
  bool running_ = false;
};

}  // namespace manipulation
}  // namespace autonomy
