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
#include "autolink/node/writer.hpp"
#include "autonomy/manipulation/motion/execution/autolink_trajectory_controller.hpp"
#include "autonomy/manipulation/motion/execution/effort_tracking_controller.hpp"
#include "autonomy/manipulation/motion/execution/joint_state_subscriber.hpp"
#include "autonomy/manipulation/motion/execution/trajectory_execution_manager.hpp"
#include "autonomy/manipulation/model/simple_robot_model.hpp"
#include "autonomy/manipulation/motion/dynamics/dynamics_factory.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/planner/pipeline/planning_pipeline.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/proto/manipulation_options.pb.h"
#include "autonomy/manipulation/motion/scene/occupancy_map_monitor.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"
#include "autonomy/manipulation/motion/scene/scene_monitor.hpp"
#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/manipulation_options.hpp"
#include "autonomy/manipulation/motion/servo/servo.hpp"

#include <automsgs/msgs/std_msgs/float64_multi_array.pb.h>
#include <automsgs/msgs/std_msgs/int32.pb.h>

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
  ManipulationServer();

  /** @brief Stop the server and release owned resources. */
  ~ManipulationServer();

  /**
   * @brief Load options, resolve URDF, and construct plugins / subsystems.
   * @param[in] options Proto config (defaults may be loaded from conf files).
   * @return true on successful initialization.
   */
  bool Init(const ManipulationOptions& options = {});

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
  const ManipulationOptions& options() const { return options_; }

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

  /** @brief Inverse-dynamics solver (Pinocchio FEATURE or stub). */
  dynamics::DynamicsSolver* dynamics_solver();

  std::shared_ptr<dynamics::DynamicsSolver> SharedDynamicsSolver() const {
    return dynamics_;
  }

 private:
  bool ResolveUrdfPath(std::string* urdf_path) const;
  bool SetupPlugins(const std::string& urdf_path, const std::string& group);

  ManipulationOptions options_;
  std::string urdf_path_;
  std::shared_ptr<autolink::Node> node_;
  std::shared_ptr<core::SimpleRobotModel> model_;
  std::shared_ptr<core::SimpleRobotState> state_;
  std::shared_ptr<scene::PlanningScene> scene_;
  std::shared_ptr<scene::SceneMonitor> scene_monitor_;
  std::unique_ptr<scene::OccupancyMapMonitor> occupancy_monitor_;
  std::shared_ptr<kinematics::KinematicsBase> kinematics_;
  std::shared_ptr<planning::PlannerBase> planner_;
  std::shared_ptr<dynamics::DynamicsSolver> dynamics_;
  std::unique_ptr<planning::PlanningPipeline> pipeline_;
  std::unique_ptr<execution::TrajectoryExecutionManager> execution_;
  std::shared_ptr<execution::JointStateSubscriber> joint_states_;
  std::unique_ptr<ManipulationActionServer> action_server_;
  std::shared_ptr<servo::DampedLeastSquaresServo> servo_;
  std::unique_ptr<servo::ServoNode> servo_node_;
  std::shared_ptr<execution::AutolinkTrajectoryController> servo_publisher_;
  std::shared_ptr<void> servo_twist_reader_;
  std::shared_ptr<void> servo_pose_reader_;
  std::shared_ptr<void> servo_jog_reader_;
  std::shared_ptr<void> servo_cmd_type_reader_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::std_msgs::Int32>>
      servo_status_writer_;
  std::shared_ptr<
      autolink::Writer<automsgs::msgs::std_msgs::Float64MultiArray>>
      effort_command_writer_;
  std::unique_ptr<execution::EffortTrackingController> effort_tracker_;
  std::unordered_map<std::string, std::shared_ptr<server::Capability>>
      capabilities_;
  bool running_ = false;
};

}  // namespace manipulation
}  // namespace autonomy
