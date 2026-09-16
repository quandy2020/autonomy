/*
 * Copyright 2026 The Openbot Authors
 *
 * move_group capability plugin base.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/kinematics/kinematics_base.hpp"
#include "autonomy/manipulation/planning/planner_base.hpp"
#include "autonomy/manipulation/scene/scene_monitor.hpp"

namespace autonomy {
namespace manipulation {

class ManipulationServer;

namespace server {

/**
 * @brief Base interface for move_group-style capability plugins.
 */
class Capability {
 public:
  virtual ~Capability() = default;

  /** @brief Stable capability name used for lookup / plugin registration. */
  virtual std::string Name() const = 0;

  /**
   * @brief Bind to the owning ManipulationServer.
   * @param[in] server Runtime server (not owned).
   * @return true on successful initialization.
   */
  virtual bool Init(ManipulationServer* server) = 0;
};

/**
 * @brief Capability that runs the planning pipeline.
 */
class PlanCapability : public Capability {
 public:
  /** @brief Returns "plan". */
  std::string Name() const override { return "plan"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Plan without executing.
   * @param[in] req Motion plan request.
   * @return Motion plan response.
   */
  planning::MotionPlanResponse Plan(const planning::MotionPlanRequest& req);

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that executes a precomputed trajectory.
 */
class ExecuteCapability : public Capability {
 public:
  /** @brief Returns "execute". */
  std::string Name() const override { return "execute"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Execute @p trajectory on the server's execution manager.
   * @param[in] trajectory Joint-space waypoints.
   * @return Error code from execution.
   */
  ErrorCode Execute(const core::RobotTrajectory& trajectory);

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that plans then executes in one call.
 */
class PlanAndExecuteCapability : public Capability {
 public:
  /** @brief Returns "plan_and_execute". */
  std::string Name() const override { return "plan_and_execute"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Plan @p req and execute the resulting trajectory on success.
   * @param[in] req Motion plan request.
   * @return Motion plan response (includes execution outcome when applicable).
   */
  planning::MotionPlanResponse Run(const planning::MotionPlanRequest& req);

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability for Cartesian-path planning.
 */
class CartesianCapability : public Capability {
 public:
  /** @brief Returns "cartesian". */
  std::string Name() const override { return "cartesian"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Plan a Cartesian path for @p req.
   * @param[in] req Motion plan request with Cartesian goals / constraints.
   * @return Motion plan response.
   */
  planning::MotionPlanResponse Plan(const planning::MotionPlanRequest& req);

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability for forward / inverse kinematics.
 */
class FkIkCapability : public Capability {
 public:
  /** @brief Returns "fk_ik". */
  std::string Name() const override { return "fk_ik"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Compute FK for @p joints into @p pose.
   * @param[in] joints Joint configuration.
   * @param[out] pose Resulting end-effector pose.
   * @return true on success.
   */
  bool ComputeFk(const core::JointState& joints, kinematics::Pose* pose) const;

  /**
   * @brief Compute IK for @p pose seeded by @p seed.
   * @param[in] pose Desired end-effector pose.
   * @param[in] seed Seed joint state.
   * @param[out] solution Joint solution on success.
   * @return Error code from the kinematics plugin.
   */
  ErrorCode ComputeIk(const kinematics::Pose& pose,
                      const core::JointState& seed,
                      core::JointState* solution) const;

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that exposes the current planning scene.
 */
class GetPlanningSceneCapability : public Capability {
 public:
  /** @brief Returns "get_planning_scene". */
  std::string Name() const override { return "get_planning_scene"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Shared pointer to the server's planning scene.
   * @return Scene pointer, or empty if unavailable.
   */
  std::shared_ptr<scene::PlanningScene> Scene() const;

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that applies a SceneDiff to the planning scene.
 */
class ApplyPlanningSceneCapability : public Capability {
 public:
  /** @brief Returns "apply_planning_scene". */
  std::string Name() const override { return "apply_planning_scene"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Apply @p diff to the server's planning scene.
   * @param[in] diff Incremental scene update.
   * @return true if the diff was applied.
   */
  bool Apply(const scene::SceneDiff& diff);

 private:
  ManipulationServer* server_ = nullptr;
};

/** @brief Result of a single-state validity query. */
struct StateValidationResult {
  bool valid = false;
  ErrorCode error_code = ErrorCode::kFailure;
  std::string error;
  std::string contact_body_a;
  std::string contact_body_b;
};

/**
 * @brief Capability for check_state_validity / multi-state validation.
 */
class StateValidationCapability : public Capability {
 public:
  std::string Name() const override { return "state_validation"; }
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Validate @p state for collision and optional path constraints.
   * @param[in] state Joint configuration to test.
   * @param[in] constraints Optional constraints (empty = collision only).
   * @return Validity result with error code.
   */
  StateValidationResult CheckState(
      const core::JointState& state,
      const planning::MotionPlanRequest& constraints = {}) const;

  /**
   * @brief Validate each state in @p states.
   * @param[in] states Joint configurations.
   * @param[in] constraints Optional shared constraints.
   * @return Per-state results (same order as @p states).
   */
  std::vector<StateValidationResult> CheckStates(
      const std::vector<core::JointState>& states,
      const planning::MotionPlanRequest& constraints = {}) const;

 private:
  ManipulationServer* server_ = nullptr;
};

/** @brief Planner interface descriptor (MoveIt PlannerInterface analogue). */
struct PlannerInterfaceInfo {
  std::string name;
  std::string pipeline_id = "manipulation";
  std::string description;
};

/**
 * @brief Capability to query planners and get/set runtime planner params.
 */
class QueryPlannersCapability : public Capability {
 public:
  std::string Name() const override { return "query_planners"; }
  bool Init(ManipulationServer* server) override;

  /** @brief Registered planner aliases / class names. */
  std::vector<PlannerInterfaceInfo> ListPlanners() const;

  /**
   * @brief Read planner runtime params from ManipulationOptions.
   * @param[in] planner_id Planner id (empty = active options.planner_id).
   * @return Key/value params (planner_id, planning_time_ms, …).
   */
  std::unordered_map<std::string, std::string> GetPlannerParams(
      const std::string& planner_id = "") const;

  /**
   * @brief Update runtime options for subsequent plans.
   * @param[in] planner_id Target planner id written into options when non-empty.
   * @param[in] params Supported keys: planner_id, planning_time_ms, max_attempts,
   *                   max_velocity, max_acceleration.
   * @return true if at least one param applied.
   */
  bool SetPlannerParams(const std::string& planner_id,
                        const std::unordered_map<std::string, std::string>& params);

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that clears occupancy (MoveIt clear_octomap).
 */
class ClearOctomapCapability : public Capability {
 public:
  std::string Name() const override { return "clear_octomap"; }
  bool Init(ManipulationServer* server) override;
  /** @brief Clear occupied points on the monitored scene. */
  bool Clear();

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that clears world collision objects (optional attachments).
 */
class ClearSceneCapability : public Capability {
 public:
  std::string Name() const override { return "clear_scene"; }
  bool Init(ManipulationServer* server) override;
  /**
   * @brief Clear world objects.
   * @param[in] clear_attached Also clear attachments when true.
   */
  bool Clear(bool clear_attached = false);

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability that validates an arbitrary trajectory against the scene.
 */
class ValidateTrajectoryCapability : public Capability {
 public:
  std::string Name() const override { return "validate_trajectory"; }
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Check path validity (collision / occupancy) for @p trajectory.
   * @param[in] trajectory Waypoints to validate.
   * @param[in] constraints Optional joint/pose path constraints.
   * @return ErrorCode::kSuccess if valid.
   */
  ErrorCode Validate(const core::RobotTrajectory& trajectory,
                     const planning::MotionPlanRequest& constraints = {}) const;

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability returning the loaded robot URDF path / text (get_urdf).
 */
class GetUrdfCapability : public Capability {
 public:
  std::string Name() const override { return "get_urdf"; }
  bool Init(ManipulationServer* server) override;

  /** @brief Resolved URDF file path used at Init. */
  std::string UrdfPath() const;

  /**
   * @brief Read URDF file contents (empty if unreadable).
   * @return URDF XML text.
   */
  std::string UrdfXml() const;

 private:
  ManipulationServer* server_ = nullptr;
};

/**
 * @brief Capability to save / load world geometry (MoveIt geometry file subset).
 */
class SaveLoadGeometryCapability : public Capability {
 public:
  std::string Name() const override { return "save_load_geometry"; }
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Save current world objects + occupancy to @p path.
   * @param[in] path Output geometry file.
   * @return true on success.
   */
  bool Save(const std::string& path) const;

  /**
   * @brief Load geometry file into the planning scene (clears world first).
   * @param[in] path Input geometry file.
   * @return true on success.
   */
  bool Load(const std::string& path);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace server
}  // namespace manipulation
}  // namespace autonomy
