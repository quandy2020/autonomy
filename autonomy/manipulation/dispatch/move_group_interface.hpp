/*
 * Copyright 2026 The Openbot Authors
 *
 * In-process client API. Cross-process clients should use
 * /autonomy/manipulation/move Action.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace interface {

/**
 * @brief In-process MoveIt-style client for ManipulationServer.
 *
 * Cross-process clients should use the /autonomy/manipulation/move Action.
 */
class MoveGroupInterface {
 public:
  /**
   * @brief Bind to a ManipulationServer (not owned).
   * @param[in] server Runtime server used for plan / execute.
   */
  explicit MoveGroupInterface(ManipulationServer* server);

  /**
   * @brief Set the planner id applied to subsequent requests.
   * @param[in] planner_id Planner plugin / pipeline id.
   */
  void SetPlannerId(const std::string& planner_id);

  /**
   * @brief Set the planning group name applied to subsequent requests.
   * @param[in] group SRDF / model group name.
   */
  void SetGroup(const std::string& group);

  /**
   * @brief Plan to a joint-space target using the configured group / planner.
   * @param[in] goal Desired joint state.
   * @return Motion plan response.
   */
  planning::MotionPlanResponse PlanToJointTarget(
      const core::JointState& goal);

  /**
   * @brief Plan to a Cartesian pose target (IK + pipeline).
   * @param[in] goal Desired end-effector pose.
   * @return Motion plan response.
   */
  planning::MotionPlanResponse PlanToPoseTarget(
      const kinematics::Pose& goal);

  /**
   * @brief Plan then execute @p request on the bound server.
   * @param[in] request Full motion plan request.
   * @return Motion plan response (includes execution outcome when applicable).
   */
  planning::MotionPlanResponse PlanAndExecute(
      const planning::MotionPlanRequest& request);

 private:
  ManipulationServer* server_ = nullptr;
  std::string planner_id_;
  std::string group_;
};

}  // namespace interface
}  // namespace manipulation
}  // namespace autonomy
