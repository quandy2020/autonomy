/*
 * Copyright 2026 The Openbot Authors
 *
 * In-process client API. Cross-process clients should use
 * /automanip/move Action.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "automanip/common/planner_interface.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

/**
 * @brief In-process MoveIt-style client for ManipulationServer.
 *
 * Cross-process clients should use the /automanip/move Action.
 */
class MoveGroupInterface {
 public:
  /**
   * @brief Define MoveGroupInterface::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(MoveGroupInterface)

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
  ::automanip::proto::MotionPlanResponse PlanToJointTarget(
      const automsgs::msgs::sensor_msgs::JointState& goal);

  /**
   * @brief Plan to a Cartesian pose target (IK + pipeline).
   * @param[in] goal Desired end-effector pose.
   * @return Motion plan response.
   */
  ::automanip::proto::MotionPlanResponse PlanToPoseTarget(
      const automsgs::msgs::geometry_msgs::Pose& goal);

  /**
   * @brief Plan then execute @p request on the bound server.
   * @param[in] request Full motion plan request.
   * @return Motion plan response (includes execution outcome when applicable).
   */
  ::automanip::proto::MotionPlanResponse PlanAndExecute(
      const planner::MotionPlanRequest& request);

 private:
  ManipulationServer* server_ = nullptr;
  std::string planner_id_;
  std::string group_;
};

}  // namespace dispatch
}  // namespace automanip
