/*
 * Copyright 2026 The Openbot Authors
 *
 * MotionPlanRequest/Response ↔ automsgs.moveit_msgs.
 */

#pragma once

#include <automsgs/msgs/moveit_msgs/motion_plan.pb.h>

#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/motion/scene/msg_convert.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Convert an internal request to automsgs MotionPlanRequest.
 * @param[in] request Internal planning request.
 * @return Protobuf message.
 */
automsgs::msgs::moveit_msgs::MotionPlanRequest ToMsg(
    const MotionPlanRequest& request);

/**
 * @brief Convert an automsgs MotionPlanRequest to internal form.
 * @param[in] msg Protobuf request.
 * @return Internal MotionPlanRequest.
 */
MotionPlanRequest FromMsg(
    const automsgs::msgs::moveit_msgs::MotionPlanRequest& msg);

/**
 * @brief Convert an internal response to automsgs MotionPlanResponse.
 * @param[in] response Internal planning response.
 * @return Protobuf message.
 */
automsgs::msgs::moveit_msgs::MotionPlanResponse ToMsg(
    const MotionPlanResponse& response);

/**
 * @brief Convert an automsgs MotionPlanResponse to internal form.
 * @param[in] msg Protobuf response.
 * @return Internal MotionPlanResponse.
 */
MotionPlanResponse FromMsg(
    const automsgs::msgs::moveit_msgs::MotionPlanResponse& msg);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
