/*
 * Copyright 2026 The Openbot Authors
 *
 * MotionPlanRequest / MotionPlanResponse ↔ automsgs.moveit_msgs.
 */

#pragma once

#include <automsgs/msgs/moveit_msgs/motion_plan.pb.h>

#include "automanip/common/planner_interface.hpp"
#include "automanip/motion/scene/scene_message_conversion.hpp"

namespace automanip {
namespace planner {

/**
 * @brief Convert an internal request to automsgs MotionPlanRequest.
 * @param[in] request Internal planning request.
 * @return Protobuf message.
 */
automsgs::msgs::moveit_msgs::MotionPlanRequest ToMessage(
    const MotionPlanRequest& request);

/**
 * @brief Convert an automsgs MotionPlanRequest to internal form.
 * @param[in] message Protobuf request.
 * @return Internal MotionPlanRequest.
 */
MotionPlanRequest FromMessage(
    const automsgs::msgs::moveit_msgs::MotionPlanRequest& message);

/**
 * @brief Convert an internal response to automsgs MotionPlanResponse.
 * @param[in] response Internal planning response.
 * @return Protobuf message.
 */
automsgs::msgs::moveit_msgs::MotionPlanResponse ToMessage(
    const ::automanip::proto::MotionPlanResponse& response);

/**
 * @brief Convert an automsgs MotionPlanResponse to internal form.
 * @param[in] message Protobuf response.
 * @return Internal MotionPlanResponse.
 */
::automanip::proto::MotionPlanResponse FromMessage(
    const automsgs::msgs::moveit_msgs::MotionPlanResponse& message);

}  // namespace planner
}  // namespace automanip
