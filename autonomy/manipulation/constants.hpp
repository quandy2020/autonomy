/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

namespace autonomy {
namespace manipulation {

/** @brief Default topic for arm joint-trajectory commands. */
constexpr char kJointTrajectoryTopic[] = "/arm_controller/joint_trajectory";

/** @brief Default action name for manipulation move goals. */
constexpr char kManipulationAction[] = "/autonomy/manipulation/move";

}  // namespace manipulation
}  // namespace autonomy
