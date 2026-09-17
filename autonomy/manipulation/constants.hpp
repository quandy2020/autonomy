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

/** @brief Default PointCloud2 topic for occupancy updates. */
constexpr char kPointCloudTopic[] = "/camera/depth/points";

/** @brief Default OctomapWithPose topic. */
constexpr char kOctomapTopic[] = "/octomap_binary";

/** @brief Default servo joint command topic (single-point trajectories). */
constexpr char kServoJointCommandTopic[] = "/servo_server/delta_joint_cmds";

/** @brief Default Cartesian twist command topic for ServoNode. */
constexpr char kServoTwistCommandTopic[] = "/servo_server/delta_twist_cmds";

/** @brief Default Cartesian pose command topic for ServoNode. */
constexpr char kServoPoseCommandTopic[] = "/servo_server/pose_target_cmds";

/** @brief Default joint-jog command topic for ServoNode. */
constexpr char kServoJointJogCommandTopic[] = "/servo_server/delta_joint_jog_cmds";

/** @brief Int32 CommandType switch: 0=Twist, 1=JointJog, 2=Pose. */
constexpr char kServoCommandTypeTopic[] = "/servo_server/command_type";

/** @brief Int32 ServoStatus publish topic. */
constexpr char kServoStatusTopic[] = "/servo_server/status";

/** @brief Float64MultiArray effort feedforward to hardware (Nm). */
constexpr char kEffortCommandTopic[] = "/arm_controller/effort_command";

}  // namespace manipulation
}  // namespace autonomy
