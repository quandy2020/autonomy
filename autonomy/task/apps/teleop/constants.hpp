/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

namespace autonomy {
namespace task {
namespace teleop {

constexpr char kTeleopClientBlackboardKey[] = "teleop_client";
constexpr char kCmdVelTopic[] = "/cmd_vel";
constexpr char kDefaultBaseFrame[] = "base_link";

constexpr double kDefaultMaxLinearSpeed = 0.5;
constexpr double kDefaultMaxAngularSpeed = 1.0;
constexpr double kDefaultWatchdogTimeoutSec = 0.5;

}  // namespace teleop
}  // namespace task
}  // namespace autonomy
