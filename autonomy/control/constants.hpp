/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

namespace autonomy {
namespace control {

constexpr char kControllerServerNodeName[] = "controller_server";
constexpr char kCmdVelChannel[] = "/cmd_vel";
constexpr char kFollowPathActionName[] = "follow_path";
constexpr char kSpinActionName[] = "spin";
constexpr char kBackUpActionName[] = "backup";
constexpr char kWaitActionName[] = "wait";

}  // namespace control
}  // namespace autonomy
