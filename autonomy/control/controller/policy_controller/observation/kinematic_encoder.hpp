/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file kinematic_encoder.hpp
 * @brief Encode pose + velocity into a kinematic feature vector.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/control/proto/policy_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Encode kinematic features (absolute or relative to path goal).
 *
 * Absolute: @c [x, y, yaw, vx, vy, wz] in pose frame.
 * Relative: @c [dx, dy, dyaw, vx, vy, wz] in robot frame vs path end pose.
 */
bool EncodeKinematic(const ObservationSnapshot& snapshot,
                     const proto::ObservationInput& input,
                     std::vector<float>* output, std::string* error,
                     bool relative_to_path_goal = false);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
