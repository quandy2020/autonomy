/*
 * Copyright 2026 The Openbot Authors
 *
 * Apply URDF mimic joint relationships on a JointState.
 */

#pragma once

#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @brief Apply mimic relationships: `q_mimic = factor * q_master + offset`.
 * @param[in] joints Joint models that declare mimic targets.
 * @param[in,out] joint_state Joint state updated in place.
 */
void ApplyMimicJoints(const std::vector<JointModel>& joints,
                      automsgs::msgs::sensor_msgs::JointState* joint_state);

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
