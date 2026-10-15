/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file path_encoder.hpp
 * @brief Encode a global path as robot-frame (x, y) samples.
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
 * @brief Sample @p path_point_count waypoints into interleaved (x, y) floats.
 *
 * Waypoints are transformed into the robot frame using the snapshot pose.
 *
 * @param[in] snapshot Must contain a non-empty path (pose optional → identity).
 * @param[in] input Policy input spec (optional shape override).
 * @param[in] path_point_count Number of samples along the path.
 * @param[out] output Length @c path_point_count * 2 (or shape product).
 * @param[out] error Optional failure reason.
 */
bool EncodePath(const ObservationSnapshot& snapshot,
                const proto::ObservationInput& input, int path_point_count,
                std::vector<float>* output, std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
