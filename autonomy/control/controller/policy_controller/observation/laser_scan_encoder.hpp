/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file laser_scan_encoder.hpp
 * @brief Encode LaserScan ranges into a fixed-length normalized vector.
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
 * @brief Downsample ranges to @p laser_scan_bin_count bins in [0, 1].
 *
 * Invalid / out-of-range samples are clamped to @c range_max before
 * normalizing by @c range_max.
 *
 * @param[in] snapshot Must contain LaserScan ranges.
 * @param[in] input Policy input spec (shape may override bin count).
 * @param[in] laser_scan_bin_count Default bin count when shape is unset.
 * @param[out] output Normalized ranges.
 * @param[out] error Optional failure reason.
 */
bool EncodeLaserScan(const ObservationSnapshot& snapshot,
                     const proto::ObservationInput& input, int laser_scan_bin_count,
                     std::vector<float>* output, std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
