/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file image_encoder.hpp
 * @brief Encode a camera Image into an NCHW float tensor.
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
 * @brief Convert @c sensor_msgs/Image to model input via network preprocess.
 *
 * Supports @c preprocess values: @c letterbox (default), @c stretch,
 * @c center_crop. Output layout follows @p input.shape when provided
 * (typically NCHW).
 *
 * @param[in] snapshot Must contain an image with non-empty data.
 * @param[in] input Policy input spec (shape / preprocess).
 * @param[out] output Flattened float tensor.
 * @param[out] error Optional failure reason.
 */
bool EncodeImage(const ObservationSnapshot& snapshot,
                 const proto::ObservationInput& input, std::vector<float>* output,
                 std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
