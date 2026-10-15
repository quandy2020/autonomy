/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file tensor_frame_encoder.hpp
 * @brief Encode precomputed TensorFrame (RAW / lidar BEV).
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
 * @brief Copy lidar BEV TensorFrame into @p output (resize if shape differs).
 */
bool EncodeLidarBev(const ObservationSnapshot& snapshot,
                    const proto::ObservationInput& input,
                    std::vector<float>* output, std::string* error);

/**
 * @brief Copy named RAW TensorFrame (key = input.name) into @p output.
 */
bool EncodeRawTensor(const ObservationSnapshot& snapshot,
                     const proto::ObservationInput& input,
                     std::vector<float>* output, std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
