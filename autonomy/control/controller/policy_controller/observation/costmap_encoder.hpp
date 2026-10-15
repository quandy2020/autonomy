/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file costmap_encoder.hpp
 * @brief Encode a local CostmapCrop into a float grid tensor.
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
 * @brief Copy or resample @ref CostmapCrop data into @p output ([0, 1]).
 *
 * Same-size crops take a single vector assignment; otherwise nearest-neighbor
 * resample to the trailing two dims of @p input.shape.
 *
 * @param[in] snapshot Must contain a non-empty costmap crop.
 * @param[in] input Policy input spec (optional HxW override).
 * @param[out] output Row-major float grid.
 * @param[out] error Optional failure reason.
 */
bool EncodeCostmap(const ObservationSnapshot& snapshot,
                   const proto::ObservationInput& input, std::vector<float>* output,
                   std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
