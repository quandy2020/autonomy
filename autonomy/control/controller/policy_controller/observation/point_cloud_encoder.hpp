/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file point_cloud_encoder.hpp
 * @brief Light PointCloud2 → fixed-length xyz float pack.
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
 * @brief Pack up to @p max_points xyz FLOAT32 samples as [N,3] (row-major).
 *
 * No voxelization / BEV — heavy lidar preprocess belongs in the ONNX export.
 * Missing points are zero-padded; extras are truncated.
 *
 * @param[in] snapshot Must hold a PointCloud2 with x/y/z fields.
 * @param[in] input Shape drives N when present (product/3 or last dims).
 * @param[in] max_points Fallback point count when shape is unset.
 * @param[out] output Length N*3.
 */
bool EncodePointCloud(const ObservationSnapshot& snapshot,
                      const proto::ObservationInput& input, int max_points,
                      std::vector<float>* output, std::string* error,
                      bool stride_sample = false);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
