/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file config.hpp
 * @brief Fixed-profile runtime configuration shared by Fathom C++ modules.
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_CONFIG_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_CONFIG_HPP_

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/** @brief Validated model and preprocessing parameters for one deployment. */
struct FathomConfig {
    /** Path to the fixed-profile model artifact. */
    std::string model_path;
    /** Common-network backend identifier: `onnx` or `tensorrt`. */
    std::string backend = "onnx";
    /** Fixed model input width in pixels. */
    int input_width = 0;
    /** Fixed model input height in pixels. */
    int input_height = 0;
    /** Multiplier that converts incoming depth samples to meters. */
    float depth_scale = 0.001F;
    /** Minimum model validity score accepted during projection. */
    float mask_threshold = 0.5F;
};

/**
 * @brief Validates a Fathom deployment profile before model construction.
 *
 * The configured spatial profile must match the exported graph. The token
 * count is baked into the graph and is not exposed by the C++ engine metadata.
 * Supported backends are the project network backend ids `onnx` and
 * `tensorrt`. The optional error is cleared on entry.
 *
 * @param config Deployment profile to validate.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when every field satisfies the runtime contract.
 */
bool ValidateFathomConfig(const FathomConfig& config,
                          std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_CONFIG_HPP_
