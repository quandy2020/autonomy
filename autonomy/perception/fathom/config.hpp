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

#ifndef AUTONOMY_PERCEPTION_FATHOM_CONFIG_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_CONFIG_HPP_

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @file config.hpp
 * @brief Fixed-profile deployment configuration for a Fathom model.
 */

/** Configuration shared by Fathom model loading and frame refinement. */
struct FathomConfig {
    std::string model_path;
    std::string backend = "onnx";
    int input_width = 0;
    int input_height = 0;
    int num_tokens = 0;
    float depth_scale = 0.001F;
    float mask_threshold = 0.5F;
};

/**
 * Validate a Fathom deployment profile before model construction.
 *
 * The exported graph fixes its spatial profile and token count, so all profile
 * fields must be valid before a frame is accepted. Supported backends are the
 * project network backend ids `onnx` and `tensorrt`.
 */
bool ValidateFathomConfig(const FathomConfig& config,
                          std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_CONFIG_HPP_
