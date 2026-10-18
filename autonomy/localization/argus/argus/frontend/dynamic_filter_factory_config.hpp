/*
 * Copyright 2026 The Openbot Authors
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
 * @file dynamic_filter_factory_config.hpp
 * @brief Factory knobs for LiDAR / visual dynamic filters.
 */

#ifndef ARGUS_FRONTEND_DYNAMIC_FILTER_FACTORY_CONFIG_HPP_
#define ARGUS_FRONTEND_DYNAMIC_FILTER_FACTORY_CONFIG_HPP_

#include <string>
#include <vector>

namespace argus {
namespace frontend {

/** @brief Dynamic-filter backend selection. */
enum class DynamicFilterType { kNone = 0, kSpatiotemporal, kSemantic };

/** @brief Parse config string (`none` / `spatiotemporal` / `semantic`). */
inline DynamicFilterType ParseDynamicFilterType(const std::string& name) {
    if (name == "spatiotemporal") {
        return DynamicFilterType::kSpatiotemporal;
    }
    if (name == "semantic") {
        return DynamicFilterType::kSemantic;
    }
    return DynamicFilterType::kNone;
}

/** @brief Factory knobs for LiDAR / visual dynamic filters. */
struct DynamicFilterFactoryConfig {
    DynamicFilterFactoryConfig();

    DynamicFilterType lidar =
        DynamicFilterType::kNone; /**< @brief LiDAR backend. */
    DynamicFilterType visual =
        DynamicFilterType::kNone; /**< @brief Visual backend. */
    double mask_threshold =
        0.5; /**< @brief Soft score threshold for semantic. */
    std::string semantic_model; /**< @brief ONNX model path. */
    /** @brief onnx | mask_file — see SemanticDynamicFilterConfig. */
    std::string semantic_backend = "onnx";
    std::string semantic_mask_path;  /**< @brief Offline HxW keep map path. */
    int semantic_input_width = 640;  /**< @brief ONNX input width. */
    int semantic_input_height = 640; /**< @brief ONNX input height. */
    std::vector<int> semantic_dynamic_class_ids = {
        0};                       /**< @brief Dynamic class ids. */
    double spatiotemporal_range_diff_m = 0.4; /**< @brief Spatiotemporal range delta (m). */
    double spatiotemporal_azimuth_res_deg = 0.4; /**< @brief Spatiotemporal az bin (deg). */
    double spatiotemporal_elevation_res_deg =
        0.4;                      /**< @brief Spatiotemporal el bin (deg). */
    double spatiotemporal_min_range_m = 1.0;  /**< @brief Spatiotemporal min range (m). */
    double spatiotemporal_max_range_m = 80.0; /**< @brief Spatiotemporal max range (m). */
};

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_DYNAMIC_FILTER_FACTORY_CONFIG_HPP_
