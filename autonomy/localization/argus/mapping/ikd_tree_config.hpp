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
 * @file ikd_tree_config.hpp
 * @brief Build / rebuild parameters for @ref IkdTree.
 */

#ifndef ARGUS_MAP_IKD_TREE_CONFIG_HPP_
#define ARGUS_MAP_IKD_TREE_CONFIG_HPP_

namespace argus {
namespace map {

/** @brief Build / rebuild and downsample parameters for IkdTree. */
struct IkdTreeConfig {
    int max_leaf_size = 16; /**< @brief Max points per leaf before split. */
    /** @brief Rebuild when size grows by this factor since last build. */
    double rebuild_ratio = 1.5;
    /** @brief Optional voxel downsample when adding (0 = disabled). */
    double downsample_resolution = 0.0;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_IKD_TREE_CONFIG_HPP_
