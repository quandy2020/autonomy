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
 * @file esikf_state_filter.hpp
 * @brief Iterated ESIKF point-to-plane state update (FAST-LIVO2 LIO backend).
 */

#ifndef ARGUS_BACKEND_STATE_FILTER_ESIKF_STATE_FILTER_HPP_
#define ARGUS_BACKEND_STATE_FILTER_ESIKF_STATE_FILTER_HPP_

#include "argus/backend/state_filter/esikf_update_options.hpp"
#include "argus/frontend/types.hpp"
#include "argus/map/voxel_map.hpp"

namespace argus {
namespace backend {

/**
 * @brief FAST-LIVO2 LIO measurement update on a voxel map.
 *
 * Optional P0/P2 path (still single-threaded): FIM degeneracy →
 * ConfidenceGating → anisotropic P inflate and/or skip / down-weight LiDAR
 * update.
 */
class EsikfStateFilter
{
public:
    /**
     * @brief Run default iterated ESIKF LiDAR update on the voxel map.
     * @param map Voxel map manager (modified in place).
     * @param state_propagated Propagated state estimate (updated in place).
     */
    static void Update(map::VoxelMapManager* map,
                       frontend::StatesGroup* state_propagated);

    /**
     * @brief LiDAR update with optional degeneracy, gating, and intensity
     * assist.
     * @param map Voxel map manager (modified in place).
     * @param state_propagated Propagated state estimate (updated in place).
     * @param options Robustness hooks and optional result outputs.
     */
    static void Update(map::VoxelMapManager* map,
                       frontend::StatesGroup* state_propagated,
                       const EsikfUpdateOptions& options);
};

}  // namespace backend
}  // namespace argus

#endif  // ARGUS_BACKEND_STATE_FILTER_ESIKF_STATE_FILTER_HPP_
