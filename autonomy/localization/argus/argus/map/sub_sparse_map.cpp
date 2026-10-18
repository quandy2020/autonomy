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

#include "argus/map/sub_sparse_map.hpp"

namespace argus {
namespace map {
namespace {

constexpr int kSizeLarge = 500;
constexpr int kSizeSmall = 100;

}  // namespace

SubSparseMap::SubSparseMap() {
    propa_errors.reserve(kSizeLarge);
    errors.reserve(kSizeLarge);
    warp_patch.reserve(kSizeLarge);
    search_levels.reserve(kSizeLarge);
    voxel_points.reserve(kSizeLarge);
    inv_expo_list.reserve(kSizeLarge);
    add_from_voxel_map.reserve(kSizeSmall);
}

void SubSparseMap::Reset() {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
}

}  // namespace map
}  // namespace argus
