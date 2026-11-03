/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRAJECTORY_EXPORT_POLICY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRAJECTORY_EXPORT_POLICY_HPP_

#include <cstddef>

namespace autonomy::localization::atlas::internal {

template <typename MapContainer>
auto SelectLargestTrajectoryMap(const MapContainer& maps) ->
    typename MapContainer::value_type {
    using MapPointer = typename MapContainer::value_type;
    MapPointer selected_map = nullptr;
    std::size_t maximum_keyframe_count = 0;
    for (MapPointer map : maps) {
        if (map == nullptr) {
            continue;
        }
        const std::size_t keyframe_count = map->GetAllKeyFrames().size();
        // Keep the first map on a tie. Atlas returns maps in ID order, so this
        // preserves deterministic export behavior.
        if (keyframe_count > maximum_keyframe_count) {
            maximum_keyframe_count = keyframe_count;
            selected_map = map;
        }
    }
    return selected_map;
}

template <typename MapType>
constexpr bool ShouldExportTrajectoryReferenceFromMap(
    const MapType* reference_map, const MapType* selected_map) {
    return reference_map != nullptr && selected_map != nullptr &&
           reference_map == selected_map;
}

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRAJECTORY_EXPORT_POLICY_HPP_
