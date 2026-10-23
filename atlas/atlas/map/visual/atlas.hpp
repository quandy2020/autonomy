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
 * @file atlas.hpp
 * @brief Collection of visual maps. Map points and covisibility stay here,
 *        not in the voxel map.
 */

#ifndef ATLAS_MAP_VISUAL_ATLAS_HPP_
#define ATLAS_MAP_VISUAL_ATLAS_HPP_

#include <cstdint>
#include <vector>

namespace atlas {
namespace map {
namespace visual {

using MapId = std::int64_t;

class Atlas {
 public:
  MapId CreateMap();
  MapId active_map() const { return active_map_; }

 private:
  std::vector<MapId> maps_;
  MapId active_map_ = -1;
  MapId next_id_ = 0;
};

}  // namespace visual
}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_VISUAL_ATLAS_HPP_
