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

#include "map/visual/atlas.hpp"

namespace atlas {
namespace map {
namespace visual {

MapId Atlas::CreateMap() {
  const MapId id = next_id_++;
  maps_.push_back(id);
  active_map_ = id;
  return id;
}

}  // namespace visual
}  // namespace map
}  // namespace atlas
