/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace render {

void SceneState::Touch() { ++revision_; }

void NullRenderBackend::Initialize(const MapViewOptions& options) {
    options_ = options;
    AINFO << "Strata NullRenderBackend initialized for container: " << options.container;
}

void NullRenderBackend::Sync(const SceneState& state) {
    AINFO << "Strata scene sync revision=" << state.revision()
          << " rectangles=" << state.rectangles.size()
          << " poi=" << state.poiMarkers.size()
          << " robots=" << state.robotMarkers.size();
}

void NullRenderBackend::SetZoom(double zoom) { options_.zoom = zoom; }
void NullRenderBackend::SetCenter(const LngLat& center) { options_.center = center; }
void NullRenderBackend::Resize(int, int) {}
void NullRenderBackend::Destroy() {}

}  // namespace render
}  // namespace strata
}  // namespace map
}  // namespace autonomy
