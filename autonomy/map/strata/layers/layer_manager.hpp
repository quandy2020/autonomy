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

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace layers {

class LayerManager
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LayerManager)

    explicit LayerManager(render::SceneState::SharedPtr scene);

    SlamMapResult LoadSlamMap(const SlamMapOptions& options);
    void EnsureLayerOrder();
    double GetLayerOrder(const std::string& layerId) const;
    void AddLayerId(const std::string& layerId);

private:
    render::SceneState::SharedPtr scene_;
    std::vector<std::string> layerIds_;
};

}  // namespace layers
}  // namespace strata
}  // namespace map
}  // namespace autonomy
