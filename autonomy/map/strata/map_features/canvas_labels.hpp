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

#include <optional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace map_features {

struct CanvasLabelInput {
    std::string id;
    std::string label;
    std::vector<LngLat> polygon;
    std::optional<LngLat> point;
};

class CanvasLabelsController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(CanvasLabelsController)

    CanvasLabelsController(render::SceneState::SharedPtr scene, CanvasLabelOptions options);

    void Update(const std::vector<CanvasLabelInput>& inputs);
    void Show();
    void Hide();
    void Remove();
    const std::vector<CanvasLabelFeature>& GetLabels() const;

    static LngLat ComputePolygonCentroid(const std::vector<LngLat>& polygon);
    static std::vector<CanvasLabelFeature> BuildFeatures(
        const std::vector<CanvasLabelInput>& inputs);

private:
    render::SceneState::SharedPtr scene_;
    CanvasLabelOptions options_;
    bool visible_{true};
};

}  // namespace map_features
}  // namespace strata
}  // namespace map
}  // namespace autonomy
