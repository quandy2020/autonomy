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

#include <functional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/layers/layer_manager.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/utils/event_emitter.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace map_features {

class FloorManager
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(FloorManager)

    explicit FloorManager(render::SceneState::SharedPtr scene);

    /** 对齐 BICMap createFloorManager({ floors, defaultFloor })。 */
    void Configure(const std::vector<FloorConfig>& floors, const std::string& defaultFloorId,
                   layers::LayerManager::SharedPtr layerManager);

    bool SwitchTo(const std::string& floorId);
    std::string GetActiveFloor() const;
    std::vector<render::FloorState> GetFloors() const;
    void HideFloor(const std::string& floorId);
    void BindLayers3D(const std::string& floorId, const std::vector<std::string>& layerIds);
    void Remove();

private:
    void ApplyBoundLayersVisibility(const std::vector<std::string>& layerIds);

    render::SceneState::SharedPtr scene_;
    layers::LayerManager::SharedPtr layerManager_;
    std::unordered_map<std::string, std::vector<std::string>> layers3DRegistry_;
};

class SemanticZones
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SemanticZones)

    explicit SemanticZones(render::SceneState::SharedPtr scene);

    render::SemanticZoneFeature* AddZone(const std::string& id, ZoneType type,
                                         const std::vector<LngLat>& polygon);
    bool RemoveZone(const std::string& id);
    void Update(const std::vector<render::SemanticZoneFeature>& zones);
    void Show();
    void Hide();
    std::vector<render::SemanticZoneFeature> GetZones() const;
    void Remove();

private:
    render::SceneState::SharedPtr scene_;
    bool visible_{true};
};

class Buildings
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Buildings)

    explicit Buildings(render::SceneState::SharedPtr scene);

    void Show();
    void Hide();
    void Remove();
    void Update(const std::vector<render::BuildingFeature>& buildings);
    void SetHeightScale(double scale);
    void SetOpacity(float opacity);
    void OnFeature(const std::string& featureId, std::function<void()> handler);

private:
    render::SceneState::SharedPtr scene_;
    utils::EventEmitter::SharedPtr events_;
};

class LayerEvents
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LayerEvents)

    LayerEvents();

    void On(const std::string& layerId, utils::EventEmitter::Handler handler);
    void Off(const std::string& layerId);
    void Once(const std::string& layerId, utils::EventEmitter::Handler handler);
    void OnFeature(const std::string& featureId, utils::EventEmitter::Handler handler);
    void OffFeature(const std::string& featureId);
    void Dispatch(const std::string& layerId, const std::string& featureId);
    void Destroy();

private:
    utils::EventEmitter::SharedPtr emitter_;
};

}  // namespace map_features
}  // namespace strata
}  // namespace map
}  // namespace autonomy
