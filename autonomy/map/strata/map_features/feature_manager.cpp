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

#include <algorithm>

#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/map_features/feature_manager.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace map_features {

FloorManager::FloorManager(render::SceneState::SharedPtr scene) : scene_(std::move(scene)) {}

void FloorManager::Configure(const std::vector<FloorConfig>& floors,
                             const std::string& defaultFloorId,
                             layers::LayerManager::SharedPtr layerManager) {
    layerManager_ = std::move(layerManager);
    scene_->floors.clear();
    scene_->floors.reserve(floors.size());
    for (const auto& config : floors) {
        render::FloorState floor;
        floor.id = config.id;
        floor.name = config.name.empty() ? config.id : config.name;
        floor.level = config.level;
        floor.slamOptions = config.slamOptions;
        floor.visible = false;
        scene_->floors.push_back(std::move(floor));
    }
    if (!defaultFloorId.empty()) {
        SwitchTo(defaultFloorId);
    } else if (!scene_->floors.empty()) {
        SwitchTo(scene_->floors.front().id);
    }
}

bool FloorManager::SwitchTo(const std::string& floorId) {
    bool found = false;
    for (auto& floor : scene_->floors) {
        floor.visible = (floor.id == floorId);
        if (floor.visible) {
            found = true;
            if (layerManager_) {
                layerManager_->LoadSlamMap(floor.slamOptions);
            }
        }
    }
    if (!found) {
        return false;
    }
    scene_->activeFloorId = floorId;
    const auto registry_it = layers3DRegistry_.find(floorId);
    if (registry_it != layers3DRegistry_.end()) {
        ApplyBoundLayersVisibility(registry_it->second);
    } else {
        ApplyBoundLayersVisibility({});
    }
    scene_->Touch();
    return true;
}

std::string FloorManager::GetActiveFloor() const { return scene_->activeFloorId; }

std::vector<render::FloorState> FloorManager::GetFloors() const { return scene_->floors; }

void FloorManager::HideFloor(const std::string& floorId) {
    for (auto& floor : scene_->floors) {
        if (floor.id == floorId) {
            floor.visible = false;
        }
    }
    scene_->Touch();
}

void FloorManager::BindLayers3D(const std::string& floorId,
                                const std::vector<std::string>& layerIds) {
    layers3DRegistry_[floorId] = layerIds;
    if (scene_->activeFloorId == floorId) {
        ApplyBoundLayersVisibility(layerIds);
    }
}

void FloorManager::ApplyBoundLayersVisibility(const std::vector<std::string>& layerIds) {
    auto contains = [&](const char* layer_id) {
        return std::find(layerIds.begin(), layerIds.end(), layer_id) != layerIds.end();
    };
    for (auto& fov : scene_->robotFovs) {
        fov.visible = contains(LayerIds::kRobotFov);
    }
    for (auto& layer : scene_->robot3DLayers) {
        layer.visible = contains(LayerIds::kRobot3D);
    }
    for (auto& building : scene_->buildings) {
        building.visible = contains(LayerIds::kBuilding3D);
    }
    scene_->Touch();
}

void FloorManager::Remove() {
    scene_->floors.clear();
    scene_->activeFloorId.clear();
    layers3DRegistry_.clear();
    scene_->Touch();
}

SemanticZones::SemanticZones(render::SceneState::SharedPtr scene) : scene_(std::move(scene)) {}

render::SemanticZoneFeature* SemanticZones::AddZone(const std::string& id, ZoneType type,
                                                    const std::vector<LngLat>& polygon) {
    render::SemanticZoneFeature zone;
    zone.id = id;
    zone.type = type;
    zone.polygon = polygon;
    const auto& preset = ZoneStylePresets().at(type);
    zone.style.fillColor = preset.fillColor;
    zone.style.fillOpacity = preset.fillOpacity;
    zone.style.outlineColor = preset.outlineColor;
    zone.style.outlineWidth = preset.outlineWidth;
    zone.style.outlineDash = preset.outlineDash;
    scene_->semanticZones.push_back(zone);
    scene_->Touch();
    return &scene_->semanticZones.back();
}

bool SemanticZones::RemoveZone(const std::string& id) {
    auto& zones = scene_->semanticZones;
    const auto it = std::remove_if(zones.begin(), zones.end(),
                                   [&](const render::SemanticZoneFeature& z) { return z.id == id; });
    if (it == zones.end()) {
        return false;
    }
    zones.erase(it, zones.end());
    scene_->Touch();
    return true;
}

void SemanticZones::Update(const std::vector<render::SemanticZoneFeature>& zones) {
    scene_->semanticZones = zones;
    scene_->Touch();
}

void SemanticZones::Show() {
    visible_ = true;
    for (auto& z : scene_->semanticZones) {
        z.visible = true;
    }
    scene_->Touch();
}

void SemanticZones::Hide() {
    visible_ = false;
    for (auto& z : scene_->semanticZones) {
        z.visible = false;
    }
    scene_->Touch();
}

std::vector<render::SemanticZoneFeature> SemanticZones::GetZones() const {
    return scene_->semanticZones;
}

void SemanticZones::Remove() {
    scene_->semanticZones.clear();
    scene_->Touch();
}

Buildings::Buildings(render::SceneState::SharedPtr scene)
    : scene_(std::move(scene)), events_(utils::EventEmitter::make_shared()) {}

void Buildings::Show() {
    for (auto& b : scene_->buildings) {
        b.visible = true;
    }
    scene_->Touch();
}

void Buildings::Hide() {
    for (auto& b : scene_->buildings) {
        b.visible = false;
    }
    scene_->Touch();
}

void Buildings::Remove() {
    scene_->buildings.clear();
    scene_->Touch();
}

void Buildings::Update(const std::vector<render::BuildingFeature>& buildings) {
    scene_->buildings = buildings;
    scene_->Touch();
}

void Buildings::SetHeightScale(double scale) {
    for (auto& b : scene_->buildings) {
        b.heightScale = scale;
    }
    scene_->Touch();
}

void Buildings::SetOpacity(float opacity) {
    for (auto& b : scene_->buildings) {
        b.opacity = opacity;
    }
    scene_->Touch();
}

void Buildings::OnFeature(const std::string& featureId, std::function<void()> handler) {
    events_->On(featureId, [handler](const std::string&, const std::string&) { handler(); });
}

LayerEvents::LayerEvents() : emitter_(utils::EventEmitter::make_shared()) {}

void LayerEvents::On(const std::string& layerId, utils::EventEmitter::Handler handler) {
    emitter_->On(layerId, std::move(handler));
}

void LayerEvents::Off(const std::string& layerId) { emitter_->Off(layerId); }

void LayerEvents::Once(const std::string& layerId, utils::EventEmitter::Handler handler) {
    emitter_->Once(layerId, std::move(handler));
}

void LayerEvents::OnFeature(const std::string& featureId, utils::EventEmitter::Handler handler) {
    emitter_->On("feature:" + featureId, std::move(handler));
}

void LayerEvents::OffFeature(const std::string& featureId) {
    emitter_->Off("feature:" + featureId);
}

void LayerEvents::Dispatch(const std::string& layerId, const std::string& featureId) {
    emitter_->Emit(layerId, featureId);
    emitter_->Emit("feature:" + featureId, layerId);
}

void LayerEvents::Destroy() { emitter_->Off("*"); }

}  // namespace map_features
}  // namespace strata
}  // namespace map
}  // namespace autonomy
