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

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace markers {

class DirectionalMarkerController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(DirectionalMarkerController)

    explicit DirectionalMarkerController(render::SceneState::SharedPtr scene);

    DirectionalMarker* Add(const LngLat& position, const std::string& id = "");
    bool SetPosition(const std::string& id, const LngLat& position);
    bool SetRotation(const std::string& id, double rotationDeg);
    bool EnableEditMode(const std::string& id);
    bool DisableEditMode(const std::string& id);
    bool ToggleEditMode(const std::string& id);
    bool IsInEditMode(const std::string& id) const;
    bool Remove(const std::string& id);
    void RemoveAll();

private:
    render::SceneState::SharedPtr scene_;
};

class PoiMarkerController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(PoiMarkerController)

    explicit PoiMarkerController(render::SceneState::SharedPtr scene);

    void AddBatch(const std::vector<PoiMarker>& markers);
    PoiMarker* AddMarker(const PoiMarker& marker);
    bool UpdateMarker(const PoiMarker& marker);
    bool RemoveMarker(const std::string& id);
    void ClearMarkers();
    std::vector<PoiMarker> GetMarkers() const;
    std::optional<PoiMarker> GetMarkerById(const std::string& id) const;
    std::vector<PoiMarker> GetSelectedMarkers() const;
    std::optional<PoiMarker> GetSelectedMarker() const;
    bool SelectById(const std::string& id);
    bool ToggleSelection(const std::string& id);
    void SetSelection(const std::vector<std::string>& ids);
    void ClearSelection();
    void SelectAll();
    void ToggleLabels(bool visible);
    void SetSelectable(bool selectable);
    bool IsSelectable() const { return selectable_; }
    void SetOnClick(MarkerClickCallback callback);
    bool HandleClick(const std::string& id);
    void Remove();

private:
    render::SceneState::SharedPtr scene_;
    bool labelsVisible_{true};
    bool selectable_{true};
    MarkerClickCallback on_click_;
};

class RobotMarkerController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(RobotMarkerController)

    explicit RobotMarkerController(render::SceneState::SharedPtr scene);

    void AddBatch(const std::vector<RobotMarker>& robots);
    RobotMarker* AddRobot(const RobotMarker& robot);
    bool UpdateRobot(const RobotMarker& robot);
    bool RemoveRobot(const std::string& id);
    void ClearRobots();
    std::vector<RobotMarker> GetRobots() const;
    std::vector<std::string> GetRobotIds() const;
    void ToggleLabels(bool visible);
    void SetVisible(bool visible);
    void SetOnClick(MarkerClickCallback callback);
    bool HandleClick(const std::string& id);
    void Remove();

private:
    render::SceneState::SharedPtr scene_;
    bool labelsVisible_{true};
    MarkerClickCallback on_click_;
};

}  // namespace markers
}  // namespace strata
}  // namespace map
}  // namespace autonomy
