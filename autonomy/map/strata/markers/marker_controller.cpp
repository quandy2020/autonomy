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

#include <chrono>

#include "autonomy/map/strata/markers/marker_controller.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace markers {

namespace {
std::string MakeId(const std::string& prefix) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return prefix + std::to_string(now);
}
}

DirectionalMarkerController::DirectionalMarkerController(render::SceneState::SharedPtr scene)
    : scene_(std::move(scene)) {}

DirectionalMarker* DirectionalMarkerController::Add(const LngLat& position,
                                                    const std::string& id) {
    DirectionalMarker marker;
    marker.id = id.empty() ? MakeId("dir-") : id;
    marker.position = position;
    scene_->directionalMarkers.push_back(marker);
    scene_->Touch();
    return &scene_->directionalMarkers.back();
}

bool DirectionalMarkerController::SetPosition(const std::string& id, const LngLat& position) {
    for (auto& m : scene_->directionalMarkers) {
        if (m.id == id) {
            m.position = position;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool DirectionalMarkerController::SetRotation(const std::string& id, double rotationDeg) {
    for (auto& m : scene_->directionalMarkers) {
        if (m.id == id) {
            m.rotationDeg = rotationDeg;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool DirectionalMarkerController::EnableEditMode(const std::string& id) {
    for (auto& m : scene_->directionalMarkers) {
        if (m.id == id) {
            m.editMode = true;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool DirectionalMarkerController::DisableEditMode(const std::string& id) {
    for (auto& m : scene_->directionalMarkers) {
        if (m.id == id) {
            m.editMode = false;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool DirectionalMarkerController::ToggleEditMode(const std::string& id) {
    for (auto& m : scene_->directionalMarkers) {
        if (m.id == id) {
            m.editMode = !m.editMode;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool DirectionalMarkerController::IsInEditMode(const std::string& id) const {
    for (const auto& m : scene_->directionalMarkers) {
        if (m.id == id) {
            return m.editMode;
        }
    }
    return false;
}

bool DirectionalMarkerController::Remove(const std::string& id) {
    auto& markers = scene_->directionalMarkers;
    const auto it = std::remove_if(markers.begin(), markers.end(),
                                   [&](const DirectionalMarker& m) { return m.id == id; });
    if (it == markers.end()) {
        return false;
    }
    markers.erase(it, markers.end());
    scene_->Touch();
    return true;
}

void DirectionalMarkerController::RemoveAll() {
    scene_->directionalMarkers.clear();
    scene_->Touch();
}

PoiMarkerController::PoiMarkerController(render::SceneState::SharedPtr scene)
    : scene_(std::move(scene)) {}

void PoiMarkerController::AddBatch(const std::vector<PoiMarker>& markers) {
    scene_->poiMarkers.insert(scene_->poiMarkers.end(), markers.begin(), markers.end());
    scene_->Touch();
}

PoiMarker* PoiMarkerController::AddMarker(const PoiMarker& marker) {
    scene_->poiMarkers.push_back(marker);
    scene_->Touch();
    return &scene_->poiMarkers.back();
}

bool PoiMarkerController::UpdateMarker(const PoiMarker& marker) {
    for (auto& m : scene_->poiMarkers) {
        if (m.id == marker.id) {
            m = marker;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool PoiMarkerController::RemoveMarker(const std::string& id) {
    auto& markers = scene_->poiMarkers;
    const auto it = std::remove_if(markers.begin(), markers.end(),
                                   [&](const PoiMarker& m) { return m.id == id; });
    if (it == markers.end()) {
        return false;
    }
    markers.erase(it, markers.end());
    scene_->Touch();
    return true;
}

void PoiMarkerController::ClearMarkers() {
    scene_->poiMarkers.clear();
    scene_->Touch();
}

std::vector<PoiMarker> PoiMarkerController::GetMarkers() const { return scene_->poiMarkers; }

std::optional<PoiMarker> PoiMarkerController::GetMarkerById(const std::string& id) const {
    for (const auto& marker : scene_->poiMarkers) {
        if (marker.id == id) {
            return marker;
        }
    }
    return std::nullopt;
}

std::vector<PoiMarker> PoiMarkerController::GetSelectedMarkers() const {
    std::vector<PoiMarker> selected;
    for (const auto& marker : scene_->poiMarkers) {
        if (marker.selected) {
            selected.push_back(marker);
        }
    }
    return selected;
}

std::optional<PoiMarker> PoiMarkerController::GetSelectedMarker() const {
    for (const auto& marker : scene_->poiMarkers) {
        if (marker.selected) {
            return marker;
        }
    }
    return std::nullopt;
}

bool PoiMarkerController::SelectById(const std::string& id) {
    bool found = false;
    for (auto& m : scene_->poiMarkers) {
        m.selected = (m.id == id);
        if (m.selected) {
            found = true;
        }
    }
    scene_->Touch();
    return found;
}

bool PoiMarkerController::ToggleSelection(const std::string& id) {
    if (!selectable_) {
        return false;
    }
    for (auto& marker : scene_->poiMarkers) {
        if (marker.id == id) {
            marker.selected = !marker.selected;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

void PoiMarkerController::SetSelection(const std::vector<std::string>& ids) {
    if (!selectable_) {
        return;
    }
    for (auto& marker : scene_->poiMarkers) {
        marker.selected = std::find(ids.begin(), ids.end(), marker.id) != ids.end();
    }
    scene_->Touch();
}

void PoiMarkerController::ClearSelection() {
    for (auto& m : scene_->poiMarkers) {
        m.selected = false;
    }
    scene_->Touch();
}

void PoiMarkerController::SelectAll() {
    if (!selectable_) {
        return;
    }
    for (auto& marker : scene_->poiMarkers) {
        marker.selected = true;
    }
    scene_->Touch();
}

void PoiMarkerController::ToggleLabels(bool visible) {
    labelsVisible_ = visible;
    scene_->Touch();
}

void PoiMarkerController::SetSelectable(bool selectable) { selectable_ = selectable; }

void PoiMarkerController::SetOnClick(MarkerClickCallback callback) {
    on_click_ = std::move(callback);
}

bool PoiMarkerController::HandleClick(const std::string& id) {
    if (!on_click_) {
        return false;
    }
    const auto marker = GetMarkerById(id);
    if (!marker.has_value()) {
        return false;
    }
    on_click_(id);
    return true;
}

void PoiMarkerController::Remove() { ClearMarkers(); }

RobotMarkerController::RobotMarkerController(render::SceneState::SharedPtr scene)
    : scene_(std::move(scene)) {}

void RobotMarkerController::AddBatch(const std::vector<RobotMarker>& robots) {
    scene_->robotMarkers.insert(scene_->robotMarkers.end(), robots.begin(), robots.end());
    scene_->Touch();
}

RobotMarker* RobotMarkerController::AddRobot(const RobotMarker& robot) {
    scene_->robotMarkers.push_back(robot);
    scene_->Touch();
    return &scene_->robotMarkers.back();
}

bool RobotMarkerController::UpdateRobot(const RobotMarker& robot) {
    for (auto& r : scene_->robotMarkers) {
        if (r.id == robot.id) {
            r = robot;
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool RobotMarkerController::RemoveRobot(const std::string& id) {
    auto& robots = scene_->robotMarkers;
    const auto it = std::remove_if(robots.begin(), robots.end(),
                                   [&](const RobotMarker& r) { return r.id == id; });
    if (it == robots.end()) {
        return false;
    }
    robots.erase(it, robots.end());
    scene_->Touch();
    return true;
}

void RobotMarkerController::ClearRobots() {
    scene_->robotMarkers.clear();
    scene_->Touch();
}

std::vector<RobotMarker> RobotMarkerController::GetRobots() const { return scene_->robotMarkers; }

std::vector<std::string> RobotMarkerController::GetRobotIds() const {
    std::vector<std::string> ids;
    ids.reserve(scene_->robotMarkers.size());
    for (const auto& robot : scene_->robotMarkers) {
        ids.push_back(robot.id);
    }
    return ids;
}

void RobotMarkerController::ToggleLabels(bool visible) {
    labelsVisible_ = visible;
    for (auto& r : scene_->robotMarkers) {
        r.showLabel = visible;
    }
    scene_->Touch();
}

void RobotMarkerController::SetVisible(bool visible) {
    for (auto& r : scene_->robotMarkers) {
        r.visible = visible;
    }
    scene_->Touch();
}

void RobotMarkerController::SetOnClick(MarkerClickCallback callback) {
    on_click_ = std::move(callback);
}

bool RobotMarkerController::HandleClick(const std::string& id) {
    if (!on_click_) {
        return false;
    }
    for (const auto& robot : scene_->robotMarkers) {
        if (robot.id == id) {
            on_click_(id);
            return true;
        }
    }
    return false;
}

void RobotMarkerController::Remove() { ClearRobots(); }

}  // namespace markers
}  // namespace strata
}  // namespace map
}  // namespace autonomy
