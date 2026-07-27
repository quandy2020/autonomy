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
#include "autonomy/map/strata/robot/visual/fov_geometry.hpp"
#include "autonomy/map/strata/robot/visual/robot_visual.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace visual {

RobotFovController::RobotFovController(render::SceneState::SharedPtr scene,
                                       RobotFovOptions options)
    : scene_(std::move(scene)) {
    state_.id = options.id;
    state_.options = std::move(options);
    state_.visible = state_.options.enabled;
    SyncState();
}

void RobotFovController::Update(const LngLat& position, double heading_deg) {
    state_.position = position;
    state_.headingDeg = heading_deg;
    state_.bands = BuildFovBands(position, heading_deg, state_.options);
    SyncState();
}

void RobotFovController::SetConfig(const RobotFovOptions& options) {
    state_.options = options;
    state_.bands = BuildFovBands(state_.position, state_.headingDeg, state_.options);
    SyncState();
}

void RobotFovController::Show() {
    state_.visible = true;
    SyncState();
}

void RobotFovController::Hide() {
    state_.visible = false;
    SyncState();
}

void RobotFovController::Remove() {
    auto& fovs = scene_->robotFovs;
    fovs.erase(std::remove_if(fovs.begin(), fovs.end(),
                              [&](const RobotFovState& fov) { return fov.id == state_.id; }),
               fovs.end());
    scene_->Touch();
}

void RobotFovController::SyncState() {
    auto& fovs = scene_->robotFovs;
    const auto it = std::find_if(fovs.begin(), fovs.end(),
                                 [&](const RobotFovState& fov) { return fov.id == state_.id; });
    if (it == fovs.end()) {
        fovs.push_back(state_);
    } else {
        *it = state_;
    }
    scene_->Touch();
}

StatusRobotMarkerController::StatusRobotMarkerController(render::SceneState::SharedPtr scene)
    : scene_(std::move(scene)) {}

void StatusRobotMarkerController::SyncStatusString(RobotMarker& robot) const {
    const auto& styles = RobotStatusStyles();
    const auto it = styles.find(robot.robotStatus);
    if (it != styles.end()) {
        robot.status = it->second.label;
    } else {
        robot.status = ToString(robot.robotStatus);
    }
}

size_t StatusRobotMarkerController::AddRobot(const RobotMarker& robot) {
    RobotMarker copy = robot;
    SyncStatusString(copy);
    scene_->robotMarkers.push_back(std::move(copy));
    scene_->Touch();
    return scene_->robotMarkers.size() - 1;
}

void StatusRobotMarkerController::AddBatch(const std::vector<RobotMarker>& robots) {
    for (auto robot : robots) {
        SyncStatusString(robot);
        scene_->robotMarkers.push_back(std::move(robot));
    }
    scene_->Touch();
}

bool StatusRobotMarkerController::UpdateRobot(const std::string& robot_id,
                                              const RobotMarkerPatch& patch) {
    for (auto& existing : scene_->robotMarkers) {
        if (existing.id == robot_id) {
            ApplyRobotMarkerPatch(existing, patch);
            SyncStatusString(existing);
            scene_->Touch();
            return true;
        }
    }
    return false;
}

bool StatusRobotMarkerController::UpdateRobot(const RobotMarker& robot) {
    for (auto& existing : scene_->robotMarkers) {
        if (existing.id == robot.id) {
            existing = robot;
            SyncStatusString(existing);
            scene_->Touch();
            return true;
        }
    }
    return false;
}

void StatusRobotMarkerController::UpdateRobots(const std::vector<RobotMarker>& robots) {
    scene_->robotMarkers.clear();
    AddBatch(robots);
}

bool StatusRobotMarkerController::RemoveRobot(const std::string& robot_id) {
    auto& robots = scene_->robotMarkers;
    const auto it = std::remove_if(robots.begin(), robots.end(),
                                   [&](const RobotMarker& robot) { return robot.id == robot_id; });
    if (it == robots.end()) {
        return false;
    }
    robots.erase(it, robots.end());
    scene_->Touch();
    return true;
}

void StatusRobotMarkerController::ClearRobots() {
    scene_->robotMarkers.clear();
    scene_->Touch();
}

std::vector<RobotMarker> StatusRobotMarkerController::GetRobots() const {
    return scene_->robotMarkers;
}

void StatusRobotMarkerController::ToggleLabels(bool show) {
    for (auto& robot : scene_->robotMarkers) {
        robot.showLabel = show;
    }
    scene_->Touch();
}

void StatusRobotMarkerController::SetVisible(bool visible) {
    for (auto& robot : scene_->robotMarkers) {
        robot.visible = visible;
    }
    scene_->Touch();
}

void StatusRobotMarkerController::Remove() { ClearRobots(); }

Robot3DLayerController::Robot3DLayerController(render::SceneState::SharedPtr scene,
                                               Robot3DModelConfig model,
                                               const Robot3DLayerState& initial_state)
    : scene_(std::move(scene)), state_(initial_state) {
    state_.model = std::move(model);
    scene_index_ = scene_->robot3DLayers.size();
    scene_->robot3DLayers.push_back(state_);
    scene_->Touch();
}

void Robot3DLayerController::Update(const LngLat& position, double heading_deg,
                                    RobotStatus status) {
    state_.position = position;
    state_.headingDeg = heading_deg;
    state_.status = status;
    SyncState();
}

void Robot3DLayerController::Show() {
    state_.visible = true;
    SyncState();
}

void Robot3DLayerController::Hide() {
    state_.visible = false;
    SyncState();
}

void Robot3DLayerController::Remove() {
    if (scene_index_ < scene_->robot3DLayers.size()) {
        scene_->robot3DLayers.erase(scene_->robot3DLayers.begin() +
                                    static_cast<long>(scene_index_));
        scene_->Touch();
    }
}

void Robot3DLayerController::SyncState() {
    if (scene_index_ < scene_->robot3DLayers.size()) {
        scene_->robot3DLayers[scene_index_] = state_;
        scene_->Touch();
    }
}

Robot3DStatusLayer::Robot3DStatusLayer(render::SceneState::SharedPtr scene,
                                       Robot3DModelConfig model)
    : scene_(std::move(scene)), model_(std::move(model)) {}

RobotStatus Robot3DStatusLayer::StatusFromPatch(const RobotMarker& robot,
                                                const RobotMarkerPatch& patch) const {
    if (patch.robotStatus) {
        return *patch.robotStatus;
    }
    if (patch.status) {
        if (*patch.status == "running") {
            return RobotStatus::kRunning;
        }
        if (*patch.status == "idle") {
            return RobotStatus::kIdle;
        }
    }
    return robot.robotStatus;
}

size_t Robot3DStatusLayer::AddRobot(const RobotMarker& robot) {
    robots_.push_back(robot);
    Robot3DLayerState state;
    state.id = robot.id + "-3d";
    state.robotId = robot.id;
    state.position = robot.lngLat;
    state.headingDeg = robot.rotationDeg;
    state.status = robot.robotStatus;
    layers_[robot.id] =
        Robot3DLayerController::make_shared(scene_, model_, state);
    return robots_.size() - 1;
}

bool Robot3DStatusLayer::UpdateRobot(const std::string& robot_id,
                                     const RobotMarkerPatch& patch) {
    const auto it = std::find_if(robots_.begin(), robots_.end(),
                                 [&](const RobotMarker& robot) { return robot.id == robot_id; });
    if (it == robots_.end()) {
        return false;
    }
    ApplyRobotMarkerPatch(*it, patch);
    const auto layer_it = layers_.find(robot_id);
    if (layer_it == layers_.end()) {
        return false;
    }
    const LngLat position = patch.lngLat ? *patch.lngLat : it->lngLat;
    const double heading = patch.rotationDeg ? *patch.rotationDeg : it->rotationDeg;
    layer_it->second->Update(position, heading, StatusFromPatch(*it, patch));
    return true;
}

void Robot3DStatusLayer::UpdateRobots(const std::vector<RobotMarker>& robots) {
    ClearRobots();
    for (const auto& robot : robots) {
        AddRobot(robot);
    }
}

bool Robot3DStatusLayer::RemoveRobot(const std::string& robot_id) {
    const auto layer_it = layers_.find(robot_id);
    if (layer_it == layers_.end()) {
        return false;
    }
    layer_it->second->Remove();
    layers_.erase(layer_it);
    robots_.erase(std::remove_if(robots_.begin(), robots_.end(),
                                 [&](const RobotMarker& robot) { return robot.id == robot_id; }),
                  robots_.end());
    return true;
}

void Robot3DStatusLayer::ClearRobots() {
    for (auto& [_, layer] : layers_) {
        layer->Remove();
    }
    layers_.clear();
    robots_.clear();
}

std::vector<RobotMarker> Robot3DStatusLayer::GetRobots() const { return robots_; }

void Robot3DStatusLayer::Remove() { ClearRobots(); }

}  // namespace visual
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
