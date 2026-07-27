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
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace visual {

class RobotFovController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(RobotFovController)

    RobotFovController(render::SceneState::SharedPtr scene, RobotFovOptions options);

    void Update(const LngLat& position, double heading_deg);
    void SetConfig(const RobotFovOptions& options);
    void Show();
    void Hide();
    void Remove();
    const RobotFovState& State() const { return state_; }

private:
    void SyncState();

    render::SceneState::SharedPtr scene_;
    RobotFovState state_;
};

class StatusRobotMarkerController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(StatusRobotMarkerController)

    explicit StatusRobotMarkerController(render::SceneState::SharedPtr scene);

    size_t AddRobot(const RobotMarker& robot);
    void AddBatch(const std::vector<RobotMarker>& robots);
    bool UpdateRobot(const std::string& robot_id, const RobotMarkerPatch& patch);
    bool UpdateRobot(const RobotMarker& robot);
    void UpdateRobots(const std::vector<RobotMarker>& robots);
    bool RemoveRobot(const std::string& robot_id);
    void ClearRobots();
    std::vector<RobotMarker> GetRobots() const;
    void ToggleLabels(bool show = true);
    void SetVisible(bool visible);
    void Remove();

private:
    void SyncStatusString(RobotMarker& robot) const;

    render::SceneState::SharedPtr scene_;
};

class Robot3DLayerController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Robot3DLayerController)

    Robot3DLayerController(render::SceneState::SharedPtr scene, Robot3DModelConfig model,
                           const Robot3DLayerState& initial_state);

    void Update(const LngLat& position, double heading_deg, RobotStatus status);
    void Show();
    void Hide();
    void Remove();
    const Robot3DLayerState& State() const { return state_; }

private:
    void SyncState();

    render::SceneState::SharedPtr scene_;
    Robot3DLayerState state_;
    size_t scene_index_{0};
};

class Robot3DStatusLayer
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Robot3DStatusLayer)

    Robot3DStatusLayer(render::SceneState::SharedPtr scene, Robot3DModelConfig model);

    size_t AddRobot(const RobotMarker& robot);
    bool UpdateRobot(const std::string& robot_id, const RobotMarkerPatch& patch);
    void UpdateRobots(const std::vector<RobotMarker>& robots);
    bool RemoveRobot(const std::string& robot_id);
    void ClearRobots();
    std::vector<RobotMarker> GetRobots() const;
    void ToggleLabels(bool /*show*/ = true) {}
    void Remove();

private:
    RobotStatus StatusFromPatch(const RobotMarker& robot, const RobotMarkerPatch& patch) const;

    render::SceneState::SharedPtr scene_;
    Robot3DModelConfig model_;
    std::vector<RobotMarker> robots_;
    std::unordered_map<std::string, Robot3DLayerController::SharedPtr> layers_;
};

}  // namespace visual
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
