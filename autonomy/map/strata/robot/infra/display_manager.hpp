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

#include <cstddef>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/robot/visual/robot_visual.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace infra {

enum class DisplayMode { k2D, k3D, kBoth };

struct DisplayManagerConfig {
    DisplayMode mode{DisplayMode::k2D};
    visual::StatusRobotMarkerController::SharedPtr renderer2d;
    visual::Robot3DStatusLayer::SharedPtr renderer3d;
};

/**
 * @brief Unified 2D/3D robot display orchestrator (BICMap DisplayManager).
 */
class DisplayManager
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(DisplayManager)

    DisplayManager();
    explicit DisplayManager(DisplayManagerConfig config);

    void SetMode(DisplayMode mode);
    DisplayMode GetMode() const { return mode_; }

    void SetRenderer2d(visual::StatusRobotMarkerController::SharedPtr renderer);
    void SetRenderer3d(visual::Robot3DStatusLayer::SharedPtr renderer);

    size_t AddRobot(const RobotMarker& robot);
    bool UpdateRobot(const std::string& id, const RobotMarkerPatch& patch);
    bool UpdateRobot(size_t index, const RobotMarkerPatch& patch);
    void UpdateRobots(const std::vector<RobotMarker>& robots);
    bool RemoveRobot(const std::string& id);
    bool RemoveRobot(size_t index);
    void ClearRobots();
    std::vector<RobotMarker> GetRobots() const;
    void ToggleLabels(bool show = true);
    void Remove();

private:
    visual::StatusRobotMarkerController::SharedPtr ActiveRenderer2d() const;
    visual::Robot3DStatusLayer::SharedPtr ActiveRenderer3d() const;
    int FindIndex(const std::string& id) const;

    DisplayMode mode_;
    visual::StatusRobotMarkerController::SharedPtr renderer2d_;
    visual::Robot3DStatusLayer::SharedPtr renderer3d_;
    std::vector<RobotMarker> robots_;
};

}  // namespace infra
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
