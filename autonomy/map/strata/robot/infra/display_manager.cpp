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

#include "autonomy/map/strata/robot/infra/display_manager.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace infra {

DisplayManager::DisplayManager() : DisplayManager(DisplayManagerConfig{}) {}

DisplayManager::DisplayManager(DisplayManagerConfig config)
    : mode_(config.mode),
      renderer2d_(std::move(config.renderer2d)),
      renderer3d_(std::move(config.renderer3d)) {}

visual::StatusRobotMarkerController::SharedPtr DisplayManager::ActiveRenderer2d() const {
    if (mode_ == DisplayMode::k2D || mode_ == DisplayMode::kBoth) {
        return renderer2d_;
    }
    return nullptr;
}

visual::Robot3DStatusLayer::SharedPtr DisplayManager::ActiveRenderer3d() const {
    if (mode_ == DisplayMode::k3D || mode_ == DisplayMode::kBoth) {
        return renderer3d_;
    }
    return nullptr;
}

int DisplayManager::FindIndex(const std::string& id) const {
    for (size_t i = 0; i < robots_.size(); ++i) {
        if (robots_[i].id == id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void DisplayManager::SetMode(DisplayMode mode) {
    const bool was2d = mode_ == DisplayMode::k2D || mode_ == DisplayMode::kBoth;
    const bool was3d = mode_ == DisplayMode::k3D || mode_ == DisplayMode::kBoth;
    mode_ = mode;
    const bool is2d = mode_ == DisplayMode::k2D || mode_ == DisplayMode::kBoth;
    const bool is3d = mode_ == DisplayMode::k3D || mode_ == DisplayMode::kBoth;

    if (renderer2d_) {
        if (was2d && !is2d) {
            renderer2d_->ClearRobots();
        } else if (!was2d && is2d) {
            renderer2d_->UpdateRobots(robots_);
        }
    }
    if (renderer3d_) {
        if (was3d && !is3d) {
            renderer3d_->ClearRobots();
        } else if (!was3d && is3d) {
            renderer3d_->UpdateRobots(robots_);
        }
    }
}

void DisplayManager::SetRenderer2d(visual::StatusRobotMarkerController::SharedPtr renderer) {
    renderer2d_ = std::move(renderer);
}

void DisplayManager::SetRenderer3d(visual::Robot3DStatusLayer::SharedPtr renderer) {
    renderer3d_ = std::move(renderer);
}

size_t DisplayManager::AddRobot(const RobotMarker& robot) {
    robots_.push_back(robot);
    if (auto renderer = ActiveRenderer2d()) {
        renderer->AddRobot(robot);
    }
    if (auto renderer = ActiveRenderer3d()) {
        renderer->AddRobot(robot);
    }
    return robots_.size() - 1;
}

bool DisplayManager::UpdateRobot(const std::string& id, const RobotMarkerPatch& patch) {
    const int idx = FindIndex(id);
    if (idx < 0) {
        return false;
    }
    return UpdateRobot(static_cast<size_t>(idx), patch);
}

bool DisplayManager::UpdateRobot(size_t index, const RobotMarkerPatch& patch) {
    if (index >= robots_.size()) {
        return false;
    }
    ApplyRobotMarkerPatch(robots_[index], patch);
    if (auto renderer = ActiveRenderer2d()) {
        renderer->UpdateRobot(robots_[index].id, patch);
    }
    if (auto renderer = ActiveRenderer3d()) {
        renderer->UpdateRobot(robots_[index].id, patch);
    }
    return true;
}

void DisplayManager::UpdateRobots(const std::vector<RobotMarker>& robots) {
    robots_ = robots;
    if (auto renderer = ActiveRenderer2d()) {
        renderer->UpdateRobots(robots_);
    }
    if (auto renderer = ActiveRenderer3d()) {
        renderer->UpdateRobots(robots_);
    }
}

bool DisplayManager::RemoveRobot(const std::string& id) {
    const int idx = FindIndex(id);
    if (idx < 0) {
        return false;
    }
    return RemoveRobot(static_cast<size_t>(idx));
}

bool DisplayManager::RemoveRobot(size_t index) {
    if (index >= robots_.size()) {
        return false;
    }
    const std::string id = robots_[index].id;
    if (auto renderer = ActiveRenderer2d()) {
        renderer->RemoveRobot(id);
    }
    if (auto renderer = ActiveRenderer3d()) {
        renderer->RemoveRobot(id);
    }
    robots_.erase(robots_.begin() + static_cast<long>(index));
    return true;
}

void DisplayManager::ClearRobots() {
    robots_.clear();
    if (auto renderer = ActiveRenderer2d()) {
        renderer->ClearRobots();
    }
    if (auto renderer = ActiveRenderer3d()) {
        renderer->ClearRobots();
    }
}

std::vector<RobotMarker> DisplayManager::GetRobots() const { return robots_; }

void DisplayManager::ToggleLabels(bool show) {
    if (auto renderer = ActiveRenderer2d()) {
        renderer->ToggleLabels(show);
    }
    if (auto renderer = ActiveRenderer3d()) {
        renderer->ToggleLabels(show);
    }
}

void DisplayManager::Remove() {
    if (renderer2d_) {
        renderer2d_->Remove();
    }
    if (renderer3d_) {
        renderer3d_->Remove();
    }
    robots_.clear();
}

}  // namespace infra
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
