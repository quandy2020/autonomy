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

#include "autonomy/map/strata/shapes/edit_controller.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace shapes {

PolygonEditController::PolygonEditController(render::SceneState::SharedPtr scene,
                                             PolygonCollection::SharedPtr collection)
    : scene_(std::move(scene)), collection_(std::move(collection)) {}

bool PolygonEditController::EnterEditMode(const std::string& polygon_id,
                                          const EditModeCallbacks& callbacks) {
    const auto feature = collection_->GetById(polygon_id);
    if (!feature.has_value() || feature->points.empty()) {
        return false;
    }
    is_edit_mode_ = true;
    selected_id_ = polygon_id;
    edit_points_ = feature->points;
    callbacks_ = callbacks;
    if (callbacks_.onEditStart) {
        callbacks_.onEditStart(polygon_id, edit_points_);
    }
    scene_->Touch();
    return true;
}

bool PolygonEditController::ExitEditMode() {
    if (!is_edit_mode_ || !selected_id_.has_value()) {
        return false;
    }
    const std::string feature_id = *selected_id_;
    if (callbacks_.onEditEnd) {
        callbacks_.onEditEnd(feature_id, edit_points_);
    }
    is_edit_mode_ = false;
    selected_id_.reset();
    edit_points_.clear();
    callbacks_ = {};
    scene_->Touch();
    return true;
}

std::vector<LngLat> PolygonEditController::GetEditPoints() const {
    return is_edit_mode_ ? edit_points_ : std::vector<LngLat>{};
}

std::optional<PolygonFeature> PolygonEditController::GetEditData() const {
    if (!is_edit_mode_ || !selected_id_.has_value()) {
        return std::nullopt;
    }
    auto feature = collection_->GetById(*selected_id_);
    if (!feature.has_value()) {
        return std::nullopt;
    }
    feature->points = edit_points_;
    return feature;
}

bool PolygonEditController::UpdateEditPoint(size_t index, const LngLat& point) {
    if (!is_edit_mode_ || index >= edit_points_.size() || !selected_id_.has_value()) {
        return false;
    }
    edit_points_[index] = point;
    NotifyUpdate();
    return true;
}

bool PolygonEditController::CommitEdit() {
    if (!is_edit_mode_ || !selected_id_.has_value()) {
        return false;
    }
    auto feature = collection_->GetById(*selected_id_);
    if (!feature.has_value()) {
        return false;
    }
    feature->points = edit_points_;
    collection_->Update(*selected_id_, *feature);
    return ExitEditMode();
}

void PolygonEditController::NotifyUpdate() const {
    if (callbacks_.onEditUpdate && selected_id_.has_value()) {
        callbacks_.onEditUpdate(*selected_id_, edit_points_);
    }
    scene_->Touch();
}

WideLineEditController::WideLineEditController(render::SceneState::SharedPtr scene,
                                               WideLineCollection::SharedPtr collection)
    : scene_(std::move(scene)), collection_(std::move(collection)) {}

bool WideLineEditController::EnterEditMode(const std::string& wide_line_id,
                                           const EditModeCallbacks& callbacks) {
    const auto feature = collection_->GetById(wide_line_id);
    if (!feature.has_value() || feature->path.empty()) {
        return false;
    }
    is_edit_mode_ = true;
    selected_id_ = wide_line_id;
    edit_points_ = feature->path;
    callbacks_ = callbacks;
    if (callbacks_.onEditStart) {
        callbacks_.onEditStart(wide_line_id, edit_points_);
    }
    scene_->Touch();
    return true;
}

bool WideLineEditController::ExitEditMode() {
    if (!is_edit_mode_ || !selected_id_.has_value()) {
        return false;
    }
    const std::string feature_id = *selected_id_;
    if (callbacks_.onEditEnd) {
        callbacks_.onEditEnd(feature_id, edit_points_);
    }
    is_edit_mode_ = false;
    selected_id_.reset();
    edit_points_.clear();
    callbacks_ = {};
    scene_->Touch();
    return true;
}

std::vector<LngLat> WideLineEditController::GetEditPoints() const {
    return is_edit_mode_ ? edit_points_ : std::vector<LngLat>{};
}

std::optional<WideLineFeature> WideLineEditController::GetEditData() const {
    if (!is_edit_mode_ || !selected_id_.has_value()) {
        return std::nullopt;
    }
    auto feature = collection_->GetById(*selected_id_);
    if (!feature.has_value()) {
        return std::nullopt;
    }
    feature->path = edit_points_;
    return feature;
}

bool WideLineEditController::UpdateEditPoint(size_t index, const LngLat& point) {
    if (!is_edit_mode_ || index >= edit_points_.size() || !selected_id_.has_value()) {
        return false;
    }
    edit_points_[index] = point;
    NotifyUpdate();
    return true;
}

bool WideLineEditController::CommitEdit() {
    if (!is_edit_mode_ || !selected_id_.has_value()) {
        return false;
    }
    auto feature = collection_->GetById(*selected_id_);
    if (!feature.has_value()) {
        return false;
    }
    feature->path = edit_points_;
    collection_->Update(*selected_id_, *feature);
    return ExitEditMode();
}

void WideLineEditController::NotifyUpdate() const {
    if (callbacks_.onEditUpdate && selected_id_.has_value()) {
        callbacks_.onEditUpdate(*selected_id_, edit_points_);
    }
    scene_->Touch();
}

}  // namespace shapes
}  // namespace strata
}  // namespace map
}  // namespace autonomy
