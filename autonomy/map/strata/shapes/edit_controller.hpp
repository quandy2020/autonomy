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
#include "autonomy/map/strata/shapes/shape_controller.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace shapes {

class PolygonEditController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(PolygonEditController)

    PolygonEditController(render::SceneState::SharedPtr scene,
                          PolygonCollection::SharedPtr collection);

    bool EnterEditMode(const std::string& polygon_id, const EditModeCallbacks& callbacks = {});
    bool ExitEditMode();
    bool IsEditMode() const { return is_edit_mode_; }
    std::optional<std::string> SelectedId() const { return selected_id_; }
    std::vector<LngLat> GetEditPoints() const;
    std::optional<PolygonFeature> GetEditData() const;
    bool UpdateEditPoint(size_t index, const LngLat& point);
    bool CommitEdit();

private:
    void NotifyUpdate() const;

    render::SceneState::SharedPtr scene_;
    PolygonCollection::SharedPtr collection_;
    bool is_edit_mode_{false};
    std::optional<std::string> selected_id_;
    std::vector<LngLat> edit_points_;
    EditModeCallbacks callbacks_;
};

class WideLineEditController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(WideLineEditController)

    WideLineEditController(render::SceneState::SharedPtr scene,
                           WideLineCollection::SharedPtr collection);

    bool EnterEditMode(const std::string& wide_line_id, const EditModeCallbacks& callbacks = {});
    bool ExitEditMode();
    bool IsEditMode() const { return is_edit_mode_; }
    std::optional<std::string> SelectedId() const { return selected_id_; }
    std::vector<LngLat> GetEditPoints() const;
    std::optional<WideLineFeature> GetEditData() const;
    bool UpdateEditPoint(size_t index, const LngLat& point);
    bool CommitEdit();

private:
    void NotifyUpdate() const;

    render::SceneState::SharedPtr scene_;
    WideLineCollection::SharedPtr collection_;
    bool is_edit_mode_{false};
    std::optional<std::string> selected_id_;
    std::vector<LngLat> edit_points_;
    EditModeCallbacks callbacks_;
};

}  // namespace shapes
}  // namespace strata
}  // namespace map
}  // namespace autonomy
