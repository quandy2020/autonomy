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

#include <chrono>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace overlay {

class LabelBubbleController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LabelBubbleController)

    LabelBubbleController(render::SceneState::SharedPtr scene, LabelBubbleOptions options);

    void Show();
    void Hide();
    void SetLngLat(const LngLat& lngLat);
    void SetHtml(const std::string& html);
    void SetScreenOffset(float offsetX, float offsetY);
    void Remove();

private:
    render::SceneState::SharedPtr scene_;
    LabelBubbleOptions options_;
    size_t index_{0};
};

class IotBubbleController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(IotBubbleController)

    IotBubbleController(render::SceneState::SharedPtr scene, int defaultDurationMs, int maxBubbles);

    std::string Emit(IotEventType type, const LngLat& position, const std::string& message = {});
    void Update(const std::string& id, IotEventType type, const std::string& message);
    void Dismiss(const std::string& id);
    void Clear();
    void Remove();
    void PruneExpired();

private:
    render::SceneState::SharedPtr scene_;
    int defaultDurationMs_;
    int maxBubbles_;
};

}  // namespace overlay
}  // namespace strata
}  // namespace map
}  // namespace autonomy
