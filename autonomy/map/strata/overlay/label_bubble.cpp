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
#include <chrono>

#include "autonomy/map/strata/overlay/label_bubble.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace overlay {

LabelBubbleController::LabelBubbleController(render::SceneState::SharedPtr scene,
                                             LabelBubbleOptions options)
    : scene_(std::move(scene)), options_(std::move(options)) {
    index_ = scene_->labelBubbles.size();
    scene_->labelBubbles.push_back(options_);
    scene_->Touch();
}

void LabelBubbleController::Show() {
    options_.visible = true;
    scene_->labelBubbles[index_] = options_;
    scene_->Touch();
}

void LabelBubbleController::Hide() {
    options_.visible = false;
    scene_->labelBubbles[index_] = options_;
    scene_->Touch();
}

void LabelBubbleController::SetLngLat(const LngLat& lngLat) {
    options_.lngLat = lngLat;
    scene_->labelBubbles[index_] = options_;
    scene_->Touch();
}

void LabelBubbleController::SetHtml(const std::string& html) {
    options_.html = html;
    scene_->labelBubbles[index_] = options_;
    scene_->Touch();
}

void LabelBubbleController::SetScreenOffset(float offsetX, float offsetY) {
    options_.offsetX = offsetX;
    options_.offsetY = offsetY;
    scene_->labelBubbles[index_] = options_;
    scene_->Touch();
}

void LabelBubbleController::Remove() {
    if (index_ < scene_->labelBubbles.size()) {
        scene_->labelBubbles.erase(scene_->labelBubbles.begin() +
                                   static_cast<long>(index_));
        scene_->Touch();
    }
}

IotBubbleController::IotBubbleController(render::SceneState::SharedPtr scene, int defaultDurationMs,
                                         int maxBubbles)
    : scene_(std::move(scene)),
      defaultDurationMs_(defaultDurationMs),
      maxBubbles_(maxBubbles) {}

std::string IotBubbleController::Emit(IotEventType type, const LngLat& position,
                                      const std::string& message) {
    if (static_cast<int>(scene_->iotBubbles.size()) >= maxBubbles_) {
        scene_->iotBubbles.erase(scene_->iotBubbles.begin());
    }
    render::IotBubbleState bubble;
    bubble.id = "iot-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    bubble.type = type;
    bubble.position = position;
    bubble.message = message;
    bubble.visible = true;
    if (defaultDurationMs_ > 0) {
        bubble.expireAtMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count() +
                            defaultDurationMs_;
    }
    scene_->iotBubbles.push_back(bubble);
    scene_->Touch();
    return bubble.id;
}

void IotBubbleController::Update(const std::string& id, IotEventType type,
                                 const std::string& message) {
    for (auto& bubble : scene_->iotBubbles) {
        if (bubble.id == id) {
            bubble.type = type;
            bubble.message = message;
            scene_->Touch();
            return;
        }
    }
}

void IotBubbleController::Dismiss(const std::string& id) {
    auto& bubbles = scene_->iotBubbles;
    const auto it = std::remove_if(bubbles.begin(), bubbles.end(),
                                   [&](const render::IotBubbleState& bubble) {
                                       return bubble.id == id;
                                   });
    if (it != bubbles.end()) {
        bubbles.erase(it, bubbles.end());
        scene_->Touch();
    }
}

void IotBubbleController::Clear() {
    scene_->iotBubbles.clear();
    scene_->Touch();
}

void IotBubbleController::Remove() {
    Clear();
}

void IotBubbleController::PruneExpired() {
    if (scene_->iotBubbles.empty()) {
        return;
    }
    const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
    auto& bubbles = scene_->iotBubbles;
    const auto it = std::remove_if(
        bubbles.begin(), bubbles.end(), [now_ms](const render::IotBubbleState& bubble) {
            return bubble.expireAtMs > 0 && bubble.expireAtMs < now_ms;
        });
    if (it != bubbles.end()) {
        bubbles.erase(it, bubbles.end());
        scene_->Touch();
    }
}

}  // namespace overlay
}  // namespace strata
}  // namespace map
}  // namespace autonomy
