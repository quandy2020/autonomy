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

#include "autonomy/map/strata/map_features/canvas_labels.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace map_features {

CanvasLabelsController::CanvasLabelsController(render::SceneState::SharedPtr scene,
                                               CanvasLabelOptions options)
    : scene_(std::move(scene)), options_(std::move(options)) {
    scene_->canvasLabels = options_.features;
    scene_->canvasLabelStyle = options_.style;
    scene_->Touch();
}

LngLat CanvasLabelsController::ComputePolygonCentroid(const std::vector<LngLat>& polygon) {
    LngLat centroid;
    if (polygon.empty()) {
        return centroid;
    }
    const size_t count = polygon.size() > 1 ? polygon.size() - 1 : polygon.size();
    for (size_t i = 0; i < count; ++i) {
        centroid.x += polygon[i].x;
        centroid.y += polygon[i].y;
    }
    centroid.x /= static_cast<double>(count);
    centroid.y /= static_cast<double>(count);
    return centroid;
}

std::vector<CanvasLabelFeature> CanvasLabelsController::BuildFeatures(
    const std::vector<CanvasLabelInput>& inputs) {
    std::vector<CanvasLabelFeature> features;
    features.reserve(inputs.size());
    for (const auto& input : inputs) {
        if (input.label.empty()) {
            continue;
        }
        CanvasLabelFeature feature;
        feature.id = input.id;
        feature.label = input.label;
        if (input.point.has_value()) {
            feature.position = *input.point;
        } else if (!input.polygon.empty()) {
            feature.position = ComputePolygonCentroid(input.polygon);
        } else {
            continue;
        }
        features.push_back(std::move(feature));
    }
    return features;
}

void CanvasLabelsController::Update(const std::vector<CanvasLabelInput>& inputs) {
    options_.features = BuildFeatures(inputs);
    scene_->canvasLabels = options_.features;
    scene_->canvasLabelStyle = options_.style;
    for (auto& label : scene_->canvasLabels) {
        label.visible = visible_;
    }
    scene_->Touch();
}

void CanvasLabelsController::Show() {
    visible_ = true;
    for (auto& label : scene_->canvasLabels) {
        label.visible = true;
    }
    scene_->Touch();
}

void CanvasLabelsController::Hide() {
    visible_ = false;
    for (auto& label : scene_->canvasLabels) {
        label.visible = false;
    }
    scene_->Touch();
}

void CanvasLabelsController::Remove() {
    scene_->canvasLabels.clear();
    options_.features.clear();
    scene_->Touch();
}

const std::vector<CanvasLabelFeature>& CanvasLabelsController::GetLabels() const {
    return scene_->canvasLabels;
}

}  // namespace map_features
}  // namespace strata
}  // namespace map
}  // namespace autonomy
