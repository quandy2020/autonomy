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

#include <cmath>

#include "autonomy/map/strata/drawing/drawing_controller.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace drawing {

DrawingController::DrawingController(render::SceneState::SharedPtr scene, DrawingStyle style)
    : scene_(std::move(scene)), style_(std::move(style)) {}

void DrawingController::EnableRectangleDrawing(const DrawCompleteCallback& onComplete) {
    mode_ = DrawingMode::kRectangle;
    onComplete_ = onComplete;
    activePoints_.clear();
    startPoint_.reset();
    drawing_ = false;
}

void DrawingController::EnablePolygonDrawing(const DrawCompleteCallback& onComplete) {
    mode_ = DrawingMode::kPolygon;
    onComplete_ = onComplete;
    activePoints_.clear();
    startPoint_.reset();
    drawing_ = false;
}

void DrawingController::EnableCircleDrawing(const DrawCompleteCallback& onComplete) {
    mode_ = DrawingMode::kCircle;
    onComplete_ = onComplete;
    activePoints_.clear();
    startPoint_.reset();
    drawing_ = false;
}

void DrawingController::EnablePolylineDrawing(double width_m,
                                            const DrawCompleteCallback& onComplete) {
    mode_ = DrawingMode::kPolyline;
    style_.polylineWidthM = width_m;
    onComplete_ = onComplete;
    activePoints_.clear();
    startPoint_.reset();
    drawing_ = false;
}

void DrawingController::Disable() {
    mode_ = DrawingMode::kNone;
    activePoints_.clear();
    startPoint_.reset();
    drawing_ = false;
    onComplete_ = nullptr;
}

void DrawingController::FinishDrawing() {
    if (mode_ == DrawingMode::kNone || activePoints_.empty()) {
        return;
    }
    FinishDrawingInternal();
}

void DrawingController::ClearDrawing() {
    activePoints_.clear();
    startPoint_.reset();
    drawing_ = false;
    scene_->Touch();
}

void DrawingController::SetWidth(double width_m) {
    style_.polylineWidthM = width_m;
}

void DrawingController::OnPointerDown(const LngLat& point) {
    if (mode_ == DrawingMode::kNone) {
        return;
    }
    if (mode_ == DrawingMode::kRectangle || mode_ == DrawingMode::kCircle) {
        startPoint_ = point;
        activePoints_ = {point, point};
        drawing_ = true;
        return;
    }
    if (mode_ == DrawingMode::kPolygon || mode_ == DrawingMode::kPolyline) {
        activePoints_.push_back(point);
        drawing_ = true;
    }
}

void DrawingController::OnPointerMove(const LngLat& point) {
    if (!drawing_) {
        return;
    }
    if (mode_ == DrawingMode::kRectangle && startPoint_) {
        activePoints_ = {*startPoint_, point};
    } else if (mode_ == DrawingMode::kCircle && startPoint_) {
        activePoints_ = {*startPoint_, point};
    }
}

void DrawingController::OnPointerUp(const LngLat& point) {
    if (!drawing_) {
        return;
    }
    if (mode_ == DrawingMode::kRectangle || mode_ == DrawingMode::kCircle) {
        if (startPoint_) {
            activePoints_ = {*startPoint_, point};
        }
        FinishDrawing();
    }
}

void DrawingController::OnKeyEscape() { Disable(); }

void DrawingController::OnKeyEnter() {
    if (mode_ == DrawingMode::kPolygon || mode_ == DrawingMode::kPolyline) {
        FinishDrawingInternal();
    }
}

void DrawingController::FinishDrawingInternal() {
    if (onComplete_ && !activePoints_.empty()) {
        std::string type;
        switch (mode_) {
            case DrawingMode::kRectangle: type = "rectangle"; break;
            case DrawingMode::kPolygon: type = "polygon"; break;
            case DrawingMode::kCircle: type = "circle"; break;
            case DrawingMode::kPolyline: type = "polyline"; break;
            default: type = "unknown"; break;
        }
        onComplete_(type, activePoints_, ComputeAreaOrRadius());
    }
    Disable();
}

double DrawingController::ComputeAreaOrRadius() const {
    if (activePoints_.size() < 2) {
        return 0.;
    }
    if (mode_ == DrawingMode::kCircle && startPoint_) {
        const auto& p = activePoints_.back();
        return std::hypot(p.x - startPoint_->x, p.y - startPoint_->y);
    }
    if (mode_ == DrawingMode::kRectangle && activePoints_.size() >= 2) {
        return std::abs(activePoints_[1].x - activePoints_[0].x) *
               std::abs(activePoints_[1].y - activePoints_[0].y);
    }
    return static_cast<double>(activePoints_.size());
}

}  // namespace drawing
}  // namespace strata
}  // namespace map
}  // namespace autonomy
