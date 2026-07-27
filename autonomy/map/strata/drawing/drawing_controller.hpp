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
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace drawing {

enum class DrawingMode { kNone, kRectangle, kPolygon, kCircle, kPolyline };

struct DrawingStyle {
    ColorRgba fillColor{0.f, 0.533f, 0.533f, 1.f};
    float fillOpacity{0.5f};
    ColorRgba lineColor{0.f, 0.267f, 0.267f, 1.f};
    float lineWidth{2.f};
    double polylineWidthM{2.0};
};

class DrawingController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(DrawingController)

    DrawingController(render::SceneState::SharedPtr scene,
                      DrawingStyle style = DrawingStyle{});

    void EnableRectangleDrawing(const DrawCompleteCallback& onComplete = {});
    void EnablePolygonDrawing(const DrawCompleteCallback& onComplete = {});
    void EnableCircleDrawing(const DrawCompleteCallback& onComplete = {});
    void EnablePolylineDrawing(double width_m = 2.0, const DrawCompleteCallback& onComplete = {});
    void Disable();

    void FinishDrawing();
    void ClearDrawing();
    bool GetDrawing() const { return drawing_; }
    void SetWidth(double width_m);
    double GetWidth() const { return style_.polylineWidthM; }
    std::vector<LngLat> GetPoints() const { return activePoints_; }

    void OnPointerDown(const LngLat& point);
    void OnPointerMove(const LngLat& point);
    void OnPointerUp(const LngLat& point);
    void OnKeyEscape();
    void OnKeyEnter();

    bool IsEnabled() const { return mode_ != DrawingMode::kNone; }
    DrawingMode mode() const { return mode_; }
    const std::vector<LngLat>& activePoints() const { return activePoints_; }

private:
    void FinishDrawingInternal();
    double ComputeAreaOrRadius() const;

    render::SceneState::SharedPtr scene_;
    DrawingStyle style_;
    DrawingMode mode_{DrawingMode::kNone};
    DrawCompleteCallback onComplete_;
    std::vector<LngLat> activePoints_;
    std::optional<LngLat> startPoint_;
    bool drawing_{false};
};

}  // namespace drawing
}  // namespace strata
}  // namespace map
}  // namespace autonomy
