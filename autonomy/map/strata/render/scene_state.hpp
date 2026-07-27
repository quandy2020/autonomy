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
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace render {

struct SemanticZoneFeature {
    std::string id;
    ZoneType type{ZoneType::kForbidden};
    std::vector<LngLat> polygon;
    StylePaint style;
    bool visible{true};
};

struct BuildingFeature {
    std::string id;
    std::vector<LngLat> footprint;
    double height{10.};
    double heightScale{1.};
    float opacity{0.8f};
    ColorRgba color{};
    bool visible{true};
};

struct FloorState {
    std::string id;
    std::string name;
    int level{0};
    SlamMapOptions slamOptions;
    std::string imageData;
    bool visible{true};
};

struct IotBubbleState {
    std::string id;
    IotEventType type{IotEventType::kCustom};
    LngLat position;
    std::string message;
    int64_t expireAtMs{0};
    bool visible{true};
};

class SceneState
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SceneState)

    MapViewOptions viewOptions;
    std::optional<SlamMapResult> slamMap;
    std::optional<SlamMapData> slamMapData;
    std::vector<CanvasLabelFeature> canvasLabels;
    std::optional<CanvasLabelStyle> canvasLabelStyle;
    std::vector<RobotFovState> robotFovs;
    std::vector<Robot3DLayerState> robot3DLayers;
    std::vector<RectangleFeature> rectangles;
    std::vector<PolygonFeature> polygons;
    std::vector<CircleFeature> circles;
    std::vector<PolylineFeature> polylines;
    std::vector<WideLineFeature> wideLines;
    std::vector<DirectionalMarker> directionalMarkers;
    std::vector<PoiMarker> poiMarkers;
    std::vector<RobotMarker> robotMarkers;
    std::vector<PointCloudPoint> pointCloud;
    PointCloudOptions pointCloudOptions;
    std::vector<LabelBubbleOptions> labelBubbles;
    std::vector<SemanticZoneFeature> semanticZones;
    std::vector<BuildingFeature> buildings;
    std::vector<FloorState> floors;
    std::string activeFloorId;
    std::vector<IotBubbleState> iotBubbles;
    std::vector<std::string> activeLayerOrder;
    std::optional<RoadGraph> roadGraph;

    void Touch();
    int64_t revision() const { return revision_; }

private:
    int64_t revision_{0};
};

class RenderBackend
{
public:
    using SharedPtr = std::shared_ptr<RenderBackend>;

    virtual ~RenderBackend() = default;

    virtual void Initialize(const MapViewOptions& options) = 0;
    virtual void Sync(const SceneState& state) = 0;
    virtual void SetZoom(double zoom) = 0;
    virtual void SetCenter(const LngLat& center) = 0;
    virtual void Resize(int width, int height) = 0;
    virtual void Destroy() = 0;
};

class NullRenderBackend : public RenderBackend
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(NullRenderBackend)

    void Initialize(const MapViewOptions& options) override;
    void Sync(const SceneState& state) override;
    void SetZoom(double zoom) override;
    void SetCenter(const LngLat& center) override;
    void Resize(int width, int height) override;
    void Destroy() override;

private:
    MapViewOptions options_;
};

}  // namespace render
}  // namespace strata
}  // namespace map
}  // namespace autonomy
