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

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace map {
namespace strata {

using LngLat = commsgs::geometry_msgs::Point;  // x=lng, y=lat, z=height

struct ColorRgba {
    float r{1.f};
    float g{1.f};
    float b{1.f};
    float a{1.f};
};

struct MapViewOptions {
    std::string container;
    LngLat center{};
    double zoom{0.1};
    double pitch{0.};
    double bearing{0.};
    double maxPitch{60.};
    bool antialias{false};
    std::string backgroundColor{"#808080"};
};

struct SlamMapOptions {
    double startX{0.};
    double startY{0.};
    int xGridCount{0};
    int yGridCount{0};
    double resolution{0.05};
    std::string imagePath;
    std::string canvasId;
    bool fitBounds{true};
    int padding{50};
    double zoomFactor{1.0};
    double originLatitude{39.9042};
    double originLongitude{116.4074};
    double scale{0.01};
    double bearing{0.};
};

struct MapCorners {
    commsgs::geometry_msgs::Point bottomLeft;
    commsgs::geometry_msgs::Point bottomRight;
    commsgs::geometry_msgs::Point topLeft;
    commsgs::geometry_msgs::Point topRight;
};

struct SlamMapResult {
    MapCorners corners;
    std::vector<LngLat> cameraBound;
    std::vector<LngLat> coordinates;
};

struct StylePaint {
    ColorRgba fillColor{};
    ColorRgba outlineColor{};
    float fillOpacity{0.3f};
    float outlineWidth{2.f};
    std::vector<float> outlineDash;
    bool filled{true};
};

struct RectangleFeature {
    std::string id;
    std::vector<LngLat> coordinates;  // [[sw],[ne]]
    StylePaint style;
    bool visible{true};
    bool highlighted{false};
};

struct PolygonFeature {
    std::string id;
    std::vector<LngLat> points;
    StylePaint style;
    bool visible{true};
    bool highlighted{false};
};

struct CircleFeature {
    std::string id;
    LngLat center;
    double radiusM{1.0};
    StylePaint style;
    bool visible{true};
};

enum class DashType { kSolid, kDashed, kDotted, kDashDot };

struct PolylineFeature {
    std::string id;
    std::vector<LngLat> points;
    StylePaint style;
    DashType dashType{DashType::kSolid};
    bool showArrow{false};
    float arrowSize{1.f};
    bool visible{true};
};

struct WideLineFeature {
    std::string id;
    std::vector<LngLat> path;
    double widthM{2.0};
    StylePaint style;
    bool visible{true};
    bool highlighted{false};
};

struct DirectionalMarker {
    std::string id;
    LngLat position;
    double rotationDeg{0.};
    bool editMode{false};
    bool draggable{true};
};

struct PoiMarker {
    std::string id;
    LngLat lngLat;
    double rotationDeg{0.};
    std::string name;
    bool selected{false};
};

enum class RobotStatus {
    kIdle,
    kRunning,
    kCharging,
    kError,
    kReturning,
};

struct RobotMarker {
    std::string id;
    LngLat lngLat;
    double rotationDeg{0.};
    std::string name;
    std::string status;
    RobotStatus robotStatus{RobotStatus::kIdle};
    float battery{100.f};
    bool showLabel{true};
    bool visible{true};
};

struct RobotMarkerPatch {
    std::optional<LngLat> lngLat;
    std::optional<double> rotationDeg;
    std::optional<std::string> name;
    std::optional<std::string> status;
    std::optional<RobotStatus> robotStatus;
    std::optional<float> battery;
    std::optional<bool> showLabel;
    std::optional<bool> visible;
};

void ApplyRobotMarkerPatch(RobotMarker& robot, const RobotMarkerPatch& patch);

struct CanvasLabelStyle {
    std::string font{"700 12px sans-serif"};
    ColorRgba textColor{1.f, 1.f, 1.f, 1.f};
    ColorRgba haloColor{0.118f, 0.161f, 0.231f, 1.f};
    float haloBlur{4.f};
    int padding{4};
    std::string labelKey{"name"};
};

struct CanvasLabelFeature {
    std::string id;
    LngLat position;
    std::string label;
    bool visible{true};
};

struct CanvasLabelOptions {
    std::string layerId;
    std::vector<CanvasLabelFeature> features;
    CanvasLabelStyle style;
    std::string beforeLayerId;
};

struct RobotFovOptions {
    std::string id{"default"};
    bool enabled{true};
    double angleDeg{60.};
    double radiusMeters{3.};
    ColorRgba fillColor{0.086f, 0.467f, 1.f, 1.f};
    float innerOpacity{0.45f};
    float outerOpacity{0.02f};
    int bands{6};
    int segments{20};
};

struct RobotFovBand {
    std::vector<LngLat> polygon;
    float opacity{0.3f};
};

struct RobotFovState {
    std::string id;
    LngLat position;
    double headingDeg{0.};
    RobotFovOptions options;
    std::vector<RobotFovBand> bands;
    bool visible{true};
};

struct Robot3DModelConfig {
    std::string modelUrl;
    double scale{1.0};
    LngLat offset{};
    double rotationDeg{0.};
    double rotateX{0.};
    double rotateY{0.};
    double rotateZ{0.};
    double metersScale{1.0};
    std::unordered_map<std::string, std::string> animations;
    std::string defaultAnimation;
};

struct Robot3DLayerState {
    std::string id;
    std::string robotId;
    Robot3DModelConfig model;
    LngLat position;
    double headingDeg{0.};
    RobotStatus status{RobotStatus::kIdle};
    bool visible{true};
};

struct SlamMapData {
    SlamMapOptions options;
    std::vector<uint8_t> imageBytes;
    bool imageLoaded{false};
};

struct EditModeCallbacks {
    std::function<void(const std::string& featureId, const std::vector<LngLat>& points)> onEditStart;
    std::function<void(const std::string& featureId, const std::vector<LngLat>& points)> onEditUpdate;
    std::function<void(const std::string& featureId, const std::vector<LngLat>& points)> onEditEnd;
};

struct PointCloudPoint {
    LngLat position;
    ColorRgba color{};
    float size{4.f};
    float height{0.f};
};

struct PointCloudOptions {
    bool use3D{false};
    bool colorByZ{true};
    float minZ{0.f};
    float maxZ{10.f};
    float pointSize{4.f};
};

struct LabelBubbleOptions {
    LngLat lngLat;
    std::string html;
    float offsetX{0.f};
    float offsetY{-10.f};
    bool visible{true};
};

struct ResourcePaths {
    std::string assetsUrl{"/bicMap/assets"};
    std::string fontsUrl{"/bicMap/assets/ttf"};
    std::string cssUrl{"/bicMap/bicMap.css"};
    std::string jsUrl{"/bicMap/bicMap.min.js"};
};

struct InitOptions {
    ResourcePaths resourcePaths;
};

struct GraphNode {
    std::string id;
    std::string type;
    LngLat coordinates;
};

struct GraphEdge {
    std::string id;
    std::string from;
    std::string to;
    double weight{1.0};
};

struct RoadGraph {
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;
};

struct ObstaclePolygon {
    std::string id;
    std::vector<LngLat> polygon;
};

/** BICMap createFloorManager({ floors, defaultFloor }) 楼层配置项。 */
struct FloorConfig {
    std::string id;
    std::string name;
    int level{0};
    SlamMapOptions slamOptions;
};

/** 分数坐标多边形（graphPathBuild 建筑碰撞检测用）。 */
using FracPolygon = std::vector<std::pair<double, double>>;

struct RouteSegment {
    std::pair<double, double> start;
    std::pair<double, double> end;
};

struct BuiltRoute {
    std::vector<std::pair<double, double>> coords;
    std::vector<size_t> segmentIndices;
};

struct PathfindingRouteResult {
    std::vector<std::pair<double, double>> coords;
    std::vector<size_t> poiIndices;
};

using DrawCompleteCallback = std::function<void(const std::string& featureType,
                                                 const std::vector<LngLat>& geometry,
                                                 double areaOrRadius)>;
using FeatureClickCallback = std::function<void(const std::string& featureId)>;
using MarkerClickCallback = std::function<void(const std::string& markerId)>;

}  // namespace strata
}  // namespace map
}  // namespace autonomy
