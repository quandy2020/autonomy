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

#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/strata_engine.hpp"
#include "autonomy/map/strata/utils/map_utils.hpp"

namespace autonomy {
namespace map {
namespace strata {

StrataEngine::StrataEngine()
    : scene_(render::SceneState::make_shared()),
      backend_(render::NullRenderBackend::make_shared()),
      layerManager_(layers::LayerManager::make_shared(scene_)) {}

StrataEngine::~StrataEngine() { backend_->Destroy(); }

void StrataEngine::Init(const InitOptions& options) {
    if (loaded_) {
        return;
    }
    resourcePaths_ = options.resourcePaths;
    loaded_ = true;
    AINFO << "StrataEngine initialized (BICMap C++ port)";
}

void StrataEngine::CreateMap(const MapViewOptions& options) {
    scene_->viewOptions = options;
    backend_->Initialize(options);
    scene_->Touch();
    SyncIfNeeded();
}

const MapViewOptions& StrataEngine::GetMapOptions() const { return scene_->viewOptions; }

void StrataEngine::AddZoomControl(const std::string& position) {
    AINFO << "Zoom control added at " << position;
}

SlamMapResult StrataEngine::LoadSlamMap(const SlamMapOptions& options) {
    auto result = layerManager_->LoadSlamMap(options);
    SyncIfNeeded();
    return result;
}

markers::DirectionalMarkerController::SharedPtr StrataEngine::AddDirectionalMarker(
    const LngLat& lngLat, const std::string& id) {
    auto controller = markers::DirectionalMarkerController::make_shared(scene_);
    controller->Add(lngLat, id);
    SyncIfNeeded();
    return controller;
}

markers::PoiMarkerController::SharedPtr StrataEngine::AddBatchPoiMarkers(
    const std::vector<PoiMarker>& points) {
    auto controller = markers::PoiMarkerController::make_shared(scene_);
    controller->AddBatch(points);
    SyncIfNeeded();
    return controller;
}

markers::RobotMarkerController::SharedPtr StrataEngine::AddRobotMarkers(
    const std::vector<RobotMarker>& robots) {
    auto controller = markers::RobotMarkerController::make_shared(scene_);
    controller->AddBatch(robots);
    SyncIfNeeded();
    return controller;
}

robot::visual::StatusRobotMarkerController::SharedPtr StrataEngine::AddStatusRobotMarkers(
    const std::vector<RobotMarker>& robots) {
    auto controller = robot::visual::StatusRobotMarkerController::make_shared(scene_);
    controller->AddBatch(robots);
    SyncIfNeeded();
    return controller;
}

robot::visual::RobotFovController::SharedPtr StrataEngine::CreateRobotFov(
    const RobotFovOptions& options) {
    auto controller = robot::visual::RobotFovController::make_shared(scene_, options);
    SyncIfNeeded();
    return controller;
}

robot::visual::Robot3DLayerController::SharedPtr StrataEngine::CreateRobot3DLayer(
    const Robot3DModelConfig& model, const Robot3DLayerState& initial_state) {
    auto controller =
        robot::visual::Robot3DLayerController::make_shared(scene_, model, initial_state);
    SyncIfNeeded();
    return controller;
}

robot::visual::Robot3DStatusLayer::SharedPtr StrataEngine::CreateRobot3DStatusLayer(
    const Robot3DModelConfig& model) {
    return robot::visual::Robot3DStatusLayer::make_shared(scene_, model);
}

robot::visual::StatusRobotMarkerController::SharedPtr
StrataEngine::CreateStatusRobotMarkers() {
    return robot::visual::StatusRobotMarkerController::make_shared(scene_);
}

point_cloud::PointCloud2DController::SharedPtr StrataEngine::CreatePointCloud(
    const std::vector<PointCloudPoint>& points, const PointCloudOptions& options) {
    auto controller = point_cloud::PointCloud2DController::make_shared(scene_, points, options);
    SyncIfNeeded();
    return controller;
}

point_cloud::PointCloud3DController::SharedPtr StrataEngine::CreatePointCloud3D(
    const std::vector<PointCloudPoint>& points, const PointCloudOptions& options) {
    auto controller = point_cloud::PointCloud3DController::make_shared(scene_, points, options);
    SyncIfNeeded();
    return controller;
}

shapes::RectangleCollection::SharedPtr StrataEngine::CreateRectangles(
    const std::vector<RectangleFeature>& rectangles) {
    auto collection = shapes::RectangleCollection::make_shared(
        scene_, &render::SceneState::rectangles, LayerIds::kRectangle);
    for (const auto& rect : rectangles) {
        collection->Add(rect);
    }
    SyncIfNeeded();
    return collection;
}

shapes::PolygonCollection::SharedPtr StrataEngine::CreatePolygons(
    const std::vector<PolygonFeature>& polygons) {
    auto collection = shapes::PolygonCollection::make_shared(
        scene_, &render::SceneState::polygons, LayerIds::kPolygon);
    for (const auto& poly : polygons) {
        collection->Add(poly);
    }
    SyncIfNeeded();
    return collection;
}

shapes::CircleCollection::SharedPtr StrataEngine::CreateCircles(
    const std::vector<CircleFeature>& circles) {
    auto collection = shapes::CircleCollection::make_shared(
        scene_, &render::SceneState::circles, LayerIds::kCircle);
    for (const auto& circle : circles) {
        collection->Add(circle);
    }
    SyncIfNeeded();
    return collection;
}

shapes::PolylineCollection::SharedPtr StrataEngine::CreatePolylines(
    const std::vector<PolylineFeature>& polylines) {
    auto collection = shapes::PolylineCollection::make_shared(
        scene_, &render::SceneState::polylines, LayerIds::kPolyline);
    for (const auto& line : polylines) {
        collection->Add(line);
    }
    SyncIfNeeded();
    return collection;
}

shapes::WideLineCollection::SharedPtr StrataEngine::CreateWideLines(
    const std::vector<WideLineFeature>& wideLines) {
    auto collection = shapes::WideLineCollection::make_shared(
        scene_, &render::SceneState::wideLines, LayerIds::kWideLine);
    for (auto wideLine : wideLines) {
        wideLine.style.filled = true;
        collection->Add(wideLine);
    }
    SyncIfNeeded();
    return collection;
}

shapes::PolygonEditController::SharedPtr StrataEngine::CreatePolygonEditor(
    shapes::PolygonCollection::SharedPtr collection) {
    return shapes::PolygonEditController::make_shared(scene_, std::move(collection));
}

shapes::WideLineEditController::SharedPtr StrataEngine::CreateWideLineEditor(
    shapes::WideLineCollection::SharedPtr collection) {
    return shapes::WideLineEditController::make_shared(scene_, std::move(collection));
}

drawing::DrawingController::SharedPtr StrataEngine::EnableRectangleDrawing(
    const DrawCompleteCallback& onComplete) {
    drawingController_ = drawing::DrawingController::make_shared(scene_);
    drawingController_->EnableRectangleDrawing(onComplete);
    return drawingController_;
}

drawing::DrawingController::SharedPtr StrataEngine::EnablePolygonDrawing(
    const DrawCompleteCallback& onComplete) {
    drawingController_ = drawing::DrawingController::make_shared(scene_);
    drawingController_->EnablePolygonDrawing(onComplete);
    return drawingController_;
}

drawing::DrawingController::SharedPtr StrataEngine::EnableCircleDrawing(
    const DrawCompleteCallback& onComplete) {
    drawingController_ = drawing::DrawingController::make_shared(scene_);
    drawingController_->EnableCircleDrawing(onComplete);
    return drawingController_;
}

drawing::DrawingController::SharedPtr StrataEngine::EnablePolylineDrawing(
    double widthM, const DrawCompleteCallback& onComplete) {
    drawingController_ = drawing::DrawingController::make_shared(scene_);
    drawingController_->EnablePolylineDrawing(widthM, onComplete);
    return drawingController_;
}

void StrataEngine::DisableDrawing() {
    if (drawingController_) {
        drawingController_->Disable();
    }
}

navigation::PathBuildService::SharedPtr StrataEngine::CreatePathBuildService() {
    return navigation::PathBuildService::make_shared();
}

navigation::GeoUtils StrataEngine::CreateGeoUtils(const navigation::GeoUtilsConfig& config) {
    return navigation::GeoUtils(config);
}

render::ExportedScene StrataEngine::ExportScene() const {
    return render::ExportScene(*scene_);
}

void StrataEngine::SetRoadGraph(const RoadGraph& graph) {
    scene_->roadGraph = graph;
    scene_->Touch();
}

const std::optional<RoadGraph>& StrataEngine::GetRoadGraph() const {
    return scene_->roadGraph;
}

overlay::LabelBubbleController::SharedPtr StrataEngine::CreateLabelBubble(
    const LabelBubbleOptions& options) {
    return overlay::LabelBubbleController::make_shared(scene_, options);
}

overlay::IotBubbleController::SharedPtr StrataEngine::CreateIoTBubbles(int defaultDurationMs,
                                                                       int maxBubbles) {
    return overlay::IotBubbleController::make_shared(scene_, defaultDurationMs, maxBubbles);
}

map_features::FloorManager::SharedPtr StrataEngine::CreateFloorManager(
    const std::vector<FloorConfig>& floors, const std::string& defaultFloorId) {
    auto manager = map_features::FloorManager::make_shared(scene_);
    if (!floors.empty()) {
        manager->Configure(floors, defaultFloorId, layerManager_);
    }
    return manager;
}

map_features::SemanticZones::SharedPtr StrataEngine::CreateSemanticZones() {
    return map_features::SemanticZones::make_shared(scene_);
}

map_features::Buildings::SharedPtr StrataEngine::CreateBuildings() {
    return map_features::Buildings::make_shared(scene_);
}

map_features::LayerEvents::SharedPtr StrataEngine::CreateLayerEvents() {
    return map_features::LayerEvents::make_shared();
}

map_features::CanvasLabelsController::SharedPtr StrataEngine::AddCanvasLabels(
    const CanvasLabelOptions& options) {
    auto controller = map_features::CanvasLabelsController::make_shared(scene_, options);
    SyncIfNeeded();
    return controller;
}

navigation::Pathfinder::SharedPtr StrataEngine::CreatePathfinder(
    const navigation::PathfinderOptions& options) {
    return navigation::Pathfinder::make_shared(options);
}

navigation::GraphPathfinder::SharedPtr StrataEngine::CreateGraphPathfinder(
    const RoadGraph& graph) {
    return navigation::GraphPathfinder::make_shared(graph);
}

robot::RobotEngine::SharedPtr StrataEngine::CreateRobotEngine() {
    return robot::RobotEngine::make_shared();
}

robot::RobotEngine::SharedPtr StrataEngine::CreateRobotEngine(
    robot::RobotEngineConfig config) {
    return robot::RobotEngine::make_shared(std::move(config));
}

robot::core::RobotProfile StrataEngine::CreateRobotProfile(
    const std::string& type, const robot::core::RobotProfile& overrides) {
    return robot::core::CreateRobotProfile(type, overrides);
}

urdf::UrdfPlugin::SharedPtr StrataEngine::CreateUrdfPlugin() {
    return urdf::UrdfPlugin::make_shared();
}

double StrataEngine::GetDistance(double lat1, double lon1, double lat2, double lon2) {
    return utils::GetDistance(lat1, lon1, lat2, lon2);
}

MapCorners StrataEngine::GetMapCorners(double startX, double startY, int xGridCount,
                                       int yGridCount, double resolution) {
    return utils::GetMapCorners(startX, startY, xGridCount, yGridCount, resolution);
}

std::string StrataEngine::GetAssetPath(const std::string& relative) const {
    return resourcePaths_.assetsUrl + "/" + relative;
}

std::string StrataEngine::GetFontPath(const std::string& fontFile) const {
    return resourcePaths_.fontsUrl + "/" + fontFile;
}

void StrataEngine::SyncRender() { SyncIfNeeded(); }

void StrataEngine::SetRenderBackend(render::RenderBackend::SharedPtr backend) {
    backend_ = std::move(backend);
    if (loaded_) {
        backend_->Initialize(scene_->viewOptions);
        SyncIfNeeded();
    }
}

void StrataEngine::SyncIfNeeded() {
    if (backend_) {
        backend_->Sync(*scene_);
    }
}

}  // namespace strata
}  // namespace map
}  // namespace autonomy
