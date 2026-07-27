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

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/drawing/drawing_controller.hpp"
#include "autonomy/map/strata/layers/layer_manager.hpp"
#include "autonomy/map/strata/map_features/canvas_labels.hpp"
#include "autonomy/map/strata/map_features/feature_manager.hpp"
#include "autonomy/map/strata/markers/marker_controller.hpp"
#include "autonomy/map/strata/navigation/geo_utils.hpp"
#include "autonomy/map/strata/navigation/graph_path_build.hpp"
#include "autonomy/map/strata/navigation/path_build.hpp"
#include "autonomy/map/strata/navigation/poi_util.hpp"
#include "autonomy/map/strata/render/scene_exporter.hpp"
#include "autonomy/map/strata/overlay/iot_bubble.hpp"
#include "autonomy/map/strata/overlay/label_bubble.hpp"
#include "autonomy/map/strata/point_cloud/point_cloud.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/robot/robot_engine.hpp"
#include "autonomy/map/strata/robot/visual/robot_visual.hpp"
#include "autonomy/map/strata/shapes/edit_controller.hpp"
#include "autonomy/map/strata/shapes/shape_controller.hpp"
#include "autonomy/map/strata/types.hpp"
#include "autonomy/map/strata/urdf/urdf_plugin.hpp"

namespace autonomy {
namespace map {
namespace strata {

/**
 * @class StrataEngine
 * @brief C++ port of BICMap (bicmap-gl.js) — unified indoor/outdoor map engine.
 *
 * Provides the same functional surface as @x-humanoid-cloud/bic-map:
 * map creation, SLAM layers, markers, shapes, drawing, point clouds,
 * navigation, robot orchestration, semantic zones, IoT bubbles, URDF viewer.
 */
class StrataEngine
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(StrataEngine)

    StrataEngine();
    ~StrataEngine();

    StrataEngine(const StrataEngine&) = delete;
    StrataEngine& operator=(const StrataEngine&) = delete;

    // --- init / lifecycle (bicmap-gl.init) ---
    void Init(const InitOptions& options = {});
    bool IsLoaded() const { return loaded_; }

    // --- map (bicmap-gl.createMap) ---
    void CreateMap(const MapViewOptions& options);
    const MapViewOptions& GetMapOptions() const;

    // --- controls ---
    void AddZoomControl(const std::string& position = "top-right");

    // --- layers (bicmap-gl.loadSlamMap) ---
    SlamMapResult LoadSlamMap(const SlamMapOptions& options);

    // --- markers ---
    markers::DirectionalMarkerController::SharedPtr AddDirectionalMarker(
        const LngLat& lngLat, const std::string& id = "");
    markers::PoiMarkerController::SharedPtr AddBatchPoiMarkers(
        const std::vector<PoiMarker>& points);
    markers::RobotMarkerController::SharedPtr AddRobotMarkers(
        const std::vector<RobotMarker>& robots);
    robot::visual::StatusRobotMarkerController::SharedPtr AddStatusRobotMarkers(
        const std::vector<RobotMarker>& robots);
    robot::visual::RobotFovController::SharedPtr CreateRobotFov(const RobotFovOptions& options);
    robot::visual::Robot3DLayerController::SharedPtr CreateRobot3DLayer(
        const Robot3DModelConfig& model, const Robot3DLayerState& initial_state);
    robot::visual::Robot3DStatusLayer::SharedPtr CreateRobot3DStatusLayer(
        const Robot3DModelConfig& model);
    robot::visual::StatusRobotMarkerController::SharedPtr CreateStatusRobotMarkers();

    // --- point clouds ---
    point_cloud::PointCloud2DController::SharedPtr CreatePointCloud(
        const std::vector<PointCloudPoint>& points, const PointCloudOptions& options = {});
    point_cloud::PointCloud3DController::SharedPtr CreatePointCloud3D(
        const std::vector<PointCloudPoint>& points, const PointCloudOptions& options = {});

    // --- shapes ---
    shapes::RectangleCollection::SharedPtr CreateRectangles(
        const std::vector<RectangleFeature>& rectangles);
    shapes::PolygonCollection::SharedPtr CreatePolygons(
        const std::vector<PolygonFeature>& polygons);
    shapes::CircleCollection::SharedPtr CreateCircles(const std::vector<CircleFeature>& circles);
    shapes::PolylineCollection::SharedPtr CreatePolylines(
        const std::vector<PolylineFeature>& polylines);
    shapes::WideLineCollection::SharedPtr CreateWideLines(
        const std::vector<WideLineFeature>& wideLines);
    shapes::PolygonEditController::SharedPtr CreatePolygonEditor(
        shapes::PolygonCollection::SharedPtr collection);
    shapes::WideLineEditController::SharedPtr CreateWideLineEditor(
        shapes::WideLineCollection::SharedPtr collection);

    // --- drawing ---
    drawing::DrawingController::SharedPtr EnableRectangleDrawing(
        const DrawCompleteCallback& onComplete = {});
    drawing::DrawingController::SharedPtr EnablePolygonDrawing(
        const DrawCompleteCallback& onComplete = {});
    drawing::DrawingController::SharedPtr EnableCircleDrawing(
        const DrawCompleteCallback& onComplete = {});
    drawing::DrawingController::SharedPtr EnablePolylineDrawing(
        double widthM = 2.0, const DrawCompleteCallback& onComplete = {});
    void DisableDrawing();

    // --- navigation services ---
    navigation::PathBuildService::SharedPtr CreatePathBuildService();
    navigation::GeoUtils CreateGeoUtils(const navigation::GeoUtilsConfig& config);

    // --- scene export (for autoviz / commsgs consumers) ---
    render::ExportedScene ExportScene() const;
    void SetRoadGraph(const RoadGraph& graph);
    const std::optional<RoadGraph>& GetRoadGraph() const;

    // --- overlay ---
    overlay::LabelBubbleController::SharedPtr CreateLabelBubble(
        const LabelBubbleOptions& options);
    overlay::IotBubbleController::SharedPtr CreateIoTBubbles(
        int defaultDurationMs = 4000, int maxBubbles = 20);

    // --- map features (not exported in bicmap-gl but in core/mapFeatures) ---
    map_features::FloorManager::SharedPtr CreateFloorManager(
        const std::vector<FloorConfig>& floors = {},
        const std::string& defaultFloorId = {});
    map_features::SemanticZones::SharedPtr CreateSemanticZones();
    map_features::Buildings::SharedPtr CreateBuildings();
    map_features::LayerEvents::SharedPtr CreateLayerEvents();
    map_features::CanvasLabelsController::SharedPtr AddCanvasLabels(
        const CanvasLabelOptions& options);

    // --- navigation (core/navigation) ---
    navigation::Pathfinder::SharedPtr CreatePathfinder(const navigation::PathfinderOptions& options);
    navigation::GraphPathfinder::SharedPtr CreateGraphPathfinder(const RoadGraph& graph);

    // --- robot engine (core/robot) ---
    robot::RobotEngine::SharedPtr CreateRobotEngine();
    robot::RobotEngine::SharedPtr CreateRobotEngine(robot::RobotEngineConfig config);
    robot::core::RobotProfile CreateRobotProfile(const std::string& type,
                                                 const robot::core::RobotProfile& overrides = {});

    // --- urdf (core/urdf) ---
    urdf::UrdfPlugin::SharedPtr CreateUrdfPlugin();

    // --- geo sources (core/utils/mapUtils) ---
    static double GetDistance(double lat1, double lon1, double lat2, double lon2);
    static MapCorners GetMapCorners(double startX, double startY, int xGridCount, int yGridCount,
                                    double resolution);

    // --- resource paths ---
    std::string GetAssetPath(const std::string& relative) const;
    std::string GetFontPath(const std::string& fontFile) const;

    // --- render sync ---
    void SyncRender();
    void SetRenderBackend(render::RenderBackend::SharedPtr backend);
    render::SceneState::SharedPtr GetSceneState() { return scene_; }

    const ResourcePaths& GetResourcePaths() const { return resourcePaths_; }

private:
    void SyncIfNeeded();

    bool loaded_{false};
    ResourcePaths resourcePaths_;
    render::SceneState::SharedPtr scene_;
    render::RenderBackend::SharedPtr backend_;
    layers::LayerManager::SharedPtr layerManager_;
    drawing::DrawingController::SharedPtr drawingController_;
};

}  // namespace strata
}  // namespace map
}  // namespace autonomy
