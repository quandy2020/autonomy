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

#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace render {

struct ExportedScene {
    commsgs::map_msgs::OccupancyGrid::SharedPtr occupancyGrid;
    std::vector<PoiMarker> poiMarkers;
    std::vector<RobotMarker> robotMarkers;
    std::vector<PolylineFeature> polylines;
    std::vector<WideLineFeature> wideLines;
    std::vector<PolygonFeature> polygons;
    std::vector<RectangleFeature> rectangles;
    std::vector<CircleFeature> circles;
    std::vector<DirectionalMarker> directionalMarkers;
    std::vector<PointCloudPoint> pointCloud;
    PointCloudOptions pointCloudOptions;
    std::vector<render::SemanticZoneFeature> semanticZones;
    std::vector<CanvasLabelFeature> canvasLabels;
    std::optional<CanvasLabelStyle> canvasLabelStyle;
    std::vector<RobotFovState> robotFovs;
    std::vector<Robot3DLayerState> robot3DLayers;
    std::vector<render::BuildingFeature> buildings;
    std::vector<FloorState> floors;
    std::string activeFloorId;
    std::vector<IotBubbleState> iotBubbles;
    std::vector<LabelBubbleOptions> labelBubbles;
    std::optional<RoadGraph> roadGraph;
};

ExportedScene ExportScene(const SceneState& state);

}  // namespace render
}  // namespace strata
}  // namespace map
}  // namespace autonomy
