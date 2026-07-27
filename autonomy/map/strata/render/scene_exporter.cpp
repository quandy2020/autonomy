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

#include "autonomy/map/strata/render/scene_exporter.hpp"
#include "autonomy/map/strata/render/slam_image_decoder.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace render {

ExportedScene ExportScene(const SceneState& state) {
    ExportedScene exported;
    exported.poiMarkers = state.poiMarkers;
    exported.robotMarkers = state.robotMarkers;
    exported.polylines = state.polylines;
    exported.wideLines = state.wideLines;
    exported.polygons = state.polygons;
    exported.rectangles = state.rectangles;
    exported.circles = state.circles;
    exported.directionalMarkers = state.directionalMarkers;
    exported.pointCloud = state.pointCloud;
    exported.pointCloudOptions = state.pointCloudOptions;
    exported.semanticZones = state.semanticZones;
    exported.canvasLabels = state.canvasLabels;
    exported.canvasLabelStyle = state.canvasLabelStyle;
    exported.robotFovs = state.robotFovs;
    exported.robot3DLayers = state.robot3DLayers;
    exported.buildings = state.buildings;
    exported.floors = state.floors;
    exported.activeFloorId = state.activeFloorId;
    exported.iotBubbles = state.iotBubbles;
    exported.labelBubbles = state.labelBubbles;
    exported.roadGraph = state.roadGraph;

    if (state.slamMapData.has_value()) {
        const auto& slam = *state.slamMapData;
        exported.occupancyGrid = commsgs::map_msgs::OccupancyGrid::make_shared();
        exported.occupancyGrid->header.frame_id = "map";
        exported.occupancyGrid->info.resolution =
            static_cast<float>(slam.options.resolution);
        exported.occupancyGrid->info.width =
            static_cast<uint32_t>(std::max(0, slam.options.xGridCount));
        exported.occupancyGrid->info.height =
            static_cast<uint32_t>(std::max(0, slam.options.yGridCount));
        const size_t cell_count = static_cast<size_t>(exported.occupancyGrid->info.width) *
                                  static_cast<size_t>(exported.occupancyGrid->info.height);
        exported.occupancyGrid->data.assign(cell_count, 0);
        exported.occupancyGrid->info.origin.position.x = slam.options.startX;
        exported.occupancyGrid->info.origin.position.y = slam.options.startY;

        if (slam.imageLoaded && !slam.imageBytes.empty()) {
            SlamImageDecodeOptions decode_options;
            decode_options.width = slam.options.xGridCount;
            decode_options.height = slam.options.yGridCount;
            const auto decoded = DecodeSlamImageToOccupancy(slam.imageBytes, decode_options);
            if (decoded.size() == cell_count) {
                exported.occupancyGrid->data = decoded;
            }
        }
    } else if (state.slamMap.has_value()) {
        exported.occupancyGrid = commsgs::map_msgs::OccupancyGrid::make_shared();
        exported.occupancyGrid->header.frame_id = "map";
        exported.occupancyGrid->info.resolution = 0.05f;
        exported.occupancyGrid->info.width = 100;
        exported.occupancyGrid->info.height = 100;
        exported.occupancyGrid->data.assign(100 * 100, 0);
    }
    return exported;
}

}  // namespace render
}  // namespace strata
}  // namespace map
}  // namespace autonomy
