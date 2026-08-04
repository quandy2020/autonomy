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
#include <string>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/commsgs/proto/visualization_msgs.pb.h"
#include "autonomy/commsgs/strata_msgs.hpp"
#include "autonomy/map/strata/render/scene_exporter.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace bridge {

using MarkerArrayProto = commsgs::proto::visualization_msgs::MarkerArray;

struct ConvertedSceneMessages {
    commsgs::strata_msgs::PoiMarkerArray poi_markers;
    commsgs::strata_msgs::RobotMarkerArray robot_markers;
    commsgs::strata_msgs::SemanticZoneArray semantic_zones;
    commsgs::strata_msgs::RoadGraph road_graph;
    commsgs::strata_msgs::FloorInfoArray floors;
    commsgs::strata_msgs::CanvasLabelArray canvas_labels;
    commsgs::strata_msgs::LabelBubbleArray label_bubbles;
    commsgs::strata_msgs::IotBubbleArray iot_bubbles;
    commsgs::strata_msgs::Robot3DLayerArray robot_3d_layers;
    MarkerArrayProto markers;
    automsgs::msgs::nav_msgs::Path path;
};

ConvertedSceneMessages ConvertExportedScene(const render::ExportedScene& scene,
                                            const std::string& frame_id,
                                            uint64_t stamp_ns);

}  // namespace bridge
}  // namespace strata
}  // namespace map
}  // namespace autonomy
