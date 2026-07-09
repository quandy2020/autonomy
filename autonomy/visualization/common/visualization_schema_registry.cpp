/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/visualization/common/visualization_schema_registry.hpp"

#include <array>

namespace autonomy {
namespace visualization {
namespace {

using Panel = VisualizationPanel;
using Strategy = VisualizationStrategy;

constexpr std::array<VisualizationSchemaRule, 39> kSchemaRules = {{
    // 3D: schema-compatible protobuf messages.
    {"autonomy.commsgs.proto.visualization_msgs.Marker",
     "foxglove.SceneUpdate", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.visualization_msgs.MarkerArray",
     "foxglove.SceneUpdate", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.geometry_msgs.PoseStamped",
     "foxglove.PoseInFrame", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.geometry_msgs.PoseArray",
     "foxglove.PosesInFrame", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.geometry_msgs.PointStamped",
     "foxglove.Point3InFrame", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.geometry_msgs.PolygonStamped",
     "foxglove.SceneUpdate", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.geometry_msgs.TransformStamped",
     "foxglove.FrameTransform", Panel::kThreeD,
     Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.geometry_msgs.TransformStampeds",
     "foxglove.FrameTransforms", Panel::kThreeD,
     Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.tf2_msgs.TFMessage", "foxglove.FrameTransforms",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.planning_msgs.Path", "foxglove.PosesInFrame",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.planning_msgs.Odometry", "foxglove.Odometry",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.LaserScan", "foxglove.LaserScan",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.MultiEchoLaserScan",
     "foxglove.LaserScan", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.PointCloud2", "foxglove.PointCloud",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.PointCloud", "foxglove.PointCloud",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.Range", "foxglove.SceneUpdate",
     Panel::kThreeD, Strategy::kPayloadAdaptation},

    // Image: protobuf layouts are ROS-like and can be decoded directly.
    {"autonomy.commsgs.proto.sensor_msgs.Image", "foxglove.RawImage",
     Panel::kImage, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.CompressedImage",
     "foxglove.CompressedImage", Panel::kImage, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.sensor_msgs.CameraInfo",
     "foxglove.CameraCalibration", Panel::kImage,
     Strategy::kPayloadAdaptation},

    // Map and navigation overlays.
    {"autonomy.commsgs.proto.sensor_msgs.NavSatFix", "foxglove.LocationFix",
     Panel::kMap, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.OccupancyGrid", "foxglove.Grid",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.OccupancyGridUpdate",
     "foxglove.Grid", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.Costmap", "foxglove.Grid",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.CostmapUpdate", "foxglove.Grid",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.GridMap", "foxglove.Grid",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.GridCells", "foxglove.SceneUpdate",
     Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.map_msgs.VoxelGrid", "foxglove.VoxelGrid",
     Panel::kThreeD, Strategy::kPayloadAdaptation},

    // Vision messages are useful for Raw/Image/3D overlays once topic pairing is
    // configured on the Foxglove side.
    {"autonomy.commsgs.proto.vision_msgs.BoundingBox2D",
     "foxglove.ImageAnnotations", Panel::kImage, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.BoundingBox2DArray",
     "foxglove.ImageAnnotations", Panel::kImage,
     Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.Detection2D",
     "foxglove.ImageAnnotations", Panel::kImage, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.Detection2DArray",
     "foxglove.ImageAnnotations", Panel::kImage,
     Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.BoundingBox3D",
     "foxglove.SceneUpdate", Panel::kThreeD,
     Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.BoundingBox3DArray",
     "foxglove.SceneUpdate", Panel::kThreeD,
     Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.Detection3D",
     "foxglove.SceneUpdate", Panel::kThreeD, Strategy::kPayloadAdaptation},
    {"autonomy.commsgs.proto.vision_msgs.Detection3DArray",
     "foxglove.SceneUpdate", Panel::kThreeD, Strategy::kPayloadAdaptation},

    // Raw/Plot-first messages.
    {"autonomy.commsgs.proto.sensor_msgs.Imu", "sensor_msgs.Imu",
     Panel::kRawPlot, Strategy::kSchemaPassThrough},
    {"autonomy.commsgs.proto.sensor_msgs.BatteryState",
     "sensor_msgs.BatteryState", Panel::kRawPlot,
     Strategy::kSchemaPassThrough},
    {"autonomy.commsgs.proto.diagnostic_msgs.DiagnosticArray",
     "diagnostic_msgs.DiagnosticArray", Panel::kRawPlot,
     Strategy::kSchemaPassThrough},
    {"autonomy.commsgs.proto.vehicle_msgs.RobotState",
     "vehicle_msgs.RobotState", Panel::kRawPlot,
     Strategy::kSchemaPassThrough},
}};

}  // namespace

const VisualizationSchemaRule* VisualizationSchemaRegistry::FindRule(
    const std::string& source_message_type) {
  for (const auto& rule : kSchemaRules) {
    if (source_message_type == rule.source_message_type) {
      return &rule;
    }
  }
  return nullptr;
}

std::vector<std::string> VisualizationSchemaRegistry::ListSourceMessageTypes(
    VisualizationPanel panel, bool include_raw_plot_only) {
  std::vector<std::string> message_types;
  for (const auto& rule : kSchemaRules) {
    if (rule.panel != panel) {
      continue;
    }
    if (!include_raw_plot_only &&
        rule.strategy == VisualizationStrategy::kRawPlotOnly) {
      continue;
    }
    message_types.emplace_back(rule.source_message_type);
  }
  return message_types;
}

bool VisualizationSchemaRegistry::IsBridgeable(
    const std::string& source_message_type) {
  return FindRule(source_message_type) != nullptr;
}

bool VisualizationSchemaRegistry::IsThreeDRenderable(
    const std::string& source_message_type) {
  const auto* rule = FindRule(source_message_type);
  return rule != nullptr && rule->panel == VisualizationPanel::kThreeD &&
         rule->strategy != VisualizationStrategy::kRawPlotOnly;
}

bool VisualizationSchemaRegistry::RequiresPayloadAdaptation(
    const std::string& source_message_type) {
  const auto* rule = FindRule(source_message_type);
  return rule != nullptr &&
         rule->strategy == VisualizationStrategy::kPayloadAdaptation;
}

std::string VisualizationSchemaRegistry::ResolveTargetSchemaName(
    const std::string& source_message_type) {
  const auto* rule = FindRule(source_message_type);
  if (rule == nullptr) {
    return source_message_type;
  }
  return std::string(rule->target_schema_name);
}

}  // namespace visualization
}  // namespace autonomy
