/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/proto/strata_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace strata_msgs {

struct ColorRgba {
    float r{0.f};
    float g{0.f};
    float b{0.f};
    float a{1.f};
};

struct PoiMarker {
    std_msgs::Header header;
    std::string id;
    geometry_msgs::Point lng_lat;
    double rotation_deg{0.};
    std::string name;
    bool selected{false};
};

struct PoiMarkerArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(PoiMarkerArray)

    std_msgs::Header header;
    std::vector<PoiMarker> markers;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.PoiMarkerArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct RobotMarker {
    std_msgs::Header header;
    std::string id;
    geometry_msgs::Point lng_lat;
    double rotation_deg{0.};
    std::string name;
    std::string status;
    float battery{100.f};
};

struct RobotMarkerArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(RobotMarkerArray)

    std_msgs::Header header;
    std::vector<RobotMarker> robots;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.RobotMarkerArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct SemanticZone {
    std_msgs::Header header;
    std::string id;
    std::string zone_type;
    std::vector<geometry_msgs::Point> polygon;
    ColorRgba fill_color;
    float fill_opacity{0.f};
    ColorRgba outline_color;
    float outline_width{0.f};
    std::string label;
};

struct SemanticZoneArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(SemanticZoneArray)

    std_msgs::Header header;
    std::vector<SemanticZone> zones;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.SemanticZoneArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct GraphNode {
    std::string id;
    std::string type;
    geometry_msgs::Point coordinates;
};

struct GraphEdge {
    std::string id;
    std::string from;
    std::string to;
    double weight{1.0};
};

struct RoadGraph {
    AUTONOMY_SMART_PTR_DEFINITIONS(RoadGraph)

    std_msgs::Header header;
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.RoadGraph";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct FloorInfo {
    std::string id;
    std::string name;
    int32_t level{0};
    std::string slam_image_path;
    double start_x{0.};
    double start_y{0.};
    int32_t x_grid_count{0};
    int32_t y_grid_count{0};
    double resolution{0.05};
};

struct FloorInfoArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(FloorInfoArray)

    std_msgs::Header header;
    std::vector<FloorInfo> floors;
    std::string active_floor_id;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.FloorInfoArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct CanvasLabelStyleMsg {
    std::string font;
    ColorRgba text_color;
    ColorRgba halo_color;
    float halo_blur{4.f};
    int32_t padding{4};
};

struct CanvasLabel {
    std_msgs::Header header;
    std::string id;
    geometry_msgs::Point position;
    std::string label;
    bool visible{true};
};

struct CanvasLabelArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(CanvasLabelArray)

    std_msgs::Header header;
    std::vector<CanvasLabel> labels;
    CanvasLabelStyleMsg style;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.CanvasLabelArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct LabelBubble {
    std_msgs::Header header;
    geometry_msgs::Point lng_lat;
    std::string html;
    float offset_x{0.f};
    float offset_y{-10.f};
    bool visible{true};
};

struct LabelBubbleArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(LabelBubbleArray)

    std_msgs::Header header;
    std::vector<LabelBubble> bubbles;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.LabelBubbleArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct IotBubble {
    std_msgs::Header header;
    std::string id;
    std::string event_type;
    geometry_msgs::Point position;
    std::string message;
    int64_t expire_at_ms{0};
    bool visible{true};
};

struct IotBubbleArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(IotBubbleArray)

    std_msgs::Header header;
    std::vector<IotBubble> bubbles;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.IotBubbleArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

struct Robot3DLayer {
    std_msgs::Header header;
    std::string id;
    std::string robot_id;
    std::string model_url;
    double scale{1.0};
    geometry_msgs::Point offset;
    double rotation_deg{0.};
    geometry_msgs::Point position;
    double heading_deg{0.};
    std::string status;
    bool visible{true};
    double rotate_x{0.};
    double rotate_y{0.};
    double rotate_z{0.};
    double meters_scale{1.0};
    std::string default_animation;
};

struct Robot3DLayerArray {
    AUTONOMY_SMART_PTR_DEFINITIONS(Robot3DLayerArray)

    std_msgs::Header header;
    std::vector<Robot3DLayer> layers;

    static std::string TypeName() {
        return "autonomy.commsgs.proto.strata_msgs.Robot3DLayerArray";
    }
    bool SerializeToString(std::string* out) const;
    bool ParseFromString(const std::string& in);
};

proto::strata_msgs::ColorRgba ToProto(const ColorRgba& data);
ColorRgba FromProto(const proto::strata_msgs::ColorRgba& proto);

proto::strata_msgs::PoiMarker ToProto(const PoiMarker& data);
PoiMarker FromProto(const proto::strata_msgs::PoiMarker& proto);

proto::strata_msgs::PoiMarkerArray ToProto(const PoiMarkerArray& data);
PoiMarkerArray FromProto(const proto::strata_msgs::PoiMarkerArray& proto);

proto::strata_msgs::RobotMarker ToProto(const RobotMarker& data);
RobotMarker FromProto(const proto::strata_msgs::RobotMarker& proto);

proto::strata_msgs::RobotMarkerArray ToProto(const RobotMarkerArray& data);
RobotMarkerArray FromProto(const proto::strata_msgs::RobotMarkerArray& proto);

proto::strata_msgs::SemanticZone ToProto(const SemanticZone& data);
SemanticZone FromProto(const proto::strata_msgs::SemanticZone& proto);

proto::strata_msgs::SemanticZoneArray ToProto(const SemanticZoneArray& data);
SemanticZoneArray FromProto(const proto::strata_msgs::SemanticZoneArray& proto);

proto::strata_msgs::GraphNode ToProto(const GraphNode& data);
GraphNode FromProto(const proto::strata_msgs::GraphNode& proto);

proto::strata_msgs::GraphEdge ToProto(const GraphEdge& data);
GraphEdge FromProto(const proto::strata_msgs::GraphEdge& proto);

proto::strata_msgs::RoadGraph ToProto(const RoadGraph& data);
RoadGraph FromProto(const proto::strata_msgs::RoadGraph& proto);

proto::strata_msgs::FloorInfo ToProto(const FloorInfo& data);
FloorInfo FromProto(const proto::strata_msgs::FloorInfo& proto);

proto::strata_msgs::FloorInfoArray ToProto(const FloorInfoArray& data);
FloorInfoArray FromProto(const proto::strata_msgs::FloorInfoArray& proto);

proto::strata_msgs::CanvasLabelStyle ToProto(const CanvasLabelStyleMsg& data);
CanvasLabelStyleMsg FromProto(const proto::strata_msgs::CanvasLabelStyle& proto);

proto::strata_msgs::CanvasLabel ToProto(const CanvasLabel& data);
CanvasLabel FromProto(const proto::strata_msgs::CanvasLabel& proto);

proto::strata_msgs::CanvasLabelArray ToProto(const CanvasLabelArray& data);
CanvasLabelArray FromProto(const proto::strata_msgs::CanvasLabelArray& proto);

proto::strata_msgs::LabelBubble ToProto(const LabelBubble& data);
LabelBubble FromProto(const proto::strata_msgs::LabelBubble& proto);

proto::strata_msgs::LabelBubbleArray ToProto(const LabelBubbleArray& data);
LabelBubbleArray FromProto(const proto::strata_msgs::LabelBubbleArray& proto);

proto::strata_msgs::IotBubble ToProto(const IotBubble& data);
IotBubble FromProto(const proto::strata_msgs::IotBubble& proto);

proto::strata_msgs::IotBubbleArray ToProto(const IotBubbleArray& data);
IotBubbleArray FromProto(const proto::strata_msgs::IotBubbleArray& proto);

proto::strata_msgs::Robot3DLayer ToProto(const Robot3DLayer& data);
Robot3DLayer FromProto(const proto::strata_msgs::Robot3DLayer& proto);

proto::strata_msgs::Robot3DLayerArray ToProto(const Robot3DLayerArray& data);
Robot3DLayerArray FromProto(const proto::strata_msgs::Robot3DLayerArray& proto);

}  // namespace strata_msgs
}  // namespace commsgs
}  // namespace autonomy
