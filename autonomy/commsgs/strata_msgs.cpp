/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/strata_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace strata_msgs {

namespace {

proto::strata_msgs::ColorRgba ToProtoColor(const ColorRgba& data) {
    proto::strata_msgs::ColorRgba proto;
    proto.set_r(data.r);
    proto.set_g(data.g);
    proto.set_b(data.b);
    proto.set_a(data.a);
    return proto;
}

ColorRgba FromProtoColor(const proto::strata_msgs::ColorRgba& proto) {
    ColorRgba data;
    data.r = proto.r();
    data.g = proto.g();
    data.b = proto.b();
    data.a = proto.a();
    return data;
}

}  // namespace

proto::strata_msgs::ColorRgba ToProto(const ColorRgba& data) {
    return ToProtoColor(data);
}

ColorRgba FromProto(const proto::strata_msgs::ColorRgba& proto) {
    return FromProtoColor(proto);
}

proto::strata_msgs::PoiMarker ToProto(const PoiMarker& data) {
    proto::strata_msgs::PoiMarker proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_id(data.id);
    *proto.mutable_lng_lat() = geometry_msgs::ToProto(data.lng_lat);
    proto.set_rotation_deg(data.rotation_deg);
    proto.set_name(data.name);
    proto.set_selected(data.selected);
    return proto;
}

PoiMarker FromProto(const proto::strata_msgs::PoiMarker& proto) {
    PoiMarker data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.id = proto.id();
    if (proto.has_lng_lat()) {
        data.lng_lat = geometry_msgs::FromProto(proto.lng_lat());
    }
    data.rotation_deg = proto.rotation_deg();
    data.name = proto.name();
    data.selected = proto.selected();
    return data;
}

proto::strata_msgs::PoiMarkerArray ToProto(const PoiMarkerArray& data) {
    proto::strata_msgs::PoiMarkerArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& marker : data.markers) {
        *proto.add_markers() = ToProto(marker);
    }
    return proto;
}

PoiMarkerArray FromProto(const proto::strata_msgs::PoiMarkerArray& proto) {
    PoiMarkerArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.markers.reserve(static_cast<size_t>(proto.markers_size()));
    for (const auto& marker : proto.markers()) {
        data.markers.push_back(FromProto(marker));
    }
    return data;
}

proto::strata_msgs::RobotMarker ToProto(const RobotMarker& data) {
    proto::strata_msgs::RobotMarker proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_id(data.id);
    *proto.mutable_lng_lat() = geometry_msgs::ToProto(data.lng_lat);
    proto.set_rotation_deg(data.rotation_deg);
    proto.set_name(data.name);
    proto.set_status(data.status);
    proto.set_battery(data.battery);
    return proto;
}

RobotMarker FromProto(const proto::strata_msgs::RobotMarker& proto) {
    RobotMarker data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.id = proto.id();
    if (proto.has_lng_lat()) {
        data.lng_lat = geometry_msgs::FromProto(proto.lng_lat());
    }
    data.rotation_deg = proto.rotation_deg();
    data.name = proto.name();
    data.status = proto.status();
    data.battery = proto.battery();
    return data;
}

proto::strata_msgs::RobotMarkerArray ToProto(const RobotMarkerArray& data) {
    proto::strata_msgs::RobotMarkerArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& robot : data.robots) {
        *proto.add_robots() = ToProto(robot);
    }
    return proto;
}

RobotMarkerArray FromProto(const proto::strata_msgs::RobotMarkerArray& proto) {
    RobotMarkerArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.robots.reserve(static_cast<size_t>(proto.robots_size()));
    for (const auto& robot : proto.robots()) {
        data.robots.push_back(FromProto(robot));
    }
    return data;
}

proto::strata_msgs::SemanticZone ToProto(const SemanticZone& data) {
    proto::strata_msgs::SemanticZone proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_id(data.id);
    proto.set_zone_type(data.zone_type);
    for (const auto& point : data.polygon) {
        *proto.add_polygon() = geometry_msgs::ToProto(point);
    }
    *proto.mutable_fill_color() = ToProtoColor(data.fill_color);
    proto.set_fill_opacity(data.fill_opacity);
    *proto.mutable_outline_color() = ToProtoColor(data.outline_color);
    proto.set_outline_width(data.outline_width);
    proto.set_label(data.label);
    return proto;
}

SemanticZone FromProto(const proto::strata_msgs::SemanticZone& proto) {
    SemanticZone data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.id = proto.id();
    data.zone_type = proto.zone_type();
    data.polygon.reserve(static_cast<size_t>(proto.polygon_size()));
    for (const auto& point : proto.polygon()) {
        data.polygon.push_back(geometry_msgs::FromProto(point));
    }
    if (proto.has_fill_color()) {
        data.fill_color = FromProtoColor(proto.fill_color());
    }
    data.fill_opacity = proto.fill_opacity();
    if (proto.has_outline_color()) {
        data.outline_color = FromProtoColor(proto.outline_color());
    }
    data.outline_width = proto.outline_width();
    data.label = proto.label();
    return data;
}

proto::strata_msgs::SemanticZoneArray ToProto(const SemanticZoneArray& data) {
    proto::strata_msgs::SemanticZoneArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& zone : data.zones) {
        *proto.add_zones() = ToProto(zone);
    }
    return proto;
}

SemanticZoneArray FromProto(const proto::strata_msgs::SemanticZoneArray& proto) {
    SemanticZoneArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.zones.reserve(static_cast<size_t>(proto.zones_size()));
    for (const auto& zone : proto.zones()) {
        data.zones.push_back(FromProto(zone));
    }
    return data;
}

proto::strata_msgs::GraphNode ToProto(const GraphNode& data) {
    proto::strata_msgs::GraphNode proto;
    proto.set_id(data.id);
    proto.set_type(data.type);
    *proto.mutable_coordinates() = geometry_msgs::ToProto(data.coordinates);
    return proto;
}

GraphNode FromProto(const proto::strata_msgs::GraphNode& proto) {
    GraphNode data;
    data.id = proto.id();
    data.type = proto.type();
    if (proto.has_coordinates()) {
        data.coordinates = geometry_msgs::FromProto(proto.coordinates());
    }
    return data;
}

proto::strata_msgs::GraphEdge ToProto(const GraphEdge& data) {
    proto::strata_msgs::GraphEdge proto;
    proto.set_id(data.id);
    proto.set_from(data.from);
    proto.set_to(data.to);
    proto.set_weight(data.weight);
    return proto;
}

GraphEdge FromProto(const proto::strata_msgs::GraphEdge& proto) {
    GraphEdge data;
    data.id = proto.id();
    data.from = proto.from();
    data.to = proto.to();
    data.weight = proto.weight();
    return data;
}

proto::strata_msgs::RoadGraph ToProto(const RoadGraph& data) {
    proto::strata_msgs::RoadGraph proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& node : data.nodes) {
        *proto.add_nodes() = ToProto(node);
    }
    for (const auto& edge : data.edges) {
        *proto.add_edges() = ToProto(edge);
    }
    return proto;
}

RoadGraph FromProto(const proto::strata_msgs::RoadGraph& proto) {
    RoadGraph data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.nodes.reserve(static_cast<size_t>(proto.nodes_size()));
    for (const auto& node : proto.nodes()) {
        data.nodes.push_back(FromProto(node));
    }
    data.edges.reserve(static_cast<size_t>(proto.edges_size()));
    for (const auto& edge : proto.edges()) {
        data.edges.push_back(FromProto(edge));
    }
    return data;
}

proto::strata_msgs::FloorInfo ToProto(const FloorInfo& data) {
    proto::strata_msgs::FloorInfo proto;
    proto.set_id(data.id);
    proto.set_name(data.name);
    proto.set_level(data.level);
    proto.set_slam_image_path(data.slam_image_path);
    proto.set_start_x(data.start_x);
    proto.set_start_y(data.start_y);
    proto.set_x_grid_count(data.x_grid_count);
    proto.set_y_grid_count(data.y_grid_count);
    proto.set_resolution(data.resolution);
    return proto;
}

FloorInfo FromProto(const proto::strata_msgs::FloorInfo& proto) {
    FloorInfo data;
    data.id = proto.id();
    data.name = proto.name();
    data.level = proto.level();
    data.slam_image_path = proto.slam_image_path();
    data.start_x = proto.start_x();
    data.start_y = proto.start_y();
    data.x_grid_count = proto.x_grid_count();
    data.y_grid_count = proto.y_grid_count();
    data.resolution = proto.resolution();
    return data;
}

proto::strata_msgs::FloorInfoArray ToProto(const FloorInfoArray& data) {
    proto::strata_msgs::FloorInfoArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& floor : data.floors) {
        *proto.add_floors() = ToProto(floor);
    }
    proto.set_active_floor_id(data.active_floor_id);
    return proto;
}

FloorInfoArray FromProto(const proto::strata_msgs::FloorInfoArray& proto) {
    FloorInfoArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.floors.reserve(static_cast<size_t>(proto.floors_size()));
    for (const auto& floor : proto.floors()) {
        data.floors.push_back(FromProto(floor));
    }
    data.active_floor_id = proto.active_floor_id();
    return data;
}

proto::strata_msgs::CanvasLabelStyle ToProto(const CanvasLabelStyleMsg& data) {
    proto::strata_msgs::CanvasLabelStyle proto;
    proto.set_font(data.font);
    *proto.mutable_text_color() = ToProtoColor(data.text_color);
    *proto.mutable_halo_color() = ToProtoColor(data.halo_color);
    proto.set_halo_blur(data.halo_blur);
    proto.set_padding(data.padding);
    return proto;
}

CanvasLabelStyleMsg FromProto(const proto::strata_msgs::CanvasLabelStyle& proto) {
    CanvasLabelStyleMsg data;
    data.font = proto.font();
    if (proto.has_text_color()) {
        data.text_color = FromProtoColor(proto.text_color());
    }
    if (proto.has_halo_color()) {
        data.halo_color = FromProtoColor(proto.halo_color());
    }
    data.halo_blur = proto.halo_blur();
    data.padding = proto.padding();
    return data;
}

proto::strata_msgs::CanvasLabel ToProto(const CanvasLabel& data) {
    proto::strata_msgs::CanvasLabel proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_id(data.id);
    *proto.mutable_position() = geometry_msgs::ToProto(data.position);
    proto.set_label(data.label);
    proto.set_visible(data.visible);
    return proto;
}

CanvasLabel FromProto(const proto::strata_msgs::CanvasLabel& proto) {
    CanvasLabel data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.id = proto.id();
    if (proto.has_position()) {
        data.position = geometry_msgs::FromProto(proto.position());
    }
    data.label = proto.label();
    data.visible = proto.visible();
    return data;
}

proto::strata_msgs::CanvasLabelArray ToProto(const CanvasLabelArray& data) {
    proto::strata_msgs::CanvasLabelArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& label : data.labels) {
        *proto.add_labels() = ToProto(label);
    }
    *proto.mutable_style() = ToProto(data.style);
    return proto;
}

CanvasLabelArray FromProto(const proto::strata_msgs::CanvasLabelArray& proto) {
    CanvasLabelArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.labels.reserve(static_cast<size_t>(proto.labels_size()));
    for (const auto& label : proto.labels()) {
        data.labels.push_back(FromProto(label));
    }
    if (proto.has_style()) {
        data.style = FromProto(proto.style());
    }
    return data;
}

proto::strata_msgs::LabelBubble ToProto(const LabelBubble& data) {
    proto::strata_msgs::LabelBubble proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    *proto.mutable_lng_lat() = geometry_msgs::ToProto(data.lng_lat);
    proto.set_html(data.html);
    proto.set_offset_x(data.offset_x);
    proto.set_offset_y(data.offset_y);
    proto.set_visible(data.visible);
    return proto;
}

LabelBubble FromProto(const proto::strata_msgs::LabelBubble& proto) {
    LabelBubble data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    if (proto.has_lng_lat()) {
        data.lng_lat = geometry_msgs::FromProto(proto.lng_lat());
    }
    data.html = proto.html();
    data.offset_x = proto.offset_x();
    data.offset_y = proto.offset_y();
    data.visible = proto.visible();
    return data;
}

proto::strata_msgs::LabelBubbleArray ToProto(const LabelBubbleArray& data) {
    proto::strata_msgs::LabelBubbleArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& bubble : data.bubbles) {
        *proto.add_bubbles() = ToProto(bubble);
    }
    return proto;
}

LabelBubbleArray FromProto(const proto::strata_msgs::LabelBubbleArray& proto) {
    LabelBubbleArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.bubbles.reserve(static_cast<size_t>(proto.bubbles_size()));
    for (const auto& bubble : proto.bubbles()) {
        data.bubbles.push_back(FromProto(bubble));
    }
    return data;
}

proto::strata_msgs::IotBubble ToProto(const IotBubble& data) {
    proto::strata_msgs::IotBubble proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_id(data.id);
    proto.set_event_type(data.event_type);
    *proto.mutable_position() = geometry_msgs::ToProto(data.position);
    proto.set_message(data.message);
    proto.set_expire_at_ms(data.expire_at_ms);
    proto.set_visible(data.visible);
    return proto;
}

IotBubble FromProto(const proto::strata_msgs::IotBubble& proto) {
    IotBubble data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.id = proto.id();
    data.event_type = proto.event_type();
    if (proto.has_position()) {
        data.position = geometry_msgs::FromProto(proto.position());
    }
    data.message = proto.message();
    data.expire_at_ms = proto.expire_at_ms();
    data.visible = proto.visible();
    return data;
}

proto::strata_msgs::IotBubbleArray ToProto(const IotBubbleArray& data) {
    proto::strata_msgs::IotBubbleArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& bubble : data.bubbles) {
        *proto.add_bubbles() = ToProto(bubble);
    }
    return proto;
}

IotBubbleArray FromProto(const proto::strata_msgs::IotBubbleArray& proto) {
    IotBubbleArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.bubbles.reserve(static_cast<size_t>(proto.bubbles_size()));
    for (const auto& bubble : proto.bubbles()) {
        data.bubbles.push_back(FromProto(bubble));
    }
    return data;
}

proto::strata_msgs::Robot3DLayer ToProto(const Robot3DLayer& data) {
    proto::strata_msgs::Robot3DLayer proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    proto.set_id(data.id);
    proto.set_robot_id(data.robot_id);
    proto.set_model_url(data.model_url);
    proto.set_scale(data.scale);
    *proto.mutable_offset() = geometry_msgs::ToProto(data.offset);
    proto.set_rotation_deg(data.rotation_deg);
    *proto.mutable_position() = geometry_msgs::ToProto(data.position);
    proto.set_heading_deg(data.heading_deg);
    proto.set_status(data.status);
    proto.set_visible(data.visible);
    proto.set_rotate_x(data.rotate_x);
    proto.set_rotate_y(data.rotate_y);
    proto.set_rotate_z(data.rotate_z);
    proto.set_meters_scale(data.meters_scale);
    proto.set_default_animation(data.default_animation);
    return proto;
}

Robot3DLayer FromProto(const proto::strata_msgs::Robot3DLayer& proto) {
    Robot3DLayer data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.id = proto.id();
    data.robot_id = proto.robot_id();
    data.model_url = proto.model_url();
    data.scale = proto.scale();
    if (proto.has_offset()) {
        data.offset = geometry_msgs::FromProto(proto.offset());
    }
    data.rotation_deg = proto.rotation_deg();
    if (proto.has_position()) {
        data.position = geometry_msgs::FromProto(proto.position());
    }
    data.heading_deg = proto.heading_deg();
    data.status = proto.status();
    data.visible = proto.visible();
    data.rotate_x = proto.rotate_x();
    data.rotate_y = proto.rotate_y();
    data.rotate_z = proto.rotate_z();
    data.meters_scale = proto.meters_scale();
    data.default_animation = proto.default_animation();
    return data;
}

proto::strata_msgs::Robot3DLayerArray ToProto(const Robot3DLayerArray& data) {
    proto::strata_msgs::Robot3DLayerArray proto;
    *proto.mutable_header() = std_msgs::ToProto(data.header);
    for (const auto& layer : data.layers) {
        *proto.add_layers() = ToProto(layer);
    }
    return proto;
}

Robot3DLayerArray FromProto(const proto::strata_msgs::Robot3DLayerArray& proto) {
    Robot3DLayerArray data;
    if (proto.has_header()) {
        data.header = std_msgs::FromProto(proto.header());
    }
    data.layers.reserve(static_cast<size_t>(proto.layers_size()));
    for (const auto& layer : proto.layers()) {
        data.layers.push_back(FromProto(layer));
    }
    return data;
}

bool PoiMarkerArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool PoiMarkerArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::PoiMarkerArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool RobotMarkerArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool RobotMarkerArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::RobotMarkerArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool SemanticZoneArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool SemanticZoneArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::SemanticZoneArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool RoadGraph::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool RoadGraph::ParseFromString(const std::string& in) {
    proto::strata_msgs::RoadGraph proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool FloorInfoArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool FloorInfoArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::FloorInfoArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool CanvasLabelArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool CanvasLabelArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::CanvasLabelArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool LabelBubbleArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool LabelBubbleArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::LabelBubbleArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool IotBubbleArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool IotBubbleArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::IotBubbleArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

bool Robot3DLayerArray::SerializeToString(std::string* out) const {
    if (out == nullptr) {
        return false;
    }
    return ToProto(*this).SerializeToString(out);
}

bool Robot3DLayerArray::ParseFromString(const std::string& in) {
    proto::strata_msgs::Robot3DLayerArray proto;
    if (!proto.ParseFromString(in)) {
        return false;
    }
    *this = FromProto(proto);
    return true;
}

}  // namespace strata_msgs
}  // namespace commsgs
}  // namespace autonomy
