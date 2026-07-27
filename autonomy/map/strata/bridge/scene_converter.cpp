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

#include <cmath>
#include <unordered_map>

#include "autonomy/map/strata/bridge/scene_converter.hpp"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/strata/constants.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace bridge {

namespace {

using commsgs::proto::geometry_msgs::Point;
using commsgs::proto::geometry_msgs::Quaternion;
using commsgs::proto::std_msgs::Header;

constexpr int32_t kMarkerAdd = 0;
constexpr int32_t kMarkerArrow = 0;
constexpr int32_t kMarkerCube = 1;
constexpr int32_t kMarkerSphere = 2;
constexpr int32_t kMarkerLineStrip = 4;

commsgs::std_msgs::Header MakeHeader(const std::string& frame_id, uint64_t stamp_ns) {
    commsgs::std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000ULL);
    header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);
    return header;
}

void FillHeader(Header* header, const std::string& frame_id, uint64_t stamp_ns) {
    header->set_frame_id(frame_id);
    header->mutable_stamp()->set_sec(static_cast<int32_t>(stamp_ns / 1000000000ULL));
    header->mutable_stamp()->set_nanosec(static_cast<uint32_t>(stamp_ns % 1000000000ULL));
}

Quaternion YawToQuaternion(double yaw_deg) {
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    Quaternion quaternion;
    quaternion.set_x(0.);
    quaternion.set_y(0.);
    quaternion.set_z(std::sin(yaw_rad / 2.0));
    quaternion.set_w(std::cos(yaw_rad / 2.0));
    return quaternion;
}

void SetPoint(Point* point, const LngLat& lng_lat) {
    point->set_x(lng_lat.x);
    point->set_y(lng_lat.y);
    point->set_z(lng_lat.z);
}

commsgs::geometry_msgs::Point ToCommsgsPoint(const LngLat& lng_lat) {
    commsgs::geometry_msgs::Point point;
    point.x = lng_lat.x;
    point.y = lng_lat.y;
    point.z = lng_lat.z;
    return point;
}

void SetMarkerStyle(commsgs::proto::visualization_msgs::Marker* marker, const StylePaint& style) {
    marker->mutable_color()->set_r(style.outlineColor.r);
    marker->mutable_color()->set_g(style.outlineColor.g);
    marker->mutable_color()->set_b(style.outlineColor.b);
    marker->mutable_color()->set_a(style.fillColor.a);
    marker->mutable_scale()->set_x(style.outlineWidth * 0.01);
}

void AppendLineStrip(MarkerArrayProto* markers, int& marker_id, const std::string& frame_id,
                     uint64_t stamp_ns, const std::string& ns, const std::vector<LngLat>& points,
                     const StylePaint& style) {
    if (points.empty()) {
        return;
    }
    auto* marker = markers->add_markers();
    marker->set_ns(ns);
    marker->set_id(marker_id++);
    marker->set_type(kMarkerLineStrip);
    marker->set_action(kMarkerAdd);
    FillHeader(marker->mutable_header(), frame_id, stamp_ns);
    for (const auto& point : points) {
        SetPoint(marker->add_points(), point);
    }
    SetMarkerStyle(marker, style);
}

void AppendBuildingExtrusion(MarkerArrayProto* markers, int& marker_id,
                             const std::string& frame_id, uint64_t stamp_ns,
                             const render::BuildingFeature& building) {
    if (building.footprint.size() < 3) {
        return;
    }
    double min_x = building.footprint.front().x;
    double max_x = min_x;
    double min_y = building.footprint.front().y;
    double max_y = min_y;
    for (const auto& point : building.footprint) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    const double width = std::max(max_x - min_x, 0.1);
    const double depth = std::max(max_y - min_y, 0.1);
    const double height = std::max(building.height * building.heightScale, 0.5);

    LngLat center;
    center.x = (min_x + max_x) * 0.5;
    center.y = (min_y + max_y) * 0.5;
    center.z = height * 0.5;

    auto* marker = markers->add_markers();
    marker->set_ns("strata_building_extrusion");
    marker->set_id(marker_id++);
    marker->set_type(kMarkerCube);
    marker->set_action(kMarkerAdd);
    FillHeader(marker->mutable_header(), frame_id, stamp_ns);
    SetPoint(marker->mutable_pose()->mutable_position(), center);
    marker->mutable_pose()->mutable_orientation()->set_w(1.);
    marker->mutable_scale()->set_x(width);
    marker->mutable_scale()->set_y(depth);
    marker->mutable_scale()->set_z(height);
    marker->mutable_color()->set_r(building.color.r);
    marker->mutable_color()->set_g(building.color.g);
    marker->mutable_color()->set_b(building.color.b);
    marker->mutable_color()->set_a(building.opacity);
}

commsgs::strata_msgs::CanvasLabelStyleMsg DefaultCanvasLabelStyle() {
    commsgs::strata_msgs::CanvasLabelStyleMsg style;
    style.font = "700 12px sans-serif";
    style.text_color = {1.f, 1.f, 1.f, 1.f};
    style.halo_color = {0.118f, 0.161f, 0.231f, 1.f};
    style.halo_blur = 4.f;
    style.padding = 4;
    return style;
}

commsgs::strata_msgs::CanvasLabelStyleMsg ToStyleMsg(const CanvasLabelStyle& style) {
    commsgs::strata_msgs::CanvasLabelStyleMsg msg;
    msg.font = style.font;
    msg.text_color.r = style.textColor.r;
    msg.text_color.g = style.textColor.g;
    msg.text_color.b = style.textColor.b;
    msg.text_color.a = style.textColor.a;
    msg.halo_color.r = style.haloColor.r;
    msg.halo_color.g = style.haloColor.g;
    msg.halo_color.b = style.haloColor.b;
    msg.halo_color.a = style.haloColor.a;
    msg.halo_blur = style.haloBlur;
    msg.padding = style.padding;
    return msg;
}

}  // namespace

ConvertedSceneMessages ConvertExportedScene(const render::ExportedScene& scene,
                                            const std::string& frame_id,
                                            uint64_t stamp_ns) {
    ConvertedSceneMessages converted;
    const auto header = MakeHeader(frame_id, stamp_ns);

    converted.poi_markers.header = header;
    for (const auto& poi : scene.poiMarkers) {
        commsgs::strata_msgs::PoiMarker marker;
        marker.header = header;
        marker.id = poi.id;
        marker.lng_lat = ToCommsgsPoint(poi.lngLat);
        marker.rotation_deg = poi.rotationDeg;
        marker.name = poi.name;
        marker.selected = poi.selected;
        converted.poi_markers.markers.push_back(std::move(marker));
    }

    converted.robot_markers.header = header;
    for (const auto& robot : scene.robotMarkers) {
        commsgs::strata_msgs::RobotMarker robot_msg;
        robot_msg.header = header;
        robot_msg.id = robot.id;
        robot_msg.lng_lat = ToCommsgsPoint(robot.lngLat);
        robot_msg.rotation_deg = robot.rotationDeg;
        robot_msg.name = robot.name;
        robot_msg.status =
            robot.status.empty() ? ToString(robot.robotStatus) : robot.status;
        robot_msg.battery = robot.battery;
        converted.robot_markers.robots.push_back(std::move(robot_msg));
    }

    converted.semantic_zones.header = header;
    int marker_id = 0;
    for (const auto& zone : scene.semanticZones) {
        commsgs::strata_msgs::SemanticZone zone_msg;
        zone_msg.header = header;
        zone_msg.id = zone.id;
        zone_msg.zone_type = ToString(zone.type);
        for (const auto& point : zone.polygon) {
            zone_msg.polygon.push_back(ToCommsgsPoint(point));
        }
        zone_msg.fill_color.r = zone.style.fillColor.r;
        zone_msg.fill_color.g = zone.style.fillColor.g;
        zone_msg.fill_color.b = zone.style.fillColor.b;
        zone_msg.fill_color.a = zone.style.fillColor.a;
        zone_msg.fill_opacity = zone.style.fillOpacity;
        zone_msg.outline_color.r = zone.style.outlineColor.r;
        zone_msg.outline_color.g = zone.style.outlineColor.g;
        zone_msg.outline_color.b = zone.style.outlineColor.b;
        zone_msg.outline_color.a = zone.style.outlineColor.a;
        zone_msg.outline_width = zone.style.outlineWidth;
        const auto& presets = ZoneStylePresets();
        const auto preset_it = presets.find(zone.type);
        zone_msg.label = preset_it != presets.end() ? preset_it->second.label
                                                    : ToString(zone.type);
        converted.semantic_zones.zones.push_back(std::move(zone_msg));

        auto* marker = converted.markers.add_markers();
        marker->set_ns("strata_semantic_zone");
        marker->set_id(marker_id++);
        marker->set_type(kMarkerLineStrip);
        marker->set_action(kMarkerAdd);
        FillHeader(marker->mutable_header(), frame_id, stamp_ns);
        for (const auto& point : zone.polygon) {
            SetPoint(marker->add_points(), point);
        }
        marker->mutable_color()->set_r(zone.style.outlineColor.r);
        marker->mutable_color()->set_g(zone.style.outlineColor.g);
        marker->mutable_color()->set_b(zone.style.outlineColor.b);
        marker->mutable_color()->set_a(zone.style.fillColor.a);
        marker->mutable_scale()->set_x(zone.style.outlineWidth * 0.01);
    }

    for (const auto& poi : scene.poiMarkers) {
        auto* marker = converted.markers.add_markers();
        marker->set_ns("strata_poi");
        marker->set_id(marker_id++);
        marker->set_type(kMarkerSphere);
        marker->set_action(kMarkerAdd);
        FillHeader(marker->mutable_header(), frame_id, stamp_ns);
        SetPoint(marker->mutable_pose()->mutable_position(), poi.lngLat);
        *marker->mutable_pose()->mutable_orientation() = YawToQuaternion(poi.rotationDeg);
        marker->mutable_scale()->set_x(0.15);
        marker->mutable_scale()->set_y(0.15);
        marker->mutable_scale()->set_z(0.15);
        marker->mutable_color()->set_r(poi.selected ? 1.f : 0.2f);
        marker->mutable_color()->set_g(poi.selected ? 0.6f : 0.6f);
        marker->mutable_color()->set_b(poi.selected ? 0.f : 1.f);
        marker->mutable_color()->set_a(1.f);
        marker->set_text(poi.name);
    }

    for (const auto& robot : scene.robotMarkers) {
        auto* marker = converted.markers.add_markers();
        marker->set_ns("strata_robot");
        marker->set_id(marker_id++);
        marker->set_type(kMarkerArrow);
        marker->set_action(kMarkerAdd);
        FillHeader(marker->mutable_header(), frame_id, stamp_ns);
        SetPoint(marker->mutable_pose()->mutable_position(), robot.lngLat);
        *marker->mutable_pose()->mutable_orientation() = YawToQuaternion(robot.rotationDeg);
        marker->mutable_scale()->set_x(0.4);
        marker->mutable_scale()->set_y(0.08);
        marker->mutable_scale()->set_z(0.08);
        marker->mutable_color()->set_r(0.f);
        marker->mutable_color()->set_g(0.4f);
        marker->mutable_color()->set_b(1.f);
        marker->mutable_color()->set_a(robot.visible ? 1.f : 0.2f);
        marker->set_text(robot.name);
    }

    for (const auto& fov : scene.robotFovs) {
        if (!fov.visible) {
            continue;
        }
        StylePaint fov_style;
        fov_style.outlineColor = fov.options.fillColor;
        fov_style.fillColor = fov.options.fillColor;
        fov_style.outlineWidth = 1.f;
        for (const auto& band : fov.bands) {
            fov_style.fillColor.a = band.opacity;
            fov_style.outlineColor.a = band.opacity;
            AppendLineStrip(&converted.markers, marker_id, frame_id, stamp_ns,
                            "strata_robot_fov", band.polygon, fov_style);
        }
    }

    if (scene.roadGraph.has_value()) {
        converted.road_graph.header = header;
        std::unordered_map<std::string, LngLat> node_index;
        for (const auto& node : scene.roadGraph->nodes) {
            commsgs::strata_msgs::GraphNode graph_node;
            graph_node.id = node.id;
            graph_node.type = node.type;
            graph_node.coordinates = ToCommsgsPoint(node.coordinates);
            converted.road_graph.nodes.push_back(std::move(graph_node));
            node_index[node.id] = node.coordinates;
        }
        for (const auto& edge : scene.roadGraph->edges) {
            commsgs::strata_msgs::GraphEdge graph_edge;
            graph_edge.id = edge.id;
            graph_edge.from = edge.from;
            graph_edge.to = edge.to;
            graph_edge.weight = edge.weight;
            converted.road_graph.edges.push_back(std::move(graph_edge));

            const auto from_it = node_index.find(edge.from);
            const auto to_it = node_index.find(edge.to);
            if (from_it == node_index.end() || to_it == node_index.end()) {
                continue;
            }
            auto* marker = converted.markers.add_markers();
            marker->set_ns("strata_road_graph");
            marker->set_id(marker_id++);
            marker->set_type(kMarkerLineStrip);
            marker->set_action(kMarkerAdd);
            FillHeader(marker->mutable_header(), frame_id, stamp_ns);
            SetPoint(marker->add_points(), from_it->second);
            SetPoint(marker->add_points(), to_it->second);
            marker->mutable_color()->set_r(0.2f);
            marker->mutable_color()->set_g(0.8f);
            marker->mutable_color()->set_b(0.4f);
            marker->mutable_color()->set_a(0.9f);
            marker->mutable_scale()->set_x(0.03);
        }
        for (const auto& node : scene.roadGraph->nodes) {
            auto* marker = converted.markers.add_markers();
            marker->set_ns("strata_road_graph_node");
            marker->set_id(marker_id++);
            marker->set_type(kMarkerSphere);
            marker->set_action(kMarkerAdd);
            FillHeader(marker->mutable_header(), frame_id, stamp_ns);
            SetPoint(marker->mutable_pose()->mutable_position(), node.coordinates);
            marker->mutable_pose()->mutable_orientation()->set_w(1.);
            marker->mutable_scale()->set_x(0.08);
            marker->mutable_scale()->set_y(0.08);
            marker->mutable_scale()->set_z(0.08);
            marker->mutable_color()->set_r(0.1f);
            marker->mutable_color()->set_g(0.9f);
            marker->mutable_color()->set_b(0.3f);
            marker->mutable_color()->set_a(1.f);
        }
    }

    for (const auto& polygon : scene.polygons) {
        if (!polygon.visible) {
            continue;
        }
        AppendLineStrip(&converted.markers, marker_id, frame_id, stamp_ns, "strata_polygon",
                        polygon.points, polygon.style);
    }

    for (const auto& rectangle : scene.rectangles) {
        if (!rectangle.visible || rectangle.coordinates.size() < 2) {
            continue;
        }
        const LngLat& sw = rectangle.coordinates[0];
        const LngLat& ne = rectangle.coordinates[1];
        const std::vector<LngLat> corners = {
            sw, {ne.x, sw.y, sw.z}, ne, {sw.x, ne.y, sw.z}, sw};
        AppendLineStrip(&converted.markers, marker_id, frame_id, stamp_ns, "strata_rectangle",
                        corners, rectangle.style);
    }

    for (const auto& circle : scene.circles) {
        if (!circle.visible) {
            continue;
        }
        auto* marker = converted.markers.add_markers();
        marker->set_ns("strata_circle");
        marker->set_id(marker_id++);
        marker->set_type(kMarkerSphere);
        marker->set_action(kMarkerAdd);
        FillHeader(marker->mutable_header(), frame_id, stamp_ns);
        SetPoint(marker->mutable_pose()->mutable_position(), circle.center);
        marker->mutable_pose()->mutable_orientation()->set_w(1.);
        const double diameter = circle.radiusM * 2.0;
        marker->mutable_scale()->set_x(diameter);
        marker->mutable_scale()->set_y(diameter);
        marker->mutable_scale()->set_z(0.05);
        SetMarkerStyle(marker, circle.style);
    }

    for (const auto& building : scene.buildings) {
        if (!building.visible || building.footprint.empty()) {
            continue;
        }
        StylePaint style;
        style.outlineColor = building.color;
        style.fillColor = building.color;
        style.fillColor.a = building.opacity;
        style.outlineWidth = 2.f;
        std::vector<LngLat> footprint = building.footprint;
        if (footprint.front().x != footprint.back().x ||
            footprint.front().y != footprint.back().y) {
            footprint.push_back(footprint.front());
        }
        AppendLineStrip(&converted.markers, marker_id, frame_id, stamp_ns, "strata_building",
                        footprint, style);
        AppendBuildingExtrusion(&converted.markers, marker_id, frame_id, stamp_ns, building);
    }

    for (const auto& marker_state : scene.directionalMarkers) {
        auto* marker = converted.markers.add_markers();
        marker->set_ns("strata_directional");
        marker->set_id(marker_id++);
        marker->set_type(kMarkerArrow);
        marker->set_action(kMarkerAdd);
        FillHeader(marker->mutable_header(), frame_id, stamp_ns);
        SetPoint(marker->mutable_pose()->mutable_position(), marker_state.position);
        *marker->mutable_pose()->mutable_orientation() =
            YawToQuaternion(marker_state.rotationDeg);
        marker->mutable_scale()->set_x(0.5);
        marker->mutable_scale()->set_y(0.1);
        marker->mutable_scale()->set_z(0.1);
        marker->mutable_color()->set_r(1.f);
        marker->mutable_color()->set_g(0.5f);
        marker->mutable_color()->set_b(0.f);
        marker->mutable_color()->set_a(1.f);
    }

    for (const auto& bubble : scene.iotBubbles) {
        if (!bubble.visible) {
            continue;
        }
        commsgs::strata_msgs::IotBubble iot_msg;
        iot_msg.header = header;
        iot_msg.id = bubble.id;
        iot_msg.event_type = ToString(bubble.type);
        iot_msg.position = ToCommsgsPoint(bubble.position);
        iot_msg.message = bubble.message;
        iot_msg.expire_at_ms = bubble.expireAtMs;
        iot_msg.visible = bubble.visible;
        converted.iot_bubbles.bubbles.push_back(std::move(iot_msg));
    }
    converted.iot_bubbles.header = header;

    converted.canvas_labels.header = header;
    converted.canvas_labels.style = scene.canvasLabelStyle.has_value()
                                        ? ToStyleMsg(*scene.canvasLabelStyle)
                                        : DefaultCanvasLabelStyle();
    for (const auto& label : scene.canvasLabels) {
        if (!label.visible) {
            continue;
        }
        commsgs::strata_msgs::CanvasLabel label_msg;
        label_msg.header = header;
        label_msg.id = label.id;
        label_msg.position = ToCommsgsPoint(label.position);
        label_msg.label = label.label;
        label_msg.visible = label.visible;
        converted.canvas_labels.labels.push_back(std::move(label_msg));
    }

    converted.label_bubbles.header = header;
    for (const auto& bubble : scene.labelBubbles) {
        if (!bubble.visible) {
            continue;
        }
        commsgs::strata_msgs::LabelBubble bubble_msg;
        bubble_msg.header = header;
        bubble_msg.lng_lat = ToCommsgsPoint(bubble.lngLat);
        bubble_msg.html = bubble.html;
        bubble_msg.offset_x = bubble.offsetX;
        bubble_msg.offset_y = bubble.offsetY;
        bubble_msg.visible = bubble.visible;
        converted.label_bubbles.bubbles.push_back(std::move(bubble_msg));
    }

    converted.robot_3d_layers.header = header;
    for (const auto& layer : scene.robot3DLayers) {
        if (!layer.visible) {
            continue;
        }
        commsgs::strata_msgs::Robot3DLayer layer_msg;
        layer_msg.header = header;
        layer_msg.id = layer.id;
        layer_msg.robot_id = layer.robotId;
        layer_msg.model_url = layer.model.modelUrl;
        layer_msg.scale = layer.model.scale;
        layer_msg.offset = ToCommsgsPoint(layer.model.offset);
        layer_msg.rotation_deg = layer.model.rotationDeg;
        layer_msg.position = ToCommsgsPoint(layer.position);
        layer_msg.heading_deg = layer.headingDeg;
        layer_msg.status = ToString(layer.status);
        layer_msg.visible = layer.visible;
        layer_msg.rotate_x = layer.model.rotateX;
        layer_msg.rotate_y = layer.model.rotateY;
        layer_msg.rotate_z = layer.model.rotateZ;
        layer_msg.meters_scale = layer.model.metersScale;
        layer_msg.default_animation = layer.model.defaultAnimation;
        converted.robot_3d_layers.layers.push_back(std::move(layer_msg));
    }

    converted.floors.header = header;
    converted.floors.active_floor_id = scene.activeFloorId;
    for (const auto& floor : scene.floors) {
        commsgs::strata_msgs::FloorInfo info;
        info.id = floor.id;
        info.name = floor.name;
        info.level = floor.level;
        info.slam_image_path = floor.slamOptions.imagePath;
        info.start_x = floor.slamOptions.startX;
        info.start_y = floor.slamOptions.startY;
        info.x_grid_count = floor.slamOptions.xGridCount;
        info.y_grid_count = floor.slamOptions.yGridCount;
        info.resolution = floor.slamOptions.resolution;
        converted.floors.floors.push_back(std::move(info));
    }

    converted.path.header = header;
    for (const auto& polyline : scene.polylines) {
        for (const auto& point : polyline.points) {
            commsgs::geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.position = ToCommsgsPoint(point);
            pose.pose.orientation.w = 1.;
            converted.path.poses.push_back(std::move(pose));
        }
    }
    for (const auto& wide_line : scene.wideLines) {
        for (const auto& point : wide_line.path) {
            commsgs::geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.position = ToCommsgsPoint(point);
            pose.pose.orientation.w = 1.;
            converted.path.poses.push_back(std::move(pose));
        }
    }

    return converted;
}

}  // namespace bridge
}  // namespace strata
}  // namespace map
}  // namespace autonomy
