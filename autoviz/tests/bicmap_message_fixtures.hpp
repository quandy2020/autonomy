/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/std_msgs/color_rgba.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/strata_msgs/canvas_label.pb.h>
#include <automsgs/msgs/strata_msgs/iot_bubble.pb.h>
#include <automsgs/msgs/strata_msgs/label_bubble.pb.h>
#include <automsgs/msgs/strata_msgs/poi_marker.pb.h>
#include <automsgs/msgs/strata_msgs/robot_3d_layer.pb.h>
#include <automsgs/msgs/strata_msgs/robot_marker.pb.h>
#include <automsgs/msgs/strata_msgs/floor_info.pb.h>
#include <automsgs/msgs/strata_msgs/road_graph.pb.h>
#include <automsgs/msgs/strata_msgs/semantic_zone.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

namespace autoviz {
namespace tests {
namespace bicmap {

inline void SetHeader(automsgs::msgs::std_msgs::Header* header,
                     const std::string& frame_id = "map") {
  header->set_frame_id(frame_id);
}

inline automsgs::msgs::nav_msgs::OccupancyGrid MakeOccupancyGrid(
    uint32_t width, uint32_t height, float resolution = 0.05f,
    int32_t fill_value = 100) {
  automsgs::msgs::nav_msgs::OccupancyGrid grid;
  SetHeader(grid.mutable_header());
  auto* info = grid.mutable_info();
  info->set_resolution(resolution);
  info->set_width(width);
  info->set_height(height);
  info->mutable_origin()->mutable_position()->set_x(-1.0);
  info->mutable_origin()->mutable_position()->set_y(-1.0);
  grid.mutable_data()->Resize(static_cast<int>(width * height), fill_value);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      if (x == 0 || y == 0 || x + 1 == width || y + 1 == height) {
        grid.set_data(y * width + x, 100);
      } else if ((x + y) % 7 == 0) {
        grid.set_data(y * width + x, -1);
      } else {
        grid.set_data(y * width + x, 0);
      }
    }
  }
  return grid;
}

inline automsgs::msgs::strata_msgs::PoiMarkerArray MakePoiMarkers(
    int count, bool long_labels = false) {
  automsgs::msgs::strata_msgs::PoiMarkerArray array;
  SetHeader(array.mutable_header());
  for (int i = 0; i < count; ++i) {
    auto* poi = array.add_markers();
    SetHeader(poi->mutable_header());
    poi->set_id("poi_" + std::to_string(i));
    poi->mutable_lng_lat()->set_x(1.0 + 0.5 * i);
    poi->mutable_lng_lat()->set_y(0.5 * i);
    poi->set_rotation_deg(45.0 * i);
    poi->set_name(long_labels ? ("高级标注点位-" + std::to_string(i) + "-BICMap")
                              : ("POI-" + std::to_string(i)));
  }
  return array;
}

inline automsgs::msgs::strata_msgs::RobotMarkerArray MakeRobotMarkers(
    int count, const std::string& status = "running") {
  automsgs::msgs::strata_msgs::RobotMarkerArray array;
  SetHeader(array.mutable_header());
  for (int i = 0; i < count; ++i) {
    auto* robot = array.add_robots();
    SetHeader(robot->mutable_header());
    robot->set_id("robot_" + std::to_string(i));
    robot->mutable_lng_lat()->set_x(0.5 + 0.3 * i);
    robot->mutable_lng_lat()->set_y(0.2 * i);
    robot->set_rotation_deg(90.0);
    robot->set_name("Robot-" + std::to_string(i));
    robot->set_status(status);
    robot->set_battery(80.f - static_cast<float>(i) * 5.f);
  }
  return array;
}

inline automsgs::msgs::strata_msgs::SemanticZoneArray MakeSemanticZones(
    const std::string& zone_type = "forbidden") {
  automsgs::msgs::strata_msgs::SemanticZoneArray array;
  SetHeader(array.mutable_header());
  auto* zone = array.add_zones();
  SetHeader(zone->mutable_header());
  zone->set_id("zone_0");
  zone->set_zone_type(zone_type);
  zone->set_label(zone_type + "_area");
  zone->mutable_fill_color()->set_r(0.2f);
  zone->mutable_fill_color()->set_g(0.6f);
  zone->mutable_fill_color()->set_b(1.f);
  zone->mutable_fill_color()->set_a(0.35f);
  zone->mutable_outline_color()->set_r(0.1f);
  zone->mutable_outline_color()->set_g(0.4f);
  zone->mutable_outline_color()->set_b(0.9f);
  zone->mutable_outline_color()->set_a(1.f);
  zone->set_fill_opacity(0.35f);
  zone->set_outline_width(0.03f);
  const std::vector<std::pair<double, double>> corners = {
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {0.0, 1.5}};
  for (const auto& [x, y] : corners) {
    auto* pt = zone->add_polygon();
    pt->set_x(x);
    pt->set_y(y);
  }
  return array;
}

inline automsgs::msgs::nav_msgs::Path MakePath(int point_count) {
  automsgs::msgs::nav_msgs::Path path;
  SetHeader(path.mutable_header());
  for (int i = 0; i < point_count; ++i) {
    auto* pose = path.add_poses();
    SetHeader(pose->mutable_header());
    pose->mutable_pose()->mutable_position()->set_x(0.2 * i);
    pose->mutable_pose()->mutable_position()->set_y(0.1 * i);
    pose->mutable_pose()->mutable_position()->set_z(0.01);
  }
  return path;
}

inline automsgs::msgs::sensor_msgs::PointCloud2 MakePointCloud2(int point_count) {
  automsgs::msgs::sensor_msgs::PointCloud2 cloud;
  SetHeader(cloud.mutable_header());
  cloud.set_height(1);
  cloud.set_width(static_cast<uint32_t>(point_count));
  cloud.set_is_bigendian(false);
  cloud.set_is_dense(true);
  cloud.set_point_step(12);
  cloud.set_row_step(12 * static_cast<uint32_t>(point_count));

  auto add_field = [&cloud](const char* name, uint32_t offset) {
    auto* field = cloud.add_fields();
    field->set_name(name);
    field->set_offset(offset);
    field->set_datatype(
        automsgs::msgs::sensor_msgs::PointField::FLOAT32);
    field->set_count(1);
  };
  add_field("x", 0);
  add_field("y", 4);
  add_field("z", 8);

  std::string bytes(static_cast<size_t>(point_count) * 12, '\0');
  for (int i = 0; i < point_count; ++i) {
    const float coords[3] = {static_cast<float>(i) * 0.05f,
                             std::sin(static_cast<float>(i) * 0.2f),
                             0.05f * static_cast<float>(i % 5)};
    std::memcpy(bytes.data() + static_cast<size_t>(i) * 12, coords,
                sizeof(coords));
  }
  cloud.set_data(bytes);
  return cloud;
}

inline automsgs::msgs::std_msgs::ColorRGBA MakeColor(float r, float g, float b,
                                                     float a = 1.f) {
  automsgs::msgs::std_msgs::ColorRGBA color;
  color.set_r(r);
  color.set_g(g);
  color.set_b(b);
  color.set_a(a);
  return color;
}

inline automsgs::msgs::visualization_msgs::Marker MakeMarker(
    automsgs::msgs::visualization_msgs::Marker::Type type, int32_t id,
    const std::string& ns = "bicmap") {
  automsgs::msgs::visualization_msgs::Marker marker;
  SetHeader(marker.mutable_header());
  marker.set_ns(ns);
  marker.set_id(id);
  marker.set_type(type);
  marker.set_action(automsgs::msgs::visualization_msgs::Marker::ADD);
  *marker.mutable_color() = MakeColor(0.2f, 0.7f, 0.9f, 0.9f);
  marker.mutable_scale()->set_x(0.2);
  marker.mutable_scale()->set_y(0.2);
  marker.mutable_scale()->set_z(0.2);
  marker.mutable_pose()->mutable_position()->set_x(1.0);
  marker.mutable_pose()->mutable_position()->set_y(0.5);
  return marker;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeLineStripMarker(
    int id, int point_count) {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* marker = array.add_markers();
  *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::LINE_STRIP,
                       id);
  marker->mutable_scale()->set_x(0.03);
  for (int i = 0; i < point_count; ++i) {
    auto* pt = marker->add_points();
    pt->set_x(0.2 * i);
    pt->set_y(0.1 * std::sin(0.5 * i));
  }
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeRectangleMarker(
    int id) {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* marker = array.add_markers();
  *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::LINE_LIST, id);
  marker->mutable_scale()->set_x(0.02);
  const std::vector<std::pair<double, double>> corners = {
      {0, 0}, {1, 0}, {1, 0.8}, {0, 0.8}, {0, 0}, {1, 0}};
  for (size_t i = 0; i + 1 < corners.size(); ++i) {
    auto* a = marker->add_points();
    a->set_x(corners[i].first);
    a->set_y(corners[i].second);
    auto* b = marker->add_points();
    b->set_x(corners[i + 1].first);
    b->set_y(corners[i + 1].second);
  }
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeCircleMarker(
    int id, int segments = 24) {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* marker = array.add_markers();
  *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::LINE_STRIP,
                       id);
  marker->mutable_scale()->set_x(0.02);
  constexpr double kRadius = 0.6;
  for (int i = 0; i <= segments; ++i) {
    const double theta = 2.0 * M_PI * i / segments;
    auto* pt = marker->add_points();
    pt->set_x(kRadius * std::cos(theta));
    pt->set_y(kRadius * std::sin(theta));
  }
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeCubeMarker(int id) {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* marker = array.add_markers();
  *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::CUBE, id);
  marker->mutable_scale()->set_x(0.5);
  marker->mutable_scale()->set_y(0.5);
  marker->mutable_scale()->set_z(0.8);
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeBuildingMarkers() {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  for (int i = 0; i < 3; ++i) {
    auto* marker = array.add_markers();
    *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::CUBE, i,
                         "outdoor_building");
    marker->mutable_pose()->mutable_position()->set_x(2.0 * i);
    marker->mutable_pose()->mutable_position()->set_y(1.0);
    marker->mutable_scale()->set_x(1.2);
    marker->mutable_scale()->set_y(0.8);
    marker->mutable_scale()->set_z(3.0 + i);
  }
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeSpaceMarkers() {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* marker = array.add_markers();
  *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::SPHERE_LIST,
                       0, "space");
  marker->mutable_scale()->set_x(0.15);
  marker->mutable_scale()->set_y(0.15);
  marker->mutable_scale()->set_z(0.15);
  for (int i = 0; i < 12; ++i) {
    auto* pt = marker->add_points();
    pt->set_x(0.1 * (i % 4));
    pt->set_y(0.1 * (i / 4));
    pt->set_z(0.05 * (i % 3));
  }
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeRobotFovMarker() {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* marker = array.add_markers();
  *marker = MakeMarker(automsgs::msgs::visualization_msgs::Marker::LINE_STRIP, 0,
                       "strata_robot_fov");
  *marker->mutable_color() = MakeColor(0.2f, 0.8f, 1.f, 0.35f);
  const std::vector<std::pair<double, double>> fan = {
      {0, 0}, {1.2, 0.4}, {1.2, -0.4}, {0, 0}};
  for (const auto& [x, y] : fan) {
    auto* pt = marker->add_points();
    pt->set_x(x);
    pt->set_y(y);
  }
  return array;
}

inline automsgs::msgs::strata_msgs::RoadGraph MakeRoadGraph() {
  automsgs::msgs::strata_msgs::RoadGraph graph;
  SetHeader(graph.mutable_header());
  auto* n0 = graph.add_nodes();
  n0->set_id("n0");
  n0->mutable_coordinates()->set_x(0.0);
  n0->mutable_coordinates()->set_y(0.0);
  auto* n1 = graph.add_nodes();
  n1->set_id("n1");
  n1->mutable_coordinates()->set_x(3.0);
  n1->mutable_coordinates()->set_y(1.0);
  auto* n2 = graph.add_nodes();
  n2->set_id("n2");
  n2->mutable_coordinates()->set_x(5.0);
  n2->mutable_coordinates()->set_y(-0.5);
  auto* edge = graph.add_edges();
  edge->set_id("e0");
  edge->set_from("n0");
  edge->set_to("n1");
  edge->set_weight(1.0);
  auto* edge2 = graph.add_edges();
  edge2->set_id("e1");
  edge2->set_from("n1");
  edge2->set_to("n2");
  edge2->set_weight(1.5);
  return graph;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeGraphicDrawingMarkers() {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  array.MergeFrom(MakeLineStripMarker(0, 4));
  array.MergeFrom(MakeCubeMarker(1));
  auto* text = array.add_markers();
  *text = MakeMarker(automsgs::msgs::visualization_msgs::Marker::TEXT_VIEW_FACING,
                     2);
  text->set_text("BICMap");
  return array;
}

inline automsgs::msgs::strata_msgs::CanvasLabelArray MakeCanvasLabels(int count = 2) {
  automsgs::msgs::strata_msgs::CanvasLabelArray array;
  SetHeader(array.mutable_header());
  for (int i = 0; i < count; ++i) {
    auto* label = array.add_labels();
    SetHeader(label->mutable_header());
    label->set_id("canvas_" + std::to_string(i));
    label->mutable_position()->set_x(1.0 + 0.4 * i);
    label->mutable_position()->set_y(0.5 * i);
    label->set_label("Canvas-" + std::to_string(i));
    label->set_visible(true);
  }
  return array;
}

inline automsgs::msgs::strata_msgs::LabelBubbleArray MakeLabelBubbles(int count = 2) {
  automsgs::msgs::strata_msgs::LabelBubbleArray array;
  SetHeader(array.mutable_header());
  for (int i = 0; i < count; ++i) {
    auto* bubble = array.add_bubbles();
    SetHeader(bubble->mutable_header());
    bubble->mutable_lng_lat()->set_x(2.0 + 0.3 * i);
    bubble->mutable_lng_lat()->set_y(0.8 + 0.2 * i);
    bubble->set_html("<b>Bubble-" + std::to_string(i) + "</b>");
    bubble->set_offset_x(12.f * static_cast<float>(i));
    bubble->set_offset_y(-8.f);
    bubble->set_visible(true);
  }
  return array;
}

inline automsgs::msgs::strata_msgs::IotBubbleArray MakeIotBubbles(int count = 2) {
  automsgs::msgs::strata_msgs::IotBubbleArray array;
  SetHeader(array.mutable_header());
  const int64_t future_ms = 4'000'000'000'000LL;
  for (int i = 0; i < count; ++i) {
    auto* bubble = array.add_bubbles();
    SetHeader(bubble->mutable_header());
    bubble->set_id("iot_" + std::to_string(i));
    bubble->set_event_type(i == 0 ? "warning" : "door_open");
    bubble->mutable_position()->set_x(3.0 + 0.2 * i);
    bubble->mutable_position()->set_y(1.0);
    bubble->set_message("IoT event " + std::to_string(i));
    bubble->set_expire_at_ms(future_ms);
    bubble->set_visible(true);
  }
  return array;
}

inline automsgs::msgs::strata_msgs::Robot3DLayerArray MakeRobot3DLayers(int count = 1) {
  automsgs::msgs::strata_msgs::Robot3DLayerArray array;
  SetHeader(array.mutable_header());
  for (int i = 0; i < count; ++i) {
    auto* layer = array.add_layers();
    SetHeader(layer->mutable_header());
    layer->set_id("r3d_" + std::to_string(i));
    layer->set_robot_id("robot_" + std::to_string(i));
    layer->set_model_url("/assets/models/guide-bot.glb");
    layer->mutable_position()->set_x(0.5 + 0.4 * i);
    layer->mutable_position()->set_y(0.3);
    layer->set_heading_deg(45.0);
    layer->set_status("running");
    layer->set_scale(1.0);
    layer->set_visible(true);
  }
  return array;
}

inline automsgs::msgs::visualization_msgs::MarkerArray MakeBuildingExtrusionMarkers() {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  auto* outline = array.add_markers();
  *outline = MakeMarker(automsgs::msgs::visualization_msgs::Marker::LINE_STRIP, 0,
                        "strata_building");
  const std::vector<std::pair<double, double>> footprint = {
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {0.0, 1.5}, {0.0, 0.0}};
  for (const auto& [x, y] : footprint) {
    auto* pt = outline->add_points();
    pt->set_x(x);
    pt->set_y(y);
  }
  auto* extrusion = array.add_markers();
  *extrusion = MakeMarker(automsgs::msgs::visualization_msgs::Marker::CUBE, 1,
                          "strata_building_extrusion");
  extrusion->mutable_pose()->mutable_position()->set_x(1.0);
  extrusion->mutable_pose()->mutable_position()->set_y(0.75);
  extrusion->mutable_pose()->mutable_position()->set_z(1.5);
  extrusion->mutable_scale()->set_x(2.0);
  extrusion->mutable_scale()->set_y(1.5);
  extrusion->mutable_scale()->set_z(3.0);
  extrusion->mutable_color()->set_r(0.4f);
  extrusion->mutable_color()->set_g(0.5f);
  extrusion->mutable_color()->set_b(0.7f);
  extrusion->mutable_color()->set_a(0.85f);
  return array;
}

inline automsgs::msgs::strata_msgs::FloorInfoArray MakeFloorInfoArray() {
  automsgs::msgs::strata_msgs::FloorInfoArray array;
  SetHeader(array.mutable_header());
  array.set_active_floor_id("F1");
  for (const auto& [id, name, level] :
       {std::tuple{"F1", "1F", 1}, {"F2", "2F", 2}}) {
    auto* floor = array.add_floors();
    floor->set_id(id);
    floor->set_name(name);
    floor->set_level(level);
    floor->set_resolution(0.05);
    floor->set_x_grid_count(100);
    floor->set_y_grid_count(80);
  }
  return array;
}

}  // namespace bicmap
}  // namespace tests
}  // namespace autoviz
