/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/tare_visualizer.hpp"

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autonomy::perception::exploration {
namespace {

using automsgs::msgs::visualization_msgs::Marker;

Marker MakeCube(const std::string& frame_id, const std::string& ns, int id,
                double x, double y, double size, float r, float g, float b) {
  Marker marker;
  marker.mutable_header()->set_frame_id(frame_id);
  marker.set_ns(ns);
  marker.set_id(id);
  marker.set_type(Marker::CUBE);
  marker.set_action(Marker::ADD);
  marker.mutable_pose()->mutable_position()->set_x(x);
  marker.mutable_pose()->mutable_position()->set_y(y);
  marker.mutable_pose()->mutable_position()->set_z(0.05);
  marker.mutable_pose()->mutable_orientation()->set_w(1.0);
  marker.mutable_scale()->set_x(size);
  marker.mutable_scale()->set_y(size);
  marker.mutable_scale()->set_z(0.08);
  marker.mutable_color()->set_r(r);
  marker.mutable_color()->set_g(g);
  marker.mutable_color()->set_b(b);
  marker.mutable_color()->set_a(0.35f);
  return marker;
}

}  // namespace

automsgs::msgs::visualization_msgs::MarkerArray BuildTareGraphMarkers(
    const GridWorld& grid_world, const KeyposeGraph& keypose_graph,
    const std::string& frame_id) {
  automsgs::msgs::visualization_msgs::MarkerArray array;
  const int cols = grid_world.cols();
  const int rows = grid_world.rows();
  const double cell_size = grid_world.cell_size();
  int marker_id = 0;

  for (int i = 0; i < cols * rows; ++i) {
    const auto status = grid_world.cell_status()[static_cast<size_t>(i)];
    if (status == CellStatus::kUnseen) {
      continue;
    }
    const auto center = grid_world.CellCenter(i);
    float r = 0.5f;
    float g = 0.5f;
    float b = 0.5f;
    if (status == CellStatus::kExploring) {
      r = 1.0f;
      g = 0.8f;
      b = 0.1f;
    } else if (status == CellStatus::kCovered) {
      r = 0.2f;
      g = 0.8f;
      b = 0.3f;
    }
    *array.add_markers() = MakeCube(frame_id, "tare_grid_cells", marker_id++,
                                    center.x(), center.y(), cell_size * 0.9,
                                    r, g, b);
  }

  Marker edge_marker;
  edge_marker.mutable_header()->set_frame_id(frame_id);
  edge_marker.set_ns("tare_keypose_edges");
  edge_marker.set_id(0);
  edge_marker.set_type(Marker::LINE_LIST);
  edge_marker.set_action(Marker::ADD);
  edge_marker.mutable_scale()->set_x(0.06);
  edge_marker.mutable_color()->set_r(0.1f);
  edge_marker.mutable_color()->set_g(0.6f);
  edge_marker.mutable_color()->set_b(1.0f);
  edge_marker.mutable_color()->set_a(0.9f);

  const auto& nodes = keypose_graph.nodes();
  const auto& adjacency = keypose_graph.adjacency();
  for (size_t i = 0; i < nodes.size(); ++i) {
    Marker sphere;
    sphere.mutable_header()->set_frame_id(frame_id);
    sphere.set_ns("tare_keypose_nodes");
    sphere.set_id(static_cast<int>(i));
    sphere.set_type(Marker::SPHERE);
    sphere.set_action(Marker::ADD);
    sphere.mutable_pose()->mutable_position()->set_x(nodes[i].x);
    sphere.mutable_pose()->mutable_position()->set_y(nodes[i].y);
    sphere.mutable_pose()->mutable_position()->set_z(0.12);
    sphere.mutable_pose()->mutable_orientation()->set_w(1.0);
    sphere.mutable_scale()->set_x(0.3);
    sphere.mutable_scale()->set_y(0.3);
    sphere.mutable_scale()->set_z(0.3);
    sphere.mutable_color()->set_r(0.1f);
    sphere.mutable_color()->set_g(0.5f);
    sphere.mutable_color()->set_b(1.0f);
    sphere.mutable_color()->set_a(0.95f);
    *array.add_markers() = sphere;

    if (i >= adjacency.size()) {
      continue;
    }
    for (int neighbor : adjacency[i]) {
      if (static_cast<int>(i) >= neighbor) {
        continue;
      }
      if (neighbor < 0 || static_cast<size_t>(neighbor) >= nodes.size()) {
        continue;
      }
      auto* pa = edge_marker.add_points();
      pa->set_x(nodes[i].x);
      pa->set_y(nodes[i].y);
      pa->set_z(0.1);
      auto* pb = edge_marker.add_points();
      pb->set_x(nodes[static_cast<size_t>(neighbor)].x);
      pb->set_y(nodes[static_cast<size_t>(neighbor)].y);
      pb->set_z(0.1);
    }
  }
  if (edge_marker.points_size() > 0) {
    *array.add_markers() = edge_marker;
  }
  return array;
}

}  // namespace autonomy::perception::exploration
