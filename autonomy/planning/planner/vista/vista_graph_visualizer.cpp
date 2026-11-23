/* Copyright 2026 The Openbot Authors. */
#include "autonomy/planning/planner/vista/vista_graph_visualizer.hpp"

#include <cmath>
#include <utility>

#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autonomy::planning::planner::vista {
namespace {
using Marker = automsgs::msgs::visualization_msgs::Marker;

Marker* AddMarker(Marker* markers, const std::string& frame_id,
                  const std::string& name, int id, Marker::Type type) {
  markers->mutable_header()->set_frame_id(frame_id);
  markers->mutable_header()->mutable_stamp()->CopyFrom(
      automsgs::msgs::builtin_interfaces::TimeNow());
  markers->set_ns(name);
  markers->set_id(id);
  markers->set_type(type);
  markers->set_action(Marker::ADD);
  markers->mutable_pose()->mutable_orientation()->set_w(1.0);
  return markers;
}
}  // namespace

VistaGraphVisualizer::VistaGraphVisualizer(std::shared_ptr<autolink::Node> node,
                                           std::string channel)
    : channel_(std::move(channel)) {
  if (!node || channel_.empty()) {
    return;
  }
  writer_ = node->CreateWriter<Markers>(channel_);
}

bool VistaGraphVisualizer::Publish(const VisibilityGraph& graph,
                                   const std::optional<VistaRoute>& route,
                                   const std::string& frame_id) {
  if (!writer_) {
    return false;
  }

  std::uint64_t route_signature = 0;
  if (route && route->success && !route->path.poses.empty()) {
    const auto& first = route->path.poses.front();
    const auto& last = route->path.poses.back();
    route_signature = static_cast<std::uint64_t>(route->path.poses.size()) * 1315423911ULL;
    route_signature ^= static_cast<std::uint64_t>(std::llround(first.x * 1000.0));
    route_signature ^= static_cast<std::uint64_t>(std::llround(first.y * 1000.0)) << 13U;
    route_signature ^= static_cast<std::uint64_t>(std::llround(last.x * 1000.0)) << 26U;
    route_signature ^= static_cast<std::uint64_t>(std::llround(last.y * 1000.0)) << 39U;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_publish_time_ != std::chrono::steady_clock::time_point{} &&
      now - last_publish_time_ < std::chrono::milliseconds(500)) {
    return true;
  }

  Markers output;
  auto* clear = output.add_markers();
  clear->set_action(Marker::DELETEALL);

  const std::string frame = frame_id.empty() ? "map" : frame_id;

  auto* edges = AddMarker(output.add_markers(), frame, "vista_edges", 1,
                          Marker::LINE_LIST);
  edges->mutable_scale()->set_x(0.018);
  edges->mutable_color()->set_r(0.0F);
  edges->mutable_color()->set_g(0.78F);
  edges->mutable_color()->set_b(0.82F);
  edges->mutable_color()->set_a(0.48F);
  edges->set_frame_locked(true);

  auto* nodes = AddMarker(output.add_markers(), frame, "vista_nodes", 2,
                          Marker::POINTS);
  nodes->mutable_scale()->set_x(0.055);
  nodes->mutable_scale()->set_y(0.055);
  nodes->mutable_color()->set_r(0.0F);
  nodes->mutable_color()->set_g(0.92F);
  nodes->mutable_color()->set_b(0.95F);
  nodes->mutable_color()->set_a(0.90F);
  nodes->set_frame_locked(true);

  for (const auto& entry : graph.nodes) {
    const auto& pose = entry.second.pose;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) {
      continue;
    }
    auto* point = nodes->add_points();
    point->set_x(pose.x);
    point->set_y(pose.y);
    point->set_z(0.055);
  }

  // Ogre software/hardware vertex buffers have a bounded single-buffer
  // capacity. Keep the visual graph representative while the planner retains
  // the complete edge set for routing.
  constexpr std::size_t kMaximumVisualizedEdges = 1500;
  const std::size_t edge_stride = std::max<std::size_t>(
      1, (graph.edges.size() + kMaximumVisualizedEdges - 1) /
             kMaximumVisualizedEdges);
  std::size_t valid_edge_index = 0;
  for (const auto& entry : graph.edges) {
    const auto& edge = entry.second;
    if (!edge.valid) {
      continue;
    }
    const auto first = graph.nodes.find(edge.id.first);
    const auto second = graph.nodes.find(edge.id.second);
    if (first == graph.nodes.end() || second == graph.nodes.end()) {
      continue;
    }
    if ((valid_edge_index++ % edge_stride) != 0) {
      continue;
    }
    const auto& a = first->second.pose;
    const auto& b = second->second.pose;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y)) {
      continue;
    }
    auto* point = edges->add_points();
    point->set_x(a.x);
    point->set_y(a.y);
    point->set_z(0.05);
    point = edges->add_points();
    point->set_x(b.x);
    point->set_y(b.y);
    point->set_z(0.05);
  }

  if (route && route->success && route->path.poses.size() >= 2) {
    auto* path = AddMarker(output.add_markers(), frame, "vista_route", 3,
                           Marker::LINE_STRIP);
    path->mutable_scale()->set_x(0.095);
    path->mutable_color()->set_r(0.18F);
    path->mutable_color()->set_g(0.30F);
    path->mutable_color()->set_b(0.96F);
    path->mutable_color()->set_a(0.96F);
    path->set_frame_locked(true);
    for (const auto& pose : route->path.poses) {
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) {
        continue;
      }
      auto* point = path->add_points();
      point->set_x(pose.x);
      point->set_y(pose.y);
      point->set_z(0.105);
    }
    if (path->points_size() >= 2) {
      auto* start = AddMarker(output.add_markers(), frame, "vista_start", 4,
                              Marker::SPHERE);
      start->mutable_pose()->mutable_position()->set_x(route->path.poses.front().x);
      start->mutable_pose()->mutable_position()->set_y(route->path.poses.front().y);
      start->mutable_pose()->mutable_position()->set_z(0.12);
      start->mutable_scale()->set_x(0.16);
      start->mutable_scale()->set_y(0.16);
      start->mutable_scale()->set_z(0.16);
      start->mutable_color()->set_r(0.20F);
      start->mutable_color()->set_g(0.92F);
      start->mutable_color()->set_b(0.42F);
      start->mutable_color()->set_a(0.95F);

      auto* goal = AddMarker(output.add_markers(), frame, "vista_goal", 5,
                             Marker::SPHERE);
      goal->mutable_pose()->mutable_position()->set_x(route->path.poses.back().x);
      goal->mutable_pose()->mutable_position()->set_y(route->path.poses.back().y);
      goal->mutable_pose()->mutable_position()->set_z(0.12);
      goal->mutable_scale()->set_x(0.18);
      goal->mutable_scale()->set_y(0.18);
      goal->mutable_scale()->set_z(0.18);
      goal->mutable_color()->set_r(1.0F);
      goal->mutable_color()->set_g(0.72F);
      goal->mutable_color()->set_b(0.16F);
      goal->mutable_color()->set_a(0.95F);
    }
  }

  last_geometry_revision_ = graph.geometry_revision;
  last_route_signature_ = route_signature;
  last_publish_time_ = now;
  return writer_->Write(output);
}

bool VistaGraphVisualizer::Clear() {
  if (!writer_) {
    return false;
  }
  Markers output;
  output.add_markers()->set_action(Marker::DELETEALL);
  return writer_->Write(output);
}

}  // namespace autonomy::planning::planner::vista
