/* Copyright 2026 The Openbot Authors. */
/**
 * @file vista_graph_visualizer.hpp
 * @brief Publish the persistent Vista visibility graph for Autoviz.
 *
 * The visualizer is observation-only. It copies a completed graph snapshot and
 * never participates in route admission, collision checking, or motion safety.
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <autolink/node/node.hpp>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/planning/planner/vista/route_planner.hpp"

namespace autonomy::planning::planner::vista {

/** @brief Converts one immutable Vista graph snapshot into native markers. */
class VistaGraphVisualizer {
 public:
  /**
   * @brief Create the graph marker channel.
   * @param node Native Autolink node owned by the planner server.
   * @param channel MarkerArray channel opened for a latest-value display.
   */
  VistaGraphVisualizer(std::shared_ptr<autolink::Node> node,
                       std::string channel);

  /**
   * @brief Publish graph topology and the last committed route.
   * @param graph Immutable graph snapshot taken after an accepted update.
   * @param route Last committed route, or empty before the first route.
   * @param frame_id Coordinate frame shared by graph, route and map.
   * @return True when the native transport accepted the marker message.
   */
  bool Publish(const VisibilityGraph& graph,
               const std::optional<VistaRoute>& route,
               const std::string& frame_id);

  /** @brief Publish a delete-all marker so a stale graph disappears. */
  bool Clear();

  /** @return Channel opened by this visualizer. */
  const std::string& channel() const { return channel_; }

 private:
  using Markers = automsgs::msgs::visualization_msgs::MarkerArray;

  std::shared_ptr<autolink::Writer<Markers>> writer_;
  std::string channel_;
  std::uint64_t last_geometry_revision_ = 0;
  std::uint64_t last_route_signature_ = 0;
  std::chrono::steady_clock::time_point last_publish_time_{};
};

}  // namespace autonomy::planning::planner::vista
