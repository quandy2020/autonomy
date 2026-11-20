// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_visualizer.hpp
 * @brief Publish native field, headland and operational swath debug geometry. */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
#include "autolink/node/node.hpp"
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
namespace autonomy::task::coverage {
/** @brief Adapt Open Navigation coverage visualization to native marker channels. */
class CoverageVisualizer {
 public:
  /** @brief Create field, headland, swath and combined marker publishers. */
  CoverageVisualizer(std::shared_ptr<autolink::Node>, const std::string& prefix);
  /** @brief Publish geometry from the exact computed Cartesian coverage result. */
  void Publish(const ComputeAction::Result&);
 private:
  using Markers = automsgs::msgs::visualization_msgs::MarkerArray;
  std::shared_ptr<autolink::Writer<Markers>> fields_;
  std::shared_ptr<autolink::Writer<Markers>> headlands_;
  std::shared_ptr<autolink::Writer<Markers>> swaths_;
  std::shared_ptr<autolink::Writer<Markers>> combined_;
  std::shared_ptr<autolink::Writer<Markers>> regions_;
};
}
