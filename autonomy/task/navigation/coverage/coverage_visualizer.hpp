/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file coverage_visualizer.hpp
 * @brief Publish native field, headland and operational swath debug geometry.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_VISUALIZER_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_VISUALIZER_HPP_

#include "autonomy/task/navigation/coverage/types.hpp"
#include "autolink/node/node.hpp"
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
namespace autonomy::task::coverage {

/**
 * @class autonomy::task::coverage::CoverageVisualizer
 * @brief Publishes the planned geometry to the native marker channels.
 *
 * Markers carry the field outline, the headland rings, the ordered swaths, one
 * translucent fill per room and a coloured stroke per planned room path, which
 * is what AutoViz draws on top of the occupancy map.
 */
class CoverageVisualizer {
 public:
  /**
   * @brief Opens the marker writers used by @ref Publish.
   * @param[in] node Autolink node owning the writers.
   * @param[in] prefix Channel prefix, normally empty (channels start with
   *             @c /coverage).
   */
  CoverageVisualizer(std::shared_ptr<autolink::Node> node,
                     const std::string& prefix);

  /**
   * @brief Publishes geometry from one computed coverage result.
   *
   * The result is expected in the Cartesian frame; regions without a planned
   * path still receive their fill so an operator can see what was left out.
   *
   * @param[in] result Coverage result to convert into markers.
   */
  void Publish(const ComputeAction::Result& result);

 private:
  // Marker array type shared by every writer below.
  using Markers = automsgs::msgs::visualization_msgs::MarkerArray;

  // Field outline writer.
  std::shared_ptr<autolink::Writer<Markers>> fields_;

  // Headland ring writer.
  std::shared_ptr<autolink::Writer<Markers>> headlands_;

  // Swath writer.
  std::shared_ptr<autolink::Writer<Markers>> swaths_;

  // Combined marker writer used by the panel display.
  std::shared_ptr<autolink::Writer<Markers>> combined_;

  // Per-room fill, outline and path writer.
  std::shared_ptr<autolink::Writer<Markers>> regions_;
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_COVERAGE_VISUALIZER_HPP_
