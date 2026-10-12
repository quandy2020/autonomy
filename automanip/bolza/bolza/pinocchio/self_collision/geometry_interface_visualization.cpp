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

#include "bolza/pinocchio/self_collision/geometry_interface_visualization.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace bolza {

GeometryInterfaceVisualization::GeometryInterfaceVisualization(PinocchioInterface pinocchioInterface,
                                                               PinocchioGeometryInterface geometryInterface,
                                                               std::string pinocchioWorldFrame)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      geometryInterface_(std::move(geometryInterface)),
      pinocchioWorldFrame_(std::move(pinocchioWorldFrame)) {}

std::vector<GeometryDistanceSegment> GeometryInterfaceVisualization::ComputeDistances(const vector_t& q) {
  const auto& model = pinocchioInterface_.GetModel();
  auto& data = pinocchioInterface_.GetData();
  pinocchio::forwardKinematics(model, data, q);
  const auto results = geometryInterface_.ComputeDistances(pinocchioInterface_);

  std::vector<GeometryDistanceSegment> segments;
  segments.reserve(results.size());
  const auto& pairs = geometryInterface_.GetGeometryModel().collisionPairs;
  for (size_t i = 0; i < results.size(); ++i) {
    GeometryDistanceSegment seg;
    seg.pointA = results[i].nearest_points[0];
    seg.pointB = results[i].nearest_points[1];
    seg.distance = results[i].min_distance;
    if (i < pairs.size()) {
      seg.collisionPair = {static_cast<size_t>(pairs[i].first), static_cast<size_t>(pairs[i].second)};
    }
    segments.push_back(seg);
  }
  return segments;
}

}  // namespace bolza
