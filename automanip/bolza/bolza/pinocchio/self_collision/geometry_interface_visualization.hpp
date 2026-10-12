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

#pragma once

/**
 * @file geometry_interface_visualization.hpp
 * @brief Nearest-point pair and signed distance for one collision pair (ROS-free).
 */

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <bolza/core/types.hpp>
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>
#include <bolza/pinocchio/self_collision/pinocchio_geometry_interface.hpp>

namespace bolza {
/**
 *
 * Nearest-point pair and signed distance for one collision pair (ROS-free).
 */
struct GeometryDistanceSegment {
  Eigen::Matrix<scalar_t, 3, 1> pointA = Eigen::Matrix<scalar_t, 3, 1>::Zero();
  Eigen::Matrix<scalar_t, 3, 1> pointB = Eigen::Matrix<scalar_t, 3, 1>::Zero();
  scalar_t distance = 0.0;
  std::pair<size_t, size_t> collisionPair{0, 0};
};

/**
 *
 * Computes self-collision distance segments for visualization backends
 *
 * (autoviz / custom). Replaces bolza GeometryInterfaceVisualization without ROS.
 */
class GeometryInterfaceVisualization {
 public:
  /**
   * @brief Constructs a GeometryInterfaceVisualization.
   * @param [in] pinocchioInterface: Pinocchio Interface.
   * @param [in] geometryInterface: Geometry Interface.
   * @param [in] pinocchioWorldFrame: Pinocchio World Frame.
   */
  GeometryInterfaceVisualization(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                 std::string pinocchioWorldFrame = "world");

  /**
 * @brief World frame name used by consumers when drawing.
 * @return Result value.
 */

  const std::string& WorldFrame() const { return pinocchioWorldFrame_; }

  /**
 * @brief Update FK and return distance segments for configuration q.
 * @param [in] q: Q.
 * @return Result value.
 */

  std::vector<GeometryDistanceSegment> ComputeDistances(const vector_t& q);

 private:
  PinocchioInterface pinocchioInterface_;
  PinocchioGeometryInterface geometryInterface_;
  std::string pinocchioWorldFrame_;
};

}  // namespace bolza
