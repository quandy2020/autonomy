/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

/**
 * @file geometric_path.hpp
 * @brief Resampling and unknown-space truncation of the geometric path.
 *
 * The search returns an irregular polyline. The trajectory optimizer needs a
 * short sequence of spatial knots, one per polynomial piece, and those knots
 * must not sit in unknown space that the prediction horizon cannot see.
 */

#pragma once

#include <vector>

#include "Eigen/Dense"

#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Arc-length resample of a polyline, keeping both endpoints.
 *
 * Samples are spaced by about `spacing` meters. The output is clamped to
 * [2, max_points]. An empty input stays empty. A one-point input is repeated
 * so the caller still has a segment.
 *
 * @param path Input polyline, world meters.
 * @param spacing Nominal spacing, meters. Non-positive spacing returns the endpoints only.
 * @param max_points Upper bound on the number of samples, including the endpoints.
 * @return Resampled polyline. Size is at least 2 when the input is non-empty.
 */
/**
 * @brief Resamples a geometric path and truncates it before unknown space.
 */
class GeometricPath {
 public:
  std::vector<Eigen::Vector2d> Resample(const std::vector<Eigen::Vector2d>& path, double spacing,
                                            int max_points) const;

/**
 * @brief Walk the path backward until unknown cells lie outside the prediction radius.
 *
 * A vertex is unsafe when an unknown cell exists inside radius meters, where
 * radius is the obstacle prediction distance. Unsafe vertices are dropped
 * from the end. The path is never shortened below two points. This is the
 * ground substitute for refusing a local goal inside an unseen region.
 *
 * @param grid Occupancy used to query unknown cells.
 * @param options Supplies the prediction horizon and the obstacle speed that set the radius.
 * @param path Polyline shortened in place. Must be non-null.
 */
  void Truncate(const OccupancyGrid& grid, const proto::SandoControllerOptions& options,
                       std::vector<Eigen::Vector2d>* path) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
