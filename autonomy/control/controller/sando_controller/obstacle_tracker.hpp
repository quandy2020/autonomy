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
 * @file obstacle_tracker.hpp
 * @brief Costmap connected components plus externally injected movers.
 *
 * Clusters are flood-filled only inside a window around the robot. Each
 * cluster stores an axis-aligned box. Velocity and acceleration are a
 * complementary filter of the box-center motion. A cluster slower than the
 * velocity threshold and the acceleration threshold is treated as static and
 * its velocity is zeroed. External obstacles survive the next association.
 */

#pragma once

#include <vector>

#include "autonomy/control/controller/sando_controller/occupancy_grid.hpp"
#include "autonomy/control/controller/sando_controller/types.hpp"
#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @class ObstacleTracker
 * @brief Associates costmap blobs with the previous dynamic tracks.
 */
class ObstacleTracker {
 public:
  /**
   * @brief Drop every track, including external ones, and reset the id counter.
   */
  void ClearObstacles() { obstacles_.clear(); }

  /**
   * @brief Insert or refresh one external mover.
   *
   * A track whose center is within 0.4 m is updated in place and marked
   * external. last_seen is set to 0 so the next TrackObstacles stamps it.
   * Otherwise a new id is allocated. Half-extents are the given radius.
   *
   * @param x Center x, meters.
   * @param y Center y, meters.
   * @param velocity_x World x velocity, m/s.
   * @param velocity_y World y velocity, m/s.
   * @param radius Circumscribed radius, meters.
   * @param robot_radius Floor on the stored radius, meters.
   */
  void AddObstacle(double x, double y, double velocity_x, double velocity_y, double radius, double robot_radius);

  /**
   * @brief Rebuild tracks from occupied connected components.
   *
   * Seeds are limited to a window around the robot. A component is kept when
   * its cell count lies in [min_cluster, max_cluster]. The measurement
   * velocity is (blob - previous) / dt and the measurement acceleration is
   * (meas_v - v) / dt. The filter is alpha * old + (1 - alpha) * measurement.
   * Speed is clamped to obst_max_vel. External tracks are matched and not
   * overwritten. Unmatched non-external tracks older than the lifetime are
   * dropped.
   *
   * @param grid Occupancy used for the flood fill. An empty grid clears the tracks.
   * @param options Cluster limits, filter gains, thresholds, and the lifetime.
   * @param now Clock time, seconds.
   * @param robot_x Window center x, meters.
   * @param robot_y Window center y, meters.
   */
  void TrackObstacles(const OccupancyGrid& grid, const proto::SandoControllerOptions& options, double now,
             double robot_x, double robot_y);

  /**
   * @brief Tracks from the last AddObstacle and TrackObstacles calls.
   * @return Reference valid until the next mutating call.
   */
  const std::vector<DynObstacle>& GetObstacles() const { return obstacles_; }

 private:
  std::vector<DynObstacle> obstacles_;  ///< Live tracks. External entries have external == true.
  int next_identifier_{1};                      ///< Next id assigned to a new cluster or a new external mover.
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
