/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#ifndef AUTOMANIP_EXAMPLES_LEGGED_ROBOT_TERRAIN_TERRAIN_HPP_
#define AUTOMANIP_EXAMPLES_LEGGED_ROBOT_TERRAIN_TERRAIN_HPP_

#include <string>
#include <vector>

#include "examples/legged_robot/common/types.hpp"

namespace bolza {
namespace legged_robot {

/**
 * Height field used for footholds and the base reference.
 * `height` is the raw surface (feet). `smoothHeight` and `normal` follow a
 * blurred copy so the torso does not pitch into each stair riser.
 */
class Terrain {
 public:
  static Terrain flat();
  /** Loads `terrain.info`. Unknown files fall back to a flat field. */
  static Terrain load(const std::string& info_file);
  /** Rows of heights. The first line is `origin_x origin_y resolution`. */
  static Terrain loadHeightFile(const std::string& path);
  static Terrain grid(double origin_x, double origin_y, double resolution, int cols, int rows,
                      std::vector<double> raw, double com_height, double max_slope, double smooth_radius,
                      std::string description);

  double height(double x, double y) const;
  double smoothHeight(double x, double y) const;
  vector3_t normal(double x, double y) const;

  bool hasRelief() const { return relief_; }
  double comHeight() const { return com_height_; }
  double maxSlope() const { return max_slope_; }
  const std::string& description() const { return description_; }

  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  double resolution() const { return resolution_; }
  int columns() const { return cols_; }
  int rows() const { return rows_; }
  double cell(int ix, int iy) const;

 private:
  double sample(const std::vector<double>& grid, double x, double y) const;
  void finish(double smooth_radius);

  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double resolution_ = 0.05;
  int cols_ = 0;
  int rows_ = 0;
  double com_height_ = 0.575;
  double max_slope_ = 0.4;
  bool relief_ = false;
  std::string description_ = "flat";
  std::vector<double> raw_;
  std::vector<double> smooth_;
};

}  // namespace legged_robot
}  // namespace bolza

#endif  // AUTOMANIP_EXAMPLES_LEGGED_ROBOT_TERRAIN_TERRAIN_HPP_
