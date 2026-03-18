/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>

#include "autonomy/tools/aviz/common/display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"
#include "autonomy/tools/aviz/common/properties/vector_property.hpp"

namespace aviz {
namespace rendering {
class Grid;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief GridDisplay
 * Grid display
 *
 * Displays a grid in either the XY, YZ, or XZ plane.
 */
class GridDisplay : public aviz::common::Display {
  Q_OBJECT

 public:
  enum Plane {
    XY,
    XZ,
    YZ,
  };

  GridDisplay();
  ~GridDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;

 private Q_SLOTS:
  void updateCellCount();
  void updateCellSize();
  void updateColor();
  void updateHeight();
  void updateLineWidth();
  void updateOffset();
  void updatePlane();
  void updateStyle();

 private:
  std::unique_ptr<aviz::rendering::Grid> grid_;  ///< Handles actually drawing the grid

  aviz::common::properties::IntProperty* cell_count_property_;
  aviz::common::properties::IntProperty* height_property_;
  aviz::common::properties::FloatProperty* cell_size_property_;
  aviz::common::properties::FloatProperty* line_width_property_;
  aviz::common::properties::EnumProperty* style_property_;
  aviz::common::properties::ColorProperty* color_property_;
  aviz::common::properties::FloatProperty* alpha_property_;
  aviz::common::properties::EnumProperty* plane_property_;
  aviz::common::properties::VectorProperty* offset_property_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
