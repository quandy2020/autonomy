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

#include "autoview/common/display.hpp"
#include "autoview/common/properties/color_property.hpp"
#include "autoview/common/properties/enum_property.hpp"
#include "autoview/common/properties/float_property.hpp"
#include "autoview/common/properties/int_property.hpp"
#include "autoview/common/properties/tf_frame_property.hpp"
#include "autoview/common/properties/vector_property.hpp"


namespace autoview {
namespace rendering {

class Grid;


/**
 * \class GridDisplay
 * \brief Displays a grid in either the XY, YZ, or XZ plane.
 *
 * For more information see Grid
 */
class GridDisplay : public Display
{
  Q_OBJECT

public:
  enum Plane
  {
    XY,
    XZ,
    YZ,
  };

  GridDisplay();
  ~GridDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

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
  std::unique_ptr<rviz_rendering::Grid> grid_;  ///< Handles actually drawing the grid

  rviz_common::properties::TfFrameProperty * frame_property_;
  rviz_common::properties::IntProperty * cell_count_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::FloatProperty * cell_size_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::EnumProperty * style_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::EnumProperty * plane_property_;
  rviz_common::properties::VectorProperty * offset_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID__GRID_DISPLAY_HPP_
